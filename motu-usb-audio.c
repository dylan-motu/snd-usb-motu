// SPDX-License-Identifier: GPL-2.0-only
/*
 * MOTU Pro Audio USB Driver
 */
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

MODULE_DESCRIPTION("MOTU Pro Audio USB Driver");
MODULE_AUTHOR("Dylan Robinson <dylan_robinson@motu.com>");
MODULE_LICENSE("GPL v2");

#define NUM_URBS            32
#define UFRAMES_PER_URB     16
#define START_URB_UFRAMES   128
#define NUM_CH              24
#define BYTES_PER_SAMPLE    3
#define BYTES_PER_FRAME     (NUM_CH * BYTES_PER_SAMPLE)
#define PERF_INTERVAL       5000

static struct usb_driver snd_usb_motu_driver;

struct motu_perf
{
    const char* name;
    u64 last;
    u64 max;
    u64 min;
    u32 num;
};

struct motu_stream
{
    bool enabled;
    struct snd_pcm_substream *substream;
    unsigned int period_frames;
    unsigned int period_count;
    unsigned int period_idx;
    unsigned int frame_pos;
    unsigned int buffer_pos;
    struct urb *start_urb;
    struct urb *urbs[NUM_URBS];
};

struct motu_usb_data
{
    spinlock_t lock;
    atomic_t streams_started;
    struct usb_device *usb;
    struct snd_card *card;
    struct workqueue_struct *start_stop_wq;
    struct work_struct start_streams_work;
    struct delayed_work stop_streams_work;
    struct motu_stream rec_stream;
    struct motu_stream pb_stream;
    struct hrtimer timer;
    struct motu_perf perf1;
    struct motu_perf perf2;
    unsigned int pb_urb_idx;
};

static struct snd_pcm_hardware snd_motu_hw = {
    .info = (SNDRV_PCM_INFO_MMAP |
             SNDRV_PCM_INFO_MMAP_VALID |
             SNDRV_PCM_INFO_INTERLEAVED |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_BATCH),
    .formats =          SNDRV_PCM_FMTBIT_S24_3LE,
    .rates =            SNDRV_PCM_RATE_48000,
    .rate_min =         48000,
    .rate_max =         48000,
    .channels_min =     NUM_CH,
    .channels_max =     NUM_CH,
    .buffer_bytes_max = BYTES_PER_FRAME * 2048,
    .period_bytes_min = BYTES_PER_FRAME * 128,
    .period_bytes_max = BYTES_PER_FRAME * 1024,
    .periods_min =      2,
    .periods_max =      16,
};

/* x86 only */
static inline u64 tsc_to_ns(void)
{
    u64 cycles;
    u32 hi, lo;

    asm volatile("rdtsc" : "=a"(lo), "=d"(hi));
    cycles = ((u64)hi << 32) | lo;

    return div64_u64(cycles * 1000000ULL, tsc_khz);
}

static inline void reset_perf(struct motu_perf *perf)
{
    perf->max = 0;
    perf->min = 0xFFFFFFFFFFFFFFFFULL;
    perf->num = 0;
}

static inline void init_perf(struct motu_perf *perf)
{
    perf->last = tsc_to_ns();
    reset_perf(perf);
}

static inline void log_perf(struct motu_perf *perf, struct usb_device *usb)
{
    dev_info(&usb->dev, "Interval (%llu, %llu) %s",
        perf->min, perf->max, perf->name);
}

static inline void update_perf(struct motu_perf *perf, struct usb_device *usb)
{
    u64 now = tsc_to_ns();
    u64 delta = now - perf->last;

    perf->last = now;

    if (delta > perf->max)
        perf->max = delta;
    
    if (delta < perf->min)
        perf->min = delta;
    
    if (++perf->num >= PERF_INTERVAL) {
        log_perf(perf, usb);
        reset_perf(perf);
    }
}

static enum hrtimer_restart timer_callback(struct hrtimer *timer)
{
    struct motu_usb_data *priv =
        container_of(timer, struct motu_usb_data, timer);
    
    hrtimer_forward_now(timer, 2000000);
    update_perf(&priv->perf2, priv->usb);

    return HRTIMER_RESTART;
}

static void reset_substream_position(struct motu_stream *stream)
{
    stream->period_idx = 0;
    stream->frame_pos = 0;
    stream->buffer_pos = 0;
}

static unsigned int substream_position(const struct motu_stream *stream)
{
    return (stream->period_idx * stream->period_frames) + stream->frame_pos;
}

static unsigned int substream_position_bytes(const struct motu_stream *stream)
{
    return substream_position(stream) * BYTES_PER_FRAME;
}

/**
 * inc_frame_position - Advance frame position and update period index
 * @stream: pointer to stream state
 * @count:  number of frames to increment
 *
 * Return: true if the period elapsed
 */
static bool inc_frame_position(struct motu_stream *stream, unsigned int count)
{
    stream->frame_pos += count;

    if (stream->frame_pos >= stream->period_frames) {
        stream->frame_pos -= stream->period_frames;
        
        if (++stream->period_idx >= stream->period_count)
            stream->period_idx = 0;
        
        return true;
    }

    return false;
}

/**
 * copy_to_usb - Copy audio data from ALSA buffer to USB buffer
 * @dst:        destination USB buffer
 * @dst_offset: offset in the destination buffer
 * @stream:     pointer to stream state
 * @length:     number of bytes to copy
 *
 * Return: true if a period has elapsed, false otherwise
 */
static bool copy_to_usb(unsigned char *dst, unsigned int dst_offset,
    struct motu_stream *stream, unsigned int length)
{
    unsigned char* src;
    unsigned int src_size;
    unsigned int src_offset;
    unsigned int src_end_offset;
    unsigned int src_over;

    if (!stream->period_count)
        return false;
    
    src = stream->substream->runtime->dma_area;
    src_size = stream->substream->runtime->dma_bytes;
    src_offset = substream_position_bytes(stream);
    src_end_offset = src_offset + length;
    src_over = (src_end_offset > src_size) ? (src_end_offset - src_size) : 0;

    memcpy(dst + dst_offset, src + src_offset, length - src_over);

    if (src_over)
        memcpy(dst + dst_offset + (length - src_over), src, src_over);
    
    return inc_frame_position(stream, length / BYTES_PER_FRAME);
}

/**
 * copy_from_usb - Copy audio data from USB buffer to ALSA buffer
 * @src:        source USB buffer
 * @src_offset: offset in the source buffer
 * @stream:     pointer to stream state
 * @length:     number of bytes to copy
 *
 * Return: true if a period has elapsed
 */
static bool copy_from_usb(const unsigned char *src, unsigned int src_offset,
    struct motu_stream *stream, unsigned int length)
{
    unsigned char* dst;
    unsigned int dst_size;
    unsigned int dst_offset;
    unsigned int dst_end_offset;
    unsigned int dst_over;

    if (!stream->period_count)
        return false;

    dst = stream->substream->runtime->dma_area;
    dst_size = stream->substream->runtime->dma_bytes;
    dst_offset = substream_position_bytes(stream);
    dst_end_offset = dst_offset + length;
    dst_over = (dst_end_offset > dst_size) ? (dst_end_offset - dst_size) : 0;

    memcpy(dst + dst_offset, src + src_offset, length - dst_over);

    if (dst_over)
        memcpy(dst, src + src_offset + (length - dst_over), dst_over);

    return inc_frame_position(stream, length / BYTES_PER_FRAME);
}

static void playback_complete_urb(struct urb *urb)
{

}

static void capture_complete_urb(struct urb *urb)
{
    struct motu_usb_data *priv = urb->context;
    struct urb* pb_urb;
    bool is_start = urb == priv->rec_stream.start_urb;
    bool rec_period_elapsed = false;
    bool pb_period_elapsed = false;
    bool locked = false;

    if (!atomic_read(&priv->streams_started))
        return;

    if (is_start) {
        init_perf(&priv->perf1);
        init_perf(&priv->perf2);
        hrtimer_start(&priv->timer, 2000000, HRTIMER_MODE_REL);
        pb_urb = priv->pb_stream.start_urb;
        priv->pb_urb_idx = 0;
    }
    else {
        update_perf(&priv->perf1, priv->usb);
        pb_urb = priv->pb_stream.urbs[priv->pb_urb_idx];
        priv->pb_urb_idx = (priv->pb_urb_idx + 1) % NUM_URBS;
    }
    
    if (!is_start)
        locked = spin_trylock(&priv->lock);

    for (int i = 0; i < urb->number_of_packets; ++i) {
        /* Nominal Packet Size */
        unsigned int length = BYTES_PER_FRAME * 6;
        
        if (!urb->iso_frame_desc[i].status)
            length = urb->iso_frame_desc[i].actual_length;
        else
            dev_info(&priv->usb->dev, "rec urb error");
        
        pb_urb->iso_frame_desc[i].length = length;

        if (locked)
        {
            rec_period_elapsed |= copy_from_usb(urb->transfer_buffer,
                urb->iso_frame_desc[i].offset, &priv->rec_stream, length);
            pb_period_elapsed |= copy_to_usb(pb_urb->transfer_buffer,
                pb_urb->iso_frame_desc[i].offset, &priv->pb_stream, length);
        }
    }

    if (locked) {
        priv->rec_stream.buffer_pos = substream_position(&priv->rec_stream);
        priv->pb_stream.buffer_pos = substream_position(&priv->pb_stream);
    }
    
    if (rec_period_elapsed)
        snd_pcm_period_elapsed(priv->rec_stream.substream);
    
    if (pb_period_elapsed)
        snd_pcm_period_elapsed(priv->pb_stream.substream);
    
    if (locked)
        spin_unlock(&priv->lock);
    
    if (!is_start)
        usb_submit_urb(urb, GFP_ATOMIC);
    
    usb_submit_urb(pb_urb, GFP_ATOMIC);
}

static void start_streaming_endpoints(struct work_struct *work)
{
    struct motu_usb_data *priv =
        container_of(work, struct motu_usb_data, start_streams_work);
    struct motu_stream *stream = &priv->rec_stream;
    struct urb **urbs = stream->urbs;

    if (atomic_cmpxchg(&priv->streams_started, 0, 1))
        return;
    
    dev_info(&priv->usb->dev, "start_streaming_endpoints");
    
    usb_set_interface(priv->usb, 2, 1); // start record
    usb_set_interface(priv->usb, 1, 1); // start playback

    usb_submit_urb(stream->start_urb, GFP_ATOMIC);

    for (int i = 0; i < NUM_URBS; ++i)
        usb_submit_urb(urbs[i], GFP_ATOMIC);
}

static void stop_streaming_endpoints(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct motu_usb_data *priv =
        container_of(dwork, struct motu_usb_data, stop_streams_work);
    
    if (!atomic_cmpxchg(&priv->streams_started, 1, 0))
        return;
    
    dev_info(&priv->usb->dev, "stop_streaming_endpoints");
    
    hrtimer_cancel(&priv->timer);

    usb_kill_urb(priv->rec_stream.start_urb);
    usb_kill_urb(priv->pb_stream.start_urb);

    for (int i = 0; i < NUM_URBS; ++i) {
        usb_kill_urb(priv->rec_stream.urbs[i]);
        usb_kill_urb(priv->pb_stream.urbs[i]);
    }
    
    usb_set_interface(priv->usb, 2, 0); // stop record
    usb_set_interface(priv->usb, 1, 0); // stop playback
}

static void free_stream_urbs(struct motu_usb_data *priv,
    struct motu_stream *stream)
{
    struct urb **urbs = stream->urbs;

    if (stream->start_urb)
    {
        usb_kill_urb(stream->start_urb);
        usb_free_urb(stream->start_urb);
        stream->start_urb = NULL;
    }

    for (int i = 0; i < NUM_URBS; ++i) {
        
        if (!urbs[i])
            continue;
        
        usb_kill_urb(urbs[i]);
        
        if (urbs[i]->transfer_buffer)
        {
            usb_free_coherent(priv->usb,
                urbs[i]->transfer_buffer_length,
                urbs[i]->transfer_buffer,
                urbs[i]->transfer_dma);
        }
        
        usb_free_urb(urbs[i]);

        urbs[i] = NULL;
    }
}

static int alloc_stream_urbs(struct motu_usb_data *priv,
    struct motu_stream *stream, struct usb_interface *intf,
    usb_complete_t completion_handler)
{
    const struct usb_host_interface *alts = usb_altnum_to_altsetting(intf, 1);
    const struct usb_endpoint_descriptor *epd = &alts->endpoint[0].desc;
    struct urb **urbs = stream->urbs;
    unsigned int pipe;
    unsigned int max_packet_size;
    unsigned int transfer_buffer_length;
    
    if (usb_endpoint_dir_in(epd))
        pipe = usb_rcvisocpipe(priv->usb, usb_endpoint_num(epd));
    else
        pipe = usb_sndisocpipe(priv->usb, usb_endpoint_num(epd));
    
    max_packet_size = usb_endpoint_maxp(epd) * usb_endpoint_maxp_mult(epd);
    transfer_buffer_length = max_packet_size * UFRAMES_PER_URB;

    for (int i = 0; i < NUM_URBS; ++i) {
        int offset = 0;

        urbs[i] = usb_alloc_urb(UFRAMES_PER_URB, GFP_KERNEL);
        
        if (!urbs[i])
            goto out_free;
        
        urbs[i]->dev = priv->usb;
        urbs[i]->pipe = pipe;
        urbs[i]->transfer_flags = URB_NO_TRANSFER_DMA_MAP | URB_ISO_ASAP;
        urbs[i]->transfer_buffer = usb_alloc_coherent(priv->usb,
            transfer_buffer_length, GFP_KERNEL, &urbs[i]->transfer_dma);
        
        if (!urbs[i]->transfer_buffer)
            goto out_free;
        
        urbs[i]->transfer_buffer_length = transfer_buffer_length;
        urbs[i]->number_of_packets = UFRAMES_PER_URB;
        urbs[i]->interval = 1;
        urbs[i]->context = priv;
        urbs[i]->complete = completion_handler;

        for (int f = 0; f < UFRAMES_PER_URB; ++f) {
            urbs[i]->iso_frame_desc[f].offset = offset;
            urbs[i]->iso_frame_desc[f].length = max_packet_size;
            offset += max_packet_size;
        }
    }

    stream->start_urb = usb_alloc_urb(START_URB_UFRAMES, GFP_KERNEL);

    if (!stream->start_urb)
        goto out_free;
    
    stream->start_urb->dev = priv->usb;
    stream->start_urb->pipe = pipe;
    stream->start_urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP | URB_ISO_ASAP;
    stream->start_urb->transfer_buffer = urbs[0]->transfer_buffer;
    stream->start_urb->transfer_dma = urbs[0]->transfer_dma;
    stream->start_urb->transfer_buffer_length = transfer_buffer_length;
    stream->start_urb->number_of_packets = START_URB_UFRAMES;
    stream->start_urb->interval = 1;
    stream->start_urb->context = priv;
    stream->start_urb->complete = completion_handler;

    for (int f = 0; f < START_URB_UFRAMES; ++f) {
        stream->start_urb->iso_frame_desc[f].offset = 0;
        stream->start_urb->iso_frame_desc[f].length = max_packet_size;
    }

    return 0;

out_free:
    free_stream_urbs(priv, stream);
    return -ENOMEM;
}

static void queue_start_streaming(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    cancel_delayed_work(&priv->stop_streams_work);
    queue_work(priv->start_stop_wq, &priv->start_streams_work);
}

static void maybe_queue_stop_streaming(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    if (priv->rec_stream.enabled || priv->pb_stream.enabled)
        return;

    cancel_work(&priv->start_streams_work);
    queue_delayed_work(priv->start_stop_wq, &priv->stop_streams_work,
        msecs_to_jiffies(2000));
}

static int capture_pcm_open(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "capture_pcm_open");

    subs->runtime->hw = snd_motu_hw;

    spin_lock(&priv->lock);
    priv->rec_stream.substream = subs;
    priv->rec_stream.period_count = 0;
    spin_unlock(&priv->lock);

    return 0;
}

static int capture_pcm_close(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "capture_pcm_close");

    spin_lock(&priv->lock);
    priv->rec_stream.substream = NULL;
    priv->rec_stream.period_count = 0;
    spin_unlock(&priv->lock);

    priv->rec_stream.enabled = false;
    maybe_queue_stop_streaming(subs);

    return 0;
}

static int capture_pcm_hw_params(struct snd_pcm_substream *subs,
            struct snd_pcm_hw_params *hw_params)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "Capture Channels: %u",
        params_channels(hw_params));
    dev_info(&priv->usb->dev, "Capture Sample Rate: %u",
        params_rate(hw_params));
    dev_info(&priv->usb->dev, "Capture Buffer Bytes: %u",
        params_buffer_bytes(hw_params));
    dev_info(&priv->usb->dev, "Capture Periods: %u",
        params_periods(hw_params));
    dev_info(&priv->usb->dev, "Capture Period Size: %u",
        params_period_size(hw_params));
    
    spin_lock(&priv->lock);
    reset_substream_position(&priv->rec_stream);
    priv->rec_stream.period_count = params_periods(hw_params);
    priv->rec_stream.period_frames = params_period_size(hw_params);
    spin_unlock(&priv->lock);
    
    return 0;
}

static int capture_pcm_prepare(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "capture_pcm_prepare");

    spin_lock(&priv->lock);
    reset_substream_position(&priv->rec_stream);
    spin_unlock(&priv->lock);

    return 0;
}

static int capture_pcm_trigger(struct snd_pcm_substream *subs, int cmd)
{
    struct motu_usb_data *priv = subs->private_data;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        dev_info(&priv->usb->dev,
            "capture_pcm_trigger start (stop_threshold:%lu)",
            subs->runtime->stop_threshold);
        priv->rec_stream.enabled = true;
        queue_start_streaming(subs);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        dev_info(&priv->usb->dev, "capture_pcm_trigger stop");
        priv->rec_stream.enabled = false;
        maybe_queue_stop_streaming(subs);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static snd_pcm_uframes_t capture_pcm_pointer(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    return priv->rec_stream.buffer_pos;
}

static const struct snd_pcm_ops capture_pcm_ops = {
    .open = capture_pcm_open,
    .close = capture_pcm_close,
    .hw_params = capture_pcm_hw_params,
    .prepare = capture_pcm_prepare,
    .trigger = capture_pcm_trigger,
    .pointer = capture_pcm_pointer,
};

static int playback_pcm_open(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_open");

    subs->runtime->hw = snd_motu_hw;

    spin_lock(&priv->lock);
    priv->pb_stream.substream = subs;
    priv->pb_stream.period_count = 0;
    spin_unlock(&priv->lock);

    return 0;
}

static int playback_pcm_close(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_close");

    spin_lock(&priv->lock);
    priv->pb_stream.substream = NULL;
    priv->pb_stream.period_count = 0;
    spin_unlock(&priv->lock);

    priv->pb_stream.enabled = false;
    maybe_queue_stop_streaming(subs);

    return 0;
}

static int playback_pcm_hw_params(struct snd_pcm_substream *subs,
            struct snd_pcm_hw_params *hw_params)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "Playback Channels: %u",
        params_channels(hw_params));
    dev_info(&priv->usb->dev, "Playback Sample Rate: %u",
        params_rate(hw_params));
    dev_info(&priv->usb->dev, "Playback Buffer Bytes: %u",
        params_buffer_bytes(hw_params));
    dev_info(&priv->usb->dev, "Playback Periods: %u",
        params_periods(hw_params));
    dev_info(&priv->usb->dev, "Playback Period Size: %u",
        params_period_size(hw_params));
    
    spin_lock(&priv->lock);
    reset_substream_position(&priv->pb_stream);
    priv->pb_stream.period_count = params_periods(hw_params);
    priv->pb_stream.period_frames = params_period_size(hw_params);
    spin_unlock(&priv->lock);
    
    return 0;
}

static int playback_pcm_prepare(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_prepare");

    spin_lock(&priv->lock);
    reset_substream_position(&priv->pb_stream);
    spin_unlock(&priv->lock);

    return 0;
}

static int playback_pcm_trigger(struct snd_pcm_substream *subs, int cmd)
{
    struct motu_usb_data *priv = subs->private_data;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        dev_info(&priv->usb->dev,
            "playback_pcm_trigger start (stop_threshold:%lu)",
            subs->runtime->stop_threshold);
        priv->pb_stream.enabled = true;
        queue_start_streaming(subs);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        dev_info(&priv->usb->dev, "playback_pcm_trigger stop");
        priv->pb_stream.enabled = false;
        maybe_queue_stop_streaming(subs);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static snd_pcm_uframes_t playback_pcm_pointer(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;
    struct motu_stream *stream = &priv->pb_stream;

    return stream->buffer_pos;
}

static const struct snd_pcm_ops playback_pcm_ops = {
    .open = playback_pcm_open,
    .close = playback_pcm_close,
    .hw_params = playback_pcm_hw_params,
    .prepare = playback_pcm_prepare,
    .trigger = playback_pcm_trigger,
    .pointer = playback_pcm_pointer,
};

static int pcm_init(struct motu_usb_data *priv)
{
    int err;
    struct snd_pcm *pcm;

    err = snd_pcm_new(priv->card, "MOTU Pro Audio", 0, 1, 1, &pcm);

    if (err)
        goto error;
    
    pcm->private_data = priv;

    strcpy(pcm->name, "MOTU Pro Audio");
    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &capture_pcm_ops);
    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &playback_pcm_ops);
    snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);
    
error:
    return err;
}

static int motu_usb_audio_probe(struct usb_interface *intf,
            const struct usb_device_id *usb_id)
{
    struct snd_card *card;
    struct motu_usb_data *priv;
    struct usb_interface *pb_intf;
    struct usb_interface *rec_intf;
    struct usb_device *dev = interface_to_usbdev(intf);
    int ifnum = intf->altsetting->desc.bInterfaceNumber;
    int err = 0;

    dev_info(&dev->dev, "Probing Interface: %d", ifnum);
    
    /* Audio Control Interface */
    if (ifnum == 0) {
        err = snd_card_new(&dev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
            THIS_MODULE, sizeof(*priv), &card);
        
        if (err)
            return err;
        
        priv = card->private_data;
        priv->usb = dev;
        priv->card = card;

        usb_set_intfdata(intf, priv);

        pb_intf = usb_ifnum_to_if(dev, 1);
        rec_intf = usb_ifnum_to_if(dev, 2);

        if (!pb_intf || !rec_intf) {
            err = -ENODEV;
            goto error_free;
        }

        err = usb_driver_claim_interface(&snd_usb_motu_driver, pb_intf, priv);

        if (err)
            goto error_free;

        err = usb_driver_claim_interface(&snd_usb_motu_driver, rec_intf, priv);

        if (err)
            goto error_free;
        
        err = alloc_stream_urbs(priv, &priv->pb_stream, pb_intf,
            playback_complete_urb);

        if (err)
            goto error_free;
        
        err = alloc_stream_urbs(priv, &priv->rec_stream, rec_intf,
            capture_complete_urb);

        if (err)
            goto error_free;
        
        err = pcm_init(priv);

        if (err)
            goto error_free;
        
        spin_lock_init(&priv->lock);

        priv->start_stop_wq = alloc_ordered_workqueue("snd_usb_motu_wq", 0);

        if (!priv->start_stop_wq ) {
            err = -ENOMEM;
            goto error_free;
        }

        INIT_WORK(&priv->start_streams_work, start_streaming_endpoints);
        INIT_DELAYED_WORK(&priv->stop_streams_work, stop_streaming_endpoints);

        hrtimer_setup(&priv->timer, timer_callback, CLOCK_MONOTONIC,
            HRTIMER_MODE_REL);

        priv->perf1.name = "Completion";
        priv->perf2.name = "hrtimer";

        strcpy(card->driver, "MOTU Driver");
        strcpy(card->shortname, "MOTU Pro Audio");
        strcpy(card->longname, "MOTU Pro Audio");
        
        err = snd_card_register(card);

        if (err)
            goto error_free;
    }

    dev_info(&dev->dev, "Probe Success!");
    return 0;

error_free:
    dev_info(&dev->dev, "Probe Error: Freeing Card");
    snd_card_free(priv->card);
    return err;
}

static void motu_usb_audio_disconnect(struct usb_interface *intf)
{
    struct usb_device *dev = interface_to_usbdev(intf);
    struct motu_usb_data *priv = usb_get_intfdata(intf);
    int ifnum = intf->altsetting->desc.bInterfaceNumber;

    dev_info(&dev->dev, "Disconnecting Interface: %d", ifnum);

    if ((ifnum == 0) && priv) {
        dev_info(&dev->dev, "Disconnect: Freeing Card");
        snd_card_disconnect_sync(priv->card);
        hrtimer_cancel(&priv->timer);
        cancel_work_sync(&priv->start_streams_work);
        cancel_delayed_work_sync(&priv->stop_streams_work);
        atomic_set(&priv->streams_started, 0);
        free_stream_urbs(priv, &priv->rec_stream);
        free_stream_urbs(priv, &priv->pb_stream);
        destroy_workqueue(priv->start_stop_wq);
        snd_card_free(priv->card);
    }
}

static const struct usb_device_id motu_usb_ids[] = {
    { USB_DEVICE(0x07fd, 0x0005) }, /* MOTU Pro Audio Devices */
    { }
};

MODULE_DEVICE_TABLE(usb, motu_usb_ids);

static struct usb_driver snd_usb_motu_driver = {
    .name = "snd-usb-motu",
    .id_table = motu_usb_ids,
    .probe = motu_usb_audio_probe,
    .disconnect = motu_usb_audio_disconnect,
};

module_usb_driver(snd_usb_motu_driver);
