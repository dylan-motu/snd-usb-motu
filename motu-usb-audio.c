// SPDX-License-Identifier: GPL-2.0-only
/*
 * MOTU Pro Audio USB Driver
 */
#include <linux/module.h>
#include <linux/once.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/pcm.h>

MODULE_DESCRIPTION("MOTU Pro Audio USB Driver");
MODULE_AUTHOR("Dylan Robinson <dylan_robinson@motu.com>");
MODULE_LICENSE("GPL v2");

#define SAMPLE_RATE         48000

#define NUM_INTERRUPT_URBS  4
#define NUM_URBS            4
#define UFRAMES_PER_URB     512
#define TOTAL_UFRAMES       (NUM_URBS * UFRAMES_PER_URB)
#define NUM_CH              24
#define BYTES_PER_SAMPLE    3
#define BYTES_PER_FRAME     (NUM_CH * BYTES_PER_SAMPLE)
#define PB_SAFETY_OFFSET    16
#define REC_SAFETY_OFFSET   16

#if SAMPLE_RATE == 44100
    #define NOM_SAMPLE_COUNT 6
#elif SAMPLE_RATE == 48000
    #define NOM_SAMPLE_COUNT 6
#elif SAMPLE_RATE == 88200
    #define NOM_SAMPLE_COUNT 11
#elif SAMPLE_RATE == 96000
    #define NOM_SAMPLE_COUNT 12
#elif SAMPLE_RATE == 176400
    #define NOM_SAMPLE_COUNT 22
#elif SAMPLE_RATE == 192000
    #define NOM_SAMPLE_COUNT 24
#else
    #error "Unsupported SAMPLE_RATE"
#endif

#define SNDRV_PCM_RATE_FLAG(sr) (               \
    (sr) == 44100  ? SNDRV_PCM_RATE_44100  :    \
    (sr) == 48000  ? SNDRV_PCM_RATE_48000  :    \
    (sr) == 88200  ? SNDRV_PCM_RATE_88200  :    \
    (sr) == 96000  ? SNDRV_PCM_RATE_96000  :    \
    (sr) == 176400 ? SNDRV_PCM_RATE_176400 :    \
    (sr) == 192000 ? SNDRV_PCM_RATE_192000 :    \
    0                                           \
)

#define CIRC_SUB(a, b, size)    (((a) + (size) - (b)) % (size))

static struct usb_driver snd_usb_motu_driver;

enum pb_start_state {
    PB_START_IDLE = 0,
    PB_START_PRIME,
    PB_START_SYNC,
    PB_START_RUNNING
};

struct motu_buf
{
    unsigned int length;
    unsigned char *data;
};

struct motu_stream
{
    bool enabled;
    spinlock_t lock;
    struct snd_pcm_substream *substream;
    unsigned int period_frames;
    unsigned int period_count;
    unsigned int period_idx;
    unsigned int frame_pos;
    unsigned int hw_ptr;
    unsigned int max_packet_size;
    unsigned int copy_pos;
    unsigned int copy_frame;
    struct motu_buf bufs[TOTAL_UFRAMES];
    struct urb *urbs[NUM_URBS];
};

struct motu_interrupt
{
    unsigned int interval;
    struct urb *urbs[NUM_INTERRUPT_URBS];
};

struct motu_usb_data
{
    spinlock_t wq_lock;
    atomic_t streams_started;
    struct usb_device *usb;
    struct snd_card *card;
    struct workqueue_struct *start_stop_wq;
    struct delayed_work start_streams_work;
    struct delayed_work stop_streams_work;
    struct motu_interrupt interrupt;
    struct motu_stream rec_stream;
    struct motu_stream pb_stream;
    unsigned int urb_idx;
    unsigned int chase_pos;
    unsigned int rx_frames;
    int pb_adj;
    int pb_adj_total;
    enum pb_start_state pb_start_state;
};

struct motu_interrupt_msg
{
    unsigned char info;
    unsigned char attr;
    unsigned int frame;
} __attribute__((packed));

struct class_interrupt_msg
{
    unsigned char info; /* Type */
    unsigned char attr; /* Attribute (Cur, Range, Mem) */
    unsigned char cn;   /* Control Number */
    unsigned char cs;   /* Control Selector */
    unsigned char intf; /* Interface */
    unsigned char id;   /* Entity ID */
} __attribute__((packed));

static struct snd_pcm_hardware snd_motu_hw = {
    .info = (SNDRV_PCM_INFO_MMAP |
             SNDRV_PCM_INFO_MMAP_VALID |
             SNDRV_PCM_INFO_INTERLEAVED |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_BATCH),
    .formats =          SNDRV_PCM_FMTBIT_S24_3LE,
    .rates =            SNDRV_PCM_RATE_FLAG(SAMPLE_RATE),
    .rate_min =         SAMPLE_RATE,
    .rate_max =         SAMPLE_RATE,
    .channels_min =     NUM_CH,
    .channels_max =     NUM_CH,
    .buffer_bytes_max = BYTES_PER_FRAME * 2048 * 4,
    .period_bytes_min = BYTES_PER_FRAME * 32,
    .period_bytes_max = BYTES_PER_FRAME * 2048,
    .periods_min =      2,
    .periods_max =      16,
};

static unsigned char motu_shred_pattern[BYTES_PER_FRAME];

static void reset_substream_position(struct motu_stream *stream)
{
    stream->period_idx = 0;
    stream->frame_pos = 0;
    stream->hw_ptr = 0;
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
        
        stream->hw_ptr = stream->period_idx * stream->period_frames;
        
        return true;
    }

    return false;
}

/* Generate the shred pattern for a single frame. */
static void init_shred_pattern(void)
{
    for (int i = 0; i < BYTES_PER_FRAME; ++i)
        motu_shred_pattern[i] = ~(unsigned char)i;
}

/* Write the shred pattern to the audio buffer. */
static void shred_sample_frames(unsigned char *buffer)
{
    unsigned char *ptr = buffer;

    for (int i = 0; i < (NOM_SAMPLE_COUNT + 1); ++i, ptr += BYTES_PER_FRAME)
        memcpy(ptr, motu_shred_pattern, BYTES_PER_FRAME);
}

/*
 * Count how many contiguous audio sample frames at the start of a USB
 * microframe buffer have been filled by the device (i.e. not equal to
 * the shred pattern).
 *
 * Return: number of contiguous filled sample frames
 */
static unsigned int count_uframe_sample_frames(const unsigned char *buffer)
{
    const unsigned char *ptr = buffer;

    for (int i = 0; i < (NOM_SAMPLE_COUNT + 1); ++i, ptr += BYTES_PER_FRAME)
        if (!memcmp(ptr, motu_shred_pattern, BYTES_PER_FRAME))
            return i;

    return (NOM_SAMPLE_COUNT + 1);
}

/*
 * Count the total number of audio sample frames that have been filled by
 * the device across consecutive USB microframe buffers, starting from
 * priv->chase_pos.
 */
static void count_sample_frames(struct motu_usb_data *priv)
{
    struct motu_stream *rec_stream = &priv->rec_stream;
    unsigned int chase_pos = priv->chase_pos;
    unsigned int prev_frames = 0;
    unsigned int count = 0;

    for (int i = 0; i < UFRAMES_PER_URB; ++i) {
        struct motu_buf *buf = &rec_stream->bufs[chase_pos];
        
        buf->length = count_uframe_sample_frames(buf->data);
        
        if (buf->length) {
            count += prev_frames;
            prev_frames = buf->length;
        }
        else if (count) {
            /* step back two to recheck last buffer next time */
            priv->chase_pos = CIRC_SUB(chase_pos, 2, TOTAL_UFRAMES);
            break;
        }
        else if (i > 32) {
            /* too many buffers without finding any frames */
            break;
        }
        
        chase_pos = (chase_pos + 1) % TOTAL_UFRAMES;
    }

    priv->rx_frames += count;
}

static void mute_period_to_usb(struct motu_stream *stream,
    unsigned int period_size)
{
    while (period_size) {
        struct motu_buf *dst_buf = &stream->bufs[stream->copy_pos];
        unsigned int dst_frames = dst_buf->length - stream->copy_frame;
        unsigned int frames_this_copy = min(period_size, dst_frames);
        unsigned int bytes_this_copy = frames_this_copy * BYTES_PER_FRAME;
        unsigned int dst_offset = stream->copy_frame * BYTES_PER_FRAME;

        memset(dst_buf->data + dst_offset, 0, bytes_this_copy);

        period_size -= frames_this_copy;
        
        stream->copy_frame += frames_this_copy;
        if (stream->copy_frame >= dst_buf->length) {
            stream->copy_frame = 0;
            stream->copy_pos = (stream->copy_pos + 1) % TOTAL_UFRAMES;
        }
    }
}

static void copy_period_to_usb(struct motu_stream *stream,
    unsigned int period_size)
{
    unsigned char* src;
    unsigned int src_offset;

    if (period_size != stream->period_frames) {
        mute_period_to_usb(stream, period_size);
        return;
    }
    
    src = stream->substream->runtime->dma_area;
    src_offset = substream_position_bytes(stream);

    while (period_size) {
        struct motu_buf *dst_buf = &stream->bufs[stream->copy_pos];
        unsigned int dst_frames = dst_buf->length - stream->copy_frame;
        unsigned int frames_this_copy = min(period_size, dst_frames);
        unsigned int bytes_this_copy = frames_this_copy * BYTES_PER_FRAME;
        unsigned int dst_offset = stream->copy_frame * BYTES_PER_FRAME;

        memcpy(dst_buf->data + dst_offset, src + src_offset, bytes_this_copy);
        
        src_offset += bytes_this_copy;
        period_size -= frames_this_copy;
        
        stream->copy_frame += frames_this_copy;
        if (stream->copy_frame >= dst_buf->length) {
            stream->copy_frame = 0;
            stream->copy_pos = (stream->copy_pos + 1) % TOTAL_UFRAMES;
        }
    }

    inc_frame_position(stream, stream->period_frames);
    snd_pcm_period_elapsed(stream->substream);
}

static void sync_period_from_usb(struct motu_stream *stream,
    unsigned int period_size)
{
    while (period_size) {
        struct motu_buf *src_buf = &stream->bufs[stream->copy_pos];
        unsigned int src_frames = src_buf->length - stream->copy_frame;
        unsigned int frames_this_copy = min(period_size, src_frames);

        if (!frames_this_copy)
            break;

        period_size -= frames_this_copy;
        
        stream->copy_frame += frames_this_copy;
        if (stream->copy_frame >= src_buf->length) {
            shred_sample_frames(stream->bufs[stream->copy_pos].data);
            stream->copy_frame = 0;
            stream->copy_pos = (stream->copy_pos + 1) % TOTAL_UFRAMES;
        }
    }
}

static void copy_period_from_usb(struct motu_stream *stream,
    unsigned int period_size)
{
    unsigned char* dst;
    unsigned int dst_offset;

    if (period_size != stream->period_frames) {
        sync_period_from_usb(stream, period_size);
        return;
    }
    
    dst = stream->substream->runtime->dma_area;
    dst_offset = substream_position_bytes(stream);

    while (period_size) {
        struct motu_buf *src_buf = &stream->bufs[stream->copy_pos];
        unsigned int src_frames = src_buf->length - stream->copy_frame;
        unsigned int frames_this_copy = min(period_size, src_frames);
        unsigned int bytes_this_copy = frames_this_copy * BYTES_PER_FRAME;
        unsigned int src_offset = stream->copy_frame * BYTES_PER_FRAME;

        if (!frames_this_copy)
            break;

        memcpy(dst + dst_offset, src_buf->data + src_offset, bytes_this_copy);

        dst_offset += bytes_this_copy;
        period_size -= frames_this_copy;
        
        stream->copy_frame += frames_this_copy;
        if (stream->copy_frame >= src_buf->length) {
            shred_sample_frames(stream->bufs[stream->copy_pos].data);
            stream->copy_frame = 0;
            stream->copy_pos = (stream->copy_pos + 1) % TOTAL_UFRAMES;
        }
    }

    inc_frame_position(stream, stream->period_frames);
    snd_pcm_period_elapsed(stream->substream);
}

static void handle_interval_interrupt(struct motu_usb_data *priv)
{
    struct motu_stream *rec_stream = &priv->rec_stream;
    struct motu_stream *pb_stream = &priv->pb_stream;
    unsigned int period_size = priv->interrupt.interval;
    
    count_sample_frames(priv);
    
    if (rec_stream->copy_pos == TOTAL_UFRAMES) {
        /* Capture startup - Sync copy position */
        if (priv->rx_frames < (period_size + REC_SAFETY_OFFSET))
            return;
        
        int discard = priv->rx_frames - (period_size + REC_SAFETY_OFFSET);
        rec_stream->copy_pos = 0;

        while (discard > 0) {
            discard -= rec_stream->bufs[rec_stream->copy_pos].length;
            shred_sample_frames(rec_stream->bufs[rec_stream->copy_pos].data);
            ++rec_stream->copy_pos;
        }

        dev_info(&priv->usb->dev, 
            "Rec Copy Sync: %d (chase: %u, count: %u)\n",
            rec_stream->copy_pos, priv->chase_pos, priv->rx_frames);
    }
    else if (priv->pb_start_state == PB_START_SYNC) {
        /* Playback startup - Sync copy position */
        int adjust = (priv->chase_pos + PB_SAFETY_OFFSET) -
            (UFRAMES_PER_URB * 2);
        
        pb_stream->copy_pos = (TOTAL_UFRAMES + adjust) % TOTAL_UFRAMES;
        priv->pb_start_state = PB_START_RUNNING;

        dev_info(&priv->usb->dev,
            "PB Copy Sync: %u (adjust: %d chase: %u)\n", 
            pb_stream->copy_pos, adjust, priv->chase_pos);
    }
    else if (priv->pb_adj) {
        pb_stream->copy_pos =
            (pb_stream->copy_pos + priv->pb_adj) % TOTAL_UFRAMES;
        priv->pb_adj_total += priv->pb_adj;
        priv->pb_adj = 0;
    }

    if (rec_stream->copy_pos != TOTAL_UFRAMES) {
        if (rec_stream->enabled && spin_trylock(&rec_stream->lock)) {
            copy_period_from_usb(rec_stream, period_size);
            spin_unlock(&rec_stream->lock);
        }
        else {
            sync_period_from_usb(rec_stream, period_size);
        }
    }

    if (priv->pb_start_state == PB_START_RUNNING) {
        if (pb_stream->enabled && spin_trylock(&pb_stream->lock)) {
            copy_period_to_usb(pb_stream, period_size);
            spin_unlock(&pb_stream->lock);
        }
        else {
            mute_period_to_usb(pb_stream, period_size);
        }
    }
}

static void handle_status_interrupt(struct class_interrupt_msg *msg,
    struct motu_usb_data *priv)
{
    /*
    dev_info(&priv->usb->dev,
        "info:%u attr:%u cn:%u cs:%u intf:%u id:%u\n",
        msg->info, msg->attr, msg->cn, msg->cs, msg->intf, msg->id);
    */
}

static void interrupt_complete_urb(struct urb* urb)
{
    struct motu_interrupt_msg *msg = urb->transfer_buffer;
    
    if (urb->status == 0) {
        if (msg->info == 0x01 && msg->attr == 0x01)
            handle_interval_interrupt(urb->context);
        else
            handle_status_interrupt(urb->transfer_buffer, urb->context);
    }

    usb_submit_urb(urb, GFP_ATOMIC);
}

static void playback_complete_urb(struct urb *urb)
{
    /*
    struct motu_usb_data *priv = urb->context;
    dev_info(&priv->usb->dev, "copy_pos:%u\n", priv->pb_stream.copy_pos);
    */
}

static void capture_complete_urb(struct urb *urb)
{
    struct motu_usb_data *priv = urb->context;
    struct urb *pb_urb;
    struct motu_buf *pb_bufs;

    if (!atomic_read(&priv->streams_started))
        return;
    
    pb_urb = priv->pb_stream.urbs[priv->urb_idx];
    pb_bufs = &priv->pb_stream.bufs[priv->urb_idx * UFRAMES_PER_URB];
    priv->urb_idx = (priv->urb_idx + 1) % NUM_URBS;
    
    for (int i = 0; i < UFRAMES_PER_URB; ++i) {
        /* Nominal Packet Size */
        unsigned int length = BYTES_PER_FRAME * NOM_SAMPLE_COUNT;
        
        if (!urb->iso_frame_desc[i].status)
            length = urb->iso_frame_desc[i].actual_length;
        else
            dev_info(&priv->usb->dev, "rec urb error\n");
        
        pb_urb->iso_frame_desc[i].length = length;
        pb_bufs[i].length = length / BYTES_PER_FRAME;
    }

    usb_submit_urb(urb, GFP_ATOMIC);

    switch (priv->pb_start_state) {
    case PB_START_IDLE:
        priv->pb_start_state = PB_START_PRIME;
        break;
    case PB_START_PRIME:
        usb_submit_urb(priv->pb_stream.urbs[0], GFP_ATOMIC);
        usb_submit_urb(priv->pb_stream.urbs[1], GFP_ATOMIC);
        priv->pb_start_state = PB_START_SYNC;
        break;
    case PB_START_SYNC:
    case PB_START_RUNNING:
        usb_submit_urb(pb_urb, GFP_ATOMIC);
        break;
    }
}

static void set_interrupt_interval(struct motu_usb_data *priv, u16 interval)
{
    int ret;
    u8 request = 0x01;
    u8 request_type = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
    u16 index = 0;
    int timeout = 500;
    
    priv->interrupt.interval = interval;
    ret = usb_control_msg(priv->usb, usb_sndctrlpipe(priv->usb, 0), request,
        request_type, interval, index, NULL, 0, timeout);

    if (ret < 0)
        dev_warn(&priv->usb->dev,
            "Failed to set interrupt interval: %d\n", ret);
}

static void restart_streaming_endpoints(struct motu_usb_data *priv)
{
    spin_lock(&priv->wq_lock);

    if (priv->start_stop_wq) {
        queue_delayed_work(priv->start_stop_wq,
            &priv->stop_streams_work, 0);
        queue_delayed_work(priv->start_stop_wq,
            &priv->start_streams_work, msecs_to_jiffies(1));
    }

    spin_unlock(&priv->wq_lock);
}

static void start_streaming_endpoints(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct motu_usb_data *priv =
        container_of(dwork, struct motu_usb_data, start_streams_work);
    unsigned int interval =
        max(priv->rec_stream.period_frames,
            priv->pb_stream.period_frames);
    struct urb **urbs;

    if (atomic_cmpxchg(&priv->streams_started, 0, 1)) {
        if (interval != priv->interrupt.interval)
            restart_streaming_endpoints(priv);
        return;
    }
    
    dev_info(&priv->usb->dev, "start_streaming_endpoints\n");
    
    urbs = priv->rec_stream.urbs;

    priv->rec_stream.copy_pos = TOTAL_UFRAMES;
    priv->rec_stream.copy_frame = 0;
    priv->pb_stream.copy_pos = TOTAL_UFRAMES;
    priv->pb_stream.copy_frame = 0;
    priv->urb_idx = 0;
    priv->chase_pos = 0;
    priv->rx_frames = 0;
    priv->pb_start_state = PB_START_IDLE;

    for (int i = 0; i < TOTAL_UFRAMES; ++i) {
        priv->rec_stream.bufs[i].length = 0;
        priv->pb_stream.bufs[i].length = 0;
    }

    for (int i = 0; i < UFRAMES_PER_URB; ++i)
        shred_sample_frames(priv->rec_stream.bufs[i].data);

    set_interrupt_interval(priv, interval);
    usb_set_interface(priv->usb, 2, 1); // start record
    usb_set_interface(priv->usb, 1, 1); // start playback

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
    
    dev_info(&priv->usb->dev, "stop_streaming_endpoints\n");

    priv->pb_adj_total = 0;

    set_interrupt_interval(priv, 0);

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

    for (int i = 0; i < NUM_URBS; ++i) {
        
        if (!urbs[i])
            continue;
        
        usb_kill_urb(urbs[i]);
        
        if ((i == 0) && urbs[i]->transfer_buffer)
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
    const struct usb_host_interface *alt = usb_altnum_to_altsetting(intf, 1);
    const struct usb_endpoint_descriptor *epd = &alt->endpoint[0].desc;
    struct urb **urbs = stream->urbs;
    unsigned int pipe;
    unsigned int max_packet_size;
    unsigned int transfer_buffer_length;
    unsigned char *transfer_buffer = NULL;
    dma_addr_t transfer_dma;
    
    if (usb_endpoint_dir_in(epd))
        pipe = usb_rcvisocpipe(priv->usb, usb_endpoint_num(epd));
    else
        pipe = usb_sndisocpipe(priv->usb, usb_endpoint_num(epd));
    
    max_packet_size = usb_endpoint_maxp(epd) * usb_endpoint_maxp_mult(epd);
    transfer_buffer_length = max_packet_size * UFRAMES_PER_URB;

    for (int i = 0; i < NUM_URBS; ++i) {
        unsigned int buf_base;

        urbs[i] = usb_alloc_urb(UFRAMES_PER_URB, GFP_KERNEL);
        
        if (!urbs[i])
            goto out_free;

        if (!transfer_buffer) {
            transfer_buffer = usb_alloc_coherent(priv->usb,
                                transfer_buffer_length,
                                GFP_KERNEL, &transfer_dma);
            
            if (!transfer_buffer)
                goto out_free;
        }
        
        urbs[i]->dev = priv->usb;
        urbs[i]->pipe = pipe;
        urbs[i]->transfer_flags = URB_NO_TRANSFER_DMA_MAP | URB_ISO_ASAP;
        urbs[i]->transfer_buffer = transfer_buffer;
        urbs[i]->transfer_dma = transfer_dma;
        urbs[i]->transfer_buffer_length = max_packet_size * UFRAMES_PER_URB;
        urbs[i]->number_of_packets = UFRAMES_PER_URB;
        urbs[i]->interval = 1;
        urbs[i]->context = priv;
        urbs[i]->complete = completion_handler;

        buf_base = i * UFRAMES_PER_URB;

        for (int f = 0; f < UFRAMES_PER_URB; ++f) {
            unsigned int offset = max_packet_size * f;

            urbs[i]->iso_frame_desc[f].offset = offset;
            urbs[i]->iso_frame_desc[f].length = max_packet_size;

            stream->bufs[buf_base + f].data = transfer_buffer + offset;
        }
    }

    stream->max_packet_size = max_packet_size;

    return 0;

out_free:
    free_stream_urbs(priv, stream);
    return -ENOMEM;
}

static void free_interrupt_ubs(struct motu_usb_data *priv)
{
    struct urb **urbs = priv->interrupt.urbs;

    for (int i = 0; i < NUM_INTERRUPT_URBS; ++i) {
        
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

static int alloc_interrupt_urbs(struct motu_usb_data *priv,
            struct usb_interface *intf, usb_complete_t completion_handler)
{
    struct urb **urbs = priv->interrupt.urbs;
    struct usb_endpoint_descriptor *epd = &intf->altsetting[0].endpoint[0].desc;
    unsigned int pipe = usb_rcvintpipe(priv->usb, usb_endpoint_num(epd));

    for (int i = 0; i < NUM_INTERRUPT_URBS; ++i) {
        urbs[i] = usb_alloc_urb(0, GFP_KERNEL);

        if (!urbs[i])
            goto out_free;

        urbs[i]->dev = priv->usb;
        urbs[i]->pipe = pipe;
        urbs[i]->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
        urbs[i]->transfer_buffer = usb_alloc_coherent(priv->usb, 6, GFP_KERNEL,
                                        &urbs[i]->transfer_dma);
        
        if (!urbs[i]->transfer_buffer)
            goto out_free;
        
        urbs[i]->transfer_buffer_length = 6;
        urbs[i]->complete = completion_handler;
        urbs[i]->context = priv;
        urbs[i]->start_frame = -1;
        urbs[i]->interval = 1;
    }

    return 0;

out_free:
    free_interrupt_ubs(priv);
    return -ENOMEM;
}

static void start_interrupt_urbs(struct motu_usb_data *priv)
{
    for (int i = 0; i < NUM_INTERRUPT_URBS; ++i)
        usb_submit_urb(priv->interrupt.urbs[i], GFP_ATOMIC);
}

static void queue_start_streaming(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    cancel_delayed_work(&priv->stop_streams_work);
    
    spin_lock(&priv->wq_lock);
    if (priv->start_stop_wq)
        queue_delayed_work(priv->start_stop_wq, &priv->start_streams_work, 0);
    spin_unlock(&priv->wq_lock);
}

static void maybe_queue_stop_streaming(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    if (priv->rec_stream.enabled || priv->pb_stream.enabled)
        return;

    cancel_delayed_work(&priv->start_streams_work);
    
    spin_lock(&priv->wq_lock);
    if (priv->start_stop_wq)
        queue_delayed_work(priv->start_stop_wq, &priv->stop_streams_work,
            msecs_to_jiffies(2000));
    spin_unlock(&priv->wq_lock);
}

static int capture_pcm_open(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "capture_pcm_open\n");

    subs->runtime->hw = snd_motu_hw;

    spin_lock(&priv->rec_stream.lock);
    priv->rec_stream.substream = subs;
    priv->rec_stream.period_frames = 0;
    spin_unlock(&priv->rec_stream.lock);

    return 0;
}

static int capture_pcm_close(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    spin_lock(&priv->rec_stream.lock);
    priv->rec_stream.substream = NULL;
    priv->rec_stream.period_frames = 0;
    spin_unlock(&priv->rec_stream.lock);

    return 0;
}

static int capture_pcm_hw_params(struct snd_pcm_substream *subs,
            struct snd_pcm_hw_params *hw_params)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "Capture Channels: %u\n",
        params_channels(hw_params));
    dev_info(&priv->usb->dev, "Capture Sample Rate: %u\n",
        params_rate(hw_params));
    dev_info(&priv->usb->dev, "Capture Buffer Bytes: %u\n",
        params_buffer_bytes(hw_params));
    dev_info(&priv->usb->dev, "Capture Periods: %u\n",
        params_periods(hw_params));
    dev_info(&priv->usb->dev, "Capture Period Size: %u\n",
        params_period_size(hw_params));
    
    if (priv->pb_stream.period_frames &&
        priv->pb_stream.period_frames != params_period_size(hw_params)) {
        
        dev_info(&priv->usb->dev,
            "Capture and Playback must use same period size!\n");
        
        return -EINVAL;
    }
    
    spin_lock(&priv->rec_stream.lock);
    reset_substream_position(&priv->rec_stream);
    priv->rec_stream.period_count = params_periods(hw_params);
    priv->rec_stream.period_frames = params_period_size(hw_params);
    spin_unlock(&priv->rec_stream.lock);
    
    return 0;
}

static int capture_pcm_hw_free(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "capture_pcm_hw_free\n");

    spin_lock(&priv->rec_stream.lock);
    priv->rec_stream.period_frames = 0;
    spin_unlock(&priv->rec_stream.lock);

    priv->rec_stream.enabled = false;
    maybe_queue_stop_streaming(subs);

    return 0;
}

static int capture_pcm_prepare(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "capture_pcm_prepare\n");

    spin_lock(&priv->rec_stream.lock);
    reset_substream_position(&priv->rec_stream);
    spin_unlock(&priv->rec_stream.lock);

    return 0;
}

static int capture_pcm_trigger(struct snd_pcm_substream *subs, int cmd)
{
    struct motu_usb_data *priv = subs->private_data;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        dev_info(&priv->usb->dev,
            "capture_pcm_trigger start (stop_threshold:%lu)\n",
            subs->runtime->stop_threshold);
        priv->rec_stream.enabled = true;
        queue_start_streaming(subs);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        dev_info(&priv->usb->dev, "capture_pcm_trigger stop\n");
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

    return priv->rec_stream.hw_ptr;
}

static const struct snd_pcm_ops capture_pcm_ops = {
    .open = capture_pcm_open,
    .close = capture_pcm_close,
    .hw_params = capture_pcm_hw_params,
    .hw_free = capture_pcm_hw_free,
    .prepare = capture_pcm_prepare,
    .trigger = capture_pcm_trigger,
    .pointer = capture_pcm_pointer,
};

static int playback_pcm_open(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_open\n");

    subs->runtime->hw = snd_motu_hw;

    spin_lock(&priv->pb_stream.lock);
    priv->pb_stream.substream = subs;
    priv->pb_stream.period_frames = 0;
    spin_unlock(&priv->pb_stream.lock);

    return 0;
}

static int playback_pcm_close(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_close\n");

    spin_lock(&priv->pb_stream.lock);
    priv->pb_stream.substream = NULL;
    priv->pb_stream.period_frames = 0;
    spin_unlock(&priv->pb_stream.lock);

    return 0;
}

static int playback_pcm_hw_params(struct snd_pcm_substream *subs,
            struct snd_pcm_hw_params *hw_params)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "Playback Channels: %u\n",
        params_channels(hw_params));
    dev_info(&priv->usb->dev, "Playback Sample Rate: %u\n",
        params_rate(hw_params));
    dev_info(&priv->usb->dev, "Playback Buffer Bytes: %u\n",
        params_buffer_bytes(hw_params));
    dev_info(&priv->usb->dev, "Playback Periods: %u\n",
        params_periods(hw_params));
    dev_info(&priv->usb->dev, "Playback Period Size: %u\n",
        params_period_size(hw_params));
    
    if (priv->rec_stream.period_frames &&
        priv->rec_stream.period_frames != params_period_size(hw_params)) {
        
        dev_info(&priv->usb->dev,
            "Capture and Playback must use same period size!\n");
        
        return -EINVAL;
    }
    
    spin_lock(&priv->pb_stream.lock);
    reset_substream_position(&priv->pb_stream);
    priv->pb_stream.period_count = params_periods(hw_params);
    priv->pb_stream.period_frames = params_period_size(hw_params);
    spin_unlock(&priv->pb_stream.lock);
    
    return 0;
}

static int playback_pcm_hw_free(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_hw_free\n");

    spin_lock(&priv->pb_stream.lock);
    priv->pb_stream.period_frames = 0;
    spin_unlock(&priv->pb_stream.lock);

    priv->pb_stream.enabled = false;
    maybe_queue_stop_streaming(subs);

    return 0;
}

static int playback_pcm_prepare(struct snd_pcm_substream *subs)
{
    struct motu_usb_data *priv = subs->private_data;

    dev_info(&priv->usb->dev, "playback_pcm_prepare\n");

    spin_lock(&priv->pb_stream.lock);
    reset_substream_position(&priv->pb_stream);
    spin_unlock(&priv->pb_stream.lock);

    return 0;
}

static int playback_pcm_trigger(struct snd_pcm_substream *subs, int cmd)
{
    struct motu_usb_data *priv = subs->private_data;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        dev_info(&priv->usb->dev,
            "playback_pcm_trigger start (stop_threshold:%lu)\n",
            subs->runtime->stop_threshold);
        priv->pb_stream.enabled = true;
        queue_start_streaming(subs);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        dev_info(&priv->usb->dev, "playback_pcm_trigger stop\n");
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

    return priv->pb_stream.hw_ptr;
}

static const struct snd_pcm_ops playback_pcm_ops = {
    .open = playback_pcm_open,
    .close = playback_pcm_close,
    .hw_params = playback_pcm_hw_params,
    .hw_free = playback_pcm_hw_free,
    .prepare = playback_pcm_prepare,
    .trigger = playback_pcm_trigger,
    .pointer = playback_pcm_pointer,
};

static void pb_adj_read(struct snd_info_entry *entry, 
                       struct snd_info_buffer *buffer)
{
    struct motu_usb_data *priv = entry->private_data;

    snd_iprintf(buffer, "pb_adj: %d\n", priv->pb_adj_total);
}

static void pb_adj_write(struct snd_info_entry *entry,
                         struct snd_info_buffer *buffer)
{
    struct motu_usb_data *priv = entry->private_data;
    char line[16];
    if (!snd_info_get_line(buffer, line, sizeof(line))) {
        int pb_adj;
        if (!kstrtoint(line, 10, &pb_adj)) {
            dev_info(&priv->usb->dev, "pb_adj: %d\n", pb_adj);
            priv->pb_adj = pb_adj;
        }
    }
}

static void motu_create_proc_entry(struct motu_usb_data *priv)
{
    struct snd_info_entry *entry;

    entry = snd_info_create_card_entry(priv->card, "pb_adj",
        priv->card->proc_root);
    
    if (!entry)
        return;

    entry->private_data = priv;
    entry->c.text.read = pb_adj_read;
    entry->c.text.write = pb_adj_write;
    
    snd_info_register(entry);
}

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
    int ifnum = intf->altsetting[0].desc.bInterfaceNumber;
    int err = 0;

    dev_info(&dev->dev, "Probing Interface: %d\n", ifnum);
    
    /* Audio Control Interface */
    if (ifnum == 0) {
        err = snd_card_new(&dev->dev, SNDRV_DEFAULT_IDX1, "motuusb",
            THIS_MODULE, sizeof(*priv), &card);
        
        if (err)
            return err;
        
        DO_ONCE(init_shred_pattern);
        
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

        err = alloc_interrupt_urbs(priv, intf, interrupt_complete_urb);

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
        
        spin_lock_init(&priv->rec_stream.lock);
        spin_lock_init(&priv->pb_stream.lock);
        spin_lock_init(&priv->wq_lock);

        priv->start_stop_wq = alloc_ordered_workqueue("snd_usb_motu_wq", 0);

        if (!priv->start_stop_wq ) {
            err = -ENOMEM;
            goto error_free;
        }

        INIT_DELAYED_WORK(&priv->start_streams_work, start_streaming_endpoints);
        INIT_DELAYED_WORK(&priv->stop_streams_work, stop_streaming_endpoints);

        strcpy(card->driver, "MOTU Driver");
        strcpy(card->shortname, "MOTU Pro Audio");
        strcpy(card->longname, "MOTU Pro Audio");
        
        motu_create_proc_entry(priv);
        
        err = snd_card_register(card);

        if (err)
            goto error_free;
        
        start_interrupt_urbs(priv);
    }

    dev_info(&dev->dev, "Probe Success!\n");
    return 0;

error_free:
    dev_info(&dev->dev, "Probe Error: Freeing Card\n");
    snd_card_free(priv->card);
    return err;
}

static void motu_usb_audio_disconnect(struct usb_interface *intf)
{
    struct usb_device *dev = interface_to_usbdev(intf);
    struct motu_usb_data *priv = usb_get_intfdata(intf);
    int ifnum = intf->altsetting->desc.bInterfaceNumber;

    dev_info(&dev->dev, "Disconnecting Interface: %d\n", ifnum);

    if ((ifnum == 0) && priv) {
        struct workqueue_struct *wq = priv->start_stop_wq;
        dev_info(&dev->dev, "Disconnect: Freeing Card\n");
        snd_card_disconnect(priv->card);
        spin_lock(&priv->wq_lock);
        priv->start_stop_wq = NULL;
        spin_unlock(&priv->wq_lock);
        cancel_delayed_work_sync(&priv->start_streams_work);
        cancel_delayed_work_sync(&priv->stop_streams_work);
        destroy_workqueue(wq);
        atomic_set(&priv->streams_started, 0);
        free_interrupt_ubs(priv);
        free_stream_urbs(priv, &priv->rec_stream);
        free_stream_urbs(priv, &priv->pb_stream);
        snd_card_free_when_closed(priv->card);
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
