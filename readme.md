# snd-usb-motu

A Linux ALSA USB driver for first-generation MOTU Pro Audio hardware.

Currently **only USB 2.0 models** are supported:

- 16A
- 1248
- 112D
- 8M
- Monitor 8
- 24Ai
- 24Ao
- Stage-B16
- UltraLite AVB
- UltraLite-mk4
- M64
- 8D
- LP32
- 828es
- 8pre-es

> **Note:** This driver is in **early development**. Expect limited functionality and noisy logging.


## Current Status

- **Audio streaming only**
- Control/configuration must be done via the device’s web UI over Ethernet
- Works only at **48 kHz** sample rate
- **High latency** at present
- Does not handle clock lock loss — you must ensure the device is locked at 48 kHz **before** connecting to the computer
- Some debug logging (`sudo dmesg -w` to view)

The primary purpose is to prove a streaming / URB submission scheme that avoids the channel-hopping and distortion issues seen with the generic USB Audio Class driver.


## Planned Improvements

- Lower latency
- Support for multiple sample rates
- Handle clock lock / unlock


## Building and Running

Tested on **Ubuntu 24.04.2 LTS** with a fresh install.

### Install prerequisites
```bash
sudo apt update
sudo apt install build-essential
```

### Clone this repository
```bash
git clone https://github.com/dylan-motu/snd-usb-motu
cd snd-usb-motu
```

### Build and load the driver
```bash
./build_and_load.sh
```

The script will:

- Remove the generic ALSA USB Audio Class (snd-usb-audio) driver
- Remove any previously loaded snd-usb-motu module
- Compile this driver
- Insert the compiled module

This does not install the driver permanently — after a reboot, the system will return to normal.

In my experience, you can run this script with the device connected. If you encounter any issues, try these steps:
1. Disconnect the device
1. Run the script
1. Ensure the device's clock is set to 48 kHz and is locked
1. Reconnect the device

## Viewing Logs
To see the driver's debug output in real-time:
```bash
sudo dmesg -w
```

## Disclaimer

This is experimental software intended for development and testing only.  
It may cause unexpected behavior, high latency, or audio glitches.  
Use at your own risk.  

**This driver is not officially supported or endorsed by MOTU, Inc.**
