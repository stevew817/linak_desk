# Matter-controlled standing desk

This project exposes a LINAK standing desk (tested with DL5IC motor with built-in controller, but should in theory work with CBD6S control box too...) to a Matter controller over Thread, such that it can easily be integrated with smart home logic. Primarily because it is cool to see things move on voice commands, but also because it allows you to do some health-related things, like put your sit/stand cycle on a timer...

# Disclaimer
The code provided here is working 'good enough' for my personal use, but I am in no way responsible should anything go wrong when you try to replicate this project. Do so entirely at your own risk.

# Materials used

- RJ45 cable (cut)
- Silicon Labs EFR32xG24 development board (BRD2601B)
- Handful of components and wiring on a protoboard
- [Silicon Labs Matter SDK](https://github.com/SiliconLabs/matter/tree/release_1.0.2-1.0) (tested with v1.0.2-1.0)

# Instructions

## Firmware
1. Clone the Silicon Labs Matter SDK (linked above) and follow the instructions to get to a working build environment (instructions for example [here](https://github.com/SiliconLabs/matter/blob/release_1.0.2-1.0/docs/silabs/general/SOFTWARE_REQUIREMENTS.md))
2. Inside the tree, clone this repository at `silabs_examples/linak_desk`
3. Manually create the folder `zzz_generated/linak_desk/zap_generated`
4. Copy `af-gen-event.h` from `zzz_generated/lighting_app/zap_generated` into `zzz_generated/linak_desk/zap_generated`
5. Update the maximum stroke length to match your desk. My desk has a maximum of 500mm, and the controller doesn't really like going to the absolute maximum, so I set my limit to 4980 (unit is 10ths of mm). Update  `silabs_examples/linak_desk/linak_desk_model_config/linak_desk.zap` manually in a text editor. Find the JSON entry with name `"InstalledOpenLimitLift"` and update its `"defaultValue"` according to your desk.
6. Generate the application framework logic. From a command prompt at the root of the Matter SDK, run `./scripts/tools/zap/generate.py silabs_examples/linak_desk/linak_desk_model_config/linak_desk.zap -o zzz_generated/linak_desk/zap-generated/`
7. Compile the firmware. From a command prompt at the root of the Matter SDK, run `./scripts/examples/gn_efr32_example.sh ./silabs_examples/linak_desk/efr32/ ./out/linak_desk BRD2601B segger_rtt_buffer_size_up=8192`
8. Flash the bootloader to the board. The bootloader is shipped in binary form as part of the SDK release, and can be found in a [zip file here](https://github.com/SiliconLabs/matter/releases/download/v1.0.2-1.0/bootloader_binaries_1.0.2-1.0.zip). Flash the file called `bootloader-storage-spiflash-single-1024k-BRD2601B-gsdk4.1.s37` to the development board. E.g. using Commander with `/Applications/Commander.app/Contents/MacOS/commander flash ~/Downloads/bootloader_binaries/bootloader-storage-spiflash-single-1024k-BRD2601B-gsdk4.1.s37 -d EFR32MG24`
9. Flash the compiled firmware using `python3 out/linak_desk/BRD2601B/chip-efr32-linak-desk.flash.py`
10. In order to get the QR code required to commission your Matter device to a Matter controller, you will need to be able to view the logging output as described [here](https://github.com/SiliconLabs/matter/blob/release_1.0.2-1.0/examples/lighting-app/silabs/efr32/README.md#viewing-logging-output)

## Hardware

1. Solder together adapter board (TODO: add instructions/schematic)
2. Connect BRD2601B to adapter board (TODO: add wiring diagram)

## Connecting the device to the controller

1. Before connecting to the adapter board, connect the fully-flashed BRD2601B to your computer and [view the logging output](https://github.com/SiliconLabs/matter/blob/release_1.0.2-1.0/examples/lighting-app/silabs/efr32/README.md#viewing-logging-output). In there, there will be a sentence saying `Copy/paste the below URL in a browser to see the QR Code:`. Do so.
2. Add the Matter device to your Matter controller by following your ecosystem's guide. Note that this is not a certified Matter accessory device, so you may have to enable developer mode or similar. This device is using `VID 0xFFF1 PID 0x8010` and the CSA development certificates, and will announce itself as a window blind.
3. Remove the USB cable and connect the BRD2601B to the adapter board, and plug it into the RJ45 control port of your desk (where the keypad is connected).
4. Wait for the Matter device to reconnect to the controller (can take 1~5 minutes)
5. Happy voice-controlling your desk!

# Known limitations

- Currently, the firmware has not implemented changing the target height while in motion. You will have to re-send a command to move the position if the previous one was sent during a time where the desk was in motion.
- The code is ugly.
