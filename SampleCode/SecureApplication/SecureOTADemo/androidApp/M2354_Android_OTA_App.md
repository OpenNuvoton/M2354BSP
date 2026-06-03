# Android OTA Update App Description

## Purpose

The Android application, `OTA_Update_App.apk`, is used as the OTA server in the
`SecureOTABankRemapDemo` sample. It runs on an Android mobile device and
provides firmware update files to the OTA client running on a NuMaker-M2354
board with an ESP8266 Wi-Fi module.

In the demo, both the Android OTA server and the M2354 OTA client connect to the
same wireless access point. After the client connects to the server, the server
provides the firmware packages used to update:

- NuBL32 system firmware
- NuBL33 application firmware

The demo updates both firmware images from version 1 to version 2. After the
update is complete, the M2354 board restarts and the two LEDs on the board flash
alternately.

## Location

The APK file is located in the M2354 BSP at:

```text
bsp\Utilities\androidApp
```

The APK file name is:

```text
OTA_Update_App.apk
```

## Required Files

Before starting the OTA server, place the following files in the `Download`
directory of the Android mobile device:

- `NuBL32_FwPack_v2.bin`
- `NuBL33_FwPack_v2.bin`
- `license.txt`

`NuBL32_FwPack_v2.bin` is the firmware package for NuBL32 version 2.
`NuBL33_FwPack_v2.bin` is the firmware package for NuBL33 version 2.

`license.txt` contains the public key information used by the OTA server. Since
the demo uses the ECC key algorithm, the file contains two public keys, with one
public key per line.

## Network Requirement

A wireless access point is required for the demo. The Android mobile device and
the M2354 OTA client must connect to the same wireless AP.

The application note recommends turning off the firewall and avoiding connection
to an external network during the demonstration.

The IP address of the Android mobile device is used as the OTA server IP address.
This IP address must be configured in the OTA client source code through the
`WIFIIP` definition in `ota_transfer.h`.

## How to Use the Android App

1. Install `OTA_Update_App.apk` on the Android mobile device.

2. Copy the following files to the `Download` directory of the Android mobile
   device:
   
   ```text
   NuBL32_FwPack_v2.bin
   NuBL33_FwPack_v2.bin
   license.txt
   ```

3. Connect the Android mobile device to the wireless AP used by the demo.

4. Check the Android mobile device network settings and record its IP address.
   This IP address is the OTA server IP address.

5. Configure the OTA client project so that `WIFIIP` in `ota_transfer.h` matches
   the Android mobile device IP address. Also configure the wireless AP name and
   password through `WIFINAME` and `WIFIPASS`.

6. Open the OTA Update App.

7. On the main screen, select the required files:
   
   - Click the first `SELECT FILE` button and select `NuBL32_FwPack_v2.bin`.
   - Click the second `SELECT FILE` button and select `NuBL33_FwPack_v2.bin`.
   - Click the third `SELECT FILE` button and select `license.txt`.

8. Click the `OTA PROCESS STARTED` button.

9. After the button is clicked, the app opens the OTA update server and waits
   for the OTA client to connect.

10. Press the reset button on the NuMaker-M2354 board. After rebooting, the OTA
    client connects to the Android OTA server and starts the OTA update process.

11. The Android app shows update progress prompts for the NuBL32 and NuBL33
    firmware updates.

12. When both firmware images are updated, the app shows that the OTA process is
    done. The OTA client then restarts.

## Expected Result

After the OTA update is complete:

- NuBL32 is updated to version 2.
- NuBL33 is updated to version 2.
- The NuMaker-M2354 board restarts.
- The two LEDs on the board flash alternately.

## Notes

- The Android app is described as the OTA server sample program in the
  application note.
- The application note does not describe the internal Android app architecture,
  source code structure, protocol implementation details, or UI implementation.
  This document only summarizes the app role and usage flow described in the
  application note.
