# Building and uploading the firmware

## 1. Arduino IDE and the STM32 core

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (2.x).
2. **File → Preferences → Additional boards manager URLs**, add:
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
3. **Tools → Board → Boards Manager**, install **"STM32 MCU based boards"** by STMicroelectronics (2.x or 3.x both work).
4. Install **[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)**. The IDE calls it for every upload method below.

No extra libraries are needed: Wire, EEPROM and IWatchdog come with the core.

## 2. Board settings (Tools menu)

| Setting | Black Pill (F411) | Blue Pill (F103) |
|---|---|---|
| Board | Generic STM32F4 series | Generic STM32F1 series |
| Board part number | BlackPill F411CE | BluePill F103C8 |
| U(S)ART support | Enabled (generic 'Serial') | Enabled (generic 'Serial') |
| USB support | None | None |
| Upload method | STM32CubeProgrammer (Serial) or (SWD) | STM32CubeProgrammer (SWD) or (Serial) |

Open `firmware/FaultIsolation/FaultIsolation.ino` and click **Verify**. It should report about 49 KB of flash on the F411 (9%) and about 51 KB on the F103 (78%).

## 3. Upload: pick one method

### A. Serial bootloader through the USB-TTL (recommended, no extra hardware)

Every STM32 has a factory bootloader in ROM that listens on A9/A10, the same pins the USB-TTL already uses.

1. Close the dashboard's connection (the upload needs the COM port).
2. **Tools → Upload method → STM32CubeProgrammer (Serial)**, and **Tools → Port** → the USB-TTL's COM port.
3. Enter the bootloader: **hold BOOT0, press and release NRST, release BOOT0.** (Blue Pill: set the BOOT0 jumper to 1, press reset, and set it back to 0 after the upload.)
4. Click **Upload**. The log ends with *"Start operation achieved successfully"*.
5. Press **NRST** to run the new firmware.

Notes:
- The board must be powered by something that **isn't the PC's USB data port**, such as the HW-131 or a phone charger on USB-C. If the Black Pill's USB-C is plugged into the PC, its bootloader picks USB instead and ignores the serial pins.
- Power it from a proper supply, not the USB-TTL's weak 3.3 V pin. Flash writes draw extra current, and a weak supply makes the upload stop halfway ("Timeout error … Write operation fails").

### B. ST-Link (SWD)

| ST-Link | Board (4-pin SWD header) |
|---|---|
| SWDIO | DIO |
| SWCLK | CLK |
| GND | GND |
| 3.3V | 3V3, **only if nothing else powers the board** |

Upload method **STM32CubeProgrammer (SWD)**. If you get *"Unable to get core ID"*, check that the board is powered, check DIO/CLK aren't swapped, or hold NRST while clicking Upload and release it when the programmer starts.

### C. USB-C DFU (Black Pill only)

Hold BOOT0, plug the USB-C cable into the PC (or press NRST), release BOOT0. Device Manager should show **STM32 BOOTLOADER**. Upload method **STM32CubeProgrammer (DFU)**.

The F411's USB bootloader can be unreliable: it may show "Unknown USB device (Device descriptor request failed)" or drop out mid-download. Warming the chip with a finger, a different cable or port, or a USB 2.0 port can help. If it keeps failing, use method A.

## 4. After the first upload

- The boot line in the dashboard console shows what was detected:
  `{"boot":{"fw":"2.1.0","imu":1,"imuAddr":104,"lcd":1,"lcdAddr":39,"relay":1}}`
- Settings live in flash. A new chip starts with the defaults; change them with `SET …` and keep them with `SAVE`.

### Arduino IDE gotcha

The IDE keeps its own copy of an open sketch and can write it back over a file that was changed outside the IDE. If you edit the `.ino` in another editor, close the sketch in the IDE first, or reopen it before uploading.
