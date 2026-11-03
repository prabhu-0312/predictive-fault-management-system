# Troubleshooting

Everything below happened during our build. Most problems were power and wiring, not code.

| Symptom | Likely cause | Fix |
|---|---|---|
| Dashboard shows no data | USB-TTL TX/RX not crossed | Adapter TXD → A10, RXD → A9; swap them if unsure |
| Upload: "Unable to get core ID" (ST-Link) | Board unpowered, DIO/CLK swapped, or bad contact | Power the board, check wires, hold NRST while uploading, or use the serial bootloader |
| Upload stops halfway, "Timeout … Write operation fails" | Board powered from the USB-TTL's weak 3.3 V pin | Power it from the HW-131 or a phone charger, use only TX/RX/GND from the adapter |
| Upload: "Activating device: KO" (serial) | Not in bootloader mode, or Black Pill USB-C plugged into the PC | Hold BOOT0, tap NRST, release BOOT0; power USB-C from a charger, not the PC |
| DFU: "Unknown USB device (Device descriptor request failed)" | F411 USB bootloader quirk | Hold BOOT0 while plugging in, warm the chip, try another cable or port, or use the serial bootloader |
| Your code changes don't seem to be on the board | Arduino IDE saved its old copy of the sketch over your edit | Close and reopen the sketch before uploading |
| Relay never clicks | Cold solder joints on the controller's header pins | Re-solder the headers; test the module with `firmware/RelayTest` |
| 5 V rail reads ~9 V | HW-131's 5 V regulator has failed and passes the input through | Remove that side's jumper and feed 5 V from USB. This killed our first board. |
| Controller suddenly dead, 3.3 V–GND reads a few ohms | Chip damaged by over-voltage | Replace the board, and find the over-voltage before connecting the new one |
| `"lcd":0` in the boot line, backlight on, no text | I²C not answering: wiring, swapped SDA/SCL, loose jumper | Check SDA/SCL pins, swap them, check continuity, press reset |
| LCD shows solid blocks or garbage | Lost a half-character on a noisy I²C line | Firm up the LCD wires; the firmware re-syncs every 4 s |
| LCD stuck on "FAULT ISOLATION SYSTEM" | Board reset and hung, usually motor noise | Watchdog restarts it; reduce motor noise (below) |
| Vibration stuck on one value, then 0 | MPU dropped off I²C (loose jumper or noise) | Solder wires to the MPU, twist SDA/SCL with GND; firmware reconnects within ~1 s |
| Vibration reads 0 only while the motor spins | Brushed-motor noise breaking I²C | Capacitor across the motor terminals, twisted and short motor wiring away from the logic, MPU on the bracket instead of the motor body. See "Known limitations" in the README. |
| Small "ghost" vibration (~0.005 g) with the motor off | Normal accelerometer noise floor | Nothing to fix |
| Current reads 0.1–0.4 A with nothing connected | Zero shifted when the relay switched on (rail sag) | Fixed in firmware 2.1 by scaling the zero with the measured supply; press reset to re-zero |
| Current reads ~0.6 A and trips "stall" with no motor | ACS712 zero drifted, or the module was over-volted | Check the `"acs"` value is near 2.5 V with no current; `ZERO` to re-zero |
| Motor trips on start | Start-up current above the trip limit | Raise `ic` (e.g. `SET ic 1.5`, `SAVE`) |
| Holding the shaft doesn't trip | Stall current below `stall` (weak 9 V battery) or shaft still creeping | Note the held-shaft current and `SET stall` a bit below it; hold the shaft fully still |
| Board resets when the motor starts | Motor inrush on a shared supply | Give the motor its own battery; never power it from the logic rails |
| Temperature reads 100–200 °C | LM35 wiring reversed, wrong pin, or damaged sensor | Check the pinout (flat face towards you: +V, OUT, GND); the tested build disables it (`TEMP_FITTED 0`) |
| A1 reads ~1.8 V with nothing connected (Blue Pill) | That board's PA1 had a pull-up | Use PA4 (the firmware does) |
