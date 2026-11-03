# How it works

## Interfaces at a glance

| Part | Interface | Pins (Black Pill) | Why |
|---|---|---|---|
| MPU6050 | I²C bus 1, 100 kHz | B7 SDA, B6 SCL | The sensor only offers I²C/SPI; I²C needs two wires |
| LCD (PCF8574) | I²C bus 2, 100 kHz | B9 SDA, B10 SCL | The backpack turns 16 LCD pins into two wires |
| ACS712 | Analog → 12-bit ADC | A2 | Output voltage follows the current |
| Supply sense | Analog → 12-bit ADC | A3 | Direct voltage measurement |
| TCRT5000 | Digital input + interrupt | B12 | One pulse per mark; no pulse may be missed |
| Relay | Digital output, open-drain | A8 | On/off; module is active-low |
| PC | UART 115200 8N1 via USB-TTL | A9 TX, A10 RX | Simplest PC link; also used for uploads |

There's no DAC: nothing needs an analog output.

## I²C

Two lines (SDA data, SCL clock), one master, devices picked by a 7-bit address: MPU6050 **0x68**, LCD backpack **0x27** (0x3F on some). A device that doesn't answer with an ACK shows up as `"lcd":0` or `"imu":0` in the boot line.

The lines are **open-drain** with pull-up resistors. The LCD backpack pulls its lines up to 5 V, so its pins must be 5 V tolerant (B9, B10 and B11 are). The MPU and LCD are on **separate buses** so the slow LCD can't hold up the 1 kHz vibration stream and the 5 V pull-ups stay off the MPU's 3.3 V lines.

If a glitch leaves the MPU holding SDA low, the firmware clocks SCL up to nine times, sends a STOP and re-initialises the sensor. The LCD runs in 4-bit mode (each character is sent in two halves); a lost half garbles everything after it, so the firmware re-syncs the display every 4 s.

## ADC and the analog sensors

The STM32's ADC is a **12-bit successive-approximation** converter: 4096 steps over 0–3.3 V, about 0.8 mV each. It samples the input onto a capacitor, then finds each bit with a binary search against an internal reference.

**ACS712.** Motor current flows through a copper strip inside the chip; a Hall-effect sensor measures its magnetic field, so the motor circuit is isolated from the logic. The output sits at **half the module's supply (about 2.5 V) at 0 A** and moves 66 mV per amp (30 A version), which is about 82 ADC steps per amp. To make that usable the firmware:

- averages **128 readings** per sample (noise falls by about √128 ≈ 11×);
- measures the **zero point at every boot** with the relay open;
- **scales the zero with the measured 5 V rail**, because the relay coil sags the rail slightly when it switches on, and the ACS712's zero moves with its supply.

The 10 kΩ resistor in series with the output limits current into A2 if the output ever exceeds 3.3 V.

**Supply sense.** Two 10 kΩ resistors halve the 5 V rail to about 2.5 V for A3.

## Speed

The TCRT5000 has an IR LED, a phototransistor and an LM393 comparator that outputs a clean digital pulse when the reflective mark passes. Each falling edge on B12 fires an interrupt that records the time since the previous edge. RPM = 60,000,000 ÷ (period in µs × pulses per revolution). Measuring the period instead of counting pulses per second stays accurate at low speeds. Edges closer than 1.5 ms apart are ignored as noise, and no pulse for 2.5 s means 0 RPM.

## Vibration

The MPU6050 is a MEMS accelerometer with its own 16-bit ADCs, set to ±4 g (8192 counts per g) and a 184 Hz low-pass filter. It fills its **FIFO buffer at exactly 1 kHz**, and the controller reads it in bursts, so no samples are lost while the CPU is busy. Every 256 samples (256 ms) the firmware computes:

- **RMS**: overall vibration energy (the main health number).
- **Peak and crest factor** (peak ÷ RMS).
- **Kurtosis**: about 3 for normal vibration, above 6 for impacts or looseness.
- **Dominant frequency**: a 256-point FFT after a **Hann window** (stops the block edges creating false frequencies), refined by interpolating around the peak bin. Resolution is 1000 ÷ 256 ≈ 3.9 Hz per bin.

## Protection logic

| Fault | Rule | Action |
|---|---|---|
| Overcurrent | Current ≥ trip limit for 0.5 s | Trip |
| Locked rotor | Current ≥ stall limit, RPM < 5, no vibration, for 1 s | Trip |
| Vibration critical | RMS ≥ trip limit for 2 s | Trip |
| Current / vibration high | Above warning level (with hysteresis) | Warn |
| Impacts | Kurtosis > 6 with vibration present | Warn |
| Under-voltage | Supply below `uv` | Warn |
| Open circuit | Relay on, current < 0.05 A | Warn |
| Speed sensor fault | Normal current, vibrating, but 0 RPM for 3 s | Warn (motor keeps running) |
| Sensor faults | IMU not answering, ADC reading out of range | Warn |

The persistence times stop start-up current and single knocks from causing trips. Warnings use hysteresis so a value sitting on a limit doesn't flicker. A trip **latches** until it's reset by the button, the dashboard or `RESET`.

**Fault isolation** comes from combining sensors. "0 RPM" alone could mean a jammed shaft, a failed speed sensor or a broken wire; current and vibration tell them apart, and only the jammed shaft trips the motor. The dashboard groups the faults into six systems: supply, motor electrical, thermal, mechanical, speed sensing and instrumentation.

**Fail-safe.** The motor is wired through the relay's normally-open contact, so it runs only while the relay is energised. The relay drops on any trip, on reset, on power loss, and when the 4-second **independent watchdog** resets a hung firmware.

## Dashboard

A single HTML file that talks to the board over **Web Serial** (Chrome or Edge; no install). On top of the device's own rules it:

- learns a **baseline** from 30 s of normal running and shows how far each signal has drifted (z-scores and an anomaly score);
- fits a **trend** to temperature and vibration and estimates the time to the warning level;
- combines protection, anomaly and trend into a **health index** (0–100);
- records **CSV**, edits the thresholds on the device, and has a **demo mode** with ten simulated fault scenarios.
