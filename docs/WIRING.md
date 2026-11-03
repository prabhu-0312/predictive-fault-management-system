# Wiring

Pin names are as printed on the board (A2 = PA2, B12 = PB12, and so on). Everything below is for the **Black Pill (STM32F411)**. On a **Blue Pill (STM32F103)** every pin is the same except the LCD's SDA, which goes to **B11** instead of B9.

## Before you start: power safety

> **Measure both rails with a multimeter before connecting the controller.** The 5 V rail must read about 5.0 V and the 3.3 V rail about 3.3 V.

We lost our first board to this: the HW-131's 5 V regulator failed and passed the 9 V input straight through to the 5 V rail. The LCD's I²C pull-ups then pushed 9 V into the microcontroller's pins. If your 5 V rail reads more than 5.5 V, don't use it. Remove that side's jumper and feed the 5 V rail from USB instead (the USB-TTL adapter's 5V pin, or the HW-131's own USB input).

Other rules:

- **Analog pins (A0–A7) are 3.3 V only.** Never connect 5 V to them directly. That's why the ACS712 goes through a 10 kΩ resistor and the supply sense through a divider.
- **Join all grounds** (HW-131, controller, USB-TTL) **except the motor battery**, which stays separate.
- With the HW-131 powering the board, **don't also connect the ST-Link's or USB-TTL's 3.3 V/5 V pins to the same rail.** One supply per rail.

## Power

| From | To |
|---|---|
| HW-131 **3.3 V** | Black Pill **3V3** pin, MPU6050 VCC |
| HW-131 **5 V** | LCD VCC, TCRT5000 VCC, ACS712 VCC, relay VCC, top of the supply divider |
| HW-131 **GND** | Black Pill **G**, every module's GND, USB-TTL GND |

The Black Pill's 5V pin and USB-C port are left unused once the board is wired (USB-C is only handy for power during first tests).

## Modules

### USB-TTL adapter (PC link and uploads)

| Adapter | Black Pill |
|---|---|
| TXD | **A10** |
| RXD | **A9** |
| GND | **G** |
| VCC / 3V3 / 5V | not connected |

TX goes to RX. If you see no data, swap these two wires first.

### MPU6050 (vibration), on the 3.3 V side

| MPU6050 | Black Pill |
|---|---|
| VCC | 3.3 V rail |
| GND | GND |
| SDA | **B7** |
| SCL | **B6** |

Mount it firmly on the motor or its bracket with double-sided tape. Solder the wires to the module's pins rather than relying on push-on jumpers: a vibrating motor shakes jumper contacts loose. Twist SDA and SCL each with a ground wire, and keep them away from the motor wires.

### 16×2 LCD with PCF8574 backpack, on the 5 V side

| LCD backpack | Black Pill |
|---|---|
| VCC | 5 V rail |
| GND | GND |
| SDA | **B9** (Blue Pill: **B11**) |
| SCL | **B10** |

The LCD is detected only at start-up, so press reset after wiring it. Backlight on but no text? Turn the blue contrast pot on the back.

### TCRT5000 (speed), on the 5 V side

| TCRT5000 | Black Pill |
|---|---|
| VCC | 5 V rail |
| GND | GND |
| DO | **B12** |
| AO | not connected |

Point it at the shaft or a disc on the shaft, 3–5 mm away, with **one** white mark on a dark surface (or black tape on a shiny one). Turn the blue pot until the module's LED flickers once per revolution. If you use more marks, set `ppr` to match.

### Supply sense (divider into A3)

```
5 V rail ──[10 kΩ]──┬──[10 kΩ]── GND
                    │
                    A3
```

A3 sees half the rail (about 2.5 V); the firmware doubles it.

### ACS712 current sensor

Signal side (3-pin header):

| ACS712 | Black Pill |
|---|---|
| VCC | 5 V rail |
| GND | GND |
| OUT | **10 kΩ resistor → A2** |

The current side (2 screw terminals) goes in series with the motor, see the motor loop below. The terminals are electrically isolated from the signal side.

### Relay module (SRD-05VDC-SL-C)

| Relay module | Black Pill |
|---|---|
| VCC | 5 V rail |
| GND | GND |
| IN | **A8** |

The module is active-low: pulling IN to ground switches it on. The firmware drives A8 as open-drain, which is safe because A8 is 5 V tolerant. Test the module on its own with `firmware/RelayTest` if in doubt.

On Chinese-labelled modules: 公共端 = **COM**, 常开 = **NO** (normally open), 常闭 = **NC**. With the relay off, a multimeter beeps between COM and NC; the other outer terminal is NO.

### Reset / manual-stop button

The Black Pill's on-board **KEY** button is already on A0, so nothing to wire. On a Blue Pill, connect a push button from **A0 to GND**. Press while running = trip; press while tripped = reset.

### Optional

- **LM35 temperature**: left leg 5 V, middle leg → A4, right leg GND (flat face towards you). Set `TEMP_FITTED` to 1 in the firmware.
- **Status LEDs**: B13 green, B14 amber, B15 red, each through 330 Ω to GND.

## Motor loop (separate 9 V battery)

```
9 V battery +  ──► ACS712 screw terminal 1
ACS712 terminal 2  ──► relay COM
relay NO  ──► motor terminal 1
motor terminal 2  ──► 9 V battery −
```

- The battery's − does **not** connect to the logic ground.
- Solder a **100 nF–0.47 µF ceramic capacitor directly across the motor terminals**. If you have two more, add one from each terminal to the motor's metal case.
- Twist the motor-loop wires together and keep the loop short and away from the I²C wires.
- A small gear motor draws about 0.1–0.5 A from 9 V. Keep demo runs short on a PP3 battery.

## Bring-up order

Connect and test one step at a time, pressing reset after each:

1. Power + USB-TTL: on-board LED blinks, dashboard receives data.
2. MPU6050: vibration about 0.005 g when still, rises when shaken.
3. LCD: shows readings about 2 s after reset.
4. TCRT5000: pulse count rises when a white card passes the sensor.
5. Supply divider: Supply reads about 5.0 V.
6. ACS712: Current about 0.00 A with nothing connected; `"acs"` in the telemetry about 2.5.
7. Relay: clicks on about 1 s after reset when there's no fault.
8. Motor loop: motor spins, current rises, **Emergency isolate** stops it.
