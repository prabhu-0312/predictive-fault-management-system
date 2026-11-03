/*
 * ============================================================================
 *  INDUSTRIAL FAULT ISOLATION SYSTEM  -  firmware 2.1
 *  Condition monitoring + on-device protection for a DC motor
 * ============================================================================
 *  MCU     : STM32F411CEU6 "Black Pill" (tested)  or  STM32F103C8T6 "Blue Pill"
 *  Core    : STM32duino 2.x / 3.x  (Boards Manager: "STM32 MCU based boards" by STMicroelectronics)
 *  Board   : Black Pill -> "Generic STM32F4 series", part number "BlackPill F411CE"
 *            Blue Pill  -> "Generic STM32F1 series", part number "BluePill F103C8"
 *  Upload  : "STM32CubeProgrammer (Serial)" through the USB-TTL (BOOT0 high + reset), or (SWD) with an ST-Link
 *  Serial  : U(S)ART support "Enabled (generic 'Serial')", USB support "None"
 *  Libs    : NONE beyond the core (Wire, EEPROM, IWatchdog ship with it).
 *            LCD (PCF8574) and MPU6050 drivers are written inline below.
 *
 *  SENSORS
 *    ACS712-30A -> PA2  (analog, OUT -> 10k series resistor -> PA2)   motor current
 *    V-sense    -> PA3  (analog, 10k/10k divider from the 5 V rail)   supply voltage
 *    TCRT5000   -> PB12 (digital DO, interrupt)                        shaft speed (RPM)
 *    MPU6050    -> I2C1  PB7=SDA  PB6=SCL  (3.3 V)                     vibration, 1 kHz via FIFO
 *    LM35       -> PA4  (optional; TEMP_FITTED = 0 in the tested build)
 *  OUTPUTS
 *    16x2 LCD (PCF8574 backpack, 5 V) -> I2C2  Black Pill: PB9=SDA PB10=SCL | Blue Pill: PB11=SDA PB10=SCL
 *    RELAY      : PA8 -> relay module IN (active-low, open-drain). Fail-safe: relay ENERGISED = motor
 *                 runs (motor wired through COM/NO). Trip, reset, hang or power loss = motor off.
 *    USB-TTL    : USART1  PA9=TX (to adapter RXD)  PA10=RX (to adapter TXD)  115200 8N1
 *    Status LEDs: PB13 green, PB14 amber, PB15 red (optional, via 330R to GND)
 *    Heartbeat  : PC13 on-board LED
 *  INPUT
 *    PA0 push button to GND (the Black Pill's KEY button): trips the motor while running,
 *    resets a trip when tripped.
 *  POWER
 *    3.3 V rail -> controller 3V3 pin + MPU6050.  5 V rail -> LCD, TCRT5000, ACS712, relay, divider.
 *    Motor -> its own 9 V battery through ACS712 IP+/IP- and relay COM/NO (not tied to logic GND).
 *
 *  SERIAL PROTOCOL (one JSON object per line, ~4 Hz)
 *    {"d":{...}}     telemetry frame          {"cfg":{...}}   configuration
 *    {"evt":"..."}   event (trip, reset ...)  {"ack":"..."}   command reply
 *    {"boot":{...}}  boot report
 *  COMMANDS (newline terminated, case-insensitive)
 *    PING | GET | RESET | STOP | ZERO | SAVE | DEFAULTS | SET <key> <value>
 *    keys: tw tc iw ic vw vc stall uv sens ppr autotrip vref
 * ============================================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <IWatchdog.h>   // part of the STM32 core: resets the MCU if the firmware ever hangs

/* ------------------------------- PIN MAP -------------------------------- */
#define PIN_LM35        PA4      // moved from PA1 (PA1 on this board is pulled up)
#define PIN_ACS712      PA2
#define PIN_VSENSE      PA3
#define PIN_RESET_BTN   PA0
#define PIN_IR          PB12
#define PIN_LED_GREEN   PB13
#define PIN_LED_AMBER   PB14
#define PIN_LED_RED     PB15
#define PIN_TRIP        PA8
#define PIN_HEARTBEAT   PC13

/* ------------------------------- RELAY ----------------------------------
 * Most 1-channel SRD-05VDC-SL-C modules are ACTIVE-LOW (relay clicks when IN is pulled to GND).
 * Test yours: power the module (VCC=5V, GND), touch IN to GND. Clicks -> keep 1. If it clicks only
 * when IN touches 5V/3.3V -> set RELAY_ACTIVE_LOW to 0.
 * Active-low modules are driven OPEN-DRAIN: the module's own pull-up holds IN at 5 V for "off"
 * (a 3.3 V push-pull HIGH often fails to release an optocoupler module). PA8 is 5 V tolerant.   */
#define RELAY_FITTED       1     // 0 = no relay: PA8 unused, trips are reported but motor keeps running
#define RELAY_ACTIVE_LOW   1
#define BOOT_AUTO_ZERO     1     // with the relay open at boot, zero the ACS712 automatically
#define TEMP_FITTED        0     // 0 = LM35 removed: temperature reported as "--", no temp faults or trips

#define FW_VERSION      "2.1.0"
#define LINK_BAUD       115200

#if __has_include("api/HardwareSerial.h")
Uart Link(PA10, PA9);                 // STM32duino core 3.x : USART1 -> USB-TTL adapter
#else
HardwareSerial Link(PA10, PA9);       // STM32duino core 2.x : USART1 -> USB-TTL adapter
#endif
TwoWire WireIMU(PB7, PB6);            // I2C1 : MPU6050  (SDA, SCL)
#if defined(STM32F4xx)
TwoWire WireLCD(PB9_ALT1, PB10);       // I2C2 : LCD  (SDA, SCL)  Black Pill: SDA on B9 (I2C2 alt function), SCL on B10
#else
TwoWire WireLCD(PB11, PB10);          // I2C2 : LCD      (SDA, SCL)
#endif

/* ---------------------------- ANALOG FRONT END -------------------------- */
#define ADC_BITS          12
#define ADC_FULL          4095.0f
#define ACS_DIV_RATIO     1.0f   // ACS712 OUT -> 10k series -> PA2 (no divider; 30A sensor stays < 3.3 V)
#define VS_DIV_RATIO      0.5f   // R5 10k top, R6 10k bottom -> x0.5  (5 V rail -> 2.5 V at PA3)
#define ANALOG_OVERSAMPLE 32
#define CURRENT_OVERSAMPLE 128  // 30 A sensor = 66 mV/A: average hard so ~0.2 A motor currents read cleanly

/* ------------------------------ TIMING ---------------------------------- */
#define REPORT_MS         250     // telemetry period
#define LCD_PAGE_MS       3000
#define HEARTBEAT_MS      500

/* ------------------------------ VIBRATION ------------------------------- */
#define VIB_N             256     // window length (samples) = 256 ms @ 1 kHz
#define VIB_FS            1000.0f
#define ACC_LSB_PER_G     8192.0f // +-4 g range

/* ----------------------------- FAULT CODES ------------------------------ */
enum {
  F_TEMP_WARN   = 1UL << 0,
  F_TEMP_CRIT   = 1UL << 1,
  F_CUR_WARN    = 1UL << 2,
  F_CUR_CRIT    = 1UL << 3,
  F_STALL       = 1UL << 4,
  F_VIB_WARN    = 1UL << 5,
  F_VIB_CRIT    = 1UL << 6,
  F_UNDERVOLT   = 1UL << 7,
  F_NO_LOAD     = 1UL << 8,   // supply present, motor draws ~0 A  -> open circuit
  F_IMPULSIVE   = 1UL << 9,   // high kurtosis -> impacts / looseness / bearing
  F_SNS_TEMP    = 1UL << 10,
  F_SNS_IMU     = 1UL << 11,
  F_SNS_CUR     = 1UL << 12,
  F_SNS_RPM     = 1UL << 13,  // motor clearly running but no speed pulses
  F_NO_SUPPLY   = 1UL << 14,
  F_MANUAL_STOP = 1UL << 15
};
#define F_CRITICAL_MASK (F_TEMP_CRIT | F_CUR_CRIT | F_STALL | F_VIB_CRIT)
#define F_SENSOR_MASK   (F_SNS_TEMP | F_SNS_IMU | F_SNS_CUR | F_SNS_RPM)

enum { ST_NORMAL = 0, ST_WARNING = 1, ST_ALARM = 2, ST_TRIPPED = 3 };

/* ------------------------------ CONFIG ---------------------------------- */
struct Config {
  uint32_t magic;
  float tempWarn, tempCrit;   // degC
  float curWarn,  curCrit;    // A
  float vibWarn,  vibCrit;    // g RMS
  float stallCur;             // A: current above this with rpm~0 => stall
  float underVolt;            // V
  float acsSens;              // V/A at module output (5A=0.185, 20A=0.100, 30A=0.066)
  float acsZero;              // V at module output for 0 A ; <0 => ratiometric (Vsupply/2)
  float vref;                 // ADC reference (Blue Pill 3.3 V rail)
  uint8_t ppr;                // pulses per revolution of the IR marker
  uint8_t autoTrip;           // 1 = latch trip on critical faults
  uint16_t pad;
};
#define CFG_MAGIC 0x46495333UL   // "FIS3" (bumped: new defaults)
Config cfg;

void loadDefaults() {
  cfg.magic     = CFG_MAGIC;
  cfg.tempWarn  = 45.0f;  cfg.tempCrit = 60.0f;
  cfg.curWarn   = 0.60f;  cfg.curCrit  = 1.00f;
  cfg.vibWarn   = 0.35f;  cfg.vibCrit  = 0.70f;
  cfg.stallCur  = 0.40f;
  cfg.underVolt = 4.40f;
  cfg.acsSens   = 0.066f;           // ACS712ELC-30A (66 mV/A).  05B = 0.185, 20A = 0.100
  cfg.acsZero   = -1.0f;            // ratiometric until ZERO is run
  cfg.vref      = 3.30f;
  cfg.ppr       = 1;
  cfg.autoTrip  = 1;
  cfg.pad       = 0;
}

void saveConfig() {
  eeprom_buffer_fill();
  const uint8_t *p = (const uint8_t *)&cfg;
  for (uint32_t i = 0; i < sizeof(Config); i++) eeprom_buffered_write_byte(i, p[i]);
  eeprom_buffer_flush();
}

void loadConfig() {
  eeprom_buffer_fill();
  uint8_t *p = (uint8_t *)&cfg;
  for (uint32_t i = 0; i < sizeof(Config); i++) p[i] = eeprom_buffered_read_byte(i);
  if (cfg.magic != CFG_MAGIC || isnan(cfg.tempWarn) || cfg.ppr == 0 || cfg.ppr > 64) loadDefaults();
}

/* ------------------------------ STATE ----------------------------------- */
struct Telemetry {
  float rpm, current, voltage, power, temp;
  float vibRms, vibPeak, crest, kurt, vibFreq, vibAmp, ax, ay, az;
  float health;
  uint32_t faults;
  uint8_t status;
  uint8_t sensorsOk;          // bit0 IMU, bit1 LM35, bit2 ACS712, bit3 V-sense
} tm;

bool     imuPresent = false, lcdPresent = false;
uint8_t  imuAddr = 0x68, lcdAddr = 0x27;
bool     tripped = false;
uint32_t tripCause = 0;
uint32_t frameNo = 0;
bool     vibFresh = false;
uint32_t vibWindows = 0;

// persistence timers for debounced faults
uint32_t tCurCrit = 0, tStall = 0, tVibCrit = 0, tTempCrit = 0, tRpmSns = 0;

/* =========================================================================
 *                               RPM (interrupt)
 * ========================================================================= */
#define RPM_MIN_PERIOD_US   1500UL     // ignore edges faster than this (debounce)
#define RPM_TIMEOUT_US      2500000UL  // no pulse for 2.5 s => 0 RPM

volatile uint32_t irLastEdgeUs  = 0;
volatile uint32_t irPeriodSumUs = 0;
volatile uint16_t irPeriodCnt   = 0;
volatile uint32_t irTotalPulses = 0;
float rpmEstimate = 0.0f;

void irISR() {
  uint32_t now = micros();
  uint32_t dt = now - irLastEdgeUs;
  if (irLastEdgeUs != 0 && dt < RPM_MIN_PERIOD_US) return;
  if (irLastEdgeUs != 0 && dt < 60000000UL) { irPeriodSumUs += dt; irPeriodCnt++; }
  irLastEdgeUs = now;
  irTotalPulses++;
}

float computeRpm() {
  noInterrupts();
  uint32_t sum = irPeriodSumUs, cnt = irPeriodCnt, last = irLastEdgeUs;
  irPeriodSumUs = 0; irPeriodCnt = 0;
  interrupts();

  uint32_t since = micros() - last;
  if (last == 0 || since > RPM_TIMEOUT_US) { rpmEstimate = 0; return 0; }
  if (cnt > 0) {
    float avgUs = (float)sum / (float)cnt;
    float r = 60.0e6f / (avgUs * cfg.ppr);
    rpmEstimate = (rpmEstimate <= 0) ? r : (0.5f * rpmEstimate + 0.5f * r);
  } else {
    // no new pulse in this window: speed can be at most what the elapsed time allows
    float upper = 60.0e6f / ((float)since * cfg.ppr);
    if (rpmEstimate > upper) rpmEstimate = upper;
  }
  return rpmEstimate;
}

/* =========================================================================
 *                               ANALOG
 * ========================================================================= */
float readVoltsN(uint32_t pin, int n) {
  uint32_t acc = 0;
  for (int i = 0; i < n; i++) acc += analogRead(pin);
  return ((float)acc / n) * cfg.vref / ADC_FULL;
}
float readVolts(uint32_t pin) { return readVoltsN(pin, ANALOG_OVERSAMPLE); }

float emaTemp = NAN, emaCur = NAN, emaVolt = NAN;
float lastAcsRawV = 0;   // module output voltage (before divider)
float acsCalSupply = 0;  // 5 V rail at the moment the zero was measured

void sampleAnalog() {
  // --- supply voltage ---
  float vs = readVolts(PIN_VSENSE) / VS_DIV_RATIO;
  emaVolt = isnan(emaVolt) ? vs : emaVolt + 0.4f * (vs - emaVolt);

  // --- current ---
  float vAdc = readVoltsN(PIN_ACS712, CURRENT_OVERSAMPLE);
  float vMod = vAdc / ACS_DIV_RATIO;
  lastAcsRawV = vMod;
  float zero = (cfg.acsZero > 0) ? cfg.acsZero * ((acsCalSupply > 3.0f && emaVolt > 3.0f) ? emaVolt / acsCalSupply : 1.0f) : (emaVolt > 3.0f ? emaVolt * 0.5f : 2.5f);   // ACS712 zero follows its 5 V supply (relay coil sags the rail)
  float amps = fabsf((vMod - zero) / cfg.acsSens);
  emaCur = isnan(emaCur) ? amps : emaCur + 0.35f * (amps - emaCur);

  // --- temperature (LM35: 10 mV / degC) ---
#if TEMP_FITTED
  float vT = readVolts(PIN_LM35);
  float t = vT * 100.0f;
  emaTemp = isnan(emaTemp) ? t : emaTemp + 0.2f * (t - emaTemp);
#else
  float vT = 0.5f;                 // keeps the "sensor OK" bit set so no fault is shown
  emaTemp = NAN;                   // sent as null -> dashboard and LCD show "--"
#endif

  tm.voltage = emaVolt;
  tm.current = (emaCur < 0.03f) ? 0.0f : emaCur;     // dead-band (~1.5 ADC counts on the 30 A sensor)
  tm.temp    = emaTemp;
  tm.power   = tm.voltage * tm.current;

  tm.sensorsOk = 0;
  if (imuPresent)                          tm.sensorsOk |= 0x01;
  if (vT > 0.02f && vT < 1.50f)            tm.sensorsOk |= 0x02;   // 2..150 degC plausible
  if (vAdc > 0.05f && vAdc < cfg.vref - 0.05f) tm.sensorsOk |= 0x04;
  if (vs > 1.0f)                           tm.sensorsOk |= 0x08;
}

void calibrateCurrentZero() {
  // Call with the motor NOT drawing current (motor unplugged or TRIP output open).
  float acc = 0;
  const int N = 64;
  for (int i = 0; i < N; i++) { acc += readVoltsN(PIN_ACS712, CURRENT_OVERSAMPLE) / ACS_DIV_RATIO; delay(2); }
  cfg.acsZero = acc / N;
  acsCalSupply = readVolts(PIN_VSENSE) / VS_DIV_RATIO;
  emaCur = NAN;
}

/* =========================================================================
 *                               MPU6050 (FIFO)
 * ========================================================================= */
bool imuWrite(uint8_t reg, uint8_t val) {
  WireIMU.beginTransmission(imuAddr);
  WireIMU.write(reg); WireIMU.write(val);
  return WireIMU.endTransmission() == 0;
}
bool imuRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  WireIMU.beginTransmission(imuAddr);
  WireIMU.write(reg);
  if (WireIMU.endTransmission(false) != 0) return false;
  if (WireIMU.requestFrom(imuAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = WireIMU.read();
  return true;
}

void imuFifoReset() {
  imuWrite(0x6A, 0x04);           // USER_CTRL: FIFO_RESET
  delay(2);
  imuWrite(0x6A, 0x40);           // USER_CTRL: FIFO_EN
}

uint16_t vibIdx = 0;
uint32_t imuErrCount = 0, lastImuOkMs = 0, lastImuRetry = 0;
#define IMU_I2C_HZ 100000          // 400k was unreliable on long jumper wires on a vibrating motor

// Free a stuck I2C bus: if the MPU6050 lost a clock mid-byte it holds SDA low forever.
// Clock SCL up to 9 times until SDA is released, then send a STOP. (NXP UM10204 3.1.16)
void imuBusRecover() {
  WireIMU.end();
  pinMode(PB7, INPUT_PULLUP);                 // SDA
  pinMode(PB6, OUTPUT_OPEN_DRAIN);            // SCL
  digitalWrite(PB6, HIGH); delayMicroseconds(10);
  for (uint8_t i = 0; i < 9 && digitalRead(PB7) == LOW; i++) {
    digitalWrite(PB6, LOW);  delayMicroseconds(10);
    digitalWrite(PB6, HIGH); delayMicroseconds(10);
  }
  pinMode(PB7, OUTPUT_OPEN_DRAIN);            // STOP: SDA low -> high while SCL high
  digitalWrite(PB7, LOW);  delayMicroseconds(10);
  digitalWrite(PB7, HIGH); delayMicroseconds(10);
  WireIMU.begin(); WireIMU.setClock(IMU_I2C_HZ);
}

bool imuInit() {
  const uint8_t addrs[2] = {0x68, 0x69};
  imuPresent = false;
  imuBusRecover();
  for (uint8_t k = 0; k < 2; k++) {
    imuAddr = addrs[k];
    uint8_t who = 0;
    if (imuRead(0x75, &who, 1)) { imuPresent = true; break; }
  }
  if (!imuPresent) return false;
  imuWrite(0x6B, 0x80); delay(100);   // device reset
  imuWrite(0x6B, 0x01); delay(10);    // wake, clock = PLL gyro X
  imuWrite(0x1A, 0x01);               // CONFIG: DLPF 184 Hz (accel), 1 kHz internal rate
  imuWrite(0x19, 0x00);               // SMPLRT_DIV: 1 kHz / (1+0) = 1 kHz
  imuWrite(0x1B, 0x00);               // gyro +-250 dps (unused)
  imuWrite(0x1C, 0x08);               // accel +-4 g
  imuWrite(0x23, 0x08);               // FIFO_EN: accelerometer only
  imuFifoReset();
  imuErrCount = 0; vibIdx = 0;
  return true;
}

int16_t vibX[VIB_N], vibY[VIB_N], vibZ[VIB_N];

void processVibrationWindow();

void serviceIMU() {
  if (!imuPresent) return;
  uint8_t c[2];
  if (!imuRead(0x72, c, 2)) {               // FIFO_COUNT
    if (++imuErrCount > 10) { imuPresent = false; lastImuRetry = 0; }   // drop fast, recover in loop
    return;
  }
  imuErrCount = 0;
  lastImuOkMs = millis();
  uint16_t count = ((uint16_t)c[0] << 8) | c[1];
  if (count >= 1020 || count % 6) {                       // FIFO overflowed -> data misaligned, restart
    imuFifoReset(); vibIdx = 0; return;
  }
  uint16_t samples = count / 6;
  while (samples > 0) {
    uint8_t take = samples > 5 ? 5 : samples;   // 30-byte bursts (safe for any Wire buffer)
    uint8_t buf[30];
    if (!imuRead(0x74, buf, take * 6)) { imuFifoReset(); vibIdx = 0; return; }
    for (uint8_t s = 0; s < take; s++) {
      const uint8_t *b = &buf[s * 6];
      vibX[vibIdx] = (int16_t)((b[0] << 8) | b[1]);
      vibY[vibIdx] = (int16_t)((b[2] << 8) | b[3]);
      vibZ[vibIdx] = (int16_t)((b[4] << 8) | b[5]);
      if (++vibIdx >= VIB_N) { processVibrationWindow(); vibIdx = 0; }
    }
    samples -= take;
  }
}

/* --- radix-2 FFT (in place) --- */
float fftRe[VIB_N], fftIm[VIB_N];

void fft256(float *re, float *im) {
  const int n = VIB_N;
  for (int i = 1, j = 0; i < n; i++) {             // bit reversal
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * (float)M_PI / len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = i + k + len / 2;
        float xr = re[b] * cr - im[b] * ci;
        float xi = re[b] * ci + im[b] * cr;
        re[b] = re[a] - xr; im[b] = im[a] - xi;
        re[a] += xr;        im[a] += xi;
        float ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr; cr = ncr;
      }
    }
  }
}

void processVibrationWindow() {
  // mean (gravity / static tilt) per axis
  float mx = 0, my = 0, mz = 0;
  for (int i = 0; i < VIB_N; i++) { mx += vibX[i]; my += vibY[i]; mz += vibZ[i]; }
  mx /= VIB_N; my /= VIB_N; mz /= VIB_N;

  float sx = 0, sy = 0, sz = 0, peak2 = 0;
  for (int i = 0; i < VIB_N; i++) {
    float dx = vibX[i] - mx, dy = vibY[i] - my, dz = vibZ[i] - mz;
    sx += dx * dx; sy += dy * dy; sz += dz * dz;
    float m2 = dx * dx + dy * dy + dz * dz;
    if (m2 > peak2) peak2 = m2;
  }
  float rx = sqrtf(sx / VIB_N) / ACC_LSB_PER_G;
  float ry = sqrtf(sy / VIB_N) / ACC_LSB_PER_G;
  float rz = sqrtf(sz / VIB_N) / ACC_LSB_PER_G;
  float rms = sqrtf(rx * rx + ry * ry + rz * rz);
  float peak = sqrtf(peak2) / ACC_LSB_PER_G;

  // dominant axis -> kurtosis + spectrum
  int16_t *d; float m, var;
  if (sx >= sy && sx >= sz) { d = vibX; m = mx; var = sx / VIB_N; }
  else if (sy >= sz)        { d = vibY; m = my; var = sy / VIB_N; }
  else                      { d = vibZ; m = mz; var = sz / VIB_N; }

  float m4 = 0;
  for (int i = 0; i < VIB_N; i++) {
    float v = d[i] - m; v *= v; m4 += v * v;
    float w = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (VIB_N - 1));   // Hann
    fftRe[i] = (d[i] - m) * w; fftIm[i] = 0;
  }
  m4 /= VIB_N;
  float kurt = (var > 1.0f) ? m4 / (var * var) : 3.0f;

  fft256(fftRe, fftIm);
  float best = 0; int bestBin = 0;
  for (int k = 2; k < VIB_N / 2; k++) {                 // skip DC + first bin
    float p = fftRe[k] * fftRe[k] + fftIm[k] * fftIm[k];
    if (p > best) { best = p; bestBin = k; }
  }
  // parabolic interpolation for a finer frequency estimate
  float freq = bestBin * VIB_FS / VIB_N;
  if (bestBin > 2 && bestBin < VIB_N / 2 - 1) {
    float a = sqrtf(fftRe[bestBin-1]*fftRe[bestBin-1] + fftIm[bestBin-1]*fftIm[bestBin-1]);
    float b = sqrtf(best);
    float c = sqrtf(fftRe[bestBin+1]*fftRe[bestBin+1] + fftIm[bestBin+1]*fftIm[bestBin+1]);
    float den = a - 2 * b + c;
    if (fabsf(den) > 1e-6f) freq = (bestBin + 0.5f * (a - c) / den) * VIB_FS / VIB_N;
  }
  float amp = 4.0f * sqrtf(best) / VIB_N / ACC_LSB_PER_G;    // Hann-corrected peak amplitude, g

  // light smoothing of the headline values
  const float a = (vibWindows == 0) ? 1.0f : 0.5f;
  tm.vibRms  += a * (rms  - tm.vibRms);
  tm.vibPeak += a * (peak - tm.vibPeak);
  tm.kurt    += a * (kurt - tm.kurt);
  tm.crest    = (tm.vibRms > 1e-4f) ? tm.vibPeak / tm.vibRms : 0;
  tm.vibFreq  = freq;
  tm.vibAmp   = amp;
  tm.ax = rx; tm.ay = ry; tm.az = rz;
  vibWindows++;
  vibFresh = true;
}

/* =========================================================================
 *                   16x2 LCD over PCF8574 (non-blocking)
 * ========================================================================= */
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

bool lcdRaw(uint8_t v, bool rs) {
  uint8_t hi = (v & 0xF0) | LCD_BL | (rs ? LCD_RS : 0);
  uint8_t lo = ((v << 4) & 0xF0) | LCD_BL | (rs ? LCD_RS : 0);
  WireLCD.beginTransmission(lcdAddr);
  WireLCD.write(hi | LCD_EN); WireLCD.write(hi);
  WireLCD.write(lo | LCD_EN); WireLCD.write(lo);
  return WireLCD.endTransmission() == 0;
}
void lcdNibbleInit(uint8_t n) {
  uint8_t v = (n << 4) | LCD_BL;
  WireLCD.beginTransmission(lcdAddr);
  WireLCD.write(v | LCD_EN); WireLCD.write(v);
  WireLCD.endTransmission();
}

char lcdWant[2][17], lcdHave[2][17];
uint8_t lcdScan = 0;

bool lcdInit() {
  const uint8_t cands[] = {0x27, 0x3F, 0x26, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E};
  lcdPresent = false;
  for (uint8_t i = 0; i < sizeof(cands); i++) {
    WireLCD.beginTransmission(cands[i]);
    if (WireLCD.endTransmission() == 0) { lcdAddr = cands[i]; lcdPresent = true; break; }
  }
  if (!lcdPresent) return false;
  delay(50);
  lcdNibbleInit(0x03); delay(5);
  lcdNibbleInit(0x03); delay(5);
  lcdNibbleInit(0x03); delayMicroseconds(200);
  lcdNibbleInit(0x02); delayMicroseconds(200);
  lcdRaw(0x28, false);   // 4-bit, 2 lines, 5x8
  lcdRaw(0x0C, false);   // display on, cursor off
  lcdRaw(0x06, false);   // entry mode
  lcdRaw(0x01, false);   // clear
  delay(3);
  memset(lcdHave, ' ', sizeof(lcdHave));
  memset(lcdWant, ' ', sizeof(lcdWant));
  lcdHave[0][16] = lcdHave[1][16] = lcdWant[0][16] = lcdWant[1][16] = 0;
  return true;
}

void lcdSetLine(uint8_t row, const char *txt) {
  uint8_t i = 0;
  for (; i < 16 && txt[i]; i++) lcdWant[row][i] = txt[i];
  for (; i < 16; i++) lcdWant[row][i] = ' ';
}

// Writes at most ONE changed character per call (~0.5 ms) so sampling never stalls.
void serviceLCD() {
  if (!lcdPresent) return;
  for (uint8_t tries = 0; tries < 32; tries++) {
    uint8_t row = lcdScan >> 4, col = lcdScan & 0x0F;
    lcdScan = (lcdScan + 1) & 0x1F;
    if (lcdWant[row][col] != lcdHave[row][col]) {
      bool ok = lcdRaw(0x80 | (row ? 0x40 : 0x00) | col, false);
      ok = lcdRaw((uint8_t)lcdWant[row][col], true) && ok;
      if (ok) lcdHave[row][col] = lcdWant[row][col];   // failed write -> retried on a later pass
      return;
    }
  }
}

// A single glitch on the LCD's I2C wires knocks the HD44780 out of 4-bit nibble sync and every
// following character becomes garbage (solid blocks). Every few seconds: re-sync the interface
// (no clear, so no flicker) and redraw all 32 characters.
void lcdResync() {
  if (!lcdPresent) return;
  lcdNibbleInit(0x03); delay(5);
  lcdNibbleInit(0x03); delayMicroseconds(200);
  lcdNibbleInit(0x03); delayMicroseconds(200);
  lcdNibbleInit(0x02); delayMicroseconds(200);
  lcdRaw(0x28, false); lcdRaw(0x0C, false); lcdRaw(0x06, false);
  memset(lcdHave, 0x01, sizeof(lcdHave));            // 0x01 never appears in text -> forces full redraw
  lcdHave[0][16] = lcdHave[1][16] = 0;
}

/* =========================================================================
 *                      PROTECTION + DIAGNOSIS
 * ========================================================================= */
static float sev(float x, float warn, float crit) {
  float start = 0.7f * warn;
  if (x <= start) return 0;
  if (x >= crit) return 1;
  return (x - start) / (crit - start);
}

const char *faultShort(uint32_t f) {
  if (f & F_MANUAL_STOP) return "MANUAL STOP";
  if (f & F_STALL)       return "ROTOR STALL";
  if (f & F_CUR_CRIT)    return "OVERCURRENT";
  if (f & F_TEMP_CRIT)   return "OVERTEMP";
  if (f & F_VIB_CRIT)    return "VIBRATION HIGH";
  if (f & F_NO_SUPPLY)   return "NO SUPPLY";
  if (f & F_SNS_IMU)     return "IMU SENSOR FLT";
  if (f & F_SNS_TEMP)    return "TEMP SENSOR FLT";
  if (f & F_SNS_CUR)     return "CUR SENSOR FLT";
  if (f & F_SNS_RPM)     return "RPM SENSOR FLT";
  if (f & F_UNDERVOLT)   return "UNDERVOLTAGE";
  if (f & F_CUR_WARN)    return "CURRENT HIGH";
  if (f & F_TEMP_WARN)   return "TEMP HIGH";
  if (f & F_VIB_WARN)    return "VIBRATION WARN";
  if (f & F_IMPULSIVE)   return "IMPACTS/LOOSE";
  if (f & F_NO_LOAD)     return "OPEN CIRCUIT";
  return "ALL NORMAL";
}

/* Motor power through the relay. Fail-safe: energised = run. */
void setMotor(bool run) {
#if RELAY_FITTED
  #if RELAY_ACTIVE_LOW
  if (run) { pinMode(PIN_TRIP, OUTPUT_OPEN_DRAIN); digitalWrite(PIN_TRIP, LOW); }   // pull IN low -> relay ON
  else     { pinMode(PIN_TRIP, OUTPUT_OPEN_DRAIN); digitalWrite(PIN_TRIP, HIGH); }  // release -> module pull-up -> OFF
  #else
  pinMode(PIN_TRIP, OUTPUT); digitalWrite(PIN_TRIP, run ? HIGH : LOW);
  #endif
#else
  (void)run;
#endif
}

void doTrip(uint32_t cause) {
  if (tripped) return;
  tripped = true;
  tripCause = cause;
  setMotor(false);
  Link.print(F("{\"evt\":\"TRIP\",\"f\":")); Link.print(cause);
  Link.print(F(",\"why\":\"")); Link.print(faultShort(cause)); Link.println(F("\"}"));
}

void clearTrip() {
  tripped = false;
  tripCause = 0;
  tCurCrit = tStall = tVibCrit = tTempCrit = 0;
  setMotor(true);
  Link.println(F("{\"evt\":\"RESET\"}"));
}

static bool persist(bool cond, uint32_t &since, uint32_t ms) {
  uint32_t now = millis();
  if (!cond) { since = 0; return false; }
  if (since == 0) since = now ? now : 1;
  return (now - since) >= ms;
}

void evaluate() {
  uint32_t f = 0;
  const bool supply = tm.sensorsOk & 0x08;
  const bool tempOk = TEMP_FITTED && (tm.sensorsOk & 0x02);
  const bool curOk  = tm.sensorsOk & 0x04;
  const bool motorEnergised = supply && !tripped;

  if (!supply)                  f |= F_NO_SUPPLY;
  if (!(tm.sensorsOk & 0x01))   f |= F_SNS_IMU;
  if (TEMP_FITTED && !tempOk)   f |= F_SNS_TEMP;
  if (supply && !curOk)         f |= F_SNS_CUR;

  // warnings use hysteresis so a value sitting on a limit doesn't chatter
  static bool hTemp = false, hCur = false, hVib = false, hImp = false;
  if (tempOk) {
    hTemp = (tm.temp >= cfg.tempWarn) || (hTemp && tm.temp >= cfg.tempWarn - 1.0f);
    if (hTemp) f |= F_TEMP_WARN;
    if (persist(tm.temp >= cfg.tempCrit, tTempCrit, 1000)) f |= F_TEMP_CRIT;
  } else hTemp = false;
  if (curOk) {
    hCur = (tm.current >= cfg.curWarn) || (hCur && tm.current >= 0.92f * cfg.curWarn);
    if (hCur) f |= F_CUR_WARN;
    if (persist(tm.current >= cfg.curCrit, tCurCrit, 500)) f |= F_CUR_CRIT;   // ignore inrush
  } else hCur = false;
  if (imuPresent) {
    hVib = (tm.vibRms >= cfg.vibWarn) || (hVib && tm.vibRms >= 0.9f * cfg.vibWarn);
    if (hVib) f |= F_VIB_WARN;
    if (persist(tm.vibRms >= cfg.vibCrit, tVibCrit, 2000)) f |= F_VIB_CRIT;
    hImp = (tm.kurt > 6.0f && tm.vibRms > 0.5f * cfg.vibWarn) || (hImp && tm.kurt > 5.0f && tm.vibRms > 0.4f * cfg.vibWarn);
    if (hImp) f |= F_IMPULSIVE;
  } else { hVib = false; hImp = false; }
  if (supply && tm.voltage < cfg.underVolt) f |= F_UNDERVOLT;

  // --- differential diagnosis: why is RPM zero? ---
  bool rpmZero = tm.rpm < 5.0f;
  bool vibrating = imuPresent && tm.vibRms > 0.04f;      // motor is clearly spinning
  if (motorEnergised && curOk && rpmZero) {
    if (tm.current >= cfg.stallCur && !vibrating) {
      if (persist(true, tStall, 1000)) f |= F_STALL;        // locked rotor
    } else tStall = 0;
    if (tm.current > 0.05f && tm.current < cfg.stallCur && vibrating) {
      if (persist(true, tRpmSns, 3000)) f |= F_SNS_RPM;     // spinning but no pulses
    } else tRpmSns = 0;
    if (tm.current < 0.05f) f |= F_NO_LOAD;                 // energised, no current
  } else { tStall = 0; tRpmSns = 0; }
  // overcurrent with a stationary, silent shaft IS a stall -> report the root cause, not just the symptom
  if ((f & F_CUR_CRIT) && motorEnergised && rpmZero && !vibrating) f |= F_STALL;

  if (tripped && (tripCause & F_MANUAL_STOP)) f |= F_MANUAL_STOP;

  // --- protection ---
  if (cfg.autoTrip && (f & F_CRITICAL_MASK)) doTrip(f & F_CRITICAL_MASK);

  // --- health index (device-side, rule based) ---
  float s[4] = {0, 0, 0, 0};
  if (tempOk) s[0] = sev(tm.temp, cfg.tempWarn, cfg.tempCrit);
  if (curOk)  s[1] = sev(tm.current, cfg.curWarn, cfg.curCrit);
  if (imuPresent) s[2] = sev(tm.vibRms, cfg.vibWarn, cfg.vibCrit);
  if (f & F_STALL) s[3] = 1;
  float mx = 0, sum = 0;
  for (int i = 0; i < 4; i++) { if (s[i] > mx) mx = s[i]; sum += s[i]; }
  float h = 100.0f - 65.0f * mx - 35.0f * (sum / 4.0f);
  if (f & F_IMPULSIVE) h -= 10;
  if (f & F_UNDERVOLT) h -= 5;
  if ((f & F_SENSOR_MASK) && h > 60) h = 60;      // can't vouch for health with a dead sensor
  if (h < 0) h = 0;
  tm.health = h;
  tm.faults = f;

  if (tripped)                                   tm.status = ST_TRIPPED;
  else if (f & (F_CRITICAL_MASK | F_NO_SUPPLY))  tm.status = ST_ALARM;
  else if (f)                                    tm.status = ST_WARNING;
  else                                           tm.status = ST_NORMAL;
}

void updateLeds() {
  static uint32_t blink = 0;
  bool fast = (millis() / 150) & 1;
  digitalWrite(PIN_LED_GREEN, tm.status == ST_NORMAL);
  digitalWrite(PIN_LED_AMBER, tm.status == ST_WARNING);
  digitalWrite(PIN_LED_RED,   tm.status == ST_TRIPPED ? HIGH : (tm.status == ST_ALARM ? fast : LOW));
  (void)blink;
}

/* =========================================================================
 *                               LCD PAGES
 * ========================================================================= */
uint8_t lcdPage = 0;
uint32_t lastPageMs = 0;

// Tiny float formatter. Avoids printf/strtod, which would pull ~10 KB into a 64 KB flash.
void fmtFloat(char *out, float v, uint8_t dec) {
  if (isnan(v) || isinf(v)) { out[0] = '-'; out[1] = '-'; out[2] = 0; return; }
  char tmp[16]; uint8_t n = 0;
  bool neg = v < 0; if (neg) v = -v;
  uint32_t mul = 1; for (uint8_t i = 0; i < dec; i++) mul *= 10;
  if (v > 4.0e9f / mul) v = 4.0e9f / mul;
  uint32_t x = (uint32_t)(v * mul + 0.5f);
  for (uint8_t i = 0; i < dec; i++) { tmp[n++] = '0' + x % 10; x /= 10; }
  if (dec) tmp[n++] = '.';
  do { tmp[n++] = '0' + x % 10; x /= 10; } while (x);
  if (neg) tmp[n++] = '-';
  for (uint8_t i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  out[n] = 0;
}

// simple decimal parser ("12", "-3.5", "0.185")
float parseNum(const char *s) {
  float sign = 1, val = 0, scale = 0;
  if (*s == '-') { sign = -1; s++; } else if (*s == '+') s++;
  for (; *s; s++) {
    if (*s == '.') { if (scale) break; scale = 1; continue; }
    if (*s < '0' || *s > '9') break;
    val = val * 10 + (*s - '0');
    if (scale) scale *= 10;
  }
  return sign * (scale ? val / scale : val);
}

// string builder helpers for the LCD lines
static char *cat(char *d, const char *s) { while (*s) *d++ = *s++; *d = 0; return d; }
static char *padTo(char *start, char *d, uint8_t width) { while ((uint8_t)(d - start) < width) *d++ = ' '; *d = 0; return d; }

void renderLCD() {
  char l0[40], l1[40], a[16], b[16];
  static const char *stName[] = {"NORMAL", "WARNING", "ALARM", "TRIPPED"};

  if (tripped) {
    bool alt = (millis() / 1500) & 1;
    lcdSetLine(0, alt ? "** MOTOR TRIP **" : "PRESS RESET/PC  ");
    lcdSetLine(1, faultShort(tripCause));
    return;
  }
  uint8_t pages = (tm.faults) ? 3 : 2;
  if (millis() - lastPageMs >= LCD_PAGE_MS) { lastPageMs = millis(); lcdPage = (lcdPage + 1) % pages; }
  if (lcdPage >= pages) lcdPage = 0;

  char *p;
  switch (lcdPage) {
    case 0:   // "RPM 201  I0.23A" / "T34.2C Vb0.12g"
      fmtFloat(a, tm.rpm, 0); fmtFloat(b, tm.current, 2);
      p = cat(l0, "RPM"); p = cat(p, a); p = padTo(l0, p, 9); p = cat(p, "I"); p = cat(p, b); cat(p, "A");
      fmtFloat(a, tm.temp, 1); fmtFloat(b, tm.vibRms, 2);
      p = cat(l1, "T"); p = cat(p, a); p = cat(p, "C"); p = padTo(l1, p, 8); p = cat(p, "Vb"); p = cat(p, b); cat(p, "g");
      break;
    case 1:   // "V5.02  P1.16W" / "HEALTH 96% NORMAL"
      fmtFloat(a, tm.voltage, 2); fmtFloat(b, tm.power, 2);
      p = cat(l0, "V"); p = cat(p, a); p = padTo(l0, p, 8); p = cat(p, "P"); p = cat(p, b); cat(p, "W");
      fmtFloat(a, tm.health, 0);
      p = cat(l1, "H"); p = cat(p, a); p = cat(p, "% "); cat(p, stName[tm.status]);
      break;
    default:
      cat(l0, tm.status >= ST_ALARM ? "!! ALARM ACTIVE" : "! WARNING");
      cat(l1, faultShort(tm.faults));
      break;
  }
  l0[16] = 0; l1[16] = 0;
  lcdSetLine(0, l0);
  lcdSetLine(1, l1);
}

/* =========================================================================
 *                          SERIAL OUTPUT (JSON)
 * ========================================================================= */
void kv(const char *k, float v, uint8_t dec, bool comma) {
  char buf[20];
  if (comma) Link.print(',');
  Link.print('"'); Link.print(k); Link.print(F("\":"));
  if (isnan(v) || isinf(v)) { Link.print(F("null")); return; }
  fmtFloat(buf, v, dec);
  Link.print(buf);
}
void kvU(const char *k, uint32_t v) { Link.print(F(",\"")); Link.print(k); Link.print(F("\":")); Link.print(v); }

void sendFrame() {
  Link.print(F("{\"d\":{"));
  kv("t", (float)millis(), 0, false);
  kvU("n", frameNo++);
  kv("rpm", tm.rpm, 1, true);
  kv("i", tm.current, 3, true);
  kv("v", tm.voltage, 2, true);
  kv("p", tm.power, 2, true);
  kv("tc", tm.temp, 2, true);
  kv("vr", tm.vibRms, 4, true);
  kv("vp", tm.vibPeak, 3, true);
  kv("cf", tm.crest, 2, true);
  kv("ku", tm.kurt, 2, true);
  kv("vf", tm.vibFreq, 1, true);
  kv("va", tm.vibAmp, 4, true);
  kv("ax", tm.ax, 4, true);
  kv("ay", tm.ay, 4, true);
  kv("az", tm.az, 4, true);
  kv("h", tm.health, 1, true);
  kvU("s", tm.status);
  kvU("f", tm.faults);
  kvU("tr", tripped ? 1 : 0);
  kvU("tf", tripCause);
  kvU("ok", tm.sensorsOk);
  kvU("pc", irTotalPulses);
  kv("acs", lastAcsRawV, 3, true);
  Link.println(F("}}"));
}

void sendConfig() {
  Link.print(F("{\"cfg\":{"));
  kv("tw", cfg.tempWarn, 1, false); kv("tc", cfg.tempCrit, 1, true);
  kv("iw", cfg.curWarn, 2, true);  kv("ic", cfg.curCrit, 2, true);
  kv("vw", cfg.vibWarn, 3, true);  kv("vc", cfg.vibCrit, 3, true);
  kv("stall", cfg.stallCur, 2, true); kv("uv", cfg.underVolt, 2, true);
  kv("sens", cfg.acsSens, 3, true); kv("zero", cfg.acsZero, 3, true); kv("vref", cfg.vref, 3, true);
  kvU("ppr", cfg.ppr); kvU("autotrip", cfg.autoTrip);
  Link.print(F(",\"fw\":\"" FW_VERSION "\"")); Link.println(F("}}"));
}

void ack(const char *what, bool ok) {
  Link.print(F("{\"ack\":\"")); Link.print(what); Link.print(F("\",\"ok\":")); Link.print(ok ? 1 : 0); Link.println('}');
}

/* =========================================================================
 *                               COMMANDS
 * ========================================================================= */
char rxLine[64];
uint8_t rxLen = 0;

void handleCommand(char *s) {
  for (char *p = s; *p; p++) *p = toupper(*p);
  char *cmd = strtok(s, " \t");
  if (!cmd) return;
  if (!strcmp(cmd, "PING"))          { ack("PING", true); }
  else if (!strcmp(cmd, "GET"))      { sendConfig(); }
  else if (!strcmp(cmd, "RESET"))    { clearTrip(); ack("RESET", true); }
  else if (!strcmp(cmd, "STOP"))     { tripped = false; doTrip(F_MANUAL_STOP); ack("STOP", true); }
  else if (!strcmp(cmd, "ZERO"))     {
#if RELAY_FITTED
    setMotor(false); delay(400);     // open the relay so the motor really draws 0 A
#endif
    calibrateCurrentZero();
#if RELAY_FITTED
    if (!tripped) setMotor(true);
#endif
    ack("ZERO", true); sendConfig(); }
  else if (!strcmp(cmd, "SAVE"))     { saveConfig(); ack("SAVE", true); }
  else if (!strcmp(cmd, "DEFAULTS")) { loadDefaults(); ack("DEFAULTS", true); sendConfig(); }
  else if (!strcmp(cmd, "SET")) {
    char *k = strtok(NULL, " \t"), *v = strtok(NULL, " \t");
    if (!k || !v) { ack("SET", false); return; }
    float x = parseNum(v); bool ok = true;
    if      (!strcmp(k, "TW"))       cfg.tempWarn = x;
    else if (!strcmp(k, "TC"))       cfg.tempCrit = x;
    else if (!strcmp(k, "IW"))       cfg.curWarn = x;
    else if (!strcmp(k, "IC"))       cfg.curCrit = x;
    else if (!strcmp(k, "VW"))       cfg.vibWarn = x;
    else if (!strcmp(k, "VC"))       cfg.vibCrit = x;
    else if (!strcmp(k, "STALL"))    cfg.stallCur = x;
    else if (!strcmp(k, "UV"))       cfg.underVolt = x;
    else if (!strcmp(k, "SENS"))     ok = (x > 0.01f && x < 1.0f) ? (cfg.acsSens = x, true) : false;
    else if (!strcmp(k, "VREF"))     ok = (x > 2.5f && x < 3.6f) ? (cfg.vref = x, true) : false;
    else if (!strcmp(k, "PPR"))      ok = (x >= 1 && x <= 64) ? (cfg.ppr = (uint8_t)x, true) : false;
    else if (!strcmp(k, "AUTOTRIP")) cfg.autoTrip = (x != 0);
    else ok = false;
    ack("SET", ok);
    if (ok) sendConfig();
  }
  else ack("UNKNOWN", false);
}

void serviceSerialRx() {
  while (Link.available()) {
    char c = Link.read();
    if (c == '\r') continue;
    if (c == '\n') { rxLine[rxLen] = 0; if (rxLen) handleCommand(rxLine); rxLen = 0; }
    else if (rxLen < sizeof(rxLine) - 1) rxLine[rxLen++] = c;
  }
}

/* --------------------------- reset button -------------------------------- */
void serviceButton() {
  static uint32_t downSince = 0;
  static bool handled = false;
  bool down = digitalRead(PIN_RESET_BTN) == LOW;
  if (!down) { downSince = 0; handled = false; return; }
  if (downSince == 0) downSince = millis();
  if (!handled && millis() - downSince > 60) { handled = true; if (tripped) clearTrip(); else doTrip(F_MANUAL_STOP); }
}

/* =========================================================================
 *                               SETUP / LOOP
 * ========================================================================= */
void setup() {
  setMotor(false);                 // motor stays OFF until the self-test and current-zero are done
  bool wdtReset = IWatchdog.isReset(true);
  IWatchdog.begin(4000000);         // 4 s: a hang (e.g. I2C lock-up from motor noise) reboots the board, relay drops = motor off
  pinMode(PIN_LED_GREEN, OUTPUT);  pinMode(PIN_LED_AMBER, OUTPUT); pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_HEARTBEAT, OUTPUT);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  pinMode(PIN_IR, INPUT_PULLUP);
  analogReadResolution(ADC_BITS);

  // lamp test
  digitalWrite(PIN_LED_GREEN, HIGH); digitalWrite(PIN_LED_AMBER, HIGH); digitalWrite(PIN_LED_RED, HIGH);

  Link.begin(LINK_BAUD);
  loadConfig();

  WireIMU.begin(); WireIMU.setClock(IMU_I2C_HZ);
  WireLCD.begin(); WireLCD.setClock(100000);

  if (lcdInit()) {
    lcdSetLine(0, "FAULT ISOLATION");
    lcdSetLine(1, "SYSTEM  v" FW_VERSION);
    for (int i = 0; i < 64; i++) serviceLCD();
  }
  IWatchdog.reload();
  imuInit();
  IWatchdog.reload();

  attachInterrupt(digitalPinToInterrupt(PIN_IR), irISR, FALLING);

  delay(400);
#if RELAY_FITTED && BOOT_AUTO_ZERO
  calibrateCurrentZero();          // relay is open -> true 0 A reading (not saved to flash)
#else
  delay(400);
#endif
  setMotor(true);                  // all good: energise relay, motor starts
  digitalWrite(PIN_LED_GREEN, LOW); digitalWrite(PIN_LED_AMBER, LOW); digitalWrite(PIN_LED_RED, LOW);

  Link.print(F("{\"boot\":{\"fw\":\"" FW_VERSION "\",\"imu\":")); Link.print(imuPresent ? 1 : 0);
  Link.print(F(",\"imuAddr\":")); Link.print(imuAddr);
  Link.print(F(",\"lcd\":")); Link.print(lcdPresent ? 1 : 0);
  Link.print(F(",\"lcdAddr\":")); Link.print(lcdAddr);
  Link.print(F(",\"relay\":")); Link.print(RELAY_FITTED);
  Link.println(F("}}"));
  sendConfig();
  if (wdtReset) Link.println(F("{\"evt\":\"WATCHDOG_RESET\"}"));
}

void loop() {
  static uint32_t lastReport = 0, lastBeat = 0;
  IWatchdog.reload();
  uint32_t now = millis();

  serviceIMU();
  serviceSerialRx();
  serviceButton();
  serviceLCD();

  if (!imuPresent && now - lastImuRetry > 1000) {     // hot-plug / recovery
    lastImuRetry = now;
    if (imuInit()) Link.println(F("{\"evt\":\"IMU_ONLINE\"}"));
  }

  if (now - lastReport >= REPORT_MS) {
    lastReport += REPORT_MS;
    if (now - lastReport > REPORT_MS) lastReport = now;   // resync after stalls
    sampleAnalog();
    tm.rpm = computeRpm();
    if (!imuPresent) { tm.vibRms = tm.vibPeak = tm.crest = tm.kurt = tm.vibFreq = tm.vibAmp = 0; }
    evaluate();
    updateLeds();
    renderLCD();
    sendFrame();
  }

  static uint32_t lastLcdSync = 0;
  if (now - lastLcdSync >= 4000) { lastLcdSync = now; lcdResync(); }

  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    digitalWrite(PIN_HEARTBEAT, !digitalRead(PIN_HEARTBEAT));
  }
}
