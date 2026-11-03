// Relay test v3: ON = pin OUTPUT LOW (sinks the relay input to GND)
//                OFF = pin INPUT (high-impedance, same as "loose")
// Toggles PA8 and PB15 every 2 s. PC13 LED lit while the relay should be ON.
void relayOn(uint32_t p)  { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
void relayOff(uint32_t p) { pinMode(p, INPUT); }
void setup() { pinMode(PC13, OUTPUT); }
void loop() {
  relayOn(PA8);  relayOn(PB15);  digitalWrite(PC13, LOW);  delay(2000);
  relayOff(PA8); relayOff(PB15); digitalWrite(PC13, HIGH); delay(2000);
}
