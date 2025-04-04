#include <avr/io.h>

// ==========================
// This is the latest working code for my drone docking 
// Beacon. This version has tunable delay variables for 
// experimenting with faster loop cycles.
// ==========================

// Header timings (µs)
unsigned int MARK_HEADER = 2000;   // Header "on" time
unsigned int SPACE_HEADER = 1000;  // Header "off" time

// Bit timings (µs)
unsigned int MARK_0 = 1000;        // Mark for '0' bit
unsigned int MARK_1 = 500;         // Mark for '1' bit
unsigned int SPACE_BIT = 500;      // Space after each bit

// Inter-packet delay (ms)
unsigned int PACKET_DELAY_MS = 25;  // Delay between packets

void setup() {
  // Carrier PWM output
  pinMode(9, OUTPUT);
  // Toggles for controlling Multiplexer
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // Clear Timer1 control registers
  TCCR1A = 0;
  TCCR1B = 0;

  // Set Timer1 to Fast PWM mode with ICR1 as TOP and enable OC1A output
  TCCR1A = (1 << WGM11) | (1 << COM1A1);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // No prescaling

  // Set ICR1 to establish a ~38kHz frequency.
  ICR1 = 418;              // ~38kHz
  OCR1A = ICR1 / 2;        // 50% duty cycle

  // Start in channel 0
  digitalWrite(5, LOW);  // A = 0
  digitalWrite(6, LOW);  // B = 0
}

void startCarrier() {
  // Turn on the PWM output by ensuring the compare output mode is enabled.
  TCCR1A |= (1 << COM1A1);
}

void stopCarrier() {
  // Turn off the PWM output by clearing the COM1A1 bit.
  TCCR1A &= ~(1 << COM1A1);
  digitalWrite(9, LOW); // Optionally drive pin 9 low
}

void loop() {
  // --- Send Number 1 (value 3: binary 0011, parity 0) ---
  // Header
  startCarrier();
  delayMicroseconds(MARK_HEADER);    // Header mark
  stopCarrier();
  delayMicroseconds(SPACE_HEADER);   // Header space
  
  // Bit 3 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 2 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 1 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 0 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Parity bit (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);

  // Channel 1:
  digitalWrite(5, HIGH);  // A = 1
  digitalWrite(6, LOW);   // B = 0
  delay(PACKET_DELAY_MS); // Inter-packet delay (ms)

  // --- Send Number 2 (value 6: binary 0110, parity 0) ---
  // Header
  startCarrier();
  delayMicroseconds(MARK_HEADER);
  stopCarrier();
  delayMicroseconds(SPACE_HEADER);
  
  // Data bits: 0, 1, 1, 0
  // Bit 3 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 2 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 1 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 0 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Parity bit (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Channel 2:
  digitalWrite(5, LOW);   // A = 0
  digitalWrite(6, HIGH);  // B = 1
  delay(PACKET_DELAY_MS);

  // --- Send Number 3 (value 9: binary 1001, parity 0) ---
  // Header
  startCarrier();
  delayMicroseconds(MARK_HEADER);
  stopCarrier();
  delayMicroseconds(SPACE_HEADER);
  
  // Data bits: 1, 0, 0, 1
  // Bit 3 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 2 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 1 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 0 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Parity bit (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Channel 3:
  digitalWrite(5, HIGH);  // A = 1
  digitalWrite(6, HIGH);  // B = 1
  delay(PACKET_DELAY_MS);

  // --- Send Number 4 (value 12: binary 1100, parity 0) ---
  // Header
  startCarrier();
  delayMicroseconds(MARK_HEADER);
  stopCarrier();
  delayMicroseconds(SPACE_HEADER);
  
  // Data bits: 1, 1, 0, 0
  // Bit 3 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 2 (1)
  startCarrier();
  delayMicroseconds(MARK_1);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 1 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Bit 0 (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Parity bit (0)
  startCarrier();
  delayMicroseconds(MARK_0);
  stopCarrier();
  delayMicroseconds(SPACE_BIT);
  
  // Back to channel 0:
  digitalWrite(5, LOW);   // A = 0
  digitalWrite(6, LOW);   // B = 0
  delay(PACKET_DELAY_MS);
}
