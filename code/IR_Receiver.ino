/*****************************************************
 * IR Receiver with Interrupts (Inverted Logic)
 * For a demodulated IR sensor (VS1838) output
 * 
 * Steps:
 * 1) On every RISING/FALLING edge, record micros() + new state
 * 2) In loop(), parse these edges to decode custom protocol
 *****************************************************/

#include <Arduino.h>

// =============================
// USER-ADJUSTABLE TIMINGS
// =============================
#define HEADER_MARK    2000   // ~2ms LOW for header
#define HEADER_SPACE   1000   // ~1ms HIGH for header
#define BIT_MARK_1     500    // ~0.5ms LOW for bit=1
#define BIT_MARK_0     1000   // ~1.0ms LOW for bit=0
#define BIT_SPACE      500    // ~0.5ms HIGH after each bit
#define TOLERANCE      200    // +/- in microseconds

// Increase if you need to handle many edges
#define RING_BUFFER_SIZE  128

// Pin connected to VS1838 output
#define IR_PIN  2  

// For direction logic
bool seenA, seenB, seenC, seenD;

// A small struct to store each edge
struct PulseEvent {
  unsigned long duration; // microseconds since previous edge
  bool level;             // the new logic level after the edge
};

// Ring buffer to hold edges from ISR
volatile PulseEvent pulseBuffer[RING_BUFFER_SIZE];
volatile uint8_t writeIndex = 0;
volatile uint8_t readIndex  = 0;

// For measuring durations
volatile unsigned long lastEdgeTime;
volatile bool lastLevel;

// Forward declarations
void IR_ISR();
bool checkInRange(unsigned long val, unsigned long target, unsigned long tol);
void parsePulses();
String interpretDirection(bool A, bool B, bool C, bool D);

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT); // or INPUT_PULLUP if your hardware requires

  // Prepare ring buffer
  writeIndex = 0;
  readIndex  = 0;
  lastEdgeTime = micros();
  lastLevel    = digitalRead(IR_PIN);

  // Attach interrupt on CHANGE
  attachInterrupt(digitalPinToInterrupt(IR_PIN), IR_ISR, CHANGE);

  Serial.println("IR Receiver w/ Interrupts (Inverted Logic) Start");
}

void loop() {
  // Continuously parse any new pulses from the ring buffer
  parsePulses();

  // (You might add a small delay(1) here if needed,
  // but often it's fine to run continuously.)
}

// ==================================================
// Interrupt Service Routine
// Called on every RISING or FALLING edge of IR_PIN
// ==================================================
void IR_ISR() {
  unsigned long now = micros();
  bool currentLevel = digitalRead(IR_PIN);

  // Calculate how long since last edge
  unsigned long duration = now - lastEdgeTime;
  lastEdgeTime = now;

  // Store in ring buffer (if there's space)
  uint8_t nextIndex = (writeIndex + 1) % RING_BUFFER_SIZE;
  if (nextIndex != readIndex) { // not full
    pulseBuffer[writeIndex].duration = duration;
    pulseBuffer[writeIndex].level    = currentLevel;
    writeIndex = nextIndex;
  }

  lastLevel = currentLevel;
}

// ==================================================
// This function reads pulses out of the ring buffer
// and tries to decode them into your custom IR protocol
// ==================================================
void parsePulses() {
  static enum { WAIT_HEADER, WAIT_SPACE, READ_BITS } decodeState = WAIT_HEADER;
  static unsigned long bitsCollected = 0; // store partial bits
  static int bitCount = 0;               // how many bits read
  static bool gotHeader = false;

  // We'll collect 4 data bits + 1 parity => 5 bits total
  // (You can store them in a separate variable if you like)

while (readIndex != writeIndex) {
  PulseEvent ev;  
  noInterrupts();
  ev.duration = pulseBuffer[readIndex].duration;
  ev.level    = pulseBuffer[readIndex].level;
  readIndex = (readIndex + 1) % RING_BUFFER_SIZE;
  interrupts();

  // Use ev as before...
  unsigned long dur = ev.duration;
  bool lev = ev.level;

     // Basic state machine
    switch (decodeState) {

      // 1) Looking for header (2ms LOW, then ~1ms HIGH)
      case WAIT_HEADER:
        // If we see a LOW pulse ~2ms, that suggests the header mark
        // But we only know the pulse length after we've transitioned
        // from LOW->HIGH. So if 'lev == HIGH', that means the LOW ended.
        if (lev == HIGH) {
          // dur is how long we were LOW
          if (checkInRange(dur, HEADER_MARK, TOLERANCE)) {
            // Great, found header mark. Next we expect ~1ms HIGH
            decodeState = WAIT_SPACE;
          }
        }
        break;

      // 2) Wait for the header space (HIGH ~1ms)
      case WAIT_SPACE:
        // If 'lev == LOW', that means the HIGH ended.
        // dur is how long we were HIGH
        if (lev == LOW) {
          if (checkInRange(dur, HEADER_SPACE, TOLERANCE)) {
            // Good, we have a full header
            gotHeader = true;
            // Now we read bits
            bitsCollected = 0;
            bitCount = 0;
            decodeState = READ_BITS;
          } else {
            // Not a valid header, go back to WAIT_HEADER
            decodeState = WAIT_HEADER;
          }
        }
        break;

      // 3) Reading bits (4 data bits + 1 parity = 5 total)
      case READ_BITS:
        // Each bit is a LOW pulse (~500µs or ~1000µs) + ~500µs HIGH space
        // We see that LOW ends when lev == HIGH. So 'dur' is how long LOW was.
        // Then the next edge (HIGH->LOW) will give us the space length.
        // We can store partial info in static variables.

        // If we just ended a LOW (lev == HIGH), 'dur' is the "mark" length
        if (lev == HIGH) {
          // Distinguish 1 vs. 0
          bool isBit1 = checkInRange(dur, BIT_MARK_1, TOLERANCE);
          bool isBit0 = checkInRange(dur, BIT_MARK_0, TOLERANCE);

          if (!isBit1 && !isBit0) {
            // Not a valid bit. Reset
            decodeState = WAIT_HEADER;
            break;
          }

          // SHIFT bitsCollected left 1 and add the new bit
          bitsCollected <<= 1;
          if (isBit1) {
            bitsCollected |= 1;
          }
          bitCount++;
        }
        // If we just ended a HIGH (lev == LOW), 'dur' is the "space" length
        // Typically ~500µs. We can check it if we want:
        else { // lev == LOW
          if (!checkInRange(dur, BIT_SPACE, TOLERANCE)) {
            // Invalid space => reset
            decodeState = WAIT_HEADER;
            break;
          }
        }

        // Check if we got all 5 bits (4 data + 1 parity)
        if (bitCount == 5) {
          // Done reading one packet
          decodeState = WAIT_HEADER; // get ready for next packet

          // Extract the lower 4 bits (the actual data)
          byte data = (bitsCollected >> 1) & 0x0F; // top 4 bits
          bool parityBit = (bitsCollected & 0x01);

          // In your code, parity=0 always, so we won't check it
          // Identify 3,6,9,12
          switch(data) {
            case 3:  seenA = true; break;
            case 6:  seenB = true; break;
            case 9:  seenC = true; break;
            case 12: seenD = true; break;
            default:
              // ignore unknown
              break;
          }

          // You could also print immediately here if you want
          // For demonstration, let's just keep the same approach:
        }
        break;
    } // end switch
  } // end while (readIndex != writeIndex)

  // If you want to display direction once per cycle, do so here
  // for example, if your beacon is sending continuous packets:
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    // interpret direction
    String dir = interpretDirection(seenA, seenB, seenC, seenD);
    // Print
    Serial.print("Seen: ");
    if (seenA) Serial.print("A ");
    if (seenB) Serial.print("B ");
    if (seenC) Serial.print("C ");
    if (seenD) Serial.print("D ");
    Serial.print(" => ");
    Serial.println(dir);

    // reset flags for next read
    seenA = seenB = seenC = seenD = false;
    lastPrint = millis();
  }
}

// -----------------------------------------------------------
// Check if 'val' is within +/- tol of 'target'
// -----------------------------------------------------------
bool checkInRange(unsigned long val, unsigned long target, unsigned long tol) {
  return (val >= (target - tol)) && (val <= (target + tol));
}

// -----------------------------------------------------------
// Decide which direction/correction to output based on
// whether we saw A, B, C, D
// (Same as your prior logic; tweak as needed.)
// -----------------------------------------------------------
String interpretDirection(bool A, bool B, bool C, bool D) {
  int count = (A?1:0) + (B?1:0) + (C?1:0) + (D?1:0);

  // Single signals
  if (count == 1) {
    if (A) return "BACK-LEFT";
    if (B) return "FORWARD-LEFT";
    if (C) return "FORWARD-RIGHT";
    if (D) return "BACK-RIGHT";
  }
  // Two-signal overlaps
  if (count == 2) {
    if (A && B) return "LEFT";
    if (B && C) return "FORWARD";
    if (C && D) return "RIGHT";
    if (D && A) return "BACK";
    if (B && D) return "CENTER B-D";
    if (A && C) return "Center A-C";
  }
  // Three-signal overlap
  if (count == 3) {
    if (A && B && D) return "SLIGHT BACK-LEFT";
    if (A && B && C) return "SLIGHT FORWARD-LEFT";
    if (B && C && D) return "SLIGHT FORWARD-RIGHT";
    if (A && C && D) return "SLIGHT BACK-RIGHT";
  }
  // All four
  if (count == 4) {
    return "CENTER";
  }
  // None
  return "NO SIGNAL";
}
