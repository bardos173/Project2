#include <Arduino.h>

// ---------------- Original variables (built-in encoder + PI controller) ----------------
int b = 0;            // time for main loop (15s)
int c = 0;            // memory for time in main loop

float s = 0;          // built-in encoder counts (5s display)
float s_2 = 0;        // built-in encoder counts (PI controller)
float rpmm = 0;       // rpm from built-in encoder

int s1 = 0, s2 = 0;   // built-in encoder channels
int r = 0;
int s2m = 0;
int directionm = 0;
int dirm;
int RPM;              // commanded RPM

int exitt = 0;

float ctrl;
float kp = .4;
float ki = .01;
float eri = 0;

int repc = 1;
int t0;
int repeat = 0;

// ---------------- Our optical quadrature encoder ----------------
// Wire channel A to pin 8 (PB0) and channel B to pin 9 (PB1). Change here if different.
#define A_BIT 1   // bit in PINB for signal A (pin 9)
#define B_BIT 0   // bit in PINB for signal B (pin 8)


#define OUR_COUNTS_PER_REV 32.0  

volatile int signalA = 0;
volatile int signalB = 0;
volatile uint8_t prevState = 0;
volatile long totalcount = 0;           // total signed count
volatile long count = 0;     // signed count, reset every 5s for RPM
volatile int8_t lastStep = 0;      // +1 = clockwise, -1 = counterclockwise

long prevcount = 0;          // historical count value
int ourDirection = 0;              // 0 = clockwise, 1 = counterclockwise

// Transition table, index = (prevState << 2) | currentState, state = (A << 1) | B
// CW:  0->2, 2->3, 3->1, 1->0  (+1)     CCW: 0->1, 1->3, 3->2, 2->0  (-1)
const int8_t stepTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void setup() {
  Serial.begin(250000);
  Serial.println("Enter the desired RPM.");

  while (Serial.available() == 0) { }

  RPM = Serial.readString().toFloat();
  if (RPM < 0) {
    analogWrite(3, 255);   // change motor direction
  }
  RPM = abs(RPM);

  // ------------------ Set up our encoder interrupts ------------------
  cli();

  pinMode(8, INPUT);
  pinMode(9, INPUT);

  PCICR  |= (1 << PCIE0);                       // enable Port B pin-change interrupts
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);      // only pins 8 and 9 (A and B)

  // initialise state from the actual pin levels
  uint8_t pb = PINB;
  signalA = (pb >> A_BIT) & 1;
  signalB = (pb >> B_BIT) & 1;
  prevState = (signalA << 1) | signalB;

  sei();
}

void loop() {
  b = millis();
  c = b;

  while ((b >= c) && (b <= (c + 15500)) && exitt == 0) {

    // ---------------- PI controller ----------------
    if (b % 13 == 0 && repc == 1) {
      eri = ki * (RPM - rpmm) + eri;
      ctrl = 50 + kp * (RPM - rpmm) + eri;
      analogWrite(6, ctrl);
      repc = 0;
    }
    if (b % 13 == 1) {
      repc = 1;
    }

    // ---------------- Built-in encoder ----------------
    s1 = digitalRead(7);
    s2 = digitalRead(5);

    if (s1 != s2 && r == 0) {
      s = s + 1;
      s_2 = s_2 + 1;
      r = 1;
    }
    if (s1 == s2 && r == 1) {
      s = s + 1;
      s_2 = s_2 + 1;
      r = 0;
    }

    b = millis();
    if (b % 100 <= 1 && repeat == 0) {
      t0 = b;
      repeat = 1;
    }

    if (b % 100 == 0) {
      rpmm = (s_2 / (2 * 114)) * 600;   // rpm each 100ms for PI controller
      s_2 = 0;

      // Direction from change in count since last check
      if ((count) > prevcount) ourDirection = 1;
      else if (count < prevcount) ourDirection = 0;

      // update the historical count with the current value
      prevcount = count;

      if ((b - t0) % 5000 == 0) {
        Serial.println();
        Serial.print("RPM from builtin encoder: ");
        Serial.println((s / 228) * 12);

        // Our values
        Serial.print("RPM from optical quadrature encoder: ");
        float ourRPM = (count / OUR_COUNTS_PER_REV) * 12;   // 5s window -> *12
        Serial.println(ourRPM);

        Serial.print("Error: ");
        Serial.println((s / 228) * 12 - abs(ourRPM));          // builtin minus ours

        Serial.print("direction read by motor's sensor: ");
        Serial.print(dirm == 0 ? "CW" : "CCW");
        Serial.print("  ,   ");

        Serial.print("direction read by sensor:  ");
        Serial.println(ourDirection == 1 ? "CW" : "CCW");

        s = 0;
        directionm = 0;
        cli();
        countWindow = 0;
        sei();
        prevCountWindow = 0;
      }
      delay(1);
    }

    // ---------------- Built-in encoder direction ----------------
    if ((s1 == HIGH) && (s2 == HIGH) && (s2m == LOW)) directionm++;
    if ((s1 == LOW) && (s2 == LOW) && (s2m == HIGH)) directionm++;
    s2m = s2;

    if (directionm > 100) dirm = 0;
    if (directionm < 20)  dirm = 1;

    b = millis();
  }

  analogWrite(6, 0);   // turn off motor
  exitt = 1;
}

// ---------------- ISR: read signals A and B and decodes ----------------
ISR(PCINT0_vect) {
  uint8_t pb = PINB;

  signalA = (pb >> A_BIT) & 1; // check if PINB bit at A is 1
  signalB = (pb >> B_BIT) & 1; // check if PINB bit at B is 1

  uint8_t currentState = (signalA << 1) | signalB; // current state is AB (e.g. 01)
   // step value is either 0, 1 or -1 depending on the difference between previous and current count
  int8_t step = stepTable[(prevState << 2) | currentState]; 

  count += step; // add the step to the count

  prevState = currentState; // store previous state for reference to check direction
}
