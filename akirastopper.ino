#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <Servo.h>

#define SERVO_PIN 0  // PB2

#define PB4_PIN 3  // PB4 = physical pin 3 (Arduino pin 4)
#define PB3_PIN 2  // PB3 = physical pin 2 (Arduino pin 3)

// volatile bool activityDetected = false;
volatile bool turnToLeft = false;
volatile bool turnToRight = false;

volatile uint8_t lastPINB;

Servo myServo;

// unsigned long lastActivityTime = 0;
// const unsigned long INACTIVITY_TIMEOUT = 5UL * 60UL * 1000UL; // 5 minutes in milliseconds

void setup() {
  cli();  //disable interrupts during setup


  // Setup pins
  pinMode(PB4_PIN, INPUT);  // Set PB4 (pin 3) as input with pull-up
  pinMode(PB3_PIN, INPUT);  // Set PB3 (pin 2) as input with pull-up

  pinMode(SERVO_PIN, OUTPUT);

  lastPINB = PINB;

  // Enable pin change interrupts for PB3 and PB4
  GIMSK |= (1 << PCIE);                    // Enable Pin Change Interrupts
  PCMSK |= (1 << PCINT3) | (1 << PCINT4);  // Enable interrupts for PB3 (PCINT3) and PB4 (PCINT4)

  sei();  // Enable global interrupts
}

void loop() {

  if (turnToLeft) {
    myServo.attach(SERVO_PIN);
    myServo.write(1);  // Move servo to 0° (neutral position)
    delay(1000);       // Wait for 1 second
    myServo.detach();
    turnToLeft = false;
  } else if (turnToRight) {
    myServo.attach(SERVO_PIN);
    myServo.write(150);  // Move servo to desired position
    delay(1000);         // Wait for 1 second
    myServo.detach();
    turnToRight = false;
  }
}

ISR(PCINT0_vect) {
  uint8_t changedPins = lastPINB ^ PINB; // XOR to find changed pins
  lastPINB = PINB; // Update last known state

  if (changedPins & (1 << PB3)) {
    // PB3 changed
    if (!(PINB & (1 << PB3))) {
      // PB3 went LOW (button press assumed)
      turnToLeft = true;          // Set turnToLeft to true
    }
  } else if (changedPins & (1 << PB4)) {
    // PB4 changed
    if (!(PINB & (1 << PB4))) {
      // PB4 went LOW
      turnToRight = true;         // Set turnToRight to true
    }
  }
}

