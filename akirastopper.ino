#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <Servo.h>

#define SERVO_PIN 2  // PB2

#define PB5_PIN 0    // PB5 = physical pin 1 (Arduino pin 0)
#define PB4_PIN 4    // PB4 = physical pin 3 (Arduino pin 4)
#define PB3_PIN 3    // PB3 = physical pin 2 (Arduino pin 3)

// volatile bool activityDetected = false;
Servo myServo;

// unsigned long lastActivityTime = 0;
// const unsigned long INACTIVITY_TIMEOUT = 5UL * 60UL * 1000UL; // 5 minutes in milliseconds

void setup() {
  // Setup servo
  myServo.attach(SERVO_PIN);

  // Setup pins
  pinMode(PB5_PIN, INPUT);
  pinMode(PB4_PIN, INPUT);
  pinMode(PB3_PIN, INPUT);

  // Enable pin change interrupts
  GIMSK |= (1 << PCIE);  // Enable Pin Change Interrupts
  PCMSK |= (1 << PB5_PIN) | (1 << PB4_PIN) | (1 << PB3_PIN); // Enable interrupts on PB5, PB4, PB3

  sei(); // Enable global interrupts

  // lastActivityTime = millis();
}

void loop() {
  // if (activityDetected) {
  //   lastActivityTime = millis();
  //   activityDetected = false;
  // }

  // // Check for inactivity
  // if ((millis() - lastActivityTime) > INACTIVITY_TIMEOUT) {
  //   goToSleep();
  // }

myServo.write(0);  // Move servo to 0° (neutral position)
  delay(1000);  // Wait for 5 seconds

  myServo.write(90);  // Move servo to +90°
  delay(1000);  // Wait for 5 seconds

  myServo.write(180);  // Move servo to -90° (if your servo supports negative degrees, otherwise use 270)
  delay(1000);  // Wait for 5 seconds

  myServo.write(90);  // Move servo to -90° (if your servo supports negative degrees, otherwise use 270)
  delay(1000);  // Wait for 5 seconds
}

void goToSleep() {
  myServo.detach();  // Turn off servo to save power
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable(); // Disable Brown-Out Detection during sleep for more power savings

  sleep_cpu();  // Go to sleep here
  
  sleep_disable(); // Continue running after wake up
  myServo.attach(SERVO_PIN); // Re-attach servo
}

// Pin Change Interrupt Service Routine
ISR(PCINT0_vect) {
  // activityDetected = true;

  // Check which pin caused the interrupt
  if (PINB & (1 << PB5_PIN)) {
    // PB5 rising edge
    myServo.write(0); // Move to -90 deg (servo "0" corresponds to leftmost typically)
  } 
  else if (PINB & (1 << PB3_PIN)) {
    // PB3 rising edge
    myServo.write(180); // Move to +90 deg (servo "180" corresponds to rightmost)
  }
  // PB4 interrupt can be used for wake-up only, no action needed
}

