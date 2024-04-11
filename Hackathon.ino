#include <ezButton.h>


// pin definitions for the motorshield
#define BRAKE_B 8 // 9 for output A
#define BRAKE_A 9 // 9 for output A
#define PWM_A 3  // 3 for output A
#define PWM_B 11  // 3 for output A
#define DIR_A 12  // 12 for output A
#define DIR_B 13  // 12 for output A

// and for the encoder:
#define ENC_A_A 2 // pin connected to external interrupt
#define ENC_A_B 4 // pin connected to normal pin
#define ENC_B_A 5 // pin for PCINT because pin 3 is in use
#define ENC_B_B 6 // pin connected to normal pin

// for the current:
#define CURRENT_A A0
#define CURRENT_B A1

// sensor
#define echoPinBL A3 //Back Left US sensor
#define echoPinBR A4 //BR is the front US sensor
#define trigPinBL 10
#define trigPinBR A5

//microswitch
ezButton left_switch(7);
ezButton right_switch(A2);

// variables
float position_A, setpoint_A, error_A, velocity_A; // controller variables as globals
float position_B, setpoint_B, error_B, velocity_B; // controller variables as globals
// variables for PID control
float previousError_A, errorRate_A, sumError_A;
float previousError_B, errorRate_B, sumError_B;

// timing:
unsigned long loopTime, printTime; // timing
volatile long encodercount_A; // encoder: global accessed from Interrupt: volatile
volatile long encodercount_B; // encoder: global accessed from Interrupt: volatile
int mode; // mode 0: direct PWM, mode 1: P control
volatile unsigned long lasttime_A, transitiontime_A;
volatile unsigned long lasttime_B, transitiontime_B;

//Control Variable
float Kd = 0.7;
float Kp = 0.035;
float D = 0.0001;
float J = 4 ;
float Km = 0.035;
float pi = 3.14;
float Vin = 12;
float fh = 155;
int torque_A;
int torque_B;
float derivError_A;
float derivError_B;
float R = 104.34;

//sensor variable
long durationBL;
long durationBR;
long durationFL;
long durationFR;

float distanceBL;
float distanceBR;
float distanceFL;

int n;

void setup() {
  Serial.begin(115200);
  pinMode(echoPinBL, INPUT);
  pinMode(echoPinBR, INPUT);
  pinMode(trigPinBL, OUTPUT);
  pinMode(trigPinBR, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  left_switch.setDebounceTime(50);
  right_switch.setDebounceTime(50);

  attachInterrupt(0, encoder_A, RISING); // std. external interrupt
  attachPinChangeInterrupt(); // different style interrupt for encoder B
  setpoint_B = 0; // remove this: just for testing!!
  setpoint_A = -setpoint_B;
  //  mode = 1; // also, your choice,
}
void loop() {
  if (millis() > loopTime + 1) { // 20 Hz output
    loopTime = millis();      // reset looptimer

    //microswitch setup
    left_switch.loop();
    right_switch.loop();
    
    sensor();
    // sample point
    // this bit for measuring velocity through pulse transition time:
    if (transitiontime_B > 0) velocity_B = 2094.40 / transitiontime_B; // in [rad/sec] so 2*pi / (30*100ppr*transitiontime)/1000.000[s] = 6283000/(3000*transitiontime) = 2094.40/transitiontime
    else velocity_B = 0;
    if (transitiontime_A > 0) velocity_A = 2094.40 / transitiontime_A; // in [rad/sec] so 2*pi / (30*100ppr*transitiontime)/1000.000[s] = 6283000/(3000*transitiontime) = 2094.40/transitiontime
    else velocity_A = 0;
    // wheel position in degrees
    position_B = (360 * (float) encodercount_B) / (100 * 30); // degrees: 100 PPR, 1:30 gear, 360 deg/rev. in [rad] it would be 2*pi*(float)encodercount/(100*30)
    position_A = (360 * (float) encodercount_A) / (100 * 30); // degrees: 100 PPR, 1:30 gear, 360 deg/rev. in [rad] it would be 2*pi*(float)encodercount/(100*30)
    // control
    // simple feedback control -- This is where the control magic happens:
    error_B = setpoint_B - position_B;  // error signal
    error_A = setpoint_A - position_A;  // error signal
    derivError_B = (error_B - previousError_B) * fh;
    derivError_A = (error_A - previousError_A) * fh;
    torque_B = (Kp * error_B) + (Kd * derivError_B);
    torque_A = (Kp * error_A) + (Kd * derivError_A);
    float pwm_motor_A = (torque_A / Km) * R * 255 / Vin;
    float pwm_motor_B = (torque_B / Km) * R * 255 / Vin;

    sumError_A = error_A + previousError_A;
    previousError_A = error_A;
    sumError_B = error_B + previousError_B;

    //turning function
    turn(distanceBR, distanceBL, pwm_motor_A, pwm_motor_B);

    //switches code

    Serial.println(distanceBL);

    int right_state = right_switch.getState();
    int left_state = left_switch.getState();
    

    if(left_state == LOW) { //turn right
      digitalWrite (DIR_A, HIGH);
      digitalWrite (DIR_B, LOW);
      analogWrite(PWM_A, 255);
      analogWrite(PWM_B, 255);
    }

    //input from the other arduino with barcode
    string end = Serial.read();
    if (end == "finish") {
      digitalWrite(DIR_A, HIGH);
      digitalWrite(DIR_B, HIGH);
      analogWrite(PWM_A, 0);
      analogWrite(PWM_B, 0);
      exit(0);
    }
  }
}

void turn(float front, float back_left, float speed_signal_A, float speed_signal_B) {

  if (front > 30.0)
    front = 30.0;
  if (back_left > 30.0)
    back_left = 30.0;
  
  if (front > 15.0 && back_left < 15.0) { // FORWARD
    digitalWrite(DIR_A, HIGH);
    digitalWrite(DIR_B, HIGH);
//    analogWrite(PWM_A, min(abs (speed_signal_A), 255));
//    analogWrite(PWM_B, min(abs (speed_signal_B), 255));
    analogWrite(PWM_A, 200);
    analogWrite(PWM_B, 200);
  }
  
  else if (back_left > 15.0 && front > 15.0) { //TURN LEFT
    digitalWrite (DIR_B, HIGH);
    digitalWrite (DIR_A, HIGH);
    analogWrite(PWM_B, 200);
    analogWrite(PWM_A, 95);
  }

  else if (front < 15.0) { //turn right if there is an obstacle (wall) in the front
    digitalWrite (DIR_A, HIGH);
    digitalWrite (DIR_B, LOW);
    analogWrite(PWM_A, 200);
    analogWrite(PWM_B, 157);
  }

  else {
    digitalWrite(DIR_A, HIGH);
    digitalWrite(DIR_B, HIGH);
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    //    delay(1500);
  }
}



////// ===== DON'T start messing with the code below this line ====////
// encoder interrupt routine
void encoder_A() {
  transitiontime_A = micros() - lasttime_A; // this bit for sensing pulse duration
  if (digitalRead(ENC_A_B))
    encodercount_A++;                   // counting the pulses (position)
  else
    encodercount_A--;
  lasttime_A = micros();
}

void encoder_B() {
  transitiontime_B = micros() - lasttime_B; // this bit for sensing pulse duration
  if (digitalRead(ENC_B_B))
    encodercount_B++;                   // counting the pulses (position)
  else
    encodercount_B--;
  lasttime_B = micros();
}


void sensor() {
  digitalWrite(trigPinBL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBL, LOW);
  durationBL = pulseIn(echoPinBL, HIGH);
  distanceBL = durationBL * 0.034 / 2;

//  digitalWrite(trigPinFL, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPinFL, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPinFL, LOW);
//  durationFL = pulseIn(echoPinFL, HIGH);
//  distanceFL = durationFL * 0.034 / 2;

  digitalWrite(trigPinBR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBR, LOW);
  durationBR = pulseIn(echoPinBR, HIGH);
  distanceBR = durationBR * 0.034 / 2;

}



// ========= voodoo for PCinterrupt on encoder B ===================
// please don't change this code, it is hard to make it work
// on a different pin, so keep it if you want to use pin 5 to
// interface encoder B
#define PCINT_PIN 5
#define PCINT_MODE RISING
#define PCINT_FUNCTION encoder_B

volatile uint8_t oldPort = 0x00;

#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCIE  digitalPinToPCICRbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))

#if (PCIE == 0)
#define PCINT_vect PCINT0_vect
#elif (PCIE == 1)
#define PCINT_vect PCINT1_vect
#elif (PCIE == 2)
#define PCINT_vect PCINT2_vect
#else
#error This board doesnt support PCINT ?
#endif


void attachPinChangeInterrupt(void) {
  // update the old state to the actual state
  oldPort = PCPIN;

  // pin change mask registers decide which pins are enabled as triggers
  PCMSK |= (1 << PCINT);

  // PCICR: Pin Change Interrupt Control Register - enables interrupt vectors
  PCICR |= (1 << PCIE);
}


ISR(PCINT_vect) {
  // get the new and old pin states for port
  uint8_t newPort = PCPIN;

  // compare with the old value to detect a rising or falling
  uint8_t change = newPort ^ oldPort;

  // check which pins are triggered, compared with the settings
  uint8_t trigger = 0x00;
#if (PCINT_MODE == RISING) || (PCINT_MODE == CHANGE)
  uint8_t rising = change & newPort;
  trigger |= (rising & (1 << PCINT));
#endif
#if (PCINT_MODE == FALLING) || (PCINT_MODE == CHANGE)
  uint8_t falling = change & oldPort;
  trigger |= (falling & (1 << PCINT));
#endif

  // save the new state for next comparison
  oldPort = newPort;

  // if our needed pin has changed, call the IRL interrupt function
  if (trigger & (1 << PCINT))
    PCINT_FUNCTION();
}
