#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <driver/gpio.h>  // Include this header for GPIO_PULLUP_ONLY

// Solenoid and button
#define SOLENOID_1 13
#define BUTTON_1 12

// Motor driver and encoder
#define BIN_1 26
#define BIN_2 25
#define ENCODER_PIN_A 14
#define ENCODER_PIN_B 32

// Motor control
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;

// Solenoid and scoring
volatile bool buttonPressed = false;
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;
volatile bool solenoidState = LOW;
int score = 0;

// ESP32Encoder object
ESP32Encoder encoder;
volatile int encoderCount = 0;
volatile bool deltaT = false;

//PID values [change me]
float Kp = 1.0; // tune me first
float Ki = 1.0; // tune me last
float Kd = 1.0; // tune me next

float integral = 0.0; // start integral at zero
float previous_error = 0.0; // start prev_erorr at zero
float derivative = 0.0; //start derivative at zero
float output = 0.0;

//state variable
state = 1.0; 

// current speed
float currentSpeed = 0.0;
float desiredSpeed = 10.0;  ////////////////////////////////// change me ////////////////////////

// Interrupt variables
hw_timer_t* timer1 = NULL; // Timer for encoder reading
hw_timer_t* solenoidTimer = NULL; // Timer to trigger solenoid every 5 seconds
hw_timer_t* solenoidOffTimer = NULL; // Timer to disengage solenoid after 2 seconds if button not pressed
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux3 = portMUX_INITIALIZER_UNLOCKED;

int omegaSpeed = 0;

// Forward declaration of ISR
void IRAM_ATTR ISR_button_pressed();

//interrupt function for encoder, gets encoder position at freq specified later
void IRAM_ATTR onEncoderTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  encoderCount = encoder.getCount();
  encoder.clearCount();
  deltaT = true;
  portEXIT_CRITICAL_ISR(&timerMux1);
}

// turn on solenoid 
void IRAM_ATTR onSolenoidTimer() {
  // Engage the solenoid
  portENTER_CRITICAL_ISR(&timerMux2);
  digitalWrite(SOLENOID_1, HIGH);
  solenoidState = HIGH;
  // Start the solenoidOffTimer
  timerRestart(solenoidOffTimer);
  timerAlarmEnable(solenoidOffTimer); // Enable the 2-second timer
  portEXIT_CRITICAL_ISR(&timerMux2);
}

// turn off solenoid
void IRAM_ATTR onSolenoidOffTimer() {
  // Disengage the solenoid if button has not been pressed
  portENTER_CRITICAL_ISR(&timerMux3);
  if (solenoidState == HIGH && !buttonPressed) {
    digitalWrite(SOLENOID_1, LOW);
    solenoidState = LOW;
  }
  // Disable the solenoidOffTimer until next solenoid activation
  timerAlarmDisable(solenoidOffTimer);
  portEXIT_CRITICAL_ISR(&timerMux3);
}

void setup() {
  Serial.begin(9600);

  pinMode(SOLENOID_1, OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON_1), ISR_button_pressed, RISING);

  // Motor control setup using ledcSetup and ledcAttachPin
  ledcSetup(ledChannel_1, freq, resolution);
  ledcAttachPin(BIN_1, ledChannel_1);

  ledcSetup(ledChannel_2, freq, resolution);
  ledcAttachPin(BIN_2, ledChannel_2);

  // ESP32Encoder::useInternalWeakPullResistors = GPIO_PULLUP_ONLY;  // Enable the weak pull-up resistors
  encoder.attachHalfQuad(ENCODER_PIN_A, ENCODER_PIN_B); // Attach encoder pins
  encoder.setCount(0); // Set starting count value

  // Timer 1: Encoder reading every 10 ms
  timer1 = timerBegin(0, 80, true);            // Timer 0, prescaler 80 (1 us per tick)
  timerAttachInterrupt(timer1, &onEncoder1, true);  // Attach onTime1 function to the timer
  timerAlarmWrite(timer1, 10000, true);      // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer1);

  // Timer 2: Solenoid activation every 5 seconds
  solenoidTimer = timerBegin(1, 80, true); // Timer 1
  timerAttachInterrupt(solenoidTimer, &onSolenoidTimer, true);
  timerAlarmWrite(solenoidTimer, 5000000, true); // 5,000,000 * 1 us = 5 seconds
  timerAlarmEnable(solenoidTimer);

  // Timer 3: Solenoid disengage after 2 seconds
  solenoidOffTimer = timerBegin(2, 80, true); // Timer 2
  timerAttachInterrupt(solenoidOffTimer, &onSolenoidOffTimer, true);
  timerAlarmWrite(solenoidOffTimer, 2000000, false); // 2,000,000 * 1 us = 2 seconds, auto-reload false
  // Do not enable yet; it will be enabled when solenoid is activated
}

void loop() {
  // Handle button press
  if (buttonPressed) {
    buttonPressed = false;
    if (solenoidState == HIGH) {
      score++;
      Serial.print("Score: ");
      Serial.println(score);
    }
    digitalWrite(SOLENOID_1, LOW);
    solenoidState = LOW;
    // Disable the solenoidOffTimer since button was pressed
    timerAlarmDisable(solenoidOffTimer);
  }

  switch (state){
      case(buttonPressed){
        state = 2; 
        // set motor speed using output ///// this is only using a single direction. 
        ledcWrite(ledChannel_1, output); //start spinning
        //display score
      }

      case(gameOver){
        state = 1; 
        ledcWrite(ledChannel_1, 0); //stop spinning
        //display start stuff
      }

  }


  // encoder speed and PID
  if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);
      currentSpeed = encoderCount / 0.010; // added divide by speed
      
      // PID stuff 
      // first calculate error, integral derivative
      error = desiredSpeed - currentSpeed; 
      integral += error; 
      derivative = error - previousError; 

      // next, use PID equation, constrain it, and
      output = Kp * error + Ki * integral + Kd * derivative;
      output = constrain(output, 0, 255);

    // debugs
    Serial.print("Speed:");
    Serial.println(currentSpeed);
  }

}

// button pressed interrupt
void IRAM_ATTR ISR_button_pressed() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = currentTime;
    buttonPressed = true;
  }
}
