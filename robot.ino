#include <LiquidCrystal_I2C.h>

#include <ezButton.h>
#include <Arduino.h>
#include <avr/io.h>
#include "Hardware/multiplexor/multiplexor.h"
#include "Hardware/stepper/distance.h"
#include <Servo.h>

#define TIMEOUT (2000)

typedef enum {
  RS_CALIBRATION,
  RS_MEASURE,
  RS_COMPUTE_X_Y,
  RS_GOTO_X_Y,
  RS_DROP
} ResistorSorterState_t;

ezButton limitSwitch(8);  // create ezButton object that attach to pin 7;
ezButton limitSwitchEnd(9);  // create ezButton object that attach to pin 8;

unsigned int stateMachine;
ResistorSorterState_t state = RS_CALIBRATION;
volatile int currentStepX, currentStepY = 0;
int setpointX, setpointY = 0;

// DOUBLE AXE SETUP
const int stepPin = 3;
const int dirPin = 2;
const int stepsPerRevolution = 200;
const float stepDelay = 1000.0; // microseconds
volatile int stepCount = 0;
volatile bool isHigh = false;



uint32_t stop_time_ms = millis();
int stop_time = 0;
int stop_time_two = 0;

volatile bool stepperEnabled = false;


// ROBOT MESURE SETUP 
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

Servo servo;
const int buttonPin = 44; 
const int servoPin = 53;
int buttonState = 0;
float res= 0.0;


void setup() {


    // DOUBLE AXE SETUP
    Serial.begin(9600);
    limitSwitch.setDebounceTime(10); // set debounce time to 10 microseconds
    limitSwitchEnd.setDebounceTime(10); // set debounce time to 10 microseconds

    cli(); // disable interrupts
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    OCR2A = (int)(16 * stepDelay - 1);
    TCCR2A |= (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS22);
    TIMSK2 |= (1 << OCIE2A);
    sei(); // enable interrupts

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    stateMachine = 4;
    state = RS_CALIBRATION;


    // ROBOT MESURE SETUP
    initialize_mux();
    servo.attach(servoPin);
    servo.write(45);

    lcd.init(); //initialize the lcd
    lcd.backlight(); //open the backlight 
    Serial.println("Place your resistance");
    Serial.println("");
    pinMode(buttonPin, INPUT);
    display_init();

}

void loop() {
  for (;;) {
    limitSwitch.loop(); // MUST call the loop() function first
    limitSwitchEnd.loop();

    stateMachineSequencer();
  }
  limitSwitch.loop(); // MUST call the loop() function first
  limitSwitchEnd.loop(); 


  switch (stateMachine)
  {
  case 0:
  {
    digitalWrite(dirPin, HIGH);
    stepperEnabled = true;
    stepCount = 0;
    // Serial.println("STATE ZERO");
    if(limitSwitch.isPressed()){
        Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
        stateMachine = 1;
        stop_time = 0;
        stop_time_ms = millis();
    }

    break;
  }
  case 1:
  {
    stepperEnabled = false;
    // Serial.println("STATE ONE");
    int switch_state = limitSwitch.getState();
    if(switch_state == HIGH){
        // Serial.println("The limit switch: UNTOUCHED");
        stop_time = 0;
        stop_time_ms = millis();
        stateMachine = 0;
    }
    else{
        // Serial.println("The limit switch: TOUCHED");
        stateMachine = 1;

        if((millis() - stop_time_ms) > TIMEOUT){
          stateMachine = 2;
          stop_time = 0;
          stop_time_ms = millis();
          Serial.println("TIMEOUT PASSED");
        } else {
          stop_time++;
        }
    }

    break;
  }

  case 2:
  {

    // Serial.println("STATE TWO");
    digitalWrite(dirPin, LOW);
    stepCount = 0;
    stepperEnabled = true;
    stateMachine = 2;

    if(limitSwitchEnd.isPressed()){
      // Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
      stateMachine = 3;
    }

    break;
  }

  case 3:
  {
    stepperEnabled = false;
    // Serial.println("STATE THREE");
    int switch_end_state = limitSwitchEnd.getState();
    if(switch_end_state == HIGH){
        // Serial.println("The limit switch: UNTOUCHED");
        stop_time_two = 0;
        stateMachine = 2;
    }
    else{
        // Serial.println("The limit switch: TOUCHED");
        stateMachine = 3;
        if(stop_time_two > 50){
          stateMachine = 0;
          stop_time_two = 0;
        } else {
          stop_time_two++;
        }
    }

    break;

  }
  case 4:
  {
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH){
        prepare_measure();
        // delay(2000);
        res = auto_calibrate();
        display_infos(res);
        drop_resistance();
        // delay(2000);
        stateMachine = 0;
    }

    break;

  }
  
  default:
    break;
  }
}

void stateMachineSequencer(void) {
  switch(state) {
    case RS_CALIBRATION: {
      stepperEnabled = true;
      digitalWrite(dirPin, HIGH);

      if (limitSwitch.isPressed()) {
        stepperEnabled = false;
        currentStepX = 0;
        currentStepY = 0;
        state = RS_MEASURE;
      }

      break;
    } case RS_MEASURE: {
      int buttonState = digitalRead(buttonPin);

      if (buttonState == HIGH){
          prepare_measure();
          // delay(2000);
          res = auto_calibrate();
          display_infos(res);
          drop_resistance();
          // delay(2000);
          state = RS_COMPUTE_X_Y;
      }

      break;
    } case RS_COMPUTE_X_Y: {
      // Calcul X Y
      digitalWrite(dirPin, LOW);
      setpointX = Distance::convert_distance_into_steps(30);
      Serial.println("THIS IS SETPOINTX");
      Serial.println(setpointX);
      setpointY = Distance::convert_distance_into_steps(30);
      Serial.println("THIS IS SETPOINTY");
      Serial.println(setpointY);
      state = RS_GOTO_X_Y;
      stepperEnabled = true;
      break;
    } case RS_GOTO_X_Y: {
      // Serial.println("THIS IS CURRENTSTEPX");
      // Serial.println(currentStepX);
      // Serial.println("THIS IS CURRENTSTEPY");
      // Serial.println(currentStepY);
      if ((currentStepX >= setpointX) && (currentStepY >= setpointY)) {
        state = RS_DROP;
        stepperEnabled = false;
      }
      break;
    } case RS_DROP: {
      state = RS_CALIBRATION;
      delay(2000);
      break;
    } default:
      state = RS_CALIBRATION;
  }
}


ISR(TIMER2_COMPA_vect) {
  if (/*stepCount < stepsPerRevolution && */ stepperEnabled) {
    if (isHigh) {
      digitalWrite(stepPin, LOW);
      isHigh = false;
    } else {
      digitalWrite(stepPin, HIGH);
      isHigh = true;
      currentStepX++;
      currentStepY++;
      stepCount++;
    }
  }
}


void measure_sequence(){
    prepare_measure();
    delay(2000);
    res = auto_calibrate();
    display_infos(res);
    drop_resistance();
    delay(2000);
    display_init();
}


void display_init(){
    lcd.clear();                 // clear display
    lcd.setCursor(0, 0);         // move cursor to   (0, 0)
    lcd.print("Place Resistor"); 
}

void display_infos(float res){
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(res);
    lcd.setCursor(0,2);
    lcd.print("ohms");
    Serial.println(res);
}
void prepare_measure(){
    servo.write(90);
}

void drop_resistance(){
    // servo.write(45);
    // delay(500);
    servo.write(0);
    delay(1000);
    servo.write(45);
}
