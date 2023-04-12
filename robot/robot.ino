#include <ezButton.h>
#include <Arduino.h>
#include <avr/io.h>

ezButton limitSwitch(7);  // create ezButton object that attach to pin 7;
ezButton limitSwitchEnd(8);  // create ezButton object that attach to pin 8;

unsigned int stateMachine;


const int stepPin = 3;
const int dirPin = 2;
const int stepsPerRevolution = 200;
const float stepDelay = 500.0; // microseconds
volatile int stepCount = 0;
volatile bool isHigh = false;

int stop_time = 0;
int stop_time_two = 0;

volatile bool stepperEnabled = false;

void setup() {
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


  stateMachine = 0;
}

void loop() {
  limitSwitch.loop(); // MUST call the loop() function first
  limitSwitchEnd.loop();

  switch (stateMachine)
  {
  case 0:
  {
    digitalWrite(dirPin, LOW);
    stepperEnabled = true;
    stepCount = 0;
    Serial.println("STATE ZERO");
    if(limitSwitch.isPressed()){
        // Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
        stateMachine = 1;
    }

    if(limitSwitch.isReleased()){
        // Serial.println("The limit switch: TOUCHED -> UNTOUCHED");   
        stateMachine = 0;
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
        stateMachine = 0;
    }
    else{
        // Serial.println("The limit switch: TOUCHED");
        stateMachine = 1;
        if(stop_time > 50){
          stateMachine = 2;
          stop_time = 0;
        } else {
          stop_time++;
        }
    }

    break;
  }

  case 2:
  {

    // Serial.println("STATE TWO");
    digitalWrite(dirPin, HIGH);
    stepCount = 0;
    stepperEnabled = true;
    stateMachine = 2;

    if(limitSwitchEnd.isPressed()){
      // Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
      stateMachine = 3;
    }

    if(limitSwitchEnd.isReleased()){
      // Serial.println("The limit switch: TOUCHED -> UNTOUCHED"); 
      stateMachine = 2;
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
  
  default:
    break;
  }
}


ISR(TIMER2_COMPA_vect) {
  if (stepCount < stepsPerRevolution && stepperEnabled) {
    if (isHigh) {
      digitalWrite(stepPin, LOW);
      isHigh = false;
    } else {
      digitalWrite(stepPin, HIGH);
      isHigh = true;
      stepCount++;
    }
  }
}
