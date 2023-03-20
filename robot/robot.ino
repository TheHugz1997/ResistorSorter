#include <ezButton.h>
#include <Arduino.h>
#include <avr/io.h>

ezButton limitSwitch(7);  // create ezButton object that attach to pin 7;

unsigned int stateMachine;


const int stepPin = 3;
const int dirPin = 2;
const int stepsPerRevolution = 200;
const float stepDelay = 500.0; // microseconds
volatile int stepCount = 0;
volatile bool isHigh = false;

int stop_time = 0;

volatile bool stepperEnabled = false;

void setup() {
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

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
  digitalWrite(dirPin, HIGH);

  stateMachine = 0;
}

void loop() {
  limitSwitch.loop(); // MUST call the loop() function first

  switch (stateMachine)
  {
  case 0:
  {
    stepperEnabled = true;
    stepCount = 0;
    Serial.println("STATE ZERO");
    if(limitSwitch.isPressed()){
        Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
        stateMachine = 1;
    }

    if(limitSwitch.isReleased()){
        Serial.println("The limit switch: TOUCHED -> UNTOUCHED");   
        stateMachine = 0;
    }

    break;
  }
  case 1:
  {
    Serial.println("STATE ONE");
    int switch_state = limitSwitch.getState();
    if(switch_state == HIGH){
        Serial.println("The limit switch: UNTOUCHED");
        stateMachine = 0;
    }
    else{
        Serial.println("The limit switch: TOUCHED");
        stateMachine = 1;
        stepperEnabled = false;
        if(stop_time > 50){
          stateMachine = 2;
        } else {
          stop_time++;
        }
    }

    break;
  }

  case 2:
  {

    Serial.println("STATE TWO");
    stop_time = 0;
    digitalWrite(dirPin, LOW);
    stepCount = 0;
    stepperEnabled = true;
    stateMachine = 2;
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
