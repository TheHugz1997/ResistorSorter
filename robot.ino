#include <LiquidCrystal_I2C.h>



#include <ezButton.h>
#include <Arduino.h>
#include <avr/io.h>
#include "Hardware/multiplexor/multiplexor.h"
#include "Hardware/stepper/distance.h"
#include <Servo.h>
#include <math.h>

#define TIMEOUT (2000)
#define MIN_Y_SLOT (1)
#define MAX_Y_SLOTS (6)
#define SLOT_X_SIZE (75)
#define SLOT_Y_SIZE (35)
#define RES_TOLERANCE (0.1)
#define CENTER_Y_OFFSET (15)
#define CENTER_X_OFFSET (15)


// Creation of the Enum that will contain the sequences of the FSM
typedef enum {
  RS_CALIBRATION,
  RS_MEASURE,
  RS_COMPUTE_X_Y,
  RS_GOTO_X_Y,
  RS_DROP
} ResistorSorterState_t;


// First state of the FSM at the start of the process
ResistorSorterState_t state = RS_CALIBRATION;
volatile int currentStepX, currentStepY = 0;
int setpointX, setpointY = 0;


/*

  ***** CARTESIAN ROBOT CONFIGURATION *****

  Take into account that the X axis stepper goes in the direction of the Arduino if dirpin is on HIGH
  Take into account that the Y axis stepper goes in the direction of the tower (moves away from X axis) if dirpin2 is on HIGH

*/

// create ezButton object (in our case switches) that are attached to a specific pin
ezButton limitSwitch(8);
ezButton limitSwitchEnd(9);
ezButton limitSwitchYaxis(11);

// X AXIS STEPPER SETUP
const int stepPin = 3;
const int dirPin = 2;

// Y AXIS STEPPER SETUP
const int stepPin2 = 6;
const int dirpin2 = 5;

// Y AXIS SERVO SETUP
Servo servo_y_axis;
const int servo_y_pin = 47;

// CONFIGURATION OF THE INTERRUPT SETTINGS
const int stepsPerRevolution = 200;
const float stepDelay = 500.0; // microseconds
volatile int stepCount = 0;
volatile bool isHigh = false;
volatile bool isHighY = false;
uint32_t stop_time_ms = millis();
int stop_time = 0;
int stop_time_two = 0;

// FLAGS USED FOR ENABLE OR DISABLE THE STEPPERS OR THE SIGNAL IF THE ROBOT IS IN CALIBRATION MODE
volatile bool stepperXEnabled = false;
volatile bool stepperYEnabled = false;
volatile bool isInCalibrationMode = true;


/*

  ***** ROBOT'S MEASURE PART CONFIGURATION *****

*/

// I2C address 0x27, 16 column and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo;
const int buttonPin = 49; 
const int servoPin = 51;
int buttonState = 0;
float res= 0.0;



void setup() {

    Serial.begin(115200);

    // SET LIMITSWITCH DEBOUNCE TIME TO 10 Micro Seconds
    limitSwitch.setDebounceTime(10); 
    limitSwitchEnd.setDebounceTime(10);
    limitSwitchYaxis.setDebounceTime(10);

    cli(); // disable interrupts
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    OCR2A = (int)(16 * stepDelay - 1);
    TCCR2A |= (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS22);
    TIMSK2 |= (1 << OCIE2A);
    sei(); // enable interrupts

    // STEPPERS I/O
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    pinMode(stepPin2, OUTPUT);
    pinMode(dirpin2, OUTPUT);

    state = RS_CALIBRATION;

    // ROBOT MESURE SETUP
    initialize_mux();
    servo.attach(servoPin);
    servo.write(45);
    lcd.init(); 
    lcd.backlight(); //open the backlight 
    pinMode(buttonPin, INPUT);
    display_init();

}

void loop() {
  for (;;) {

    // MUST call the loop() function first for the limit switch

    limitSwitch.loop();
    limitSwitchEnd.loop();
    limitSwitchYaxis.loop();

    // STATE MACHINE

    stateMachineSequencer();
  }
}

void stateMachineSequencer(void) {
  switch(state) {
    case RS_CALIBRATION: {
      servo.detach();
      servo_y_axis.attach(servo_y_pin);
      servo_y_axis.write(35);
      isInCalibrationMode = true;
      stepperXEnabled = limitSwitch.getState();
      stepperYEnabled = limitSwitchYaxis.getState();

      digitalWrite(dirPin, HIGH);
      digitalWrite(dirpin2, HIGH);

      if (!stepperXEnabled && !stepperYEnabled) {
        isInCalibrationMode = false;
        currentStepX = 0;
        currentStepY = 0;
        state = RS_MEASURE;
      }
      break;
    } case RS_MEASURE: {

      int buttonState = digitalRead(buttonPin);

      servo_y_axis.detach();
      servo.attach(servoPin);

      if (buttonState == HIGH){
          servo_up();
          delay(4000);
          res = auto_calibrate();

          if(res< 10000000){
            display_infos(res);
            delay(3000);
            finish_sequence_measure();
            servo.write(45);
            state = RS_COMPUTE_X_Y;
            if(res<100.0){
              res = 10.0; 
              display_message("< 100 ohms");
            }
            servo.write(0);
            delay(500);
            servo.write(60);
            delay(500);
            servo.write(0);
            delay(500);
            servo.write(45);
            
          }
          else{
            print_measure_failed();
            servo.write(45);
          }
      }
      break;
    } case RS_COMPUTE_X_Y: {
      servo.detach();
      servo_y_axis.attach(servo_y_pin);


      digitalWrite(dirPin, LOW);
      digitalWrite(dirpin2, LOW);
      computeSlotDistance(res);

      state = RS_GOTO_X_Y;
      stepperXEnabled = true;
      stepperYEnabled = true;
      display_init();
      break;
    } case RS_GOTO_X_Y: {
      if (!limitSwitchEnd.getState()) {
        stepperXEnabled = false;
        if ((currentStepY >= setpointY)) {
          state = RS_DROP;
        }
      }
      if ((currentStepX >= setpointX) && (currentStepY >= setpointY)) {
        state = RS_DROP;
        stepperXEnabled = false;
        stepperYEnabled = false;
      }

      break;
    } case RS_DROP: {
      delay(2000);
      servo_y_axis.write(0);
      delay(2000);
      servo_y_axis.write(35);
      delay(2000);
      state = RS_CALIBRATION;
      break;
    } default:
      state = RS_CALIBRATION;
  }
}


void computeSlotDistance(uint32_t value) {
  int xSlot = 0, ySlot = 0;

  // Y axis
  for (int i = MIN_Y_SLOT; i < MAX_Y_SLOTS + 1; i++) {
    float rest = (float)value / pow(10.0, (float)i);
    if (rest < 10) {
      ySlot = i;
      break;
    }
  }

  // X axis
  double lowValue = (double)value / pow(10,  ySlot);

  if ((1.0 - RES_TOLERANCE <= lowValue) && (lowValue <= 1.0 + RES_TOLERANCE))
    xSlot = 0;
  else if ((1.2 - RES_TOLERANCE <= lowValue) && (lowValue <= 1.2 + RES_TOLERANCE))
    xSlot = 1;
  else if ((1.5 - RES_TOLERANCE <= lowValue) && (lowValue <= 1.5 + RES_TOLERANCE))
    xSlot = 2;
  else if ((1.8 - RES_TOLERANCE <= lowValue) && (lowValue <= 1.8 + RES_TOLERANCE))
    xSlot = 3;
  else if ((2.2 - RES_TOLERANCE <= lowValue) && (lowValue <= 2.2 + RES_TOLERANCE))
    xSlot = 4;
  else if ((2.7 - RES_TOLERANCE <= lowValue) && (lowValue <= 2.7 + RES_TOLERANCE))
    xSlot = 5;
  else if ((3.3 - RES_TOLERANCE <= lowValue) && (lowValue <= 3.3 + RES_TOLERANCE))
    xSlot = 6;
  else if ((3.9 - RES_TOLERANCE <= lowValue) && (lowValue <= 3.9 + RES_TOLERANCE))
    xSlot = 7;
  else if ((4.7 - RES_TOLERANCE <= lowValue) && (lowValue <= 4.7 + RES_TOLERANCE))
    xSlot = 8;
  else if ((5.6 - RES_TOLERANCE <= lowValue) && (lowValue <= 5.6 + RES_TOLERANCE))
    xSlot = 9;
  else if ((6.8 - RES_TOLERANCE <= lowValue) && (lowValue <= 6.8 + RES_TOLERANCE))
    xSlot = 10;
  else if ((8.2 - RES_TOLERANCE <= lowValue) && (lowValue <= 8.2 + RES_TOLERANCE))
    xSlot = 11;
  
  // IF THE X SLOT IS LOCATED OVER THE 33 SLOT THEN THE SLOTS ARE 7.5 CM WITDH AND YOU DON'T USE THE OFFSET ANYMORE
  if (xSlot < 6){
    setpointX = Distance::convert_distance_into_steps(xSlot * SLOT_X_SIZE);
  } else {
    setpointX = Distance::convert_distance_into_steps(xSlot * SLOT_X_SIZE + CENTER_X_OFFSET);
  }

  // IF THE Y SLOT IS THE FIRST ONE DON'T USE THE OFFSET (BASED ON PHYSICAL DIMENSIONS OF THE GRID)
  if (ySlot == MIN_Y_SLOT) {
    setpointY = Distance::convert_distance_into_steps((MAX_Y_SLOTS - ySlot) * SLOT_Y_SIZE + 5);
  } else {
    setpointY = Distance::convert_distance_into_steps((MAX_Y_SLOTS - ySlot) * SLOT_Y_SIZE + CENTER_Y_OFFSET);
  }
}


ISR(TIMER2_COMPA_vect) {
  if (stepperXEnabled && (isInCalibrationMode || (currentStepX < setpointX) )) {
    if (isHigh) {
      digitalWrite(stepPin, LOW);
      isHigh = false;
    } else {
      digitalWrite(stepPin, HIGH);
      isHigh = true;
      currentStepX++;
    }
  }

  if (stepperYEnabled && (isInCalibrationMode || (currentStepY < setpointY) )) {
    if (isHighY) {
      digitalWrite(stepPin2, LOW);
      isHighY = false;
    } else {
      digitalWrite(stepPin2, HIGH);
      isHighY = true;
      currentStepY++;
    }
  }
}


void measure_sequence(){
    servo_up();
    delay(2000);
    res = auto_calibrate();
    display_infos(res);
    drop_resistance();
    delay(2000);
    display_init();
}


void display_init(){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Place Resistor"); 
}


void display_message(String msg){
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(msg);
}


void print_measure_failed(){
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Measure failed");
  lcd.setCursor(0,2);
  lcd.print("Replace resistor");
}


void display_infos(float res){
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(res);
    lcd.setCursor(0,2);
    lcd.print("ohms");
    Serial.println(res);
}


void servo_up(){
    servo.write(82);
}


void drop_resistance(){
    servo.write(0);
    delay(1000);
    servo.write(45);
}


void finish_sequence_measure(){
  display_infos(res);
  delay(500);
  drop_resistance();
  delay(1000);

}
