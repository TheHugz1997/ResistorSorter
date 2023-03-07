#include <Servo.h>
#include "./Hardware/stepper/stepper.h"
#include "./Hardware/multiplexor/multiplexor.h"


Stepper stepper = Stepper(2,3);
Servo servo;


int pos = 0;

void setup() {
    intialize_servo();
    initialize_mux();
    Serial.begin(9600);
}


void loop() {
    //___________________SERVO___________________
    
    //___________________STEPPER___________________
    stepper.set_direction_spin(LOW);
    //Serial.print("THIS IS HIGH DIRECTION");
    stepper.rotate_n(1, 10);
    delay(1000);
       
    stepper.set_direction_spin(HIGH);
    //Serial.print("THIS IS HIGH DIRECTION");
    stepper.rotate_n(1, 10);
    delay(1000);



    //___________________Resistor mesurement___________________
    // auto_calibrate();
    // delay(15000);
}