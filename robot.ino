
#include "Hardware/stepper/stepper.h"
#include "Hardware/multiplexor/multiplexor.h"
#include <Servo.h>


Stepper stepper = Stepper(2,3);
Servo servo;

int pos = 0;

void setup() {
    // initialize_servo();
    initialize_mux();
    servo.attach(6);
    servo.write(0);
    Serial.begin(9600);
}


void loop() {
    //___________________STEPPER___________________
    // stepper.set_direction_spin(LOW);
    // //Serial.print("THIS IS HIGH DIRECTION");
    // stepper.rotate_n(1, 10);
    // delay(1000);
       
    // stepper.set_direction_spin(HIGH);
    // //Serial.print("THIS IS HIGH DIRECTION");
    // stepper.rotate_n(1, 10);
    // delay(1000);



    //___________________Resistor mesurement___________________
    // servo.write(45);
    // Serial.println("Place your resistance");
    // Serial.println("");
    // delay(3000);
    // servo.write(90);
    auto_calibrate();
    delay(1000);
    // drop_resistance();
    // delay(2000);
}




void drop_resistance(){
    servo.write(45);
    delay(1000);
    servo.write(0);
    delay(1000);
}


