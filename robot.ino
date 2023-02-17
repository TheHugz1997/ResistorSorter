#include <Servo.h>


#include "./Hardware/stepper/stepper.h"


Stepper stepper = Stepper(2,3);
Servo servo;



int pos = 0;
void setup() {
    servo.attach(5);
}

void loop() {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
       servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    } 
    // serv.rotate(90);
    stepper.rotate_n(10,3);
    stepper.set_direction_spin(LOW);

    delay(4000);
}