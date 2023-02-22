#include <Servo.h>


#include "./Hardware/stepper/stepper.h"


Stepper stepper = Stepper(2,3);
Servo servo;


int pos = 0;

void setup() {

    servo.attach(5);

}


void loop() {

    stepper.set_direction_spin(HIGH);

    stepper.rotate_n(1, 5);

    delay(4000);

    stepper.set_direction_spin(LOW);

    stepper.rotate_n(1, 5);

    delay(4000);
    
}