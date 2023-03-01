#include <Servo.h>


#include "./Hardware/stepper/stepper.h"
#include "./Hardware/stepper/distance.h"


//Stepper stepper = Stepper(2,3);
Servo servo;

Distance distance = Distance(2, 3);




int pos = 0;

void setup() {

    servo.attach(5);



}


void loop() {

    // Low in the direction of the X axis positive

    // The argument is the distance you want to travel in mm
    delay(6000);

    int steps = distance.convert_distance_into_steps(600);

    Serial.println(steps);
    
    distance.set_direction_pin(LOW);
    
    distance.rotate_n(steps);

    delay(5000);

    int stepsBack = distance.convert_distance_into_steps(600);
    
    distance.set_direction_pin(HIGH);
    
    distance.rotate_n(stepsBack);

    delay(5000);


}