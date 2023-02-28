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

    // The argument is the distance you want to travel in mm
    int steps = distance.convert_distance_into_steps(500);

    Serial.println(steps);
    
    distance.set_direction_pin(LOW);
    
    distance.rotate_n(steps);

    delay(5000);


}