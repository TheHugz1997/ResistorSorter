#include <Servo.h>

int servo_pin = 5;
Servo servo;

void initialize_servo(){
    servo.attach(servo_pin);
}


void rotate_servo(int angle){
    servo.write(angle);
}