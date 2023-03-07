#include <Servo.h>

int servo_pin = 5;
Servo servo;

initialize_servo(){
    servo.attach(servo_pin);
}


rotate_servo(int angle)(
    servo.write(angle);
)