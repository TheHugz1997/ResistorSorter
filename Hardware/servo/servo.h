#include <Servo.h>

Servo servo;

class MyServo{
    public:
        MyServo::MyServo(int pin);
        void rotate(int pin);
    private:
        Servo servo;
};


MyServo::MyServo(int pin){
    MyServo::servo = Servo();
    MyServo::servo.attach(pin);
}

void MyServo::rotate(int angle){
    MyServo::servo.write(angle);
}