#define stepsPerRevolution 200


class Stepper{

    public:
        Stepper::Stepper(int dirPin, int stepPin);
        void set_direction_spin(byte clockwise);
        void rotate_n(int speedfactor,int n);

    private:
        int directionPin;
        int stepPin;
};


Stepper::Stepper(int directionPin, int stepPin){
    Stepper::directionPin = directionPin;
    Stepper::stepPin = stepPin;
    
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    // set to clockwise by default
    digitalWrite(directionPin, HIGH);

}

void Stepper::set_direction_spin(byte clockwise){
    digitalWrite(directionPin, clockwise);
}

void Stepper::rotate_n(int speedfactor, int n){
    
    // set the motor to one rotation 
    for (int i = 0; i < n * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
    }
}