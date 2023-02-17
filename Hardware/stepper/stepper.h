#define stepsPerRevolution 200


class Stepper{
    public:
        Stepper::Stepper(int dirPin,int stepPin);
        void set_direction_spin(byte clockwise);
        void rotate_n(int speedfactor,int n);
    private:
        int dirPin;
        int stepPin;
};


Stepper::Stepper(int dirPin,int stepPin){
    Stepper::dirPin = dirPin;
    Stepper::stepPin = stepPin;
    
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    // set to clockwise by default
    digitalWrite(dirPin, HIGH);

}

void Stepper::set_direction_spin(byte clockwise){
    digitalWrite(dirPin, clockwise);
}

void Stepper::rotate_n(int speedfactor,int n){
    
    // set the motor to one rotation 
    for (int i = 0; i < n *stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10000/speedfactor);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10000/speedfactor);
  }
}