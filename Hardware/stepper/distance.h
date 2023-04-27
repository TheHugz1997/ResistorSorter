#include <math.h>

// Diameter of the pulley in mm
#define DIAMETER_OF_THE_PULLEY 12
// This distance is in mm
#define DISTANCE_BY_STEP 0.1885
// This is the constant error correction on the number of steps
#define CORRECTION_STEP 238

class Distance{

    public:
        Distance(int dirPin, int theDistanceToTravel);
        void set_direction_pin(byte clockwise);
        static int convert_distance_into_steps(int distance);
        void rotate_n(int n);


    private:
        int directionPin;
        int stepPin;
};

Distance::Distance(int directionPin, int stepPin){
    Distance::directionPin = directionPin;
    Distance::stepPin = stepPin;
    
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(directionPin, OUTPUT);

    // set to clockwise by default
    digitalWrite(directionPin, HIGH);

}

int Distance::convert_distance_into_steps(int distance) {
    if(distance > 950) {
        return 950;
    } 
    else {
        float stepsNumber = distance / DISTANCE_BY_STEP;
        Serial.println(stepsNumber);
        int approxStepsNumber = static_cast<int>(stepsNumber);
        Serial.println(approxStepsNumber);
        int correctedStepsNumber = approxStepsNumber; //  - CORRECTION_STEP;
        Serial.println(correctedStepsNumber);
        return correctedStepsNumber;
    }
}

void Distance::set_direction_pin(byte clockwise){
    digitalWrite(directionPin, clockwise);
}

void Distance::rotate_n(int n){

    for (int i = 0; i < n; i++) {
    // These four lines result in 1 step:
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
    }
}











