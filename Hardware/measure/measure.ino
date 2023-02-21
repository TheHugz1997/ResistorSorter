//int analogPin = A0;
//uint8_t raw = 0;
int Vin = 0;
float Vout = 0;
float R1 =1000;
float R2 =0;
float buffer = 0;


void setup(){
    Serial.begin(9600);
    Serial.println("Start");
    pinMode(A0, INPUT);
}


void loop(){
    int sensorValue = analogRead(A0);
    Serial.println(sensorValue);
    float result = sensorValue * (5.0 / 1023.0);
    Serial.println(result);
    Serial.println("V mesurée");

    
    delay(1000);
    
}