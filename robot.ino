#include <LiquidCrystal_I2C.h>

#include "Hardware/stepper/stepper.h"
#include "Hardware/multiplexor/multiplexor.h"
#include <Servo.h>

//#include <LiquidCrystal_I2C.h> // Library for LCD

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal_I2C lcd(0x27, 16, 4); // I2C address 0x27, 16 column and 2 rows


Stepper stepper = Stepper(2,3);
Servo servo;

const int buttonPin = 6; 
int buttonState = 0;
float res= 0.0;


void setup() {
    // initialize_servo();
    initialize_mux();
    servo.attach(7);
    servo.write(45);
    Serial.begin(9600);

    lcd.init(); //initialize the lcd
    lcd.backlight(); //open the backlight 
    Serial.println("Place your resistance");
    Serial.println("");
    pinMode(buttonPin, INPUT);
}


void loop() {
    lcd.clear();                 // clear display
    lcd.setCursor(0, 0);         // move cursor to   (0, 0)
    lcd.print("Place Resistor"); 
    //___________________Resistor mesurement___________________
    buttonState = digitalRead(buttonPin);
    if (buttonState==HIGH){
        prepare_measure();
        delay(2000);
        res = auto_calibrate();
        display_infos(res);
        drop_resistance();
        delay(2000);
    }
}




void display_infos(float res){
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print(res);
    lcd.setCursor(0,3);
    lcd.print("ohms");
    Serial.println(res);
}
void prepare_measure(){
    servo.write(89);
}

void drop_resistance(){
    // servo.write(45);
    // delay(500);
    servo.write(0);
    delay(1000);
    servo.write(45);
}


