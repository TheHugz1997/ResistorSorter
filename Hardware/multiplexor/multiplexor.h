//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;

//Mux in "SIG" pin
int SIG_pin = A0;

float resistance;
double mux_resistance[16] = {392.0,0,3920.0,0,2710.0,0,38800.0,0,27000.0,0,393000.0, 0,270000.0,0,3920000.0,0};

void initialize_mux(){
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

}


float read_mux(int channel){
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0  : 392 ohms
    {1,0,0,0}, //channel 1 
    {0,1,0,0}, //channel 2  : 3,92k ohms
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4  : 2,71k ohms
    {1,0,1,0}, //channel 5 
    {0,1,1,0}, //channel 6  : 38,8k ohms
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8  : 27k ohms
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10 : 393k ohms
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12 : 270k ohms
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14 : 3,92M ohms
    {1,1,1,1}  //channel 15
  };

  
  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(SIG_pin);
  
  int mean = 0;
  for(int i = 0; i <3;i++){
    mean += analogRead(SIG_pin);
  }
  mean = mean/3;
  //return the value
  float voltage = (mean * 5.0) / 1024.0;
  return voltage;
}



float calc_res(int channel,float voltage){
  
  //Serial.print("Res value : ");
  double Rmux = mux_resistance[channel];
  //float ans;
  //if (channel==0){
    // for the first channel, resistance is only of 392 ohms -> we cannot neglect the input resistance of the mux : 70 ohms
  resistance = ((Rmux+70.0) * voltage) / (5.0 - voltage);
  //}
  //else{
    //Serial.println("Range >1");
    //ans = (Rmux * voltage) / (5.0 - voltage);
  //}
  return resistance;
}

void auto_calibrate(){
  // Umes = (Rinc/Rmux+Rinc)* 5V
  // pour Rinc grand -> tension tend vers 5V
  // pour Rinc petit -> tension tend vers 0V 


  // pour Rmux grand -> si Rinc grand , tension tend vers 5v (break vers 4.85V)
  //                 -> si Rinc petit, tenion tend vers 0v
  Serial.println("auto_calibrate");
  for(int channel =14; channel>=0;channel-=2){ 

    Serial.print("Input channel ");
    Serial.println(channel);
    Serial.print("Voltage :");
    float voltage = read_mux(channel);

    Serial.println(voltage);
    Serial.print("Res value :");
    float res = calc_res(channel,voltage);
    Serial.println(res);

    Serial.print("Your resistance :");
    Serial.println(res);
    
    if (channel==14 && voltage >0.95){
      Serial.println("Should be greater than 1M");
      Serial.println("");

      break;
        
    }
    if (channel ==8 && voltage > 3.9){
      Serial.println("Should be between 100k and 1M");
      break;
    }
    if (channel == 6 && voltage > 0.9){
      Serial.println("Should be between 10k and 100k");
      break;
    }
    if (channel == 2 && voltage > 0.9){
      Serial.println("Should be between 1k and 10k");
      break;
    }
    if (channel == 0 && voltage > 0.01){
      Serial.println("Should be lower than 1k");
      break;
    }
    //delay(4000);
  }
}

void debug(){
  //Loop through and read all 16 values
  for(int i = 0; i < 16; i ++){
    Serial.print("Value at channel ");
    Serial.print(i);
    Serial.print("is : ");
    Serial.println(read_mux(i));
    delay(1000);
  }
}