//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;

//Mux in "SIG" pin
int SIG_pin = A0;

float resistance;
double mux_resistance[16] = {392.0,0,3920.0,0,2710.0,0,38800.0,0,27000.0,0,393000.0, 0,270000.0,0,3920000.0,0};
const float ranges[7][12] = {
    {1.0,1.2,1.5,1.8,2.2,2.7,3.3,3.9,4.7,5.6,6.8,8.2}, 
    {10.0,12.0,15.0,18.0,22.0,27.0,33.0,39.0,47.0,56.0,68.0,82.0}, 
    {100.0,120.0,150.0,180.0,220.0,270.0,330.0,390.0,470.0,560.0,680.0,820.0},
    {1000.0,1200.0,1500.0,1800.0,2200.0,2700.0,3300.0,3900.0,4700.0,5600.0,6800.0,8200.0},
    {10000.0,12000.0,15000.0,18000.0,22000.0,27000.0,33000.0,39000.0,47000.0,56000.0,68000.0,82000.0},
    {100000.0,120000.0,150000.0,180000.0,220000.0,270000.0,330000.0,390000.0,470000.0,560000.0,680000.0,820000.0},
    {1000000.0,1200000.0,1500000.0,1800000.0,2200000.0,2700000.0,3300000.0,3900000.0,4700000.0,5600000.0,6800000.0,8200000.0}
    
    
    
    };

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
  double Rmux = mux_resistance[channel];
  //input resistance of the mux : 70 ohms
  resistance = ((Rmux+60.0) * voltage) / (5.0 - voltage);

  return resistance;
}

void to_norm_E12(float res,int range){
  for (int i = 0; i<11;i++){
    float curr_diff = abs(ranges[range][i] - res);
    float next_diff = abs(ranges[range][i+1] -res);
    if(curr_diff<next_diff){
      Serial.print("Res E12 value :");
      Serial.println(ranges[range][i]);
      Serial.print("Index");
      Serial.println(i);
      Serial.print("Range :");
      Serial.println(range);
      break;
    }
    if (i==10){
      Serial.print("Res E12 value :");
      Serial.println(ranges[range][11]);
      break;
    }
    
  }

}

void auto_calibrate(){
  // Umes = (Rinc/Rmux+Rinc)* 5V
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

    
    
    if (channel==14 && voltage >0.95){
      Serial.println("Should be greater than 1M");
      to_norm_E12(res,6);

      break;
        
    }
    if (channel ==8 && voltage > 3.9){
      Serial.println("Should be between 100k and 1M");
      to_norm_E12(res,5);
      break;
    }
    if (channel == 6 && voltage > 0.9){
      Serial.println("Should be between 10k and 100k");
      to_norm_E12(res,4);
      break;
    }
    if (channel == 2 && voltage > 0.9){
      Serial.println("Should be between 1k and 10k");
      to_norm_E12(res,3);
      break;
    }
    if (channel == 0 && voltage > 0.01){
      if(res>100.0){
        // force channel two for more precision
        voltage = read_mux(2);
        res = calc_res(2,voltage);
        Serial.print("Recalibrated res : ");
        Serial.println(res);
        to_norm_E12(res,2);
        Serial.println("Should be between 100 and 1k");

      }
      else if(res>10.0 && res<100.0){
        Serial.println("Should be between 10 and 100");
        to_norm_E12(res,1);
      }
      else{
        Serial.println("Should be lower than 10");
        to_norm_E12(res,0);
      }
      break;
    }
    
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