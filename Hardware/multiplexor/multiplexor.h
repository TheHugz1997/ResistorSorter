//Mux control pins
int s0 = 46;
int s1 = 48;
int s2 = 50;
int s3 = 52;
//Mux arduino mega control pins
// int s0 = 3;
// int s1 = 4;
// int s2 = 5;
// int s3 = 6;

//Mux in "SIG" pin
int SIG_pin = A0;
// int realVcc_pin = A1;
// float realVccGlobal = 0.0;
// float adcVal = 0.0;


float resistance;
// breadboard resistances
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


// float getRealVccGlobal() {
//   return realVccGlobal;
// }


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

  int val = analogRead(SIG_pin);
  float voltage = (val * 5.0) / 1024.0;
  return voltage;
}



float calc_res(int channel,float voltage){
  double Rmux = mux_resistance[channel];
  resistance = ((Rmux+65) * voltage) / (5.0 - voltage);
  return resistance;
}


float to_norm_E12(float res,int range){

  float out = 0.0;
  for (int i = 0; i<11;i++){
    float curr_diff = abs(ranges[range][i] - res);
    float next_diff = abs(ranges[range][i+1] -res);
    if(curr_diff<next_diff){
      Serial.print("Res E12 value :");
      Serial.println(ranges[range][i]);
      out = ranges[range][i];
      // return out;
      break;
    }
    if (i==10){
      float next_range_diff = abs(ranges[range+1][0] -res);
      if(curr_diff<next_diff){
        Serial.print("Res E12 value :");
        Serial.println(ranges[range][i]);
        out = ranges[range][i];
        // return out;
        break;
      }
      else{
        if (next_range_diff < next_diff){
          out = ranges[range+1][0];
          break;
        }
        else{
          out = ranges[range][11];
          break;
        }
      }
      break;
    }
  }
  return out;

}

float auto_calibrate(){
  float output = 0.0;
  Serial.println("auto_calibrating...");
  Serial.println("");
  for(int channel =14; channel>=0;channel-=2){ 
    float voltage = read_mux(channel);
    float mean = 0;
    for(int i = 0; i <50;i++){
      mean += read_mux(channel);
      // delay(50);
    }
    mean = mean/50;
    voltage = mean;


    float res = calc_res(channel,voltage);
    delay(50);

    
    if (channel==14 && voltage >0.5){
      Serial.println(res);
      output = to_norm_E12(res,6);
      if (output<1000000.0){
        Serial.println("(Should be between 100k and 1M)");
        output = to_norm_E12(res,5);
      }else{
        Serial.println("(Should be greater than 1M)");
        if (res>10000000){
          output = 10000000;
        }
      }
      // return output;
      break;
        
    }
    if (channel ==8 && voltage > 3.9){
      Serial.println(res);
      Serial.println("(Should be between 100k and 1M)");
      output = to_norm_E12(res,5);
      // return output;
      break;
    }
    if (channel == 6 && voltage > 0.95){
      Serial.println(res);
      Serial.println("(Should be between 10k and 100k)");
      output = to_norm_E12(res,4);
      // return output;
      break;
    }
    if (channel == 2 && voltage > 0.9){
      Serial.println(res);
      Serial.println("(Should be between 1k and 10k)");
      output = to_norm_E12(res,3);
      // return output;
      break;
    }
    if (channel == 0 && voltage > 0.01){
      Serial.println(res);
      if(res>92.0){
        output = to_norm_E12(res+5.0,2);
        Serial.println("(Should be between 100 and 1k)");
        // return output;

      }
      else if(res>9.2 && res<92.0){
        Serial.println(res);
        Serial.println("(Should be between 10 and 100)");
        output = to_norm_E12(res-1.7,1);
        // return output;
      }
      else{
        Serial.println(res);
        Serial.println("(Should be lower than 10)");
        output = to_norm_E12(res-1.7,0);
        // return output;
      }
      break;
    }
  }return output;
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