
class Multiplexor{
  public:
    void setPinSetup(int s0,int s1,int s2,int s3);
    float measure(int channel);
  private:
    int controlPin[4];
};

void Multiplexor::setPinSetup(int s0,int s1,int s2,int s3){
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

}

