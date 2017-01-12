

const byte addrA=10;
const byte addrB=11;
const byte addrC=12;
const byte enableA=9;
const byte enableB=8;

const byte readPin=A7;
const byte headerMap[]={3,0,5,7,1,6,4,2,3,0,1,5,7,2,6,4};
void setup() {
  // put your setup code here, to run once:
pinMode(addrA, OUTPUT);
pinMode(addrB, OUTPUT);
pinMode(addrC, OUTPUT);
pinMode(enableA, OUTPUT);
pinMode(enableB, OUTPUT);
digitalWrite(enableA,HIGH);
digitalWrite(enableB,HIGH);

  Serial.begin(250000);
}

void loop() {

for (byte i=0;i<=15;i++)
  {
    setMUX(i);

    Serial.println(readMUX()); 
    Serial.println(readMUX()); 
    Serial.println(readMUX()); 
  
  }

}

void setMUX(byte pin){
  //select chip select pin
    digitalWrite(enableA,(pin<8));  
    digitalWrite(enableB,!(pin<8));
  //decode binary address
pin=headerMap[pin];
  digitalWrite(addrA,1-((pin   )&0x1));
  digitalWrite(addrB,1-((pin>>1)&0x1));
  digitalWrite(addrC,1-((pin>>2)&0x1));

}
int readMUX(){
  return analogRead(readPin);
}

