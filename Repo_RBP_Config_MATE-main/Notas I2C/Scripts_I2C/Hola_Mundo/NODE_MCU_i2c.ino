//Wire esclavo receptor 

//05/07/2022

#include <Wire.h> 
static_assert(LOW == 0, "Esperando LOW para que sea 0"); 
int ledPin = 13;
void setup() 
{
  Wire.begin(0x8); 
  Wire.onReceive(receiveEvent); 
  pinMode(ledPin, OUTPUT); 
  digitalWrite(ledPin, LOW); 
  Serial.begin(9600);
}


void loop() {
delay(100);
}
int receiveEvent(){
  while(Wire.available()){
    int c = Wire.read(); 
    Serial.println(c);
    return c; 
    digitalWrite(ledPin, c);
  }


  
}
