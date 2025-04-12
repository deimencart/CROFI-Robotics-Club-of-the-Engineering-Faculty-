#define motIzq2 13
#define motIzq1 2
#define motDer1 12
#define motDer2 7
#define pwmIzq 11
#define pwmDer 5
#define setPoint 2.38 //2.38
#define velStd 255 //255
#define Kp 550  //550
#define Kd 150 //150
#define Ki 0  //0
#define button 9 
int senales[6];
int sensores[6] = {A0, A1, A2, A3, A4, A5};
long black[6] = {0,0,0,0,0,0};
int white[6] = {1021,1021,1021,1021,1021,1021};
int velIzq, velDer;
bool izq = true, der = false;
float posicion, error, lastError, derivative, control;
long double integral = 0;

void setup() {
  pinMode(button, INPUT_PULLUP);
  pinMode(motIzq1, OUTPUT);
  pinMode(motIzq2, OUTPUT);
  pinMode(motDer1, OUTPUT);
  pinMode(motDer2, OUTPUT);
  pinMode(pwmIzq, OUTPUT);
  pinMode(pwmDer, OUTPUT);
  for (int x = 0; x < 6; x++){
    pinMode(sensores[x], INPUT);
  }
  digitalWrite(motDer1,HIGH);
  digitalWrite(motDer2,LOW);
  digitalWrite(motIzq1,HIGH);
  digitalWrite(motIzq2,LOW);
  Serial.begin(9600);
  lastError = 0;
}

void loop(){
  
  start:{
    digitalWrite(LED_BUILTIN,LOW);
    if (!digitalRead(button)) goto calibrate;
    else goto start;
  }
  
  calibrate:{
    digitalWrite(LED_BUILTIN,HIGH);
    for(int i = 0; i < 10000; i ++){
      for(int x = 0; x < 6; x++){
      senales[x] = analogRead(sensores[x]); //Leer sensores
      if(senales[x] > black[x]) black[x] = senales[x];
      else if(senales[x] < white[x]) white[x] = senales[x]; 
      }
    }
    while(true){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      if(!digitalRead(button)){
        delay(2000);
        goto main;
      }
    }
  }
  
  main:{
    for(int x = 0; x < 6; x++){
      senales[x] = map(analogRead(sensores[x]), black[x], white[x], 0, 100); //Leer sensores
    }

    if((senales[0] > 10)){
      izq = true;
      der = false;
    }
    else if((senales[5] > 10)){
      izq = false;
      der = true;
    }
    
    if((senales[0] < 5) and (senales[1] < 5) and (senales[2] < 5) and (senales[3] < 5)  and (senales[4] < 5) and (senales[5] < 5)){
      if(izq){
        goto left;
      }
      else if (der){
        goto right;
      }
    }
    posicion = (float)(0*senales[0]+1*senales[1] + 2*senales[2] + 3*senales[3] + 4*senales[4] + 5*senales[5])/(float)(senales[0] + senales[1] + senales[2] + senales[3] + senales[4] + senales[5]);   
  
    error = setPoint - posicion;
    derivative = error-lastError;
    integral += error;
    
    control = Kp*error - Kd*derivative + Ki*integral;
    
    //velIzq = velStd - control;
    //velDer = velStd + control;

    velIzq = velStd - control;
    velDer = velStd + control;
    
    if(velIzq < 0) velIzq = 0;
    else if( velIzq > 255) velIzq = 255;
    
    if(velDer < 0)velDer = 0;
    else if(velDer > 255) velDer = 255;
    
    lastError = error;
  
    analogWrite(pwmIzq, velIzq);
    analogWrite(pwmDer, velDer);
    goto main;
  }
  
  
  left:{
    analogWrite(pwmIzq,0);
    analogWrite(pwmDer,255);
    if(map(analogRead(sensores[1]), black[1], white[1], 0, 100) > -1){
      lastError = 0;
      goto main;
    }
    goto left;
  }
  
  right:{
    analogWrite(pwmIzq,255);
    analogWrite(pwmDer,0);
    if(map(analogRead(sensores[4]), black[4], white[4], 0, 100) > -1){
      lastError = 0;
      goto main;
    }
    goto right;
  }
  goto main;
}
