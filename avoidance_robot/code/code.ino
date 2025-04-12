// CURSO ROBOT LABERINTO. CLUB DE ROBÓTICA DE LA FACULTAD DE INGENIERÍA.   Por: Adrián Ricárdez Ortigosa
#include <avr/wdt.h>    // Biblioteca del Watchdog Timer para evitar que se atasque infinitamente el robot

// =================================== PINES ======================================
const int Luz_pin[4] = {A3, A2, A5, A4};            // {lpin_supizq, lpin_supder, lpin_infder, lpin_infizq}
const int US_trigger_pin[3] = {6, 2, 4};            // {Trigger_izq, Trigger_cent, Trigger_pin}
const int US_echo_pin[3] = {7, 3, 5};               // {Echo1, Echo2, Echo3}
const int Motor_pin[4] = {9, 8, A1, A0};            // {Izq_Adel, Izq_Atr, Der_Adel, Der_Atr}
const int PWM_pin[2] = {11, 10};                    // {PWM_Izq, PWM_Der}
const int Boton = 12;                               // Botón contador de uso general
const int TESTIGO = 13;                             // LED testigo
int Luz_valor[4] = {0, 0, 0, 0};                    // Valor del pull down de las foto resistencias
int US_lectura_mm[3] = {0, 0, 0};                   // Distancias de obstáculos en mm
unsigned long Duration[3] = {0, 0, 0};              // Duración que tarda en llegar del Trigger al Echo
long rango_US[3] = {1800, 1800, 1800};              // Rango de detección de cada ultrasónico, con 1800 son 209 mm de detección aproximádamente
short valor_US_max = 209;                           // Valor máximo de detección en función del rango de detección en milímetros
short valor_US_choque = valor_US_max - 30;          // Valor de tolerancia máximo para poder decidir alguna rutina de giro antes de chocar
float PWM_izq = 0;                                  // Valor de la potencia izquierda
float PWM_der = 0;                                  // Valor de la potencia derecha
int vi_izq = 100;                                   // Velocidad inicial izquierda
int vi_der = 100;                                   // Velocidad inicial derecha
// Constantes de proporcion de ley de control
const float K_dist = 1.26;
const float K_dist_c = 0.2;
const float K_luz = 0.18;
// ================================================================================

// ================================ VOID SETUP ====================================
void setup()
{
  // Entradas
  for (int i = 0; i < 3; i++)
    pinMode(Luz_pin[i], INPUT);
  for(int i = 0; i < 3; i++)
    pinMode(US_echo_pin[i],INPUT);
  pinMode(Boton, INPUT);

  // Salidas
  for (int i = 0; i < 4; i++)
    pinMode(Motor_pin[i], OUTPUT);
  for (int i= 0; i < 3; i++)
    pinMode(US_trigger_pin[i],OUTPUT);

  pinMode(PWM_pin[0], OUTPUT);
  pinMode(PWM_pin[1], OUTPUT);
  pinMode(TESTIGO, OUTPUT);

  Serial.begin(115200);   // Configurar este en el monitor serial en la esquina inferior derecha

  // Potencia inicial
  analogWrite(PWM_pin[0], 0);
  analogWrite(PWM_pin[1], 0);

  delay(1000);    // Delay de inicio

  // Rutina inicial
  Dir_Atr();
  analogWrite(PWM_pin[0], vi_izq);
  analogWrite(PWM_pin[1], vi_der);
  delay(450);
  Dir_Izq();
  delay(500);

  wdt_enable(WDTO_8S);    // Comienza a contar hasta 8 segundos y se resetea el robot si no detecta demasiado cerca alguna pared
}
// ================================================================================

// ------------------------------------- FUNCIONES ---------------------------------------------
void Dir_Adel()   // Cambio de dirección hacia adelante
{
  digitalWrite(Motor_pin[0], HIGH);
  digitalWrite(Motor_pin[1], LOW);
  digitalWrite(Motor_pin[2], HIGH);
  digitalWrite(Motor_pin[3], LOW);
}

void Dir_Atr()    // Cambio de dirección hacia atrás
{
  digitalWrite(Motor_pin[0], LOW);
  digitalWrite(Motor_pin[1], HIGH);
  digitalWrite(Motor_pin[2], LOW);
  digitalWrite(Motor_pin[3], HIGH);
}

void Dir_Der()    // Cambio de dirección hacia la derecha
{
  digitalWrite(Motor_pin[0], LOW);
  digitalWrite(Motor_pin[1], HIGH);
  digitalWrite(Motor_pin[2], HIGH);
  digitalWrite(Motor_pin[3], LOW);
}

void Dir_Izq()    // Cambio de dirección hacia la izquierda
{
  digitalWrite(Motor_pin[0], HIGH);
  digitalWrite(Motor_pin[1], LOW);
  digitalWrite(Motor_pin[2], LOW);
  digitalWrite(Motor_pin[3], HIGH);
}

void Retroceso_Der()    // Rutina con giro hacia la derecha
{
  digitalWrite(TESTIGO,HIGH);
  Dir_Atr();
  analogWrite(PWM_pin[0], vi_izq);
  analogWrite(PWM_pin[1], vi_der);
  delay(450);
  Dir_Der();
  delay(500);
  wdt_reset();    // Se le dice al watchdog que todo está bien y se resetea el contador
}

void Retroceso_Izq()    // Rutina con giro hacia la izquierda
{
  digitalWrite(TESTIGO,HIGH);
  Dir_Atr();
  analogWrite(PWM_pin[0], vi_izq);
  analogWrite(PWM_pin[1], vi_der);
  delay(450);
  Dir_Izq();
  delay(500);
  wdt_reset();    // Se le dice al watchdog que todo está bien y se resetea el contador
}

void Navegacion()   // Rutina de navegación (aquí se realiza el control)
{
  digitalWrite(TESTIGO,LOW);    // Apagamos el LED siempre que no choquemos
  
  // Ley de Control
  PWM_izq = vi_izq + K_dist*(US_lectura_mm[0] - US_lectura_mm[2]) + K_luz*( - Luz_valor[0] + Luz_valor[1] + Luz_valor[2] - Luz_valor[3]);
  PWM_der = vi_der + K_dist*(US_lectura_mm[2] - US_lectura_mm[0]) + K_luz*( Luz_valor[0] - Luz_valor[1] - Luz_valor[2] + Luz_valor[3]);
  
  // Suavizado de navegación por medio del ultrasónico frontal para que disminuya su velocidad si va a chocar de frente
  if(PWM_izq > 0)
    PWM_izq -= K_dist_c*US_lectura_mm[1];
  else if(PWM_izq < 0)
    PWM_izq += K_dist_c*US_lectura_mm[1];
  if(PWM_der > 0)
    PWM_der -= K_dist_c*US_lectura_mm[1];
  else if(PWM_der < 0)
    PWM_der += K_dist_c*US_lectura_mm[1];

  // Cambios de dirección, en -255 hacia atrás, en 255 hacia adelante por cada motor
  if (PWM_der > 0 && PWM_izq > 0)
    Dir_Adel();
  else if (PWM_der > 0 && PWM_izq < 0)
    Dir_Izq();
  else if (PWM_izq > 0 && PWM_der < 0)
    Dir_Der();
  else if (PWM_izq < 0 && PWM_der < 0)
    Dir_Atr();

  Serial.print("  ");Serial.print(PWM_izq);Serial.print("  ");Serial.println(PWM_der);    // Impresión de datos de potencias
  
  // Límites de velocidades, rango de PWM [-255,255] por cada motor para estancar el número
  if (PWM_izq < 0)
    PWM_izq = PWM_izq * (-1);
  if (PWM_der < 0)
    PWM_der = PWM_der * (-1);
  if (PWM_izq > 255)
    PWM_izq = 255;
  if (PWM_der > 255)
    PWM_der = 255;

  // Escritura de potencia en el L293D
  analogWrite(PWM_pin[0], PWM_izq);
  analogWrite(PWM_pin[1], PWM_der);
}

long fDistancia(long tiempo)    // Cálculo de la distancia en mm
{
  // ((tiempo)*(Velocidad del sonido)/ el camino se hace dos veces)
  long DistanceCalc; // Variable para los cálculos
  DistanceCalc = (tiempo / 2.9) / 2; // Cálculos en milímetros
  return DistanceCalc; // Devolvemos el calculo
}
// --------------------------------------------------------------------------------------------------

// ======================================================= VOID LOOP ================================================================
void loop()
{ 
  // Obtención de los valores de las fotorresistencias
  for(int i = 0; i < 4; i++)
    Luz_valor[i] = analogRead(Luz_pin[i]);

  // Calibración de offsets de luz natural (no es necesario, la prueba será a obscuridad total)
  /*Luz_valor[0] -= 300;
  Luz_valor[1] -= 480;
  Luz_valor[2] -= 450;
  Luz_valor[3] -= 520;*/

  // Estancamiento de valores de luz, ya que no hay luz negativa (en programación)
  if(Luz_valor[0] < 0)
    Luz_valor[0] = 0;
  if(Luz_valor[1] < 0)
    Luz_valor[1] = 0;
  if(Luz_valor[2] < 0)
    Luz_valor[2] = 0;
  if(Luz_valor[3] < 0)
    Luz_valor[3] = 0;

  // Obtención de los valores de los ultrasónicos
  for(int i = 0;i < 3; i++)
  {
    // Obtención de datos del sonar
    digitalWrite(US_trigger_pin[i],HIGH);
    delayMicroseconds(10);
    digitalWrite(US_trigger_pin[i],LOW);
    Duration[i] = pulseIn(US_echo_pin[i], HIGH, rango_US[i]);
    US_lectura_mm[i] = fDistancia(Duration[i]);
    // Cambio de valores a complementario para suavizar comportamiento US
    if( US_lectura_mm[i] != 0) 
      US_lectura_mm[i] = valor_US_max - US_lectura_mm[i];
  }

    // Estancamiento de valores de distancia, ya que no hay distancias negativas
    if(US_lectura_mm[0] < 0)
      US_lectura_mm[0] = 0;
    if(US_lectura_mm[1] < 0)
      US_lectura_mm[1] = 0;
    if(US_lectura_mm[2] < 0)
      US_lectura_mm[2] = 0;
  
  // Impresión de valores de distancia y luz
  Serial.print("USizq UScen USder Lsupizq Lsupder Linfder Linfizq PWM_izq PWM_der: ");
  Serial.print(US_lectura_mm[0]);Serial.print("  ");Serial.print(US_lectura_mm[1]);Serial.print("  ");Serial.print(US_lectura_mm[2]);Serial.print("  ");
  Serial.print(Luz_valor[0]);Serial.print("  ");Serial.print(Luz_valor[1]);Serial.print("  ");Serial.print(Luz_valor[2]);Serial.print("  ");Serial.print(Luz_valor[3]);

  // Si esta muy cerca de algún objeto, realizar rutina de rescate, o rutina de navegación si todo está bien
  if(US_lectura_mm[0] > valor_US_choque)
    Retroceso_Der();
  else if(US_lectura_mm[2] > valor_US_choque)
    Retroceso_Izq();
  // Los siguientes dos casos son tipo "flip"
  else if(US_lectura_mm[1] > valor_US_choque && (US_lectura_mm[0] - US_lectura_mm[2]) <= 0)
    Retroceso_Izq();
  else if(US_lectura_mm[1] > valor_US_choque && (US_lectura_mm[0] - US_lectura_mm[2]) >= 0)
    Retroceso_Der();
  // Si está evadiendo sin estar en "peligro" de atascarse, que siga la luz y evada paredes
  else
    Navegacion();
}

// ==================================================================================================================

// CURSO ROBOT LABERINTO. CLUB DE ROBÓTICA DE LA FACULTAD DE INGENIERÍA.   Por: Adrián Ricárdez Ortigosa
