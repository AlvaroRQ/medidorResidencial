
#define MUESTRAS 64
// 5.799 sin carga 4.125 con carga <-(1000uF), 4.250V (2200uF) 5.025 para la fuente externa; Para el circuito prueba: 4.954
#define AREF 4.954

//#define UPDIV 7.4031

//19.91kohm/2.092Mohm -> 105.0728277
#define UPDIV 105.0728277

//23.43 para el mcp del medidor 
#define IDES 2
#define IDESP 20.5

// Recta fluke AN2I=0.023570225
#define AN2I 0.18844
#define AN2IP 0.036414
#define ISQR2 0.707106781
#define ICORR 1
#define VCORR 1
#define PI3 3.14159265
#include <LiquidCrystal.h>
#include <avr/wdt.h>
#include <util/delay.h>



// Inicializando:
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int voltaje[MUESTRAS]={0};
int corriente[MUESTRAS]={0};
int corrientePrecisa[MUESTRAS]={0};
// Factores de potencia
//double factor[8]={1,0.99803,0.99212,0.98229,0.96858,0.95106,0.92978,0.90483};//,0.87631,0.84433};
//double complementoFactor[8]={0,0.062790,0.12533,0.18738,0.24869,0.30902,0.36812,0.42578};//,0.48175,0.53583};
double potenciaActiva=0;
double potenciaReactiva=0;
boolean capacitivo = 0;

long tiempoInicial=0;
long tiempoFinal=0;

int voltajeMaximo=0;
double voltajeMaximoReal=0;
long sumaVoltaje=0;

int corrienteMaxima=0;
double corrienteMaximaReal=0;
long sumaCorriente=0;

int corrienteMaximaPrecisa=0;
double corrienteMaximaRealPrecisa=0;
long sumaCorrientePrecisa=0;

long tiempoInicialEnvio=0;
long tiempoFinalEnvio=0;
//int faseVoltaje=0;
//int faseCorriente=0;
long tiempoInicioDisplay=0;
long tiempoDisplay=0;

int alternador=0;

double energiaActiva=0;
double ultimoDestello=0;

long tiempoParpadeo=0;

void setup() {
  //Biblioteca Display
  lcd.begin(16,2);
  
  Serial.begin(9600);
  ADCSRA &= 0b11111110;     // Se modifica el prescaler para tener una mayor frecuencia de muestreo
  //watchdogSetup();
  pinMode(A3, OUTPUT);
  digitalWrite(A3,LOW);
}

void loop() {
  // Capturando valores
  tiempoInicial=micros();
  // Se tomaran 64 muestras bien distanciadas:
  for(int i = 0; i<MUESTRAS; i++){
    voltaje[i] = analogRead(A0);
    corriente[i] = analogRead(A1);
    corrientePrecisa[i]=analogRead(A2);
    delayMicroseconds(141);      //87 para dos entradas analogas
  }
  tiempoFinal=micros();

  //Estimación de los valores maximos y el tiempo
  // Voltaje:
  voltajeMaximo=0;
  sumaVoltaje=0;
  for(int i = 0; i<MUESTRAS; i++){
    sumaVoltaje+=voltaje[i];
    //if(voltaje[i]>voltajeMaximo){
      //voltajeMaximo = voltaje[i];
      //faseVoltaje=i+1;      // +1 para generar que la numeración inicie en 1 y no en 0, esto es relativo
    //} 
  }
  voltajeMaximo=(int)((double)(sumaVoltaje/MUESTRAS)*PI3);

  
  // Corriente:
  corrienteMaxima=0;
  sumaCorriente=0;
  for(int i = 0; i<MUESTRAS; i++){
    if((corriente[i]-IDES)>0){
      sumaCorriente=sumaCorriente+corriente[i]-IDES;
    }
  }
  corrienteMaxima=(int)((double)(sumaCorriente/MUESTRAS)*PI3);
  if(corrienteMaxima<0){
    corrienteMaxima=0;
  }

  // Corriente Precisa:
  corrienteMaximaPrecisa=0;
  sumaCorrientePrecisa=0;
  for(int i = 0; i<MUESTRAS; i++){
    if((corriente[i]-IDESP)>0){
      sumaCorrientePrecisa=sumaCorrientePrecisa+corrientePrecisa[i]-IDESP;
    }
  }
  corrienteMaximaPrecisa=(int)((double)(sumaCorrientePrecisa/MUESTRAS)*PI3);
  if(corrienteMaximaPrecisa<0){
    corrienteMaximaPrecisa=0;
  }

  //Los valores reales
  voltajeMaximoReal=(double)voltajeMaximo*AREF*UPDIV*VCORR/1023.0;
  corrienteMaximaReal=((double)corrienteMaxima)*AN2I*ICORR;
  corrienteMaximaRealPrecisa=((double)corrienteMaximaPrecisa)*AN2IP*ICORR;

  //Calculo de los valores requeridos:
/*
  //Factor de potencia:
  capacitivo=0;             // Por defecto se asume una carga resistiva o indictiva
  if(faseVoltaje>faseCorriente){
    capacitivo=1;
  }
  if(capacitivo){
    potenciaActiva=(double)voltajeMaximoReal*(double)corrienteMaximaReal*factor[faseVoltaje-faseCorriente]/2.0;
    potenciaReactiva=(double)voltajeMaximoReal*(double)corrienteMaximaReal*complementoFactor[faseVoltaje-faseCorriente]/2.0;
  }
  if(!capacitivo){
    potenciaActiva=(double)voltajeMaximoReal*(double)corrienteMaximaReal*factor[faseCorriente-faseVoltaje]/2.0;
    potenciaReactiva=(double)voltajeMaximoReal*(double)corrienteMaximaReal*complementoFactor[faseVoltaje-faseCorriente]/2.0;
  }
*/
  potenciaActiva=(double)voltajeMaximoReal*(double)corrienteMaximaReal*0.5;
  potenciaReactiva=(double)voltajeMaximoReal*(double)corrienteMaximaReal*0*0.5;


  energiaActiva += potenciaActiva*277.7778;     // La potencia se calculara en un segundo, esta variable almacenara los datos en uWh
  if(energiaActiva-ultimoDestello>600000){           // Inicialmente aproximadamentecada medio Wh, es decir 600000 uWh
    // Se almacena la energia activa en la EEPROM y se hace parpadear al LED
    digitalWrite(A3,HIGH);
    ultimoDestello=energiaActiva;
    tiempoParpadeo=micros();
  }
  if(micros()-tiempoParpadeo>100000){
    digitalWrite(A3,LOW);
  }

  tiempoInicialEnvio=micros();
  // Mostrando valores de voltaje
  Serial.print("Voltaje: {");
  for(int i = 0; i<MUESTRAS; i++){
    Serial.print(voltaje[i]);
    Serial.print(",");
  }
  Serial.println("}");

  // Mostrando valores de corriente
  Serial.print("Corriente: {");
  for(int i = 0; i<MUESTRAS; i++){
    Serial.print(corriente[i]);
    Serial.print(",");
  }
  Serial.println("}");

  // Mostrando valores de corriente Precisa
  Serial.print("Corriente Precisa: {");
  for(int i = 0; i<MUESTRAS; i++){
    Serial.print(corrientePrecisa[i]);
    Serial.print(",");
  }
  Serial.println("}");
  
  tiempoFinalEnvio=micros();
  
  //Periodo de lectura:
  Serial.print("Periodo de lectura [us]: ");
  Serial.println(tiempoFinal - tiempoInicial);
  Serial.print("Periodo de envio Serial [us]: ");
  Serial.println(tiempoFinalEnvio - tiempoInicialEnvio);
  
  Serial.print("El voltaje Maximo ADC: ");
  Serial.print(voltajeMaximo);
  //Serial.print(" a una fase de: ");
  //Serial.print(faseVoltaje);
  Serial.print(" El voltaje Eficaz Real: ");
  Serial.println(voltajeMaximoReal*ISQR2);

  Serial.print("La corriente Maxima ADC: ");
  Serial.print(corrienteMaxima);
  //Serial.print(" a una fase de: ");
  //Serial.print(faseCorriente);
  Serial.print(" La corriente Eficaz Real: ");
  Serial.println(corrienteMaximaReal*ISQR2);

  Serial.print("La corriente Maxima Precisa ADC: ");
  Serial.print(corrienteMaximaPrecisa);
  //Serial.print(" a una fase de: ");
  //Serial.print(faseCorriente);
  Serial.print(" La corriente Eficaz Real: ");
  Serial.println(corrienteMaximaRealPrecisa*ISQR2);

  Serial.print("Potencia Activa: ");
  Serial.print(potenciaActiva);
  Serial.println(" [W]");

  Serial.print("Potencia Reactiva: ");
  Serial.println(potenciaReactiva);
  Serial.println(" [VAR]");

  Serial.print("Energia Activa: ");
  Serial.print(energiaActiva);
  Serial.println(" [uWh]");

  tiempoInicioDisplay=micros();

  if((alternador/8)%2){
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("pA:");
    lcd.setCursor(4, 0);
    lcd.print(potenciaActiva);
    lcd.setCursor(13, 0);
    lcd.print("[W]");
    
    lcd.setCursor(0,1);
    lcd.print("E:");
    lcd.setCursor(4, 1);
    lcd.print(energiaActiva*0.000001);
    lcd.setCursor(12, 1);
    lcd.print("[Wh]");
  }
  else{
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("V: ");
    lcd.setCursor(3, 0);
    lcd.print(voltajeMaximoReal*ISQR2);
    lcd.setCursor(13, 0);
    lcd.print("[V]");
  
    lcd.setCursor(0,1);
    lcd.print("A: ");
    lcd.setCursor(3, 1);
    lcd.print(corrienteMaximaReal*ISQR2);
    lcd.setCursor(12, 1);
    lcd.print(" [A]");
  }
  tiempoDisplay=micros()-tiempoInicial;

  Serial.print("Tiempo de Muestra en Display:  ");
  Serial.println(tiempoDisplay);

  //_delay_ms (2000.0);
  //delay(6000);
  //wdt_reset(); to prevent the reset
  alternador++;
}

void watchdogSetup(){
  cli();
  wdt_reset();
//  WDTCR |=(1<<WDCE)|(1<<WDE); //WDCE =4 WDTCSR for atmega328
//  WDTCR = (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0); //WDIE = 6??      // Solo para atmega8
  sei();
  //atmega8 set 111 for wdt, resulta en un periodo de 1.778716667 seg
  // para el programa oficial se recurira a medición cada medio segundo
}
