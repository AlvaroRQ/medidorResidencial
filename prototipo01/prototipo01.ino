/*EN ESTE PROTOTIPO SE TIENE UN RUIDO FANTASMA EN EL AMPLIFICADOR NORMAL, EL AMPLIFICADOR DE RPESICIÓN ESTA FUNCIONANDO CON UN VALOR DE GANANCIA ANOMALO
 * Y CON UN DESFASE QUE SERÁ CORREGIDO EN EL PROGRAMA SIGUIENTE, EL OFFSET DEL MISMO TAMBIEN ES DEMASIADO ELEVADO
 El ruido se debe a que el medidor presente es asimétrico*/
/*aMPLIFICADOR NORMAL ANULADO*/
#define MUESTRAS 64
// 5.799 sin carga 4.125 con carga <-(1000uF), 4.250V (2200uF) 5.025 para la fuente externa; Para el circuito prueba: 4.954
#define AREF 4.71
//2.069moHM, 19.74kOhm

//19.91kohm/2.092Mohm -> 105.0728277
#define UPDIV 79.183196

//Prototipo 2:

#define IDES 0
#define IDESP 134

#define AN2I 0
#define AN2IP 0.0232902

#define IMINP 11

#define ISQR2 0.707106781
#define ICORR 1
#define VCORR 1
#define PI3 3.14159265

//***************************** DEFINICIÓN DE PINES ****************************************/
#define LED1 10
#define LED2 9
#define BK_LCD A3
#define SWITCH1 6
#define SWITCH2 7
#define BUTTON2 8

#include <LiquidCrystal.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <EEPROM.h>


// Inicializando:
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int voltaje[MUESTRAS] = {0};
int corriente[MUESTRAS] = {0};
int corrientePrecisa[MUESTRAS] = {0};

int8_t faseAuxiliar = 0;
double factorPotencia = 1;
double factorPotenciaComplemento = 0;
double potenciaActiva = 0;
double potenciaReactiva = 0;
boolean capacitivo = 0;

int voltajeMaximo = 0;
double voltajeMaximoReal = 0;
long sumaVoltaje = 0;

int corrienteMaxima = 0;
double corrienteMaximaReal = 0;
long sumaCorriente = 0;

int corrienteMaximaPrecisa = 0;
double corrienteMaximaRealPrecisa = 0;
long sumaCorrientePrecisa = 0;

// Temporizaciones:
long tiempoInicial = 0;     // numero grande
unsigned int tiempoMuestreo = 0;
unsigned int tiempoEnergia = 0;
unsigned int tiempoParametros =0;
unsigned int tiempoDisplay = 0;
long tiempoSerial = 0;

int alternador = 0;


double energiaActiva = 0;
double ultimoDestello = 0;

long tiempoParpadeo = 0;

double vLCD=0;
double iLCD=0;
double pLCD=0;
double eLCD=0;

void setup() {
  //Biblioteca Display
  lcd.begin(16, 2);

  Serial.begin(9600);
  ADCSRA &= 0b11111110;     // Se modifica el prescaler para tener una mayor frecuencia de muestreo
  //watchdogSetup();
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);

  // Modo normal y modo Debug, LOW MODO NORMAL, HIGH modo debug
  pinMode(SWITCH1,INPUT);
  // en modo normal, tiene que esperar a los 40ms, tiene que calcular la potencia de modo diferente
  // tiene que usar una nueva variable para el lcd y tiene que bloquear el serial

  //Para regular la luz de fondo se dejara el pin en nivel alto
  pinMode(BK_LCD, OUTPUT);
  digitalWrite(BK_LCD, HIGH);
  // prototipo 1:
  //digitalWrite(BK_LCD, LOW);
}

void loop() {
  /*************************************************************************** ADQUISICIÓN DE DATOS ******************************************************************************/
  tiempoInicial = micros();
  // Se tomaran 64 muestras bien distanciadas:
  for (int i = 0; i < MUESTRAS; i++) {
    voltaje[i] = analogRead(A0);
    corriente[i] = analogRead(A1);
    corrientePrecisa[i] = analogRead(A2);
    delayMicroseconds(141);      //87 para dos entradas analogas
  }
  tiempoMuestreo = micros()-tiempoInicial;

  /******************************************************************** PARÁMETROS DE VOLTAJE CORRIENTE Y FASE ********************************************************************/

  // Se inicializa aqui dentro el factor de potencia
  boolean faseVoltaje[MUESTRAS] = {0};
  boolean faseCorriente[MUESTRAS] = {0};

  //Estimación de los valores maximos y el tiempo
  // VOLTAJE
  voltajeMaximo = 0;
  sumaVoltaje = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    sumaVoltaje += voltaje[i];
    if (voltaje[i] == 0) {
      faseVoltaje[i] = true;
    }
  }
  voltajeMaximo = (int)((double)(sumaVoltaje / MUESTRAS) * PI3);


  // CORRIENTE:
  corrienteMaxima = 0;
  sumaCorriente = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    if ((corriente[i] - IDES) > 0) {
      sumaCorriente = sumaCorriente + corriente[i] - IDES;
    }
  }
  corrienteMaxima = (int)((double)(sumaCorriente / MUESTRAS) * PI3);

  //corrienteMaxima = corrienteMaxima-IMINP;

  //La corriente no puede ser negativa:
  if (corrienteMaxima < 0) {
    corrienteMaxima = 0;
  }

  // Corriente Precisa:
  corrienteMaximaPrecisa = 0;
  sumaCorrientePrecisa = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    if ((corrientePrecisa[i] - IDESP) > 0) {
      sumaCorrientePrecisa = sumaCorrientePrecisa + corrientePrecisa[i] - IDESP;
    }
    if (corrientePrecisa[i] - IDESP <= 0) {
      faseCorriente[i] = true;
    }
  }
  corrienteMaximaPrecisa = (int)((double)(sumaCorrientePrecisa / MUESTRAS) * PI3);
  
  //La corriente no puede ser negativa:
  if (corrienteMaximaPrecisa < 0) {
    corrienteMaximaPrecisa = 0;
  }

  //Factor de potencia:
  faseAuxiliar = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    if (faseVoltaje[i]&faseCorriente[i]) {
      faseAuxiliar++;
    }
  }
  // Se invierte la fase auxiliar por conveniencia:
  faseAuxiliar = 32 - faseAuxiliar;

  
  // Determinando el tipo de carga reactiva:
  int8_t contadorTipo = 0;
  int8_t cambioVoltaje = MUESTRAS;
  int8_t cambioCorriente = MUESTRAS;
  while(MUESTRAS/2-contadorTipo){
    if(faseVoltaje[contadorTipo]!=faseVoltaje[contadorTipo+1]){
      cambioVoltaje = contadorTipo;
    }
    if(faseCorriente[contadorTipo]!=faseCorriente[contadorTipo+1]){
      cambioCorriente = contadorTipo;
    }
    contadorTipo++;
    if(cambioCorriente<cambioVoltaje){
      capacitivo=true;
      contadorTipo = MUESTRAS/2;
    }
    if(cambioCorriente>cambioVoltaje){
      capacitivo=false;
      contadorTipo = MUESTRAS/2;
    }
  }

  tiempoParametros = micros() - tiempoInicial-tiempoMuestreo;
  
/********************************************************************** VALORES REALES INSTANTÁNEOS **********************************************************************/

  voltajeMaximoReal = (double)voltajeMaximo * AREF * UPDIV * VCORR / 1023.0;
  corrienteMaximaReal = ((double)corrienteMaxima) * AN2I * ICORR;
  corrienteMaximaRealPrecisa = ((double)corrienteMaximaPrecisa) * AN2IP * ICORR;

  factorPotencia = 1.0 - ((double)(faseAuxiliar * PI3) * 0.03125) * ((double)(faseAuxiliar * PI3) * 0.03125) * 0.5 + ((double)(faseAuxiliar * PI3) * 0.03125) * ((double)(faseAuxiliar * PI3) * 0.03125) * ((double)(faseAuxiliar * PI3) * 0.03125) * ((double)(faseAuxiliar * PI3) * 0.03125) * 0.041667;
  factorPotenciaComplemento = ((double)(faseAuxiliar * PI3) * 0.03125) - ((double)(faseAuxiliar * PI3) * 0.03125) * ((double)(faseAuxiliar * PI3) * 0.03125) * ((double)(faseAuxiliar * PI3) * 0.03125) * 0.166667;

  // Filtrando el ruido para el factor de potencia:
  if (corrienteMaximaRealPrecisa == 0) {
    factorPotencia = 1.0;
    factorPotenciaComplemento = 0;
    capacitivo = 0;
  }

/************************************** POTENCIA ***************************************/
  //Para los calculos se descarta la corriente de menor precisión.
  if(corrienteMaximaReal<22){
    potenciaActiva = (double)voltajeMaximoReal * (double)corrienteMaximaRealPrecisa * factorPotencia * 0.5;
    potenciaReactiva = (double)voltajeMaximoReal * (double)corrienteMaximaRealPrecisa * factorPotenciaComplemento * 0.5;
  }
  else{
    potenciaActiva = (double)voltajeMaximoReal * (double)corrienteMaximaReal * factorPotencia * 0.5;
    potenciaReactiva = (double)voltajeMaximoReal * (double)corrienteMaximaReal * factorPotenciaComplemento * 0.5;
  }

/************************************** ENERGÍA ***************************************/
  
/****NORMAL-DEBUG****/
// Si esta en modo NORMAL la potencia se calcula en 40ms
// si esta en modo DEBUG, La potencia se calculara en un segundo, esta variable almacenara los datos en uWh
  if(!digitalRead(6)){
    energiaActiva = energiaActiva + potenciaActiva * 11.111111;
  }
  else{
    energiaActiva += potenciaActiva * 277.7778;
  }

  tiempoEnergia = micros() - tiempoInicial-tiempoMuestreo-tiempoParametros;
/********************************************************************** CONTROL Y VISUALIZACIÓN **********************************************************************/
  if (energiaActiva - ultimoDestello > 600000) {     // Inicialmente aproximadamentecada medio Wh, es decir 600000 uWh
    // Se almacena la energia activa en la EEPROM y se hace parpadear al LED
    digitalWrite(LED1, HIGH);
    ultimoDestello = energiaActiva;
    tiempoParpadeo = micros();
  }
  if (micros() - tiempoParpadeo > 25000) {
    digitalWrite(LED1, LOW);
  }

/************************************** LCD ***************************************/
/**********NORMAL DEBUG**********/
int8_t parametroAlternancia=0;
if(!digitalRead(6)){parametroAlternancia=400;}
else{parametroAlternancia=8;}

  if((alternador%25)==0){
// Luego de 50 alteraciones se promedia:
    iLCD=iLCD/25;
    vLCD=vLCD/25;
    pLCD=pLCD/25;

// Se muestra;
  if ((alternador / parametroAlternancia) % 2) {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("pA:");
    lcd.setCursor(4, 0);
    lcd.print(pLCD);    //potenciaActiva
    lcd.setCursor(13, 0);
    lcd.print("[W]");

    lcd.setCursor(0, 1);
    lcd.print("E:");
    lcd.setCursor(4, 1);
    lcd.print(energiaActiva * 0.000001);
    lcd.setCursor(12, 1);
    lcd.print("[Wh]");
  }
  else {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("V: ");
    lcd.setCursor(3, 0);
    lcd.print(vLCD);    //voltajeMaximoReal * ISQR2
    lcd.setCursor(13, 0);
    lcd.print("[V]");

    lcd.setCursor(0, 1);
    lcd.print("A: ");
    lcd.setCursor(3, 1);

    lcd.print(1000*iLCD);    //1000*corrienteMaximaRealPrecisa * ISQR2
    lcd.setCursor(11, 1);
    lcd.print(" [mA]");
  }
// Se resetea
    iLCD=0;
    vLCD=0;
    pLCD=0;
  }
  else{
    vLCD+=voltajeMaximoReal * ISQR2;
    if(corrienteMaximaReal<22){
      iLCD+=corrienteMaximaRealPrecisa * ISQR2;
    }
    else{
      iLCD+=corrienteMaximaReal * ISQR2;
    }
    pLCD+=potenciaActiva;
    }
  
  alternador++;
  tiempoDisplay = micros() - tiempoInicial-tiempoMuestreo-tiempoParametros-tiempoEnergia;

/********************************************************************** DEBUG SERIAL **********************************************************************/
/****NORMAL-DEBUG****/
if(!digitalRead(6)){
  Serial.println(voltajeMaximoReal * ISQR2);
  Serial.println(corrienteMaximaReal * ISQR2);
  Serial.println(corrienteMaximaRealPrecisa * ISQR2);
  Serial.println(factorPotencia);
  Serial.println(potenciaActiva);
  Serial.println(energiaActiva);
  while(micros()-tiempoInicial<40000){
    true;
    }
  }
  else{
  // Mostrando valores de voltaje
  Serial.print("Voltaje: {");
  for (int i = 0; i < MUESTRAS; i++) {
    Serial.print(voltaje[i]);
    Serial.print(",");
  }
  Serial.println("}");

  // Mostrando valores de corriente
  Serial.print("Corriente: {");
  for (int i = 0; i < MUESTRAS; i++) {
    Serial.print(corriente[i]);
    Serial.print(",");
  }
  Serial.println("}");

  // Mostrando valores de corriente Precisa
  Serial.print("Corriente Precisa: {");
  for (int i = 0; i < MUESTRAS; i++) {
    Serial.print(corrientePrecisa[i]);
    Serial.print(",");
  }
  Serial.println("}");

  Serial.print("El voltaje Maximo ADC: ");
  Serial.print(voltajeMaximo);
  //Serial.print(" a una fase de: ");
  //Serial.print(faseVoltaje);
  Serial.print(" El voltaje Eficaz Real: ");
  Serial.println(voltajeMaximoReal * ISQR2);

  Serial.print("La corriente Maxima ADC: ");
  Serial.print(corrienteMaxima);
  //Serial.print(" a una fase de: ");
  //Serial.print(faseCorriente);
  Serial.print(" La corriente Eficaz Real: ");
  Serial.print(corrienteMaximaReal * ISQR2);
  Serial.println(" [A] ");

  Serial.print("La corriente Maxima Precisa ADC: ");
  Serial.print(corrienteMaximaPrecisa);
  //Serial.print(" a una fase de: ");
  //Serial.print(faseCorriente);
  Serial.print(" La corriente Eficaz Real: ");
  Serial.print(1000*corrienteMaximaRealPrecisa * ISQR2);
  Serial.println(" [mA] ");

  Serial.print("Factor de potencia: ");
  Serial.print(faseAuxiliar);
  Serial.print(" correspondiente a: ");
  Serial.print(factorPotencia);
  Serial.print(" (c.");
  Serial.print(factorPotenciaComplemento);
  Serial.print(") de tipo ");
  if (capacitivo) {
    Serial.println("Capacitivo");
  }
  else {
    Serial.println("Inductivo");
  }

  Serial.print("Potencia Activa: ");
  Serial.print(potenciaActiva);
  Serial.println(" [W]");

  Serial.print("Potencia Reactiva: ");
  Serial.print(potenciaReactiva);
  Serial.println(" [VAR]");

  Serial.print("Energia Activa: ");
  Serial.print(energiaActiva);
  Serial.println(" [uWh]");

  //Periodos de lectura:
  Serial.print("Periodo [us]: ");
  Serial.println(tiempoMuestreo);
  Serial.print("Parametros [us]: ");
  Serial.println(tiempoParametros);
  Serial.print("Calculo de valor real [us]: ");
  Serial.println(tiempoEnergia);
  Serial.print("Tiempo control y LCD [us]: ");
  Serial.println(tiempoDisplay);

  Serial.print("Periodo de envio Serial [us]: ");
  tiempoSerial = micros() - tiempoInicial-tiempoMuestreo-tiempoParametros-tiempoEnergia-tiempoDisplay;
  Serial.println(tiempoSerial);
  }
  //_delay_ms (2000.0);
  //delay(6000);
  //wdt_reset(); to prevent the reset
}
/*
void watchdogSetup() {
  cli();
  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE); //WDCE =4 WDTCR for atmega8
  WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0); //WDIE = 6??      // Solo para atmega8
  sei();
  //atmega8 set 111 for wdt, resulta en un periodo de 1.778716667 seg
  // para el programa oficial se recurira a medición cada medio segundo
}*/

void EEPROM_writeDouble(int ee, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
}

double EEPROM_readDouble(int ee)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return value;
}

/*
  void guardarDoubleEEPROM(int place, double value){
  byte four= (value& 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  EEPROM.write(place, four);
  EEPROM.write(place+1, three);
  EEPROM.write(place+2, two);
  EEPROM.write(place+3, one);
  }
  double leerDoubleEEPROM(int place){
  double value=0;
  value|=EEPROM.read(place);
  value|=(EEPROM.read(place+1))<<8;
  value|=(EEPROM.read(place+2))<<16;
  value|=(EEPROM.read(place+3))<<24;
  }*/

