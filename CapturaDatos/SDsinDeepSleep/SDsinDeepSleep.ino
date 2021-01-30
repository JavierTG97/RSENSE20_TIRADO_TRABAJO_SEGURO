// Import required libraries
#include <MPU9250_asukiaaa.h>
#include "WiFi.h"
//#include "ESPAsyncWebServer.h"
//#include "SPIFFS.h"
#include "time.h"
// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// --- VARIABLES QUE DETERMINAN EL MUESTREO ----
//Tiempo total de muestreo (en segundos):
const int ttotal = 20; //segundos
//Periodo de muestreo (en us):
const int period = 10000; // Cada 100.000 us (10 Hz)
const float period_s = 0.01; // lo mismo que lo anterior pero en seg
//Numero de muestras a tomar en base a lo anterior:
const int maxCont = round(ttotal*1000000/period);
//LED para saber que se están capturando datos:
const int ledPIN = 26; //Sera de utilidad para la toma de datos
//Condicion para realizar el muestreo:
bool cogerDatos = true;

// ------------ SD ----------------------
//Pin al que se conecta la tarjeta SD:
//#define SD_CS 5 //Creo que no hace falta ya que esta integrada en el ESP32
//Para el ID de los datos:
int readingID = 0;
//Dato en String que se guarda en el txt
String dataMessage;
//*************************************************
//Nombre del archivo que generamos y que 
//podemos usar para la clase de dato que guardamos:
String filename = "/pruebasincable.txt";
//*************************************************

// ------------- IMU (I2C) ---------------
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float dax=0.0, day=0.0, daz=0.0, iax=0.0, iay=0.0, iaz=0.0, norm; //Derivadas, integradores y modulo de la aceleracion
float aX_prev=0.0, aY_prev=0.0, aZ_prev = 0.0; 
//float offsetX, offsetY, offsetZ, modulo2;
//float aXtotal = 0.0, aYtotal = 0.0, aZtotal = 0.0;

// ------------- Wi-Fi -----------------
// Replace with your network credentials
const char* ssid = "OrdenadorJavier";
const char* password = "holaquetal";

// -------------- NTP ------------------
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;
int h, m, s; // Hora, minuto, segundo
String horaS;

// ---------- INTERRUPCION -------------
//Variable para detectar interrupción, y contador:
volatile int interruptCounter;
//Creamos un puntero que represente a un timer 
hw_timer_t * timer = NULL;
//Creamos una variable de tipo portMUX_type que nos permita sincronizar la variable volatile
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//Contador de interrupciones hechas:
int cont = 0;
//Interrupcion:
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux); 
}


// ------------- SETUP ----------------- 
void setup(){
  // **** SET-UP del puerto serie *****
  //Serial.begin(115200);
  //while(!Serial);
  delay(5000); //5 segundos anhadidos por si se quiere esperar a ejecutar lo de abajo 

  // ******** SET-UP del IMU **********
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
    mySensor.setWire(&Wire);
  #endif
  mySensor.beginAccel();
  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
  } else {
    //Serial.println("Cannot read accel values");
  }

  // ******** SET-UP del Wi-Fi **********
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    //Serial.println("Connecting to WiFi..");
  }

  // **** SET-UP de la tarjeta SD *****
  SD.begin(); 
  if(!SD.begin()) {
    //Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    //Serial.println("No SD card attached");
    return;
  }
  //Serial.println("Initializing SD card...");
  if (!SD.begin()) {
    //Serial.println("ERROR - SD card initialization failed!");
    return; // init failed
  }

  // **** SET-UP del archivo *****
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open(filename.c_str());
  if(!file) {
    //Serial.println("File doens't exist");
    //Serial.println("Creating file...");
    writeFile(SD, filename.c_str(), "ax;ay;az;temp;dax;day;daz;iax;iay;iaz;norm\r\n");
  }
  else {
    //Serial.println("File already exists");  
  }
  file.close();

  // ******** SET-UP del NTP **********
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // ******** SET-UP del timer **********
  timer = timerBegin(0, 80, true); //Por orden: asociamos el timer al canal 0, prescaler de 80 -> freq=80/80=1 MHz, y el conteo es en sentido ascendente.
  timerAttachInterrupt(timer, &onTimer, true); //Asignamos el timer a la interrip. onTimer, y la interrupción se genera tras producirse el flanco de timer correspondiente. 
  timerAlarmWrite(timer, period, true); //Interrupcion = 100.000 conteos (100 ms) e indicamos que cada vez que llegue a ese valor, el conteo se renueve.
  timerAlarmEnable(timer); //activamos la interrupción.

  // ****** SET-UP LED ******
  pinMode(ledPIN , OUTPUT); 
  delay(10000); // 10 segundos de espera para preparar el movimiento
  digitalWrite(ledPIN , HIGH); // Se enciende el LED avisando de que comienza la toma de datos
}

// ------------- LOOP -----------------  
void loop(){
  
  //Entramos a la interrupcion si han pasado 100 ms:
  if ((interruptCounter > 0) and (cogerDatos)) {
    //Inicializamos de nuevo la variable, para detectar de nuevo la itnerrupción en el if
    portENTER_CRITICAL(&timerMux);
    interruptCounter=0;
    portEXIT_CRITICAL(&timerMux);

    //Actualizamos el contador:
    ++cont;

    // ********** SENSOR (IMU) *********
    //Leemos aceleraciones:
    if (mySensor.accelUpdate() == 0) {
        aX = mySensor.accelX();      
        aY = mySensor.accelY();      
        aZ = mySensor.accelZ(); 
      } 
    else {
      //Serial.println("Cannot read accel values");
    }
    //Para Debugging:
    //Serial.println(String(aX) + " " + String(aY) + " " + String(aZ));

    // ******** LEEMOS HORA NTP ********
    getLocalTime(&timeinfo);
    h = timeinfo.tm_hour;
    m = timeinfo.tm_min;
    s = timeinfo.tm_sec;
    horaS = String(h) + String(m) + String(s);

    // ****** Calculamos las derivadas e integrales *****
    //Derivadas:
    if (cont > 1){
      dax = (aX - aX_prev)/period_s;
      day = (aY - aY_prev)/period_s;
      daz = (aZ - aZ_prev)/period_s;
    }
    aX_prev = aX;
    aY_prev = aY;
    aZ_prev = aZ;

    //Integrales:
    iax = iax + period_s*aX;
    iay = iay + period_s*aY;
    iaz = iaz + period_s*aZ;

    //Norma:
    norm = pow(aX,2.0) + pow(aY,2.0) + pow(aZ,2.0);

    // ****** ESCRIBIMOS EN LA SD ******
    // Aumentamos en 1 el identificador:
    readingID++;
    dataMessage = String(aX) + ";" + String(aY) + ";" + String(aZ) + ";" + 
                  String(horaS) + ";" + String(dax) + ";" + String(day) + ";" + 
                  String(daz) + ";" + String(iax) + ";" + String(iay) + ";" + 
                  String(iaz) + ";" + String(norm) + "\r\n";
    appendFile(SD, filename.c_str(), dataMessage.c_str());

    if (cont > maxCont) {
      cogerDatos = false;
      digitalWrite(ledPIN , LOW); // Se apaga el LED avisando de que termina la toma de datos
    }
    
  }
  
}

//---------------- FUNCIONES DE ESCRITURA EN LA SD -------------------
// Write to the SD card:
void writeFile(fs::FS &fs, const char * path, const char * message) {
  //Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    //Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    //Serial.println("File written");
  } else {
    //Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card:
void appendFile(fs::FS &fs, const char * path, const char * message) {
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    //Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
  }
  file.close();
}
