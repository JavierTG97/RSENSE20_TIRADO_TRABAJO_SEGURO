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
const int ttotal = 400; //segundos
//Periodo de muestreo (en us):
const int period = 50000; // Cada 100.000 (us) -> (10 Hz)
const float period_s = float(period)/1000000.0; // lo mismo que lo anterior pero en seg
//Numero de muestras a tomar en base a lo anterior:
const int maxCont = round(ttotal*1000000/period);
//LED para saber que se están capturando datos:
const int ledPIN = 26; //Sera de utilidad para la toma de datos
//Condicion para realizar el muestreo:
bool cogerDatos = true;
//Tamaño de ventana (numero de segundos y muestras):
const int w_seg = 4; //segundos
const int w_muestras = round(float(w_seg)/period_s); //muestras 

// ------------ SD ----------------------
//Pin al que se conecta la tarjeta SD:
//#define SD_CS 5 //Creo que no hace falta ya que esta integrada en el ESP32
//Dato en String que se guarda en el txt
String dataMessage = "";
String encabezado;
//*************************************************
//Principio del nombre del archivo que generamos y que 
//podemos usar para la clase de dato que guardamos:
String filename = "/sent2.txt";
//*************************************************

// ------------- IMU (I2C) ---------------
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float asumax=0.0, asumay=0.0, asumaz=0.0, asumax2=0.0, asumay2=0.0, asumaz2=0.0; // para la desviacion tipica de la ventana A
float bsumax=0.0, bsumay=0.0, bsumaz=0.0, bsumax2=0.0, bsumay2=0.0, bsumaz2=0.0; // para la desviacion tipica de la ventana B
float abuffer[w_muestras*3+3]; // Ventana A de 2 segundos -> 20 muestras, 20x3ejes = 60, 60 + 3 de desv. tipica.
float bbuffer[w_muestras*3+3]; // Ventana B de 2 segundos -> 20 muestras, 20x3ejes = 60, 60 + 3 de desv. tipica.

int cont_buffer1 = 0;
int cont_buffer2 = 0;
bool inicio = true; //para añadir el desfase entre buffers

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

  // ******** LEEMOS HORA NTP ********
    getLocalTime(&timeinfo);
    h = timeinfo.tm_hour;
    m = timeinfo.tm_min;
    s = timeinfo.tm_sec;
    horaS = String(h) + String(m) + String(s);

  //**** Completamos nombre del archivo ****
  //filename = filename + horaS + ".txt";

  // **** SET-UP del archivo *****
  // Hacemos el encabezado:
  encabezado = "ax1;ay1;az1;";
  for (int i = 2; i <= w_muestras; i++) {
    encabezado = encabezado + "ax" + String(i) + ";ay" + String(i) + ";az" + String(i) + ";" ;
  }
  encabezado = encabezado + "varx;vary;varz" ;
  encabezado = encabezado + "\r\n" ;
  
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open(filename.c_str());
  if(!file) {
    //Serial.println("File doens't exist");
    //Serial.println("Creating file...");
    writeFile(SD, filename.c_str(), encabezado.c_str());
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

    //**** ACTUALIZAMOS BUFFERS Y SUS CONTADORES *****    
    abuffer[cont_buffer1]   = aX;
    abuffer[cont_buffer1+1] = aY;
    abuffer[cont_buffer1+2] = aZ;
    cont_buffer1 = cont_buffer1 + 3;

    if ((cont_buffer1 == ((w_muestras*3/2) + 3)) and (inicio)){ //Aqui se añade el desfase necesario
      cont_buffer2 = 0;
      bsumax = 0.0;
      bsumay = 0.0;
      bsumaz = 0.0;
      bsumax2 = 0.0;
      bsumay2 = 0.0;
      bsumaz2 = 0.0;
      inicio = false;
    }
    
    bbuffer[cont_buffer2]   = aX;
    bbuffer[cont_buffer2+1] = aY;
    bbuffer[cont_buffer2+2] = aZ;
    cont_buffer2 = cont_buffer2 + 3;

    //**** CÁLCULOS ASOCIADOS A LA VARIANZA DE LOS DATOS EN LA VENTANA *****
    asumax = asumax + aX;
    asumay = asumay + aY;
    asumaz = asumaz + aZ;
    bsumax = bsumax + aX;
    bsumay = bsumay + aY;
    bsumaz = bsumaz + aZ;

    asumax2 = asumax2 + pow(aX,2.0);
    asumay2 = asumay2 + pow(aY,2.0);
    asumaz2 = asumaz2 + pow(aZ,2.0);
    bsumax2 = bsumax2 + pow(aX,2.0);
    bsumay2 = bsumay2 + pow(aY,2.0);
    bsumaz2 = bsumaz2 + pow(aZ,2.0);    

     
    // ****** ESCRIBIMOS EN LA SD ******
    if (cont_buffer1 == (w_muestras*3)){
      //Varianza:
      abuffer[w_muestras*3]   = asumax2/float(w_muestras) - (pow(asumax,2.0)/pow(float(w_muestras),2.0));
      abuffer[w_muestras*3+1] = asumay2/float(w_muestras) - (pow(asumay,2.0)/pow(float(w_muestras),2.0));
      abuffer[w_muestras*3+2] = asumaz2/float(w_muestras) - (pow(asumaz,2.0)/pow(float(w_muestras),2.0));

      //Mensaje a escribir:
      dataMessage = String(abuffer[0]);
      for (int i = 1; i <= (w_muestras*3+2); i++) {
        dataMessage = dataMessage + ";" + String(abuffer[i]) ;
      }
      dataMessage = dataMessage + "\r\n" ;

      appendFile(SD, filename.c_str(), dataMessage.c_str());
      cont_buffer1 = 0;
      asumax2 =0.0; asumay2 =0.0; asumaz2 =0.0;
      asumax =0.0; asumay =0.0; asumaz =0.0;
      dataMessage = "";
    }

    if (cont_buffer2 == (w_muestras*3)){
      //Varianza:
      bbuffer[w_muestras*3]   = bsumax2/float(w_muestras) - (pow(bsumax,2.0)/pow(float(w_muestras),2.0));
      bbuffer[w_muestras*3+1] = bsumay2/float(w_muestras) - (pow(bsumay,2.0)/pow(float(w_muestras),2.0));
      bbuffer[w_muestras*3+2] = bsumaz2/float(w_muestras) - (pow(bsumaz,2.0)/pow(float(w_muestras),2.0));

      //Mensaje a escribir:
      dataMessage = String(bbuffer[0]);
      for (int i = 1; i <= (w_muestras*3+2); i++) {
        dataMessage = dataMessage + ";" + String(bbuffer[i]) ;
      }
      dataMessage = dataMessage + "\r\n" ;
      
      appendFile(SD, filename.c_str(), dataMessage.c_str());
      cont_buffer2 = 0;
      bsumax2 =0.0; bsumay2 =0.0; bsumaz2 =0.0;
      bsumax =0.0; bsumay =0.0; bsumaz =0.0;
      dataMessage = "";
    }
    
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
