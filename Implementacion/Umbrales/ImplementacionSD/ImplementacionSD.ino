/* Redes de Sensores Electronicos
 * Implementacion usada para la prediccion de los movimientos
 */

// LIBRERIAS:
#include <MPU9250_asukiaaa.h>                //Acelerometro
#include "WiFi.h"                            //WiFi
#include "time.h"                            //Hora
#include "FS.h"                              //SD
#include "SD.h"                              //SD
#include <SPI.h>                             //SD

// VARIABLES CAPTURA DE DATOS
const int ttotal = 40;                                 // Tiempo total (segundos) de captura de datos
const int period = 50000;                              // Tmuestreo en useg (20 Hz)
const float period_s = float(period)/1000000.0;        // Tmuestreo en seg  (20 hz)
const int maxCont = round(ttotal*1000000/period);      // Numero de muestras a tomar
const int w_seg = 4;                                   // Tiempo en seg de cada ventana
const int w_muestras = round(float(w_seg)/period_s);   // Num. de veces que se capturan datos por ventana
int cont_buffer1 = 0;                                  // Contador para controlar abuffer
int cont_buffer2 = 0;                                  // Contador para controlar bbuffer
bool inicio = true;                                    // Sirve para desfasar un 50% los buffers
const int ledPIN = 26;                                 // LED que se enciende mientras se toman datos
bool cogerDatos = true;                                // Condicion para realizar o no, el muestreo

// VARIABLES ACELERACION
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
MPU9250_asukiaaa mySensor;
float aX, aY, aZ;                            // Aceleraciones de cada eje
float aXprev, aYprev, aZprev;                // Valores previos de acelración
float asumay=0.0, asumaz=0.0;                // Para las "integrales" del abuffer 
float bsumay=0.0, bsumaz=0.0;                // Para las "integrales" del bbuffer 
float xdiff = 0.0, ydiff = 0.0, zdiff = 0.0; // Sumatorio de variaciones en abbufer // Sumatorio de variaciones en bbbufer
float asumaTdiff = 0.0, bsumaTdiff = 0.0;    // Sumatorio abs. de variaciones en todos los ejes

// VARIABLES TARJETA SD
String dataMessage = "";                                     // Mensaje que escribimos en la SD
String encabezado  = "Predicciones realizadas con fecha: ";  // Encabezado
String filename = "/predicciones.txt";                       // Nombre del archivo que escribimos

// WIFI
const char* ssid = "OrdenadorJavier";
const char* password = "holaquetal";

// ESTAMPA DE TIEMPO / NTP
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;        // Madrid: UTC + 1h -> offset = 3600 segundos.
const int   daylightOffset_sec = 3600;   // Retrasos/ adelantos de 1 hora
struct tm timeinfo;                      // Struct temporal
char estampa[50];                        // array de chars que contiene la fecha formateada  

// PREDICCION
String prediccion;

// INTERRUPCION PARA EL MUESTREO
volatile int interruptCounter;                        // Para detectar si se ha entrado en la interrupcion
hw_timer_t * timer = NULL;                            // Seleccion de timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Para sincronizar variable entre loop e interrupcion
int cont = 0;                                         // Contador del numero de interrupciones

void IRAM_ATTR onTimer() {                            // Funcion de interrupcion
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux); 
}

// SET-UP
void setup() {  
  delay(5000);

  // Comunicacion I2C
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
    mySensor.setWire(&Wire);
  #endif
  mySensor.beginAccel();
  if (mySensor.accelUpdate() == 0) {
    aXprev = mySensor.accelX();
    aYprev = mySensor.accelY();
    aZprev = mySensor.accelZ();
  } else { 
    // No hacer nada
  }

  // Wi-fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // Solo salir del bucle si se ha conectado a la red wifi;
  }

  // NTP / estampa de tiempo:
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);               // Pedimos fecha al servidor NTP
  getLocalTime(&timeinfo);                                                // Pedimos fecha al ESP32
  strftime(estampa, sizeof(estampa), "%A, %B %d %Y %H:%M:%S", &timeinfo); // Formateamos la fecha
  encabezado = encabezado + String(estampa) + "\r\n";                              // Actulizamos encabezado
  
  // SD
  SD.begin(); 
  if(!SD.begin()) {
    return; // Fallo
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    return; // Fallo
  }
  if (!SD.begin()) {
    return; // Fallo
  }  

  File file = SD.open(filename.c_str());                  // Creamos txt si no existe
  if(!file) {
    writeFile(SD, filename.c_str(), encabezado.c_str());  // Para ello, escribimos encabezado
  }
  else {
    // No hacer nada
  }
  file.close();

  // Timer
  timer = timerBegin(0, 80, true);             // Canal 0, prescaler de 80 -> freq=80/80=1 MHz, y conteo ascendente
  timerAttachInterrupt(timer, &onTimer, true); // Asignamos el timer a la interrupcion tras producirse el flanco
  timerAlarmWrite(timer, period, true);        // Interrupcion = 50000 conteos (20 Hz). Cada vez que llegue, el conteo se renueva
  timerAlarmEnable(timer);                     // Activamos la interrupcion
  
  // Led
  pinMode(ledPIN , OUTPUT); 
  delay(5000);                 // 5 segundos mas de espera
  digitalWrite(ledPIN , HIGH); // Se enciende el LED avisando de que comienza la toma de datos
  
}

// LOOP
void loop() {

  if ((interruptCounter > 0) and (cogerDatos)) {   // Entrada a la interrupcion
    portENTER_CRITICAL(&timerMux);                 // Quitamos flag de interrupcion
    interruptCounter=0;
    portEXIT_CRITICAL(&timerMux);

    ++cont; // Actualizamos contador

    if (mySensor.accelUpdate() == 0) {   // Leemos aceleraciones
        aX = mySensor.accelX();      
        aY = mySensor.accelY();      
        aZ = mySensor.accelZ(); 
      } 
    else {
      // Nada;
    }

    cont_buffer1 = cont_buffer1 + 3;                              // Actualizamos nmuestras del abuffer

    if ((cont_buffer1 == ((w_muestras*3/2) + 3)) and (inicio)){   // Desfase entre buffers
      cont_buffer2 = 0;
      bsumay = 0.0;
      bsumaz = 0.0;
      inicio = false;
    }

    cont_buffer2 = cont_buffer2 + 3;                              // Actualizamos nmuestras del bbuffer

    asumay = asumay + aY;
    asumaz = asumaz + aZ;
    bsumay = bsumay + aY;
    bsumaz = bsumaz + aZ;

    if ((aX - aXprev)<0.0){          // calculo de las variaciones de aceleracion
      xdiff = aXprev - aX;
    }
    else{
      xdiff = aX - aXprev;
    }

    if ((aY - aYprev)<0.0){
      ydiff = aYprev - aY;
    }
    else{
      ydiff = aY - aYprev;
    }

    if ((aZ - aZprev)<0.0){
      zdiff = aZprev - aZ;
    }
    else{
      zdiff = aZ - aZprev;
    }

    asumaTdiff = asumaTdiff + xdiff + ydiff + zdiff;
    bsumaTdiff = asumaTdiff + xdiff + ydiff + zdiff;

    if (cont_buffer1 == (w_muestras*3)){ // Cuando se llena abuffer
      
      realizarPrediccion(asumay, asumaz, asumaTdiff);
      dataMessage = prediccion + "\r\n" ;                     // Hacemos la prediccion y lo pasamos a String       
      appendFile(SD, filename.c_str(), dataMessage.c_str());  // Escribimos la prediccion  
      
      cont_buffer1 = 0;          // Inicializamos contador
      asumay =0.0; asumaz =0.0;  // Inicializamos "integral"
      asumaTdiff = 0.0;          // Inicializamos suma de diferencias
      dataMessage = "";
      
    }

    if (cont_buffer2 == (w_muestras*3)){ // Cuando se llena bbuffer

      realizarPrediccion(bsumay, bsumaz, bsumaTdiff);
      dataMessage = prediccion + "\r\n" ;                     // Hacemos la prediccion y lo pasamos a String       
      appendFile(SD, filename.c_str(), dataMessage.c_str());  // Escribimos la prediccion  
      
      cont_buffer2 = 0;         // Inicializamos contador
      bsumay =0.0; bsumaz =0.0; // Inicializamos "integral"
      bsumaTdiff = 0.0;         // Inicializamos suma de diferencias
      dataMessage = "";
      
    }

    aXprev = aX;  // Actualizamos valores anteriores
    aYprev = aY;
    aZprev = aZ;

    if (cont > maxCont) {
      cogerDatos = false;
      digitalWrite(ledPIN , LOW); // Se apaga el LED avisando de que termina la toma de datos
    }

  }

}

void realizarPrediccion(float y, float z, float delta){

  //Primero comprobamos si hay poco movimiento (estado sentado):
  if (delta < 4.0){
    //Miramos que forma de sentarse es:
    if (z > 50.0){
      prediccion = "p_sent1";
    }
    else {
      prediccion = "p_sent2";
    }    
  }    
  //Comprobamos si es EPIL1:
  else if (z>42.0){
    prediccion = "p_epil1";
  }
  //Comprobamos si es EPIL2:
  else if (y < -10.0) {
    prediccion = "p_epil2";
  }
  else{
    prediccion = "p_andar";
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
