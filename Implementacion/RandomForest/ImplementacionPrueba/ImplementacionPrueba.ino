/* Redes de Sensores Electronicos
 * Implementacion usada para la prediccion de los movimientos
 */

// LIBRERIAS:
#include <MPU9250_asukiaaa.h>                //Acelerometro
#include "WiFi.h"                            //WiFi
#include "time.h"                            //Hora
#include "model.h"                           //RandomForest
Eloquent::ML::Port::RandomForest classifier; //RandomForest

// VARIABLES CAPTURA DE DATOS
const int ttotal = 1000;                                 // Tiempo total (segundos) de captura de datos
const int period = 50000;                              // Tmuestreo en useg (20 Hz)
const float period_s = float(period)/1000000.0;        // Tmuestreo en seg  (20 hz)
const int maxCont = round(ttotal*1000000/period);      // Numero de muestras a tomar
const int w_seg = 4;                                   // Tiempo en seg de cada ventana
const int w_muestras = round(float(w_seg)/period_s);   // Num. de veces que se capturan datos por ventana
float abuffer[w_muestras*3+3];                         // Buffer que almacena los datos de 3ejes*w_muestras + 3 integrales
float bbuffer[w_muestras*3+3];                         // Mismo buffer pero con solape del 50%
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
float aX, aY, aZ;                         // Aceleraciones de cada eje
float asumax=0.0, asumay=0.0, asumaz=0.0; // Para las "integrales" del abuffer 
float bsumax=0.0, bsumay=0.0, bsumaz=0.0; // Para las "integrales" del bbuffer 

// WIFI
const char* ssid = "OrdenadorJavier";
const char* password = "holaquetal";

// ESTAMPA DE TIEMPO / NTP
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;        // Madrid: UTC + 1h -> offset = 3600 segundos.
const int   daylightOffset_sec = 3600;   // Retrasos/ adelantos de 1 hora
struct tm timeinfo;                      // Struct temporal

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
  Serial.begin(115200);
  delay(5000);

  // Comunicacion I2C
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
    // No hacer nada
  }

  // Wi-fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // Solo salir del bucle si se ha conectado a la red wifi;
  }

  // Timer
  timer = timerBegin(0, 80, true);             // Canal 0, prescaler de 80 -> freq=80/80=1 MHz, y conteo ascendente
  timerAttachInterrupt(timer, &onTimer, true); // Asignamos el timer a la interrupcion tras producirse el flanco
  timerAlarmWrite(timer, period, true);        // Interrupcion = 50000 conteos (20 Hz). Cada vez que llegue, el conteo se renueva
  timerAlarmEnable(timer);                     // Activamos la interrupcion
  
  // Led
  pinMode(ledPIN , OUTPUT); 
  //delay(5000);                 // 5 segundos mas de espera
  digitalWrite(ledPIN , HIGH); // Se enciende el LED avisando de que comienza la toma de datos

  // NTP / estampa de tiempo:
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  getLocalTime(&timeinfo);
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");  
  
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

    abuffer[cont_buffer1]   = aX;                                 // Las introducimos en el abuffer
    abuffer[cont_buffer1+1] = aY;
    abuffer[cont_buffer1+2] = aZ;
    cont_buffer1 = cont_buffer1 + 3;                              // Actualizamos nmuestras del abuffer

    if ((cont_buffer1 == ((w_muestras*3/2) + 3)) and (inicio)){   // Desfase entre buffers
      cont_buffer2 = 0;
      bsumax = 0.0;
      bsumay = 0.0;
      bsumaz = 0.0;
      inicio = false;
    }

    bbuffer[cont_buffer2]   = aX;                                 // Las introducimos en el bbuffer
    bbuffer[cont_buffer2+1] = aY;
    bbuffer[cont_buffer2+2] = aZ;
    cont_buffer2 = cont_buffer2 + 3;                              // Actualizamos nmuestras del bbuffer

    asumax = asumax + aX; // Calculo de las "integrales" del abuffer
    asumay = asumay + aY;
    asumaz = asumaz + aZ;
    bsumax = bsumax + aX; // Calculo de las "integrales" del bbuffer
    bsumay = bsumay + aY;
    bsumaz = bsumaz + aZ;

    if (cont_buffer1 == (w_muestras*3)){ // Cuando se llena abuffer
      abuffer[w_muestras*3]   = asumax;  // Introducimos las "integrales"
      abuffer[w_muestras*3+1] = asumay;
      abuffer[w_muestras*3+2] = asumaz;

      Serial.println(classifier.predictLabel(abuffer));        
      
      cont_buffer1 = 0;                                                // Inicializamos contador
      asumax =0.0; asumay =0.0; asumaz =0.0;                           // Inicializamos "integral"
      
    }

    if (cont_buffer2 == (w_muestras*3)){ // Cuando se llena bbuffer
      bbuffer[w_muestras*3]   = bsumax;  // Introducimos las "integrales"
      bbuffer[w_muestras*3+1] = bsumay;
      bbuffer[w_muestras*3+2] = bsumaz;
      
      Serial.println(classifier.predictLabel(bbuffer));    

      //Serial.println(classifier.predictLabel(abuffer));
      
      cont_buffer2 = 0;                                                // Inicializamos contador
      bsumax =0.0; bsumay =0.0; bsumaz =0.0;                           // Inicializamos "integral"      
    }

    if (cont > maxCont) {
      cogerDatos = false;
      digitalWrite(ledPIN , LOW); // Se apaga el LED avisando de que termina la toma de datos
    }

  }

}
