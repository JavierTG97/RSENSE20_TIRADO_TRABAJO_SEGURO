/* Redes de Sensores Electronicos
 * Implementacion usada para la prediccion de los movimientos
 */

// LIBRERIAS:
#include <MPU9250_asukiaaa.h>                //Acelerometro
#include "WiFi.h"                            //WiFi
#include "time.h"                            //Hora
#include "config.h"                          //MQTT
#include "model.h"                           //RandomForest
Eloquent::ML::Port::RandomForest classifier; //RandomForest

// VARIABLES CAPTURA DE DATOS
const int ttotal = 40;                                 // Tiempo total (segundos) de captura de datos
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

// MQTT (Adafruit IO)
//int epil1_IO = 0; // Variables asociadas a los indicadores de luz
//int epil2_IO = 0;
//int andar_IO = 0;
//int sent1_IO = 0;
//int sent2_IO = 0;
String icon_IO = "";
String textIO = "";
//AdafruitIO_Feed *andarIO = io.feed("andarIO"); // Preparamos todas las feeds
//AdafruitIO_Feed *epil1IO = io.feed("epil1IO");
//AdafruitIO_Feed *epil2IO = io.feed("epil2IO");
//AdafruitIO_Feed *sent1IO = io.feed("sent1IO");
//AdafruitIO_Feed *sent2IO = io.feed("sent1IO");
AdafruitIO_Feed *iconIO = io.feed("iconIO");
AdafruitIO_Feed *text_IO = io.feed("text_IO");
bool epil = false;                              // Para saber si enviar o no, dato correspondiente a iconIO

// PREDICCION
String prediccion_a; // Para el buffer a
String prediccion_b; // Para el buffer b

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
  //Serial.begin(115200);
  delay(5000);

  io.connect();                        // Nos conectamos con io.adafruit.com
  while(io.status() < AIO_CONNECTED) { // Esperamos a conectarnos
    delay(100);
  }

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
  delay(5000);                 // 5 segundos mas de espera
  digitalWrite(ledPIN , HIGH); // Se enciende el LED avisando de que comienza la toma de datos
  
}

// LOOP
void loop() {
  
  io.run(); // Requerimiento de Adafruit IO

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

      prediccion_a = String(classifier.predictLabel(abuffer)); // Llevamos a cabo la prediccion
      //enviarMQTT();                                          // Enviamos la prediccion a Adafruit
      
      cont_buffer1 = 0;                                                // Inicializamos contador
      asumax =0.0; asumay =0.0; asumaz =0.0;                           // Inicializamos "integral"
      
    }

    if (cont_buffer2 == (w_muestras*3)){ // Cuando se llena bbuffer
      bbuffer[w_muestras*3]   = bsumax;  // Introducimos las "integrales"
      bbuffer[w_muestras*3+1] = bsumay;
      bbuffer[w_muestras*3+2] = bsumaz;

      prediccion_b =  String(classifier.predictLabel(bbuffer)); // Llevamos a cabo la prediccion
      enviarMQTT();                                             // Enviamos las predicciones a Adafruit       
      
      cont_buffer2 = 0;                                                // Inicializamos contador
      bsumax =0.0; bsumay =0.0; bsumaz =0.0;                           // Inicializamos "integral"      
    }

    if (cont > maxCont) {
      cogerDatos = false;
      digitalWrite(ledPIN , LOW); // Se apaga el LED avisando de que termina la toma de datos
    }

  }

}


void enviarMQTT(){ // Funcion a la que se entra cada 2 segundos para enviar los datos
  textIO = prediccion_a + "; " + prediccion_b;
  text_IO->save(textIO);

  if (((prediccion_a == "p_epil1") or (prediccion_b == "p_epil2")) and (not epil)){
    icon_IO = "warning";
    iconIO->save(icon_IO);
    epil = true;
  }
  
}
