/* Redes de Sensores Electronicos
 * Implementacion usada para la prediccion de los movimientos
 */

// LIBRERIAS:
#include <MPU9250_asukiaaa.h>                //Acelerometro
#include "WiFi.h"                            //WiFi
#include "time.h"                            //Hora
#include "config.h"                          //MQTT

// VARIABLES CAPTURA DE DATOS
const int ttotal = 16;                               // Tiempo total (segundos) de captura de datos
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
float aXprev, aYprev, aZprev;                // Valores previos de aceleracion
float asumay=0.0, asumaz=0.0;                // Para las "integrales" del abuffer 
float bsumay=0.0, bsumaz=0.0;                // Para las "integrales" del bbuffer 
float xdiff = 0.0, ydiff = 0.0, zdiff = 0.0; // Sumatorio de variaciones en abbufer // Sumatorio de variaciones en bbbufer
float asumaTdiff = 0.0, bsumaTdiff = 0.0;    // Sumatorio abs. de variaciones en todos los ejes

// WIFI
const char* ssid = "OrdenadorJavier";
const char* password = "holaquetal";

// MQTT (Adafruit IO)
String icon_IO = "";                            // Variables asociadas a los indicadores de luz
String textIO = "";
AdafruitIO_Feed *iconIO = io.feed("iconIO");    // Preparamos todas las feeds
AdafruitIO_Feed *text_IO = io.feed("text_IO");
bool epil = false;                              // Para saber si enviar o no, dato correspondiente a iconIO

// PREDICCION
String prediccion_a; // Para el buffer a
String prediccion_b; // Para el buffer b
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
  Serial.begin(115200);
  delay(5000);

  io.connect();                        // Nos conectamos con io.adafruit.com
  while(io.status() < AIO_CONNECTED) { // Esperamos a conectarnos
    delay(1000);
    Serial.println("... ");
  }

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
  //WiFi.begin(ssid, password);
  //while (WiFi.status() != WL_CONNECTED) {
    //delay(1000);
    // Solo salir del bucle si se ha conectado a la red wifi;
  //}

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

    cont_buffer1 = cont_buffer1 + 3;                              // Actualizamos nmuestras del abuffer

    if ((cont_buffer1 == ((w_muestras*3/2) + 3)) and (inicio)){   // Desfase entre buffers
      cont_buffer2 = 0;
      bsumay = 0.0;
      bsumaz = 0.0;
      bsumaTdiff = 0.0;
      inicio = false;
    }
    
    cont_buffer2 = cont_buffer2 + 3; // Actualizamos nmuestras del bbuffer

    asumay = asumay + aY;            // Calculo de las "integrales" del abuffer
    asumaz = asumaz + aZ;      
    bsumay = bsumay + aY;            // Calculo de las "integrales" del bbuffer
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

      realizarPrediccion(asumay, asumaz, asumaTdiff); // Llevamos a cabo la prediccion
      prediccion_a = prediccion;
      //enviarMQTT();                                 // Enviamos la prediccion a Adafruit
      
      cont_buffer1 = 0;          // Inicializamos contador
      asumay =0.0; asumaz =0.0;  // Inicializamos "integral"
      asumaTdiff = 0.0;          // Inicializamos suma de diferencias
      
    }

    if (cont_buffer2 == (w_muestras*3)){ // Cuando se llena bbuffer

      realizarPrediccion(bsumay, bsumaz, bsumaTdiff); // Llevamos a cabo la prediccion
      prediccion_b = prediccion;
      enviarMQTT();                                   // Enviamos las predicciones a Adafruit       
      
      cont_buffer2 = 0;         // Inicializamos contador
      bsumay =0.0; bsumaz =0.0; // Inicializamos "integral"  
      bsumaTdiff = 0.0;         // Inicializamos suma de diferencias     
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


void enviarMQTT(){ // Funcion a la que se entra cada 2 segundos para enviar los datos
  textIO = prediccion_a + "; " + prediccion_b;
  text_IO->save(textIO);

  if (((prediccion_a == "p_epil1") or (prediccion_b == "p_epil2")) and (not epil)){
    icon_IO = "warning";
    iconIO->save(icon_IO);
    epil = true;
  }
  
}
