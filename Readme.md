# Trabajo de la asignatura de Redes de Sensores Electrónicos

## Detección de epilepsias de mi abuelo

El objetivo de este proyecto es elaborar un detector inteligente de **epilepsias** (y otros movimientos detallados más adelante) utilizando para ello el *SoC* **ESP32** y el acelerómetro del sensor **MPU9250**. La motivación de dicho proyecto reside en ser capaz de detectar episodios de epilepsias similares a los que ya ha tenido mi abuelo.

## Movimientos a detectar

Todos los episodios que ha tenido mi abuelo de epilepsias han ocurrido mientras estaba sentado, viendo la televisión. Por ello los tipos de datos recogidos tienen que ver con el entorno de esta zona. A continuación se enumeran los tipos de datos recogidos:

* **Andar**: Forma de caminar de mi abuelo (se pensó  que podían ser útiles para cuando sale/ entra del salón).
* **Sentado con la pierna arriba**: Junto con la siguiente posición, mi abuelo suele sentarse con la pierna apoyada en una mesita que tiene delante.
* **Sentado con la pierna abajo**: La otra posición común de estar sentado, es simplemente con la pierna abajo..
* **Epilepsia con movimiento cíclico**: Se corresponde con la siguiente figura.

![Movimiento 1](https://github.com/JavierTG97/RSENSE20_TIRADO_TRABAJO_SEGURO/blob/master/Imagenes/Imagen1.png?raw=true)

* **Epilepsia con movimiento de juntar-separar las piernas**: Se corresponde con la siguiente figura.

![Movimiento 2](https://github.com/JavierTG97/RSENSE20_TIRADO_TRABAJO_SEGURO/blob/master/Imagenes/Imagen2.png?raw=true)