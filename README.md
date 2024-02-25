## Implementación de una video cámara y de un mando para controlar la orientación de dicha cámara y el sistema de movimiento de un robot movil

# Índice

1. [Introducción](#introducción)
2. [Videos para la demostración del funcionamiento](#videos-para-la-demostración-del-funcionamiento)
3. [Programas utilizados](#programas-utilizados)
4. [Materiales usados](#materiales-usados)
5. [Implementación del mando](#implementación-del-mando)
   - [Diseño del circuito](#diseño-del-circuito)
   - [Programación y lógica de funcionamiento](#programación-y-lógica-de-funcionamiento)
6. [Implementación de la cámara y su sistema de movimiento](#implementación-de-la-cámara-y-su-sistema-de-movimiento)
   - [Diseño del circuito](#diseño-del-circuito-1)
   - [Programación y lógica de funcionamiento](#programación-y-lógica-de-funcionamiento-1)
   - [Diseño 3D de la carcasa de la cámara y su sistema de movimiento](#diseño-3d-de-la-carcasa-de-la-cámara-y-su-sistema-de-movimiento)
     - [Carcasa de la ESP32 Wrover Cam](#carcasa-de-la-esp32-wrover-cam)
     - [Eje de movimiento](#eje-de-movimiento)
     - [Zócalo para los componentes del sistema de movimiento](#zócalo-para-los-componentes-del-sistema-de-movimiento)
     - [Caja del sistema de movimiento](#caja-del-sistema-de-movimiento)
     - [Diseño conjunto](#diseño-conjunto)
7. [Implementación de la pasarela Wifi para el Arduino Mega 2560](#implementación-de-la-pasarela-wifi-para-el-arduino-mega-2560)
8. [Diseño del flujo de Nodered](#diseño-del-flujo-de-nodered)
9. [Usabilidad del entorno Dashboard de NodeRed y de Telegram](#usabilidad-del-entorno-dashboard-de-nodered-y-de-telegram)


### Introducción

Esta implementación constituye el sistema de seguridad del proyecto para la asignatura de Informática Industrial y el sistema de visión del robot móvil de la asignatura de Laboratorio de Robótica.

La parte extra del proyecto consiste principalmente en tres implementaciones:

- Uso de una cámara para poder ver lo que ve el robot el tiempo real. Además, poder controlar la posición de la cámara para orientarla en 360 grados, sacar fotos, guardarlas en una base de datos para poder recuperarlas posteriormente y usar el flash.
  
- Uso de una pasarela Wifi para el Arduino Mega 2560, y así poder mandarle comandos de velocidad y por tanto, de posición de manera manual.

- Uso de un mando para controlar tanto la orientación de la cámara como los comandos emitidos al Piero.

### Videos para la demostración del funcionamiento

El funcionamiento real del sistema puede comprobarse accediendo a los siguientes enlaces:

- Control de la cámara incorporada al robot móvil en funcionamiento autónomo a través de Telegram y NodeRed: https://youtu.be/6Vd62COOqGI
- Control de la cámara mediante el mando: https://youtu.be/HZeDFFgzWQ8
  
### Programas utilizados

Para el desarrollo de esta implementación haremos uso de dos entornos dos entornos de programación:

- Arduino IDE: con esta herramienta, programaremos todos los microncontroladores usados.
  
- NodeRed: con esta herramienta, prograremos la lógica del sistema de control con el que los dispositivos se interconectaran entre sí. La conexión entre ellos y con el propio servidor en el que está corriendo NodeRed (en este caso, uno de nuestros ordenadores) principalmente se realizará mediante MQTT, haciendo uso de distintos topics para el envío y recepción de datos.

### Materiales usados
A continuación, se detallan algunas características relevantes de todos los materiales usados en esta ampliación del proyecto.

- ESP32 Wrover Cam: La ESP32 WROVER-CAM, basada en el chip ESP32, destaca por su diseño compacto y versátil, integrando conectividad Wi-Fi y Bluetooth junto con una cámara que emplea el sensor OV-2640. Este módulo es especialmente adecuado para proyectos de IoT, video vigilancia, captura de imágenes y transmisión en tiempo real, gracias a su potente procesador dual-core Xtensa LX6 que garantiza un rendimiento eficiente.Uno de los puntos destacados de este módulo son sus GPIOs multipropósito, que ofrecen flexibilidad en el diseño y la implementación de proyectos. Estos pines pueden configurarse para diversas funciones, como entrada analógica, salida digital o comunicación serial, lo que amplía las posibilidades de integración con otros dispositivos y sensores. Este módulo será nuestra cámara para el robot.

<p align="center">
  <img src="/fotos/image-1.png" alt="Descripción de la /fotos/imagen">
</p>
  
- ESP32 Wroom 32: La ESP32 WROOM-32, basada en el chip ESP32, es un módulo compacto y versátil que incorpora conectividad Wi-Fi y Bluetooth, siendo una opción ideal para proyectos de IoT, automatización, y aplicaciones inalámbricas. Equipada con un procesador dual-core Xtensa LX6, ofrece un rendimiento eficiente para tareas complejas. Los GPIOs versátiles proporcionan flexibilidad en el diseño, permitiendo configurar los pines para diversas funciones, como entrada/salida digital, comunicación serial, o lectura analógica. Esto facilita la integración con sensores y otros componentes externos, adaptándose a diferentes requisitos de proyectos. La capacidad para ejecutar aplicaciones web directamente en el módulo facilita la visualización remota de datos y la configuración a través de interfaces basadas en navegador. Este módulo será el encargado de darle una unidad de procesamiento a nuestro mando.

<p align="center">
  <img src="/fotos/image-2.png" alt="Descripción de la /fotos/imagen">
</p>
  
- ESP8266 NodeMCU ESP-12E: esta es una placa de desarrollo versátil que cuenta con 17 pines GPIO digitales, permitiendo la implementación de PWM de 10 bits en los pines GPIO0 al GPIO15. Algunos GPIO, como el GPIO16, tienen usos específicos, como despertar la placa del modo de suspensión profunda. Sin embargo, se recomienda evitar el uso de ciertos pines, como GPIO6 al GPIO11, que no son accesibles a través de la placa de desarrollo (excepto el 9 y el 10). Además, se destaca la presencia de pines dedicados para SPI y la conversión analógica a digital (ADC0) en A0 con un rango de entrada de 0 a 3.3V. Este módulo será el que haga de pasarela WIFI a nuestra Arduino Mega del Piero.
  
<p align="center">
  <img src="/fotos/image-3.png" alt="Descripción de la /fotos/imagen">
</p>

- Arduino Nano: es una placa de desarrollo compacta basada en microcontroladores de la familia ATmega, fabricada por Arduino. Se destaca por su tamaño reducido y su capacidad para realizar diversas tareas de programación y control. El Nano cuenta con entradas y salidas digitales y analógicas, puertos de comunicación, etc. Este módulo será el encargado de controlar los movimientos del motor paso a paso, descrito en el siguiente punto.
  
<p align="center">
  <img src="/fotos/image-4.png" alt="Descripción de la /fotos/imagen">
</p>

- Motor paso a paso 28BYJ-48: este motor unipolar, aunque no es demasiado potente ni excesivamente rápido, cumple con los requisitos necesarios para el desarrollo de su aplicación en este proyecto. Se trata de un motor de 4 Fases con una resistencia de 50 Ω y un par motor de 34 Newton / metro más o menos 0,34 Kg por cm. Consume unos 55 mA y es de 8 pasos por vuelta por lo que, para completar un giro completo de 360º, necesita 512 impulsos. Este motor será el encargado de darle el movimiento a la cámara. Debido a que un motor paso a paso consume una corriente elevada y puede necesitar una tensión mayor a los 5 VDC de nuestro Arduino, no podemos realizar una conexión directa entre este y la placa Arduino. Se requiere un manejador (driver) o interface para brindar al motor la tensión y la corriente necesarias, por eso utilizamos el siguiente módulo.

<p align="center">
  <img src="/fotos/image-5.png" alt="Descripción de la /fotos/imagen">
</p>
  
- Driver ULN2003A: este módulo cuenta con el controlador ULN2003 que posee una configuración Darlington y se encarga de incrementar el nivel de corriente para suplir la potencia necesaria que solicita el motor paso a paso. permite la adaptación de cualquier motor paso a paso cuando se conecta directamente. Posee cinco lí­neas de conexión para el motor, además de cuatro fases.
  
<p align="center">
  <img src="/fotos/image-6.png" alt="Descripción de la /fotos/imagen">
</p>

- Sensor de final de carrera: es un dispositivo que conmuta internamente sus conexiones al recibir una pulsación mecánica, haciendo que la corriente de la base pase a circular por el pin NO (Normally Open) en vez de por el pin NC (Normally Closed). En nuestro caso, haremos uso de este dispositivo para detectar cuando el motor que controla la posición de la cámara llega al origen del recorrido y realizar así su calibración.

<p align="center">
  <img src="/fotos/image-8.png" alt="Descripción de la /fotos/imagen">
</p>

- Convertidor de tensión Buck Mini-360-DC: este es un dispositivo compacto y eficiente diseñado para convertir tensiones de entrada mayores a tensiones de salida más bajas de manera controlada. Su tensión de salida se ajusta mediante el potenciómetro integrado. Esto permite adaptar la salida a las necesidades específicas de un proyecto o dispositivo.  Este tipo de convertidor es comúnmente utilizado en aplicaciones electrónicas para alimentar dispositivos que requieren una tensión de alimentación menor que la disponible. En nuestro caso, lo usaremos para adaptar la salida de 12 Voltios de un cojunto de pilas y así poder alimentar dispositivos como el mando que controle la cámara, o el propio ecosistema de la cámara.

<p align="center">
  <img src="/fotos/image-7.png" alt="Descripción de la /fotos/imagen">
</p>

- Pilas: usaremos 3 pilas 3 4 Voltios y 3700 mAh para darle energía a nuestro sistema de la cámara.

<p align="center">
  <img src="/fotos/image-10.png" alt="Descripción de la /fotos/imagen">
</p>

- Diodos led: se usarán 3 diodos led de colores rojo, verde y blanco para tener indicadores de conexión WIFI (rojo sin conexión, verde con conexión) y para tener un flash para la cámara (led blanco). Además, para alimentar estos diodos haremos uso de resistencias de 220 Ω y así limitar la corriente que pase por ellos.

<p align="center">
  <img src="/fotos/image-9.png" alt="Descripción de la /fotos/imagen">
</p>

- Módulos de joystick: estos módulos constan de dos potenciómetros en perpenticular que leen la posición del joystick en función de la corriente que pase por ellos. Además, cuentan con un pulsador para hacer la función de click.

<p align="center">
  <img src="/fotos/image-14.png" alt="Descripción de la /fotos/imagen">
</p>

### Implementación del mando

El mando que vamos a usar consta de una ESP32 Wroom-32 que leerá los datos de posición de dos joysticks conectados a ella. Cada uno de estos joysticks será la herramienta con la que nos comunicaremos para construir comandos para la cámara y para el Piero. 

Los comandos realizados con el joystick para la cámara seguirán la siguiente lógica:

- Desplizamiento vertical positivo: Activar/desactiva el flash de la cámara.
- Desplazamiento vertical negativo: Centrar la cámara en su orientación de origen.
- Desplazamiento horizontal positivo: Girar la cámara hacia la derecha.
- Desplazamiento horizontal negativo: Girar la cámara hacia la izquierda.
- Click: Hacer una foto.

Los comandos realizados con el joystick para el piero seguirán la siguiente lógica:

- Desplizamiento vertical positivo: Velocidad lineal del robot positiva, magnitud en función de la posición del joystick.
- Desplazamiento vertical negativo: Velocidad lineal del robot negativa, magnitud en función de la posición del joystick.
- Desplazamiento horizontal positivo: Velocidad angular del robot positiva, magnitud en función de la posición del joystick.
- Desplazamiento horizontal negativo: Velocidad angular del robot negativa, magnitud en función de la posición del joystick.
- Click: Cambiar el modo de funcionamiento, para pasar al modo de funcionamiento de navegación reactiva o al modo de navegación manual controlado mediante el mando.

Además, todas las funciones del mando podrán ser igualmente accesibles tanto desde la Dashboard de Nodered como desde un bot de Telegram que hemos programado para dicha tarea.

#### Diseño del circuito

A continuación, aparece el esquema del circuito diseñado para el mando.

<p align="center">
  <img src="/fotos/image.png" alt="Descripción de la /fotos/imagen">
</p>

El montaje real del prototipo queda así: 

<p align="center">
  <img src="/fotos/image-13.png" alt="Descripción de la /fotos/imagen">
</p>

#### Programación y lógica de funcionamiento

Los joysticks son dispositivos analógicos. Debido a la naturaleza de estos dispositivos, los pines de posición horizontal y vertical de cada uno tienen que ir conectados a pines que estén conectados al conversor analógico-digital de la placa, para poder hacer así una lectura de la información. Este es el motivo principal por el que se ha usado una ESP32 en lugar de una ESP8266 para el mando, ya que la ESP8266 cuenta con un único pin conectado a un conversor analógico-digital integrado, claramente insuficiente para esta tarea. 

Otro de los problemas que encontramos es que, aunque la ESP32 tenga pines analógicos más que suficientes para leer la información de los dos joysticks, el conversor analógico-digital que incorpora no es totalmente lineal, sino que tiene el comportamiento reflejado en la siguiente imagen.

<p align="center">
  <img src="/fotos/image-15.png" alt="Descripción de la /fotos/imagen">
</p>

Como se puede observar, a partir de un voltaje de entrada aproximado de 3,1 Voltios, el conversor se satura y devuelve un valor digital máximo de aproximadamente 4095. De igual forma ocurre con el límite inferior del rango de tensión, ya que hasta que no se superan los 0,1 con Voltios de tensión de entrada, el conversor devolverá un valor digital de prácticamente 0. 

Además, la tanto la calidad de la fabricación de los potenciómetros de los joysticks como la propia calidad de lectura del conversor analógico-digital de la placa crea un ruido bastante notable en los resultados, arrojando una lectura muy inestable e imprecisa. Esto crea un desafío a la hora de convertir las señales analógicas de los joysticks en comandos de movimiento. 

La solución que le hemos dado a este problema es el acotamiento de la tensión que recibe el conversor en 7 niveles, por lo que, si antes se contaba con una sensibilidad de lectura entre 0 y 4095, ahora la información de los joysticks se ha codificado en valores que van desde -3 hasta 3 para los dos ejes de lectura. Con ello, se reduce el ruido producido en la lectura haciéndolo imperceptible, se eliminan los movimientos fantasmas, y se ofrece una sensibilidad para los comandos más que suficiente para la aplicación de este proyecto.

A continuación, se adjunta el programa contenido en la ESP32 Wroom 32.

Primero se declaran todas las librerías, topics y variables necesarias:
```arduino
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>

String ssid = "MiFibra-9B77";
String password = "oRfwyF6X";
String mqtt_server = "iot.ac.uma.es"; 
String mqtt_user = "II10";
String mqtt_pass = "wD9IYehI";
String ID_PLACA = String(ESP.getEfuseMac());
String topicPiero = "II10/ESP"+ID_PLACA+"/mando/piero";
String topicCamara = "II10/ESP"+ID_PLACA+"/mando/camara";
String topic_PUB_conexion = "II10/ESP"+ID_PLACA+"/mando/conexion";
String mensaje_conexion = "{\"mando\":connected}";
String LWT = "{\"mando\":disconnected}";

WiFiClient wClient;
PubSubClient mqtt_client(wClient);
// Pines para los indicadores de conexión
const int pinRojo = 5;
const int pinVerde = 18;
// Pines para los joysticks
const int pinxCamara = 33;
const int pinyCamara = 32;
const int pinButtonCamara = 17;

const int pinxPiero = 34;
const int pinyPiero = 39;
const int pinButtonPiero = 16;

const int rango = 6;

// Tamaño del buffer para mensajes JSON
const size_t bufferSize = 512;

// Variables para almacenar las lecturas
int oldxValueCamara = 0, newxValueCamara = 0;
int oldyValueCamara = 0, newyValueCamara = 0;
int oldButtonCamara = 1, newButtonCamara = 1;

int oldxValuePiero = 0, newxValuePiero = 0;
int oldyValuePiero = 0, newyValuePiero = 0;
int oldButtonPiero = 1, newButtonPiero = 1;
```

Se añaden todas las subfunciones necesarias para el programa. La siguiente, es la encargada de leer los datos de los joysticks. Aquí se acota la tensión de cada potenciómetro del joystick en un valor del -3 al 3, tal y como se explicó anteriormente:

```arduino
size_t leerJoyStick(int pinx, int piny, int pinButton, int &oldx, int &newx, int &oldy, int &newy, int &oldButton, int &newButton) {
  newx = analogRead(pinx);
  newy = analogRead(piny);
  newButton = !digitalRead(pinButton);

  if (newx >= 0) {
    newx = (round((4095 - newx) * rango / 4095) - rango/2);
  } else {
    newx = (round((-4095 + newx) * rango / 4095) - rango/2);
  }

  if (newy >= 0) {
    newy = (round((4095 - newy) * rango / 4095) - rango/2);
  } else {
    newy = (round((-4095 + newy) * rango / 4095) - rango/2);
  }

  if ((newx != oldx) || (newy != oldy) || (newButton != oldButton)) {
    // Actualizamos el valor de las variables antiguas
    oldx = newx;
    oldy = newy;
    oldButton = newButton;
    return true;
  } else {
    return false;
  }
}
```

Se añaden todas las subfunciones necesarias para realizar las conexiones de Wifi y MQTT:

```arduino
//-----------------------------------------------------
void conecta_wifi() {
  Serial.println("Connecting to " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
}
//-----------------------------------------------------
void conecta_mqtt() {// Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");// Attempt to connect
    if (mqtt_client.connect(ID_PLACA.c_str(),mqtt_user.c_str(),mqtt_pass.c_str(),topic_PUB_conexion.c_str(),2,false,LWT.c_str())){
      Serial.println(" conectado a broker: " + String(mqtt_server.c_str()));
    } else {
      Serial.println("ERROR:"+ String(mqtt_client.state()) +" reintento en 5s" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
```

Setup del programa. Se realizan todas las incializaciones necesarias:

```arduino
void setup() {
  Serial.begin(115200);
  Serial.println("TOPIC: " + topicPiero);
  pinMode(pinButtonCamara, INPUT_PULLUP);
  pinMode(pinButtonPiero, INPUT_PULLUP);
  pinMode(pinVerde,OUTPUT);
  pinMode(pinRojo,OUTPUT);
  digitalWrite(pinVerde,LOW);
  digitalWrite(pinRojo,HIGH);
  conecta_wifi();
  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512); 
  conecta_mqtt();
  Serial.println("Identificador placa: "+ String(ID_PLACA));
  Serial.println("Topic publicacion control Piero: "+ String(topicPiero));
  Serial.println("Topic publicacion control Camara: "+ String(topicCamara));
  StaticJsonDocument<bufferSize> mensajePiero;
  String mensajePieroString;
  mensajePiero["X"] = 0;
  mensajePiero["Y"] = 0;
  mensajePiero["Button"] = 0;
  serializeJson(mensajePiero, mensajePieroString);
  StaticJsonDocument<bufferSize> mensajeCamara;
  String mensajeCamaraString;
  mensajeCamara["X"] = 0;
  mensajeCamara["Y"] = 0;
  mensajeCamara["Button"] = 0;
  serializeJson(mensajeCamara, mensajeCamaraString);
  mqtt_client.publish(topicPiero.c_str(),mensajePieroString.c_str());
  mqtt_client.publish(topicCamara.c_str(),mensajeCamaraString.c_str());
  Serial.println("Termina setup en " +  String(millis()) + " ms");
}
```

Bucle principal del programa. Se comprueba la conexión de la placa y la lectura de los joysticks. En el caso de que la lectura de los joysticks cambie, se envía su nueva posición por MQTT.

```arduino
void loop() {
  static unsigned long ultimo_mensaje=0;
  static unsigned long ahora=0;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexión WiFi perdida. Intentando reconectar...");
    digitalWrite(pinVerde,LOW);
    digitalWrite(pinRojo,HIGH);
    conecta_wifi();
  }else{
    digitalWrite(pinVerde,HIGH);
    digitalWrite(pinRojo,LOW);
  }

  if (!mqtt_client.connected()) {
    conecta_mqtt();
  }
  mqtt_client.loop(); // esta llamada para que la librería recupere el control
  
  ahora = millis();

  if (ahora - ultimo_mensaje >= 100) {
    ultimo_mensaje = ahora;
    if (leerJoyStick(pinxCamara, pinyCamara, pinButtonCamara, oldxValueCamara, newxValueCamara, oldyValueCamara, newyValueCamara, oldButtonCamara, newButtonCamara)){ // mandamos mensaje si cambian los valores leidos
      // Actualizamos el valor del mensaje a mandar
      StaticJsonDocument<bufferSize> mensajeCamara;
      String mensajeCamaraString;
      mensajeCamara["X"] = oldxValueCamara;
      mensajeCamara["Y"] = oldyValueCamara;
      mensajeCamara["Button"] = oldButtonCamara;
      serializeJson(mensajeCamara, mensajeCamaraString);
      Serial.print("Comando de Cámara enviado: ");
      Serial.println(mensajeCamaraString);
      mqtt_client.publish(topicCamara.c_str(),mensajeCamaraString.c_str());
    }

    if (leerJoyStick(pinxPiero, pinyPiero, pinButtonPiero, oldxValuePiero, newxValuePiero, oldyValuePiero, newyValuePiero, oldButtonPiero, newButtonPiero)) { // mandamos mensaje si cambian los valores leidos
      //Actualizamos el valor del mensaje a mandar
      StaticJsonDocument<bufferSize> mensajePiero;
      String mensajePieroString;
      mensajePiero["X"] = oldxValuePiero;
      mensajePiero["Y"] = oldyValuePiero;
      mensajePiero["Button"] = oldButtonPiero;
      serializeJson(mensajePiero, mensajePieroString);
      Serial.print("Comando de Piero enviado: ");
      Serial.println(mensajePieroString);
      mqtt_client.publish(topicPiero.c_str(),mensajePieroString.c_str());
    }
  }
}
```
### Implementación de la cámara y su sistema de movimiento
#### Diseño del circuito

A continuación, aparece el esquema del circuito diseñado para la cámara y su sistema de movimiento.
 
<p align="center">
  <img src="/fotos/image-12.png" alt="Descripción de la /fotos/imagen">
</p>

El montaje real del prototipo queda así: 

<p align="center">
  <img src="/fotos/image-11.png" alt="Descripción de la /fotos/imagen">
</p>

#### Programación y lógica de funcionamiento

En este proyecto, al conectarse a nuestra red la ESP32 Wrover Cam crea un servidor local donde se envía constantemente la /fotos/imagen tomada por su sensor, simulando un vídeo en streaming.

Para saber en qué IP local se encuentra conectada la cámara, esta informa de ello enviando su IP actula por MQTT a NodeRed. Por tanto, la /fotos/imagen de la cámara puede verse en la dirección http://192.168.1.XXX:81/stream.

La toma de fotos de la cámara se realiza mediante un htto request a dicho servidor, haciendo uso del API http://192.168.1.XXX:/capture.

Además, la placa se suscribe a los topics correspondientes para poder recibir los comandos de movimiento de la misma, así como los comandos para controlar el flash.

Sin embargo, la librería que se ha usado para controlar el motor paso a paso hace que cada vez que se quiera hacer girar dicho motor el programa principal quede bloqueado y por tanto "pausando" la /fotos/imagen en directo de la cámara. Para solventar esto, hemos hecho uso del anteriormente mencionado Arduino Nano, que queda conectado a la ESP32 Wrover Cam con dos pines digitales. Dichos pines codifican 4 estados posibles, con el siguiente significado:

| Pin "Left" | Pin "Right" | Significado|
|--------------|--------------|--------------|
| 0    |0   | Detener motor    |
| 0    | 1    | Girar hacia la derecha  |
| 1  | 0   | Girar hacia la izquierda   |
| 1    | 1   | Llevar cámara al origen |

A continuación, se adjunta el programa de Arduino para la ESP32 Wrover Cam.

Primero, se incluyen todas las librerias, variables y pines necesarios:

```arduino
#define CAMERA_MODEL_WROVER_KIT 
#define LED_BUILTIN 2
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"] "
#include "esp_camera.h"
#include <WiFi.h>
#include <string>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Stepper.h>
#include "camera_pins.h"

const int ledVerde = 33;
const int ledRojo = 32;
const int left = 14;
const int rigth = 12;
const int pinflash = 15;

const String ssid = "Iphone de David";
const String password = "12345678";

camera_config_t config;
void startCameraServer();
void config_init();

WiFiClient wClient;
PubSubClient mqtt_client(wClient);
const String mqtt_server = "iot.ac.uma.es";
const String mqtt_user = "II10";
const String mqtt_pass = "wD9IYehI";
const String ID_PLACA = String(ESP.getEfuseMac());
const String ID_PLACA_EMISORA = "36221669107172";
const String topicCamara = "II10/ESP" + ID_PLACA_EMISORA + "/mando/camara";
const String topic_PUB_conexion = "II10/ESP"+ID_PLACA+"/camara/conexion";
const String topic_PUB_flash = "II10/ESP"+ID_PLACA+"/camara/responseFlash";
const String topic_SUB_flash = "II10/ESP"+ID_PLACA+"/camara/requestFlash";
const String topic_PUB_foto = "II10/ESP"+ID_PLACA+"/camara/fotoRutinaria";
const String topic_SUB_tiempo = "II10/ESP"+ID_PLACA+"/camara/tiempo";
const String topic_PUB_ip = "II10/ESP"+ID_PLACA+"/camara/IP";
const String topic_SUB_ip = "II10/ESP"+ID_PLACA+"/camara/IPRequest";
const String mensaje_conexion = "{\"online\":true}";
const String mensajefoto = "{\"foto_rutinaria\":1}";
const String LWT = "{\"online\":false}";


int tiempo_foto = 60; // por defecto, se toma una foto de seguridad cada 60 minutos
```
Se añaden todas las subfunciones necesarias para realizar las conexiones de Wifi y MQTT:

```arduino
void conecta_wifi() {
  Serial.println(DEBUG_STRING+"Conectando a " + ssid);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    delay(200);
    Serial.print(".");
    digitalWrite(ledRojo,HIGH);
    digitalWrite(ledVerde,LOW);
  }
  digitalWrite(LED_BUILTIN,HIGH);
  Serial.println();
  Serial.println(DEBUG_STRING+"WiFi conectada, dirección IP: " + WiFi.localIP().toString());
}
//-----------------------------------------------------------
void conecta_mqtt() {// Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print(DEBUG_STRING+"Attempting MQTT connection...");// Attempt to connect
    if (mqtt_client.connect(ID_PLACA.c_str(),mqtt_user.c_str(),mqtt_pass.c_str(),topic_PUB_conexion.c_str(),2,false,LWT.c_str())) {
      Serial.println(" conectado a broker: " + mqtt_server);
        mqtt_client.subscribe(topic_SUB_flash.c_str());  
        mqtt_client.subscribe(topic_SUB_tiempo.c_str()); 
        mqtt_client.subscribe(topicCamara.c_str());
        mqtt_client.subscribe(topic_SUB_ip.c_str());
        mqtt_client.publish(topic_PUB_ip.c_str(), (WiFi.localIP().toString()).c_str());
    } else {
      Serial.println("ERROR:"+ String(mqtt_client.state()) +" reintento en 5s" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
```
Subfunción para la gestión de los mensajes recibidos por MQTT. Aquí se le pasa al Arduino Nano la información de los comandos de movimiento de la cámara, se actúa sobre el estado del flash y se manda la IP de la cámara:
```arduino
void procesa_mensaje(char* topic, byte* payload, unsigned int length) { 
  String mensaje=String(std::string((char*) payload,length).c_str());
  Serial.println("Mensaje recibido ["+ String(topic) +"] "+ mensaje);
  if(String(topic)==topic_SUB_tiempo){
    DynamicJsonDocument mensaje_json(50);
    DeserializationError error = deserializeJson(mensaje_json,mensaje);
    if(not error){
      tiempo_foto = mensaje_json["tiempo"];
      Serial.println(DEBUG_STRING + "Actualizando tiempo de foto de seguridad a "+tiempo_foto+" minutos.");
    }
  }else if(String(topic)==topic_SUB_ip){
    mqtt_client.publish(topic_PUB_ip.c_str(), (WiFi.localIP().toString()).c_str());
  }else if(String(topic)==topic_SUB_flash){
    DynamicJsonDocument mensaje_json(50);
    DeserializationError error = deserializeJson(mensaje_json,mensaje);
    if(not error){
      if(mensaje_json["flash"]==1){
        digitalWrite(pinflash, HIGH);
        Serial.println(DEBUG_STRING + "Encendiendo Flash...");
      }else{
        digitalWrite(pinflash, LOW);
        Serial.println(DEBUG_STRING + "Apagando Flash...");
      }
      mqtt_client.publish(topic_PUB_flash.c_str(), mensaje.c_str());
    }
  }else if (String(topic) == topicCamara) {
    DynamicJsonDocument mensaje_json(200);
    DeserializationError error = deserializeJson(mensaje_json, mensaje);
    if (!error) {
      if(int(mensaje_json["X"])>0){
        Serial.println("Girando a la derecha");
        digitalWrite(left, LOW);
        digitalWrite(rigth, HIGH);
      }else if(int(mensaje_json["X"])<0){
        Serial.println("Girando a la izquierda");
        digitalWrite(left, HIGH);
        digitalWrite(rigth, LOW);
      }else if (int(mensaje_json["Y"])>0){
        Serial.println("Cambiando flash");
        digitalWrite(pinflash, !digitalRead(pinflash));
        String msg;
        if(digitalRead(pinflash)){
          msg = "{\"flash\":1}";
        }else{
          msg = "{\"flash\":0}";
        }
        mqtt_client.publish(topic_PUB_flash.c_str(), msg.c_str());
      }else if (int(mensaje_json["Y"])<0){
        Serial.println("Centrando cámara");
        digitalWrite(left, HIGH);
        digitalWrite(rigth, HIGH);
      }else if (int(mensaje_json["X"])==0){
        Serial.println("Parando motor");
        digitalWrite(left, LOW);
        digitalWrite(rigth, LOW);
      }
    }
  }
}
```
Subfunción para la configuración de la instancia de la cámara. Dicha clase se define en la librería "esp_camera.h", incluida en este programa. Lo más relevante en esta subfunción es la calidad de /fotos/imagen configurada, y el número de buffers de lectura del sensor:
```arduino
void config_init() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_HD;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;
}
```
Setup del programa. Se realizan todas las incializaciones necesarias:
```arduino
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  pinMode(left,OUTPUT);
  pinMode(rigth,OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);
  pinMode(pinflash, OUTPUT);
  config_init();
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 0);        //1-Upside down, 0-No operation
  s->set_hmirror(s, 0);      //1-Reverse left and right, 0-No operation
  s->set_brightness(s, 1);   //up the blightness just a bit
  s->set_saturation(s, -1);  //lower the saturation

  conecta_wifi();
  startCameraServer();
  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes
  mqtt_client.setCallback(procesa_mensaje);
  conecta_mqtt();

  String flash_inicial = "{\"flash\": 0}";
  if (!mqtt_client.connected()) {
    conecta_mqtt();
  }else{
    mqtt_client.publish(topic_PUB_flash.c_str(),flash_inicial.c_str());
  }
}
```
Bucle principal del programa. Se comprueba constantemente la conexión de la cámara tanto a Wifi como a MQTT, y se manda un comando de foto en un intervalo de tiempo configurado por defecto a 60 minutos, aunque es configurable por el usuario mediante MQTT en  la función callback"procesa_mensaje" descrita anteriormente:
```arduino
void loop() {
  static unsigned long ultimo_mensaje=0;
  static unsigned long ahora=0;
  ahora = millis();

  if(WiFi.status() != WL_CONNECTED) {
    Serial.println(DEBUG_STRING + "Reconectando a Wifi...");
    conecta_wifi();
    digitalWrite(ledRojo,HIGH);
    digitalWrite(ledVerde,LOW);
  }else{
    digitalWrite(ledRojo,LOW);
    digitalWrite(ledVerde,HIGH);
  }

  if (!mqtt_client.connected()) {
    Serial.println(DEBUG_STRING + "Reconectando a MQTT...");
    conecta_mqtt();
  }

  mqtt_client.loop(); // esta llamada para que la librería recupere el control

  if (ahora - ultimo_mensaje >= tiempo_foto*60000) { // Mandamos foto automáticamente cada "tiempo_foto" minutos
    ultimo_mensaje = ahora;
    mqtt_client.publish(topic_PUB_foto.c_str(), mensajefoto.c_str());
  }
}
```

A continuación, se adjunta el código del Arduino Nano.

Primero, se incluyen todas las librerias, variables y pines necesarios:
```arduino
#include <Stepper.h>

// Pines de control del motor paso a paso
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;
const int buttonPin = 6;
const int left = 7;
const int rigth = 8;

int posicionActual = 0;
const int angle = 10;
bool origin = false; // inicializamos al modo de funcionamiento normal
const int stepsPerRevolution = 2048;  // Cambiar según la especificación del motor
const int vel = 8;

Stepper myStepper(stepsPerRevolution,IN1,IN3,IN2,IN4);
```
Subfunciones auxiliares para la conversión de pasos a grados y viceversa:
```arduino
//-----------------------------------------------------
int degree2steps(const int degrees) {
  int steps = int(degrees * stepsPerRevolution / 360);
  return steps;
}
//-----------------------------------------------------
int steps2degree(const int steps) {
  int degree = int(steps * 360 / stepsPerRevolution);
  return degree;
}
```
Subfunción para realizar la calibración de la cámara. Se hace girar el motor en sentido antihorario hasta que se detecte una pulsación del final de carrera, momento en el que se considera que la cámara está en la posición 0.
```arduino
void recuperarOrigen() {
  // Nos movemos al origen
  while (!origin) {
    if(digitalRead(buttonPin)) {
      Serial.println("Pulsación detectada");
      origin = true;
    } else {
      myStepper.step(-1);
    }
  }
  // Fijamos de nuevo el origen compensando el desfase para poder hacer movimientos relativos
  posicionActual = 0;
  Serial.println("Calibracion completada");
}
```
Subfunción para el control del motor. En función del comando recibido a través de los pines configurados, se realiza un movimiento en un sentido u otro:
```arduino
void controlMotor(const int comando) {
  if ((comando > 0) && (posicionActual < 360 )) { // Comprobamos que estamos dentro del rango
    Serial.println("Moving " + String(angle) + " degrees");
    myStepper.step(degree2steps(angle));
    posicionActual = posicionActual + angle;
    Serial.println("Movement completed");
  } else if ((comando < 0) && (posicionActual > 0)) {  // Comprobamos que estamos dentro del rango
    Serial.println("Moving " + String(angle) + " degrees");
    myStepper.step(degree2steps(-angle));
    posicionActual = posicionActual - angle;
    Serial.println("Movement completed");
  }
}
```
Setup del programa. Se hacen todas las inicializaciones necesarias, y se llama a la función "calibrarOrigen" para llevar a la cámara a su posición inicial: 
```arduino
void setup() {
  // Inicializar la comunicación serial
  Serial.begin(115200);
  pinMode(buttonPin, INPUT); // Configuramos el pin del botón
  pinMode(left,INPUT);
  pinMode(rigth,INPUT);
  // Configura el objeto AccelStepper
  myStepper.setSpeed(vel);  
  recuperarOrigen();
  origin = false;
}
```
Bucle principal del programa. Se comprueba constantemente el estado de los pines de comando para realizar su acción correspondiente en consecuencia:
```arduino
void loop() {
  if((digitalRead(left))&&(digitalRead(rigth))){
    Serial.println("Centrando cámara");
    recuperarOrigen();
  }else if((digitalRead(left))&&(!digitalRead(rigth))){
    Serial.println("Girando a la izquierda");
    controlMotor(-1);
    origin = false;
  }else if((!digitalRead(left))&&(digitalRead(rigth))){
    Serial.println("Girando a la derecha");
    controlMotor(1);
    origin = false;
  }
}
```
#### Diseño 3D de la carcasa de la cámara y su sistema de movimiento

Para alojar la cámara y todos los elementos que componen su sistema de movimiento, se ha usado la herramienta Fusion 360 de Autodesk. El diseño está compuesto de cuatro partes principales.

##### Carcasa de la ESP32 Wrover Cam

Para el diseño de dicha carcasa, se ha partido de un diseño de internet para una ESP32 común, y se le han realizado todas las modificaciones necesarias para poder alojar nuestra ESP32 Wrover Cam. La carcasa se ha diseñado con un sistema de capas para la conexión de los cables y su traspaso a la caja por el eje que une la cámara al motor. Además, en la parte frontal se ha implantado un diseño de rejilla para proporcionarle ventilación al chip de la placa. Dicho diseño queda reflejado en la siguiente /fotos/imagen.

<p align="center">
  <img src="/fotos/image-16.png" alt="Descripción de la /fotos/imagen">
</p>

##### Eje de movimiento

Para unir la carcasa de la cámara con el eje del motor, se ha diseñado un eje que además de realizar la función de unión entre ambos componentes, proporcione tanto un hueco para el cable de la cámara como para el resto de cables de los pines, uniéndose la desembocadura de ambos zócalos al final del eje. Además, se han añadido elementos para la unión de las piezas despúes de su impresión, y un mástil que será el encargado de accionar el final de carrera en la calibración de la posición de la cámara. También se ha añadido un anillo al rededor del eje que evite el movimiento de la cámara en el eje Z. Dicho diseño queda reflejado en la siguiente /fotos/imagen.

<p align="center">
  <img src="/fotos/image-17.png" alt="Descripción de la /fotos/imagen">
</p>

##### Zócalo para los componentes del sistema de movimiento

La siguiente pieza es el zócalo donde se alojan todos los componentes del sistema de movimiento, como son el propio motor paso a paso, su driver, el Arduino Nano y el final de carrera. Se une a la caja del sistema de movimiento mediante unos taladros donde posteriormente irán alojados unos tornillos. Se han suavizado toods los bordes de la pieza para evitar que un cable accidentalmente pueda dañarse con algún borde filoso. Dicho diseño queda reflejado en la siguiente /fotos/imagen.

<p align="center">
  <img src="/fotos/image-18.png" alt="Descripción de la /fotos/imagen">
</p>

##### Caja del sistema de movimiento

La siguiente pieza es la caja donde se aloja todo el sistema de movimiento de la cámara. De nuevo, en la tapa se ha instaurado una rejilla para permitir la ventilación de todos los componentes. Disponde de agujeros para la colocación de los leds de indicación de conexión y el flash, y para la salida de los cables de alimentación. Dicho diseño queda reflejado en la siguiente /fotos/imagen.

<p align="center">
  <img src="/fotos/image-19.png" alt="Descripción de la /fotos/imagen">
</p>

##### Diseño conjunto

Todo el diseño en conjunto viene reflejado en la siguiente /fotos/imagen.

<p align="center">
  <img src="/fotos/image-20.png" alt="Descripción de la /fotos/imagen">
</p>

Después de la impresión y el montaje, obtenemos el resultado reflejado en la siguiente /fotos/imagen.

<p align="center">
  <img src="/fotos/image-21.png" alt="Descripción de la /fotos/imagen">
</p>

### Implementación de la pasarela Wifi para el Arduino Mega 2560

Finalmente, no hemos sido capaces de terminar de implementar esto, ya que podemos recibir los comandos de movimiento por MQTT pero no hemos podido hacer que el programa de Simulink de la Arduino Mega reciba dichos datos desde la pasarela. Nos ha dado problemas la comunicacíon serie entre ambos dispositivos. Hemos usado un logic level shifter para realizar dicha comunicación y asegurar que no hubiese problemas con la diferencia de niveles de tensión de procesamiento de ambos dispositivos, pero aún así finalmente no hemos conseguido establecer dicha comunicación, ya que constantemente aparecían valores aleatorios o sin sentido en el buffer de datos de llegada, por lo que no hemos podido establecer el control manual para el Piero, aunque se deja abierta la opción de implementarla en un futuro.

### Diseño del flujo de Nodered

En NodeRed, tenemos un conjunto de bloques para el manejo de la cámara de seguridad y sus funciones, los cuales se detallarán a continuación.

El primer bloque es el dedicado a la lógica para hacer fotos de forma manual, comando que se puede realizar a través de las tres plataformas ofrecidas al usuario, como son NodeRed, Telegram y el mando wifi. Además, se ofrece al usuario la posibilidad de escoger entre hacer la foto con o sin flash. La foto se realiza mediante una petición http al API “/capture” del servidor local de la cámara, gracias al nodo http request. Dicha foto es devuelta en formato buffer de datos, e introducida en el campo content del mensaje de respuesta al nodo Telegram sender. Además, dicha foto es convertida a formato cadena para su almacenamiento en la base de datos denominada “fotos_manual”. Para añadir robustez al programa, se ha añadido una comprobación para verificar si el intento de hacer la foto se ha llevado a cabo satisfactoriamente o no, valorando si el payload que devuelve el http request devuelve error o no, de cara a no almacenar ni enviar fotos corruptas.

<p align="center">
  <img src="/fotos/image-22.png" alt="Descripción de la /fotos/imagen">
</p>

El siguiente bloque es el dedicado a la toma de fotos automáticas, la cual se hace por defecto cada 60 minutos, tal y como se explicó en su correspondiente programa de Arduino. En el dashboard de NodeRed se dispone de un nodo slider para poder configurar el tiempo de guardado de dichas fotos automáticas, el cual puede configurarse desde 1 a 60 minutos. Dichas fotos se guardan en una base de datos denominada “fotos_auto”. 

<p align="center">
  <img src="/fotos/image-23.png" alt="Descripción de la /fotos/imagen">
</p>

El siguiente bloque es el dedicado a la recuperación de las fotos guardas en las dos bases de datos, en función del botón que use el usuario en la petición desde Telegram. De nuevo, se hace una comprobación del estado de la foto antes de mandarla.

<p align="center">
  <img src="/fotos/image-24.png" alt="Descripción de la /fotos/imagen">
</p>

El siguiente bloque es el dedicado a la gestión de las peticiones de control del motor de la cámara y del flash manual, tanto desde la dashboard de NodeRed como desde los botones disponibles en Telegram.

<p align="center">
  <img src="/fotos/image-25.png" alt="Descripción de la /fotos/imagen">
</p>

El siguiente bloque es el dedicado a la gestión de las peticiones de control del Piero, tanto desde la dashboard de NodeRed como desde los botones disponibles en Telegram.

<p align="center">
  <img src="/fotos/image-26.png" alt="Descripción de la /fotos/imagen">
</p>

El siguiente bloque es el encargado de gestionar el video en streaming en la dashboard de NodeRed. Consta de un nodo template en el que hay un código html básico que encierra el contenido de la imagen que transmite en directo la cámara, haciendo una petición al servidor que implanta la cámara en el área local. Además, también se gestiona la petición desde Telegram para ver el video en streaming, devolviendo ante dicha petición la URL del servidor donde se encuentra alojado el video en streaming de la cámara.

<p align="center">
  <img src="/fotos/image-27.png" alt="Descripción de la /fotos/imagen">
</p>

Por último, nos encontramos unos nodos auxiliares para hacer un vaciado rápido de las fotos de las bases de datos en caso de que necesitemos hacer una depuración. Estos no se utilizarán normalmente.

<p align="center">
  <img src="/fotos/image-28.png" alt="Descripción de la /fotos/imagen">
</p>

### Usabilidad del entorno Dashboard de NodeRed y de Telegram 

La dasboard que implementa NodeRed con el flujo anteriormente descrito tiene el siguiente aspecto:

<p align="center">
  <img src="/fotos/image-29.png" alt="Descripción de la /fotos/imagen">
</p>

Como se puede observar, por un lado aparece la imagen de la cámara en directo, y además una serie de botones con los que poder interactuar con todos los elementos del sistema, cumpliendo la misma función que el mando diseñado.

Para Telegram, hemos creado un bot controlado mediante el flujo de NodeRed explicado anteriormente. Al iniciarlo, nos ofrece un menú con el que podemos interactuar y de nuevo, realizar la mismas funciones que cumplía el mando.

Al realizar cualquier acción, se notificará a Telegram la acción realizada. Además, haciendo uso del botón "Ver Streaming" del menú "Control Cámara", se nos devolverá un enlace a la dasboard de NodeRed, donde podemos ver el video en directo y de nuevo, interactuar con los elementos del sistema.

<div style="display: flex; justify-content: space-between;">
    <img src="/fotos/IMG_5391.PNG" style="width: 30%;">
    <img src="/fotos/IMG_5392.PNG" style="width: 30%;">
    <img src="/fotos/IMG_5393.PNG" style="width: 30%;">
</div>
