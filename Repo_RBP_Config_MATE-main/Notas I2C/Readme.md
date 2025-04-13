# CheatSheet del protocolo I2C


## Definición

- [Definición](#definicion)
- [Bus I2C en RBP](#bus-i2c-en-rbp-y-arduino)
- [Bus I2C en NODEMCU](#bus-i2c-en-nodemcu)
- [Conclusiones](#conclusiones)
- [Autor](#autor)
- [Referencias](#referencias)

## Definición

Inter Integrated Circuit, es un bus serie desarrollado por Phillips semiconductors ampliamente utilizado en la industria electrónica. Está formado por dos hilos que puede conectar varios dispositivos mediante un hardware sencillo (como se muestra en la siguiente imagen) 

![image](https://user-images.githubusercontent.com/20031100/172984479-cdf5ca22-ab79-4b45-9e5f-5ffd2f11864d.png)

Por los dos Hilos se produce la comunicación serie, bit a bit (unidad mínima de información). 
Donde:

- SCL, (Serial Clock) Es una señal de reloj que se utiliza para la sincronización
    
- SDA (Serial Data) Es la línea para la transferencia serie de Datos 
    
    
Los dispositivos concetados en el Bus I2C mantiene una relación entre ellos de maestro/esclavo: 

- El maestro inicia y termina la transferencia de información, además de controlar la señal de reloj (Generalmente es un Microcontrolador, pero en este caso será la RaspBerry) 
- El esclavo es el circiuto direccionado por le maestro

La línea SDA es bidireccional, por lo que el maestro y el esclavo actúan como transmisores o receptores de datos.

Es necesario considerar que dentro del cableado del circuito será prudente utilizar dos resistencias en configuración Pull-Up ya que dos o más señales a través del mismo cable pueden causar conflicto.

Se pueden conectar 128 dispositivos a la vez ya que las direcciones que maneja el BUS son de 7 bits. Aunque este tiene reservado 16 direccioens por lo que al final se pueden usar un máximo de 112 nodos entre sí. 



## Bus I2C en RBP y Arduino
Establecer comunicación entre la Raspberry Pi y Alguna tarjeta controladora puede ser útil ya que se puede aporvechar la potencia de computaciín y las entradas y salidas de cualquier tarjeta/microcontrolador. Se recomienda que la Raspberry y la tarjeta de control estén lo más cerca posible ya que al final los datos viajaran por un BUS de datos en dos cables y entre más largo sea, podría haber problemas de comunicación entre ellos. 


### Hardware 
- Computadora
- Arduino UNO x1
- Raspberry Pi 3B+
- Cables de arranque x3


### Diagrama 
- SDA BCM2(RPI) <-> SDA A4(Arduino)
- SCL BCM3(RPI) <-> SCL A5(Arduino)
- GND (RPI) <-> GND(Arduino)


![raspberry-pi-arduino-i2c-communication_bb](https://user-images.githubusercontent.com/20031100/179326716-03da0167-f8f1-4987-b03f-629e2c615427.png)


### Configuración de la Raspberry 
Para usar la interfaz I2C de Raspberry, debe estar habilitada en el menú de configuración. Se accede a él con el siguiente comando:

```
sudo raspi-config
```

(esta copnfiguración se detalla en la sección del repositorio sobre la instalación de UBUNTU MATE  de la raspberry) 

Se debe instalar también las herramientas para poder "interactuar" con el bus de Datos de la Raspberry. Este paquere se llama "i2c-tools". Para ello debemos ejecutar el siguiente comando: 


```
sudo apt-get install -y i2c-tools
```
Para ver los dispositivos conectados en el BUS i2c se usa el siguiente comando: 

```
i2cdetect -y 1
```
Obteniendo en la terminal algo similar a esto:

```
pi@raspberrypi:~ $ i2cdetect -y 1
    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- 0b -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

Deberemos seguir con la instalación de la biblioteca smbus2 que permite gestionar la comunicación i2c en la Raspberry Pi: 
```
pip3 install smbus2
```

Después de haber instalado las bibliotecas y los paquetes anteriores debemos escribir los códigos para hacer nuesto "Hola Mundo" del i2c de la raspberry: 

El siguiente código es hecho en python3, habiliatndo la configuración del maestro (la Raspberry) hacia el dispositivo esclavo (el Arduino) 

((Poner el código modificado de la raspberry)) 

```
from smbus import SMBus

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
bus.write_byte(addr, 0x1) # switch it on
input("Press return to exit")
bus.write_byte(addr, 0x0) # switch it on

```

y el siguiente código será el esclavo (el Arduino) recibiendo los datos para enceder y apagar un led dentro de la tarjeta (recibiendo 1 y 0) con la biblioteca "wire.h" de arduino, iniciando la comunicación i2c. 
```
#include <Wire.h> 
static_assert(LOW == 0, "Esperando LOW para que sea 0"); 
int ledPin = 13;
void setup() 
{
  Wire.begin(0x8); 
  Wire.onReceive(receiveEvent); 
  pinMode(ledPin, OUTPUT); 
  digitalWrite(ledPin, LOW); 
  Serial.begin(9600);
}


void loop() {
delay(100);
}
int receiveEvent(){
  while(Wire.available()){
    int c = Wire.read(); 
    Serial.println(c);
    return c; 
    digitalWrite(ledPin, c);
  } 
}

```

Conectando la Raspberry y el Arduino correctamente, en el bus de Datos debería verse la dirección 0x8 del bus encendida, ya que así fue puesto en el código del esclavo. (Algunos sensores/perifericos ya tienen direcciones predeterminadas)
```
pi@raspberrypi:~ $ i2cdetect -y 1
    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- 08 -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```


## Bus I2C en NODEMCU






## Conclusiones
 
## Referencias (Configurar Referencias)
https://www.youtube.com/watch?v=RkQbK9ek59c&ab_channel=ScottyD
https://www.aranacorp.com/es/comunicacion-i2c-entre-raspberry-pi-y-arduino/


https://www.allaboutcircuits.com/technical-articles/i2c-design-mathematics-capacitance-and-resistance/
https://www.todopic.com.ar/foros/index.php?topic=5979.0
https://www.diarioelectronicohoy.com/blog/introduccion-al-i2c-bus
https://aprendiendoarduino.wordpress.com/2017/07/09/i2c/#:~:text=I2C%20es%20un%20bus%20de,velocidades%20de%203.4%20Mbit%2Fs.
https://programarfacil.com/blog/arduino-blog/comunicacion-i2c-con-arduino/



<a id="1">[1]</a> "Choose an architecture | Download". Ubuntu MATE. https://ubuntu-mate.org/download/ (accedido el 13 de mayo de 2022).
<a id="2">[2]</a> "Noetic/Installation/Ubuntu - ROS wiki". Documentation - ROS Wiki. http://wiki.ros.org/noetic/Installation/Ubuntu (accedido el 13 de mayo de 2022).

<a id="3">[3]</a> "ROS/Tutorials/CreatingPackage - ROS wiki". Documentation - ROS Wiki. http://wiki.ros.org/ROS/Tutorials/CreatingPackage (accedido el 13 de mayo de 2022).

<a id="4">[4]</a> "ROS/Tutorials/BuildingPackages - ROS wiki". Documentation - ROS Wiki. http://wiki.ros.org/ROS/Tutorials/BuildingPackages (accedido el 13 de mayo de 2022).

<a id="5">[5]</a> "Vision_opencv - ROS wiki". Documentation - ROS Wiki. http://wiki.ros.org/vision_opencv (accedido el 13 de mayo de 2022).

<a id="6">[6]</a> "Install raspi-config on Ubuntu MATE 20.10 and higher". Ubuntu MATE Community. https://ubuntu-mate.community/t/install-raspi-config-on-ubuntu-mate-20-10-and-higher/23974 (accedido el 13 de mayo de 2022).

<a id="7">[7]</a> "How to install python3-smbus ubuntu package on ubuntu 20.04/ubuntu 18.04/ubuntu 19.04/ubuntu 16.04". Modern Server and App Hosting Control Panel. https://zoomadmin.com/HowToInstall/UbuntuPackage/python3-smbus (accedido el 13 de mayo de 2022).

<a id="8">[8]</a> "Arm_lib". PyPI. https://pypi.org/project/Arm_lib/ (accedido el 13 de mayo de 2022).
 
## Autor
| Iniciales  | Description |
| ----------:| ----------- |
| **DMC**  | Diego Méndez Carter [GitHub profile](https://github.com/Laos198) |
| **MGR-MX** | Mechatronics Research Group, México [GitHub profile](https://github.com/mrg-mx) |

