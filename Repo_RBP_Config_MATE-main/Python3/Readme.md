## Lineamientos los códigos de python

# Contenido 

- [Consideraciones para la Raspberry usando Ubuntu Mate](#consideraciones-para-la-raspberry-usando-ubuntu-mate)
- [Instalando API Arm_ Lib y Smbus](#instalando-api-arm_-lib-y-smbus)
- [Referencias](#referencias)
# Consideraciones para la Raspberry usando Ubuntu Mate

- Como esta distribución de ubuntu no contempla las modificaciones de de la raspberry se debe instalar el paquete de "raspi-config" para poder habilitar los pines de comunicacipon i2c, el puerto serial, los puertos GPIO, entre otros. [[1]](#1)

1) Abrir una terminal y escribir: 

```
sudo su
```
(Si no se escribe, no funcionará)

2) Copiar y pegar. (línea por linea): 
```
$ echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list

$ apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E

$ apt-get update && apt-get install raspi-config
```

# Instalando API Arm_ Lib y Smbus
- Para que el manipulador del bobot, se ocupa la API de Arm_lib, que tiene comunicación i2c con los servos. Solamente se debe ingresar la siguiente línea a la terminal para poder usarla en los scripts de python3: 
```
sudo pip3 install Arm_lib
```

- Finalmente para que la comunicación funcione correctamente, se necesita instalar el paquete smbus: 

```
sudo apt-get install -y python3-smbus
```

- Con esto ya se podrían correr los scripts que están en la carpeta (después de haber instalado OpenCV con el paquete de ROS vision_opencv)

# Referencias 
<a id="1">[1]</a> "Install raspi-config on Ubuntu MATE 20.10 and higher". Ubuntu MATE Community. https://ubuntu-mrate.community/t/install-raspi-config-on-ubuntu-mate-20-10-and-higher/23974 (accedido el 13 de mayo de 2022).
