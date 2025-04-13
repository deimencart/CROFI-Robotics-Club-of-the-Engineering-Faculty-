## Bitácora

Hoy he llamado al robot "Dofbot"- por lo que me iré refiriendo
a él de desde ahora con ese nombre. 
En la Raspberry tiene las siguientes carcaterísticas:
 
	- Imagen con Raspbian Buster (legacy) 
	- Instalación de ROS Melodic

Para la instalación del sistema utilicé la siguiente WIKI
- https://www.linkedin.com/pulse/easiest-way-install-ros-melodic-raspberrypi-4-shubham-nandi
- http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi
NOTAS: 
La tarjeta arduino Uno del maestro Erik ya no funciona (al parecer) 
he dejado instalada una provisional que a veces falla. 



## 20/04/2022

Se volvió a instalar ROS en el sistema de Raspberry siguiendo los links de arriba y se establecieron las palabras claves del proyecto delimitando 
las fechas de entrega para 6 semanas. 

Palabras clave: 
- Diseño Mecatrónico
- Controlador de Motores 
- Robot Móvil Omnidireccional 
- Sistema Embebido Jerarquico 
- Raspberry con ROS 
## Punto de trabajo futuro
- Microcontrolador
- Diseño eléctrónico (PCB)
- Comunicación 
- Control de Velocidad



resolucion del problema smBus: 
https://github.com/johnbryanmoore/VL53L0X_rasp_python/issues/13
habilitando los ṕuertos i2c de la raspberry. 
https://blog.csdn.net/jcfszxc/article/details/123448078

## 27/04/2022

Se instaló OpenCV en la Raspberry y así mismo se bajo la base móvil del robot de Irene 
se quitaron Drivers MD25 y se piensa usar el por un momento los motores de estos 

Para instalar openCV y solucionar un problema con Numpy se ocuparon estas fuentes. 
https://www.piwheels.org/project/opencv-contrib-python/
https://stackoverflow.com/questions/20518632/importerror-numpy-core-multiarray-failed-to-import
https://programarfacil.com/blog/vision-artificial/opencv-raspberry-pi/

