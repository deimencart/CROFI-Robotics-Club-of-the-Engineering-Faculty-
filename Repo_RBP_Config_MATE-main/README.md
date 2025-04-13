# Manipulador Móvil "Babas"


## 

| Código | Description |
| ------:| ----------- |
| ***Asignatura*** | Código del Trabajo o Número de Tarea | 
| **MRG-C002** | Robótica Móvil - Babas -  V1 |

## Contenido

- [Objetivo](#objetivo)
- [Introducción](#introduccion)
- [Desarrollo](#desarrollo)
- [Conclusiones](#conclusiones)
- [Autor](#autor)
- [Referencias](#referencias)


## Objetivo
Diseñar un robot omnidireccional mediante el análisis de ingeniería inversa en el DOFBOT y así obtener los parámetros de diseño para la base móvil

## Introducción
Palabras Clave
Palabras claves en forma Jerarquizada
- Diseño Mecatrónico
- Controlador de Motores
- Robot Móvil Omnidireccional
- Sistema Embebido Jerarquico
- Raspberry con ROS
- Microcontrolador
- Diseño electrónico (PCB)
- Comunicación
- Control de Velocidad


## Definiendo el tema

Se analizó el funcionamiento del robot DOFBOT con el fin de implementar un control utilizando el sistema embebido Raspberry Pi y una tarjeta de desarrollo programada en Arduino para adaptarlo a una base móvil omnidireccional obteniendo un manipulador móvil. 

## Desarrollo
Metas del proyecto
Están relacionadas directamente con las palabras clave citadas anteriormente, Se presentan los ŕimeros cinco puntos en esta lista. Se dejó instalado un sistema funcional en ROS NOETIC que incorpora un paquete dedicado al proyecto, enlazado con otro paquete llamado "vision_opencv" que es el puente entre las imágenes generadas por OpenCV a los mensajes que manda ROS internamente.  

Tiene un funcionamiento jerárquico para obtener los parámetros de diseño de una base móvil  omnidireccional y se obtuvo la información de su funcionamiento mediante la implementación de la ingeniería inversa. 


Productos: 

- Repositorio de Github
- Instrucciones de como se instala ROS en Raspbian 
- Implementación de la API de Arm_lib
- Programas Muestra 
- Topología del diseño del sistema
- Parámetros del diseño 


## Conclusiones
 
## Referencias
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


