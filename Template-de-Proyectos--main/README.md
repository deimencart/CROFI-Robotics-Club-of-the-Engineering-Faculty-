# [Título del trabajo/tarea/(módulo del proyecto)] 


| Código | Description |
| ------:| ----------- |
| ***Asignatura*** | Código del Trabajo o Número de Tarea | 
| **TSR-2022-I** | Tarea *01 ... n* |


## Contenido

- [Objetivo](#objetivo)
- [Introducción](#introduccion)
- [Desarrollo](#desarrollo)
- [Conclusiones](#conclusiones)
- [Autor](#autor)
- [Referencias](#referencias)

## Objetivo

Redacción del objetivo del trabajo o texto que describe a la tarea (_según sea el caso_).

## Introducción

Párrafo de introducción del trabajo o tarea (_si aplica_). 

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed
do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim
veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo
consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse
cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non
proident, sunt in culpa qui officia deserunt mollit anim id est laborum.


## Desarrollo

Para consultar el formato a este documento, visitar [Markdown 101](https://github.com/decidim-archive/docs-template/blob/master/es/markdown-101.md).
ver en [texto plano](https://raw.githubusercontent.com/decidim-archive/docs-template/master/es/markdown-101.md)

Ejemplo de párrafo

Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod
tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At
vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren,
no sea takimata sanctus est Lorem ipsum dolor sit amet.

Bloques de cita

> "If it weren't for my lawyer, I'd still be in prison.
>  It went a lot faster with two people digging."
>
> --- Joe Martin

Bloque de cita con vínculo de referencia. 

> Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod
> tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. 
>
> At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren,
> no sea takimata sanctus est Lorem ipsum dolor sit amet [[1]](#1). **<- referencia insertada** 

vinculo a otro [documento](/docs/document-template.md) en el repositorio (_ruta relativa_).

Imágenes

![Markdown Guide](/images/markdown-logo.png) Markdown Guide [Basic Syntax](https://www.markdownguide.org/basic-syntax/), [Extended Syntax](https://www.markdownguide.org/extended-syntax/)

Ejemplo de la estructura del directorios del repositorio: 

Proyecto de **ROS** con los directorios **adicionales** para almacenar **imágenes** y **documentos** referentes al **proyecto**.

**Nota:** La estructura mostrada representa -en su mayoría- a los directorios más usados dentro de un proyecto de **ROS**.

```text
 proyecto/
    ├── images/
    │   ├── imagen1.png
    │   ├── imagen2.png
    │   ├── imagen3.gif
    │   └── imagen5.svg
    ├── docs/
    │   ├── archivo1.md
    │   ├── archivo2.pdf
    │   ├── archivo3.txt
    │   └── archivo2.pdf
    ├── src/
    │   ├── include/
    │   │   ├── lib1.h
    │   │   └── libcpp.so
    │   ├── config/
    │   │   ├── config-file01.yaml
    │   │   └── config-file02.yaml
    │   ├── meshes/
    │   │   ├── visual/
    │   │   │   ├── mesh-file.stl
    │   │   │   └── mesh-file.dae
    │   │   └── collision/
    │   │       ├── mesh-file.stl
    │   │       └── mesh-file.dae
    │   ├── launch/
    │   │   ├── launch-file1.launch
    │   │   ├── rviz-file1.rviz
    │   │   ├── rviz-file2.rviz
    │   │   └── launch-file3.launch
    │   ├── msg/
    │   │   ├── Message-file1.msg
    │   │   └── Message-file2.msg
    │   ├── srv/
    │   │   ├── ServiceMsg-file1.srv
    │   │   └── ServiceMsg-file2.srv
    │   ├── action/
    │   │   ├── ActionMsg-file1.action
    │   │   └── ActionMsg-file2.action
    │   ├── urdf/
    │   │   ├── urdf-file.urdf
    │   │   └── xacro-file.xacro
    │   ├── scripts/
    │   │   ├── pyscript-file.py
    │   │   └── cpp-programm.cpp
    │   ├── src/
    │   │   ├── cpp-programm.cpp
    │   │   └── pyprogramm-file.py
    │   ├── Otros-archivos-del-proyecto.txt
    │   ├── package.xml
    │   └── CMakeLists.txt
    ├── .gitignore
    ├── CMakeLists.txt
    ├── LICENSE.md
    ├── Otros-archivos-generales.txt
    └── Readme.md
```

***Bloques de código***

Bloques de código "cercados"

```
Texto de ejemplo aquí...
```

De acuerdo a la sintaxis del lenguaje de programación, ver más [información](https://docs.github.com/es/github/writing-on-github/working-with-advanced-formatting/creating-and-highlighting-code-blocks).

``` js
var foo = function (bar) {
  return bar++;
};

console.log(foo(5));
```

**Ecuaciones**

Hay dos modos en los que representaremos ecuaciones matemáticas en Github - Markdown:
- Localmente (con MathJax y SVG relativo), y ...
- De forma remota (con el servidor de renderizado LaTeX de GitHub).

En realidad, esto es un truco. GitHub **no representará ecuaciones LaTeX** dentro de lugares normales como GitHub README, pero puede representarlas en cuadernos de Jupyter, por lo que aprovechamos esta función, utilizando el servidor de representación de ecuaciones de GitHub para incrustar ecuaciones SVG (formato gráfico vectorial) en GitHub. (Consulte aquí para obtener más detalles: [un truco para mostrar fórmulas LaTeX en GitHub markdown](https://gist.github.com/a-rodin/fef3f543412d6e1ec5b6cf55bf197d7b)).

Básicamente, podemos convertir una ecuación matemática estándar de LaTeX como la _Distribución Normal Gaussiana_...

```latex
$$
P(x) = \frac{1}{\sigma\sqrt{2\pi}} e^{\frac{-(x-\mu)^2}{2\sigma^2}}
$$
```
a una etiqueta de imagen renderizada con la ayuda del servidor de renderizado matemático de GitHub:

```html
<div align="center"><img style="background: white;" src="https://render.githubusercontent.com/render/math?math=P(x)%20%3D%20%5Cfrac%7B1%7D%7B%5Csigma%5Csqrt%7B2%5Cpi%7D%7D%20e%5E%7B%5Cfrac%7B-(x-%5Cmu)%5E2%7D%7B2%5Csigma%5E2%7D%7D"></div>
```
<div align="center"><img style="background: white;" src="https://latex.codecogs.com/svg.latex?P(x)%20%3D%20%5Cfrac%7B1%7D%7B%5Csigma%5Csqrt%7B2%5Cpi%7D%7D%20e%5E%7B%5Cfrac%7B-(x-%5Cmu)%5E2%7D%7B2%5Csigma%5E2%7D%7D"></div>

Además del servidor de renderizado de GitHub, también agregamos soporte para el servidor de renderizado de [CodeCogs](https://latex.codecogs.com/), [documentación](https://editor.codecogs.com/docs/):

```html
<div align="center"><img style="background: white;" src="https://latex.codecogs.com/svg.latex?P(x)%20%3D%20%5Cfrac%7B1%7D%7B%5Csigma%5Csqrt%7B2%5Cpi%7D%7D%20e%5E%7B%5Cfrac%7B-(x-%5Cmu)%5E2%7D%7B2%5Csigma%5E2%7D%7D"></div>
```

<div align="center"><img style="background: white;" src="https://latex.codecogs.com/svg.latex?P(x)%20%3D%20%5Cfrac%7B1%7D%7B%5Csigma%5Csqrt%7B2%5Cpi%7D%7D%20e%5E%7B%5Cfrac%7B-(x-%5Cmu)%5E2%7D%7B2%5Csigma%5E2%7D%7D"></div>

Para más información, puedes consultar ["_VS Code Math to Image: Write LaTeX Math Equations in GitHub Markdown the Easy Way!_"](https://medium.com/spencerweekly/vs-code-math-to-image-write-latex-math-equations-in-github-markdown-the-easy-way-9fa8b81dc910), o su [repositorio](https://github.com/TeamMeow/vscode-math-to-image#readme) de GitHub.

Ejemplos:

```latex
\mathbb{R} \ =\ \begin{bmatrix}
n & o & a
\end{bmatrix} \ =\ \begin{bmatrix}
n_{x} & o_{x} & a_{x}\\
n_{y} & o_{y} & a_{y}\\
n_{z} & o_{z} & a_{z}
\end{bmatrix}
```

Forma 1, usando _div_:

<div align="center"><img style="background: white;" src="https://latex.codecogs.com/svg.image?%5Cmathbb%7BR%7D%20%5C%20=%5C%20%5Cbegin%7Bbmatrix%7Dn%20&%20o%20&%20a%5Cend%7Bbmatrix%7D%20%5C%20=%5C%20%5Cbegin%7Bbmatrix%7Dn_%7Bx%7D%20&%20o_%7Bx%7D%20&%20a_%7Bx%7D%5C%5Cn_%7By%7D%20&%20o_%7By%7D%20&%20a_%7By%7D%5C%5Cn_%7Bz%7D%20&%20o_%7Bz%7D%20&%20a_%7Bz%7D%5Cend%7Bbmatrix%7D"></div>

Forma 2, usando sintaxis de _img_ markdown: 

![Ecuacion 1](https://latex.codecogs.com/svg.image?%5Cmathbb%7BR%7D%20%5C%20=%5C%20%5Cbegin%7Bbmatrix%7Dn%20&%20o%20&%20a%5Cend%7Bbmatrix%7D%20%5C%20=%5C%20%5Cbegin%7Bbmatrix%7Dn_%7Bx%7D%20&%20o_%7Bx%7D%20&%20a_%7Bx%7D%5C%5Cn_%7By%7D%20&%20o_%7By%7D%20&%20a_%7By%7D%5C%5Cn_%7Bz%7D%20&%20o_%7Bz%7D%20&%20a_%7Bz%7D%5Cend%7Bbmatrix%7D)

Ejemplo de inserción de funcion en texto con referencia bibliográfica:

**Convertir un cuaternión en una matriz de rotación**

Dado un cuaternión <img style="background: white;" src="https://latex.codecogs.com/svg.image?%5Cinline%20q%20=%20%5Cleft%20(%20q_%7B0%7D,%20q_%7B1%7Di,%20q_%7B2%7Dj,%20q_%7B3%7Dk%20%5Cright%20)%20">, puede encontrar la matriz de rotación tridimensional correspondiente utilizando la siguiente fórmula [[2]](#2). **<- referencia insertada**

<div align="center"><img src="https://latex.codecogs.com/svg.image?%5Cmathbb%7BR%7D%5Cleft%20(%20Q%20%5Cright%20)=%20%5Cbegin%7Bbmatrix%7D%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B1%7D%5E%7B2%7D%20%5Cright%20)%20-%201&%202%5Cleft%20(%20q_%7B1%7Dq_%7B2%7D%20-%20%20q_%7B0%7Dq_%7B3%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B1%7Dq_%7B3%7D%20&plus;%20%20q_%7B0%7Dq_%7B2%7D%20%5Cright%20)%20%5C%5C2%5Cleft%20(%20q_%7B1%7Dq_%7B2%7D%20&plus;%20%20q_%7B0%7Dq_%7B3%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B2%7D%5E%7B2%7D%20%5Cright%20)%20-%201%20&%202%5Cleft%20(%20q_%7B2%7Dq_%7B3%7D%20-%20%20q_%7B0%7Dq_%7B1%7D%20%5Cright%20)%20%5C%5C2%5Cleft%20(%20q_%7B1%7Dq_%7B3%7D%20-%20%20q_%7B0%7Dq_%7B2%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B2%7Dq_%7B3%7D%20&plus;%20%20q_%7B0%7Dq_%7B1%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B3%7D%5E%7B2%7D%20%5Cright%20)%20-%201%20%5C%5C%5Cend%7Bbmatrix%7D"></div>

**Nota**: Para los que usamos el tema obscuro de GitHub las imágenes no se visualizan correctamente ya que la fuente es color negro sobre fondo transparente, la solución es presentar la ecuación como png de la siguiente manera:

```html
<div align="center"><img src="https://latex.codecogs.com/png.image?%5Cdpi%7B110%7D%20%5Cbg_white%20%5Cmathbb%7BR%7D%5Cleft%20(%20Q%20%5Cright%20)=%20%5Cbegin%7Bbmatrix%7D%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B1%7D%5E%7B2%7D%20%5Cright%20)%20-%201&%202%5Cleft%20(%20q_%7B1%7Dq_%7B2%7D%20-%20%20q_%7B0%7Dq_%7B3%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B1%7Dq_%7B3%7D%20&plus;%20%20q_%7B0%7Dq_%7B2%7D%20%5Cright%20)%20%5C%5C2%5Cleft%20(%20q_%7B1%7Dq_%7B2%7D%20&plus;%20%20q_%7B0%7Dq_%7B3%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B2%7D%5E%7B2%7D%20%5Cright%20)%20-%201%20&%202%5Cleft%20(%20q_%7B2%7Dq_%7B3%7D%20-%20%20q_%7B0%7Dq_%7B1%7D%20%5Cright%20)%20%5C%5C2%5Cleft%20(%20q_%7B1%7Dq_%7B3%7D%20-%20%20q_%7B0%7Dq_%7B2%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B2%7Dq_%7B3%7D%20&plus;%20%20q_%7B0%7Dq_%7B1%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B3%7D%5E%7B2%7D%20%5Cright%20)%20-%201%20%5C%5C%5Cend%7Bbmatrix%7D"></div>
```

<div align="center"><img src="https://latex.codecogs.com/png.image?%5Cdpi%7B110%7D%20%5Cbg_white%20%5Cmathbb%7BR%7D%5Cleft%20(%20Q%20%5Cright%20)=%20%5Cbegin%7Bbmatrix%7D%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B1%7D%5E%7B2%7D%20%5Cright%20)%20-%201&%202%5Cleft%20(%20q_%7B1%7Dq_%7B2%7D%20-%20%20q_%7B0%7Dq_%7B3%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B1%7Dq_%7B3%7D%20&plus;%20%20q_%7B0%7Dq_%7B2%7D%20%5Cright%20)%20%5C%5C2%5Cleft%20(%20q_%7B1%7Dq_%7B2%7D%20&plus;%20%20q_%7B0%7Dq_%7B3%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B2%7D%5E%7B2%7D%20%5Cright%20)%20-%201%20&%202%5Cleft%20(%20q_%7B2%7Dq_%7B3%7D%20-%20%20q_%7B0%7Dq_%7B1%7D%20%5Cright%20)%20%5C%5C2%5Cleft%20(%20q_%7B1%7Dq_%7B3%7D%20-%20%20q_%7B0%7Dq_%7B2%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B2%7Dq_%7B3%7D%20&plus;%20%20q_%7B0%7Dq_%7B1%7D%20%5Cright%20)%20&%202%5Cleft%20(%20q_%7B0%7D%5E%7B2%7D%20&plus;%20q_%7B3%7D%5E%7B2%7D%20%5Cright%20)%20-%201%20%5C%5C%5Cend%7Bbmatrix%7D"></div>


Youtube videos

[![Markdown, Curso Práctico para principiantes y desarrolladores](https://img.youtube.com/vi/oxaH9CFpeEE/0.jpg)](https://www.youtube.com/watch?v=oxaH9CFpeEE)

## Conclusiones

Conclusiones o cierre al trabajo realizado.

Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod
tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At
vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren,
no sea takimata sanctus est Lorem ipsum dolor sit amet [[3]](#3). **<- referencia insertada en el párrafo** . 

## Autor

***[Nombre del autor o listado de los integrantes del equipo]***


o en caso de tratarse de un equipo

| Iniciales  | Description |
| ----------:| ----------- |
| **DMC**  | Diego Méndez Carter [GitHub profile](https://github.com/Laos198) |

## Referencias

_Referencia simple_

<a id="1">[1]</a>  I.A. Glover and P.M. Grant, Digital Communications, 3rd ed.  Harlow: Prentice Hall, 2009. 

_Referencia con vínculo insertado en el título del libro_

<a id='2'>[2]</a>	J. B. Kuipers, [Quaternions and rotation sequences](https://amzn.to/2RY2lwI). Princeton, NJ: Princeton University Press, 2002. (Chapter 5,  Section 5.14 “Quaternions to Matrices”, pg. 125)

_Referencia con url externo visible_

<a id="3">[3]</a>  H.-L. Pham, V. Perdereau, B. Adorno, en P. Fraisse, “Position and Orientation Control of Robot Manipulators Using Dual Quaternion Feedback”, 11 2010, bll 658–663. <https://www.researchgate.net/publication/224200087_Position_and_Orientation_Control_of_Robot_Manipulators_Using_Dual_Quaternion_Feedback>

**Nota**:

> Listado de referencias documentales consultadas para realizar el trabajo:
> consultar el siguiente [vínculo](https://www.bath.ac.uk/publications/library-guides-to-citing-referencing/attachments/ieee-style-guide.pdf)
> para el formato de referencia estilo **IEEE**.
> 
> ```text
> [Num ref] Iniciales del autor. Apellido del autor, Título del libro, edicion (si no es la primera). 
> Lugar de publicación: Publicador, Año.
> ```
