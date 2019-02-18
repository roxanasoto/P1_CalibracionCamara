# P1_CalibracionCamara
En este repositorio presentamos la implementacion de calibracion de càmara que es muy util para reconstrucción 3D, el cual fue implementado paso a paso usando un entorno de programacion Qt en c++, siguiendo el proceso de reconocimiento de circulos, anillos y haciendo el tracking y finalmente la calibracion de camara y el refinamiento siguiento el paper ![alt text](https://www.ri.cmu.edu/pub_files/2009/10/Calib.pdf "paper doc pdf"). para el proceso de refinamiento y el proceso de reconocimiento se utilizo un conjunto setting manual de 25 frames obteniendo un accuracy promedo de 0.3 al final para el video de prueba que se encuentra en la carpeta video(sin compresion y distorsion tomados en clase). el pipeline general se muestra acontinuacion.
![alt text](https://github.com/roxanasoto/P1_CalibracionCamara/blob/master/pipeline_calibration.png)

Estudiantes:
- Roxana Soto
- Wilderd Mamani

En este repositorio se presentan dos trabajos con respecto al curso de Imagenes Trabajo1 (Calibracion de Camara) y Trabajo 2 (Review de la Reconstruccion 3D de una imagen, mas detalles en carpeta trabajo 2).

## Dependencias y Ejecucion (Calibracion Camara)
- Qt creator (desarrollo visual windows)
- opencv2 (calibracion y funciones utils)
- mimpack (libreria levergentquack...)

## Trabajo 1
Proyecto de Calibracion de Camara usando videos en tiempo real, grabados en clases a 60fps sin compresion y distorsion.
### Codigos
#### CalibrationOpenCV
Este codigo hace uso de Opencv y de sus librerias para identificar los patrones de CHESSBOARD y CIRCLES
#### RCirculos
Este codigo hace uso de Opencv, de sus librerias y de métodos propios para identificar los patrones de CIRCLES y RING, además del tracking.

### Demos
#### Ejecucion de P1_calibracion
abrir el codigo con Qt creator todas las librerias y archivos estan incluidos en el proyecto, asi que solo abrimos el archivo RCirculos.pro y hacemos build en Qt creator y ejecutamos en run cargamos el archivos , elegimos anillos en la interface que nos muestra el Qt y probamos todos los metodos, los resultados de los diferentes procesmos se mostraran en las ventanitas y tambien algunos archivos de rms en cvs y tmb los frames capturados pueden ser guardados.
#### Semana2
#### Semana3

### Papers
#### Semana1
#### Semana2
#### Semana3

## Trabajo 2
Face 3D
