# triangulacionDeBayas

CONTENIDO DE ESTE ARCHIVO
-------------------------

 * Introduccion
 * Dependencias
 * Instalación 
 * Uso


INTRODUCCION
------------

Este programa genera reconstrucciones 3D basadas en técnicas de Structure From Motion a partir de correspondencias de puntos 2D obtenidas manualmente por humanos en lugar de correspondencias visuales obtenidas por algún algoritmo como SIFT u ORB.


DEPENDENCIAS
------------
Este programa fue testeado en Ubuntu 20.04, pero debería ser facil de compilar en otras plataformas. Para funcionar correctamente, requiere haber instalado las siguientes librerías:

* C++11 o superior
* Eigen3: g2o requiere al menos la versión 3.1.0.

        sudo apt install libeigen3-dev
* g2o: es un framework C++ de código abierto para optimizar funciones de error no lineales basadas en grafos. La versión modificada necesaria para ejecutar este programa se encuentra en la carpeta include/Thirdparty/g2o.
* OpenCV: Usamos OpenCV para manipular imágenes y trabajar con datos tanto 2D como 3D. Se requiere al menos la versión 2.4.3. Probado con OpenCV 4.2.0. 

        sudo apt install libopencv-dev



INSTALACION
------------
    
Compilar triangulacionDeBayas con g++

        cd Release
        #editar la variables del archivo build.sh para que coincidan con los lugares donde se instalaron las dependencias en su PC
        chmod +x build.sh
        ./build.sh



USO
---

Antes de ejecutar el programa se tiene que setear la variable del sistema con el path a la librería g2o compilada anteriormente:
    
    export LD_LIBRARY_PATH="/path_al_repo/triangulacionDeBayas/include/Thirdparty/g2o/lib/"
    
Luego, para ejecutarlo

    Release/triangulacionDeBayas \
      -c ruta_archivo_de_calibracion \
      -d ruta_archivo_de_correspondencias \
      -x distancia_en_cm_entre_esquinas_de_qr \
      -i ruta_carpeta_con_frames \
      -o ruta_carpeta_de_salida \
      --init_distance distancia_mínima_entre_frames_de_inicializacion


La carpeta /data contiene archivos de ejemplo que sirven como input para la siguiente llamada

    Release/triangulacionDeBayas \
      -c data/calib/MotorolaG200_Javo_Vertical.yaml \
      -d data/VID_20230322_173621/bundles.csv \
      -x 2.1 \
      -i data/VID_20230322_173621/ \
      -o data/VID_20230322_173621/output \
      --init_distance 15
