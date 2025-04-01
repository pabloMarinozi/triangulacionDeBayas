from cmath import nan
import os
import argparse
import sys
import pandas as pd
import statistics
import math
import numpy
from pathlib import Path

def generate_triangulaciones(base_dir, camera_int, scale_dist, init_distance, output_dir):
    for path in Path(base_dir).rglob('prueba_bundles_qr_0.4.csv'):
        print("Triangulate:",path)
        os.system("export LD_LIBRARY_PATH=../Triangulacion/Release/lib/;export LD_LIBRARY_PATH=../Triangulacion/include/Thirdparty/g2o/lib;"+
         "../Triangulacion/Release/triangulacionDeBayas " +
         " -c "+ str(camera_int) +
         " -d " + str(path) + " -x " + str(scale_dist) +
           " -i " + str(path.parents[1]) +
            " -o " + str(path.parents[0]) + " --init_distance "+ str(init_distance))

def main(args=None):
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
        description= """Lee archivos csv con los bundles de keypoints
        -INPUT:
            --input_dir: Carpeta que contiene los csv con los bundles de los keypoints y las imagenes para realizar la reconstruccion.
            --camera_int: Archivo yaml que contienen los parametros intrinsicos de la camara.
            --qr_dist: Distancia en cm entre las esquinas de los qr para dar escala a la nube 3d.
            --sufix: Sufijo de csv de salida
            --init_distance: distancia mínima en frames para seleccionar los frames de inicialización
        -OUTPUT:
            Carpeta de salida, se almacenan los frames con los keypoints dibujados y un csv que contiene la informacion de todas las reconstrucciones.

        -EXAMPLE:
            python triangular_racimos_from_folder.py -i ./data/ -c ../data/camaraCarlos1080.yaml -s 8.0 -o ./output/
        """
    )
    parser.add_argument('-i','--input_dir', type=str, required=True)
    parser.add_argument('-m','--images_dir', type=str, required=True)
    parser.add_argument('-c','--camera_int', type=str, required=True)
    parser.add_argument('-x','--qr_dist', type=str, required=True)
    parser.add_argument('-o','--output_dir', type=str, required=True)
    parser.add_argument('--sufix', type=str, required=True)
    parser.add_argument('--init_distance', type=str, required=True)
    args = parser.parse_args(args)
    final_outputdir= args.output_dir + args.sufix
    generate_triangulaciones(args.input_dir, args.camera_int, args.qr_dist, args.init_distance, final_outputdir)


if __name__ == "__main__":
    main()
