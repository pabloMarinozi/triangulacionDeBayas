//============================================================================
// Name        : InputReader.cpp
// Author      : Pablo
// Version     :
// Copyright   : Hecho para Óptima.
// Description : Escribe las salidas al algoritmo
//============================================================================

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <OutputWriter.h>
#include <dirent.h>
#include <stddef.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

OutputWriter::OutputWriter(string strOutputPath, string strMatchesPath){
	string directory;
	const size_t last_slash_idx = strMatchesPath.find_last_of('/');
	if (std::string::npos != last_slash_idx)
	{
	    directory = strMatchesPath.substr(0, last_slash_idx);
	}
	const size_t last_slash_idx2 = directory.find_last_of('/');
	directory = directory.substr(last_slash_idx2);


	char* ruta;
	sprintf(ruta, "%s", strOutputPath.c_str());
	DIR *dirp = opendir(ruta);
	if (dirp == NULL) {
		char* comando = new char[510];

		sprintf(comando, "mkdir %s", ruta);
		std::system(comando);
		cout << "Se ha creado el directorio de salida en la ruta " << ruta
				<< endl;
	} else {
		cout << "Ya existia el directorio de salida en la ruta " << ruta
				<< endl;
	}
	sprintf(ruta, "%s%s", strOutputPath.c_str(),directory.c_str());
	DIR *dirp2 = opendir(ruta);
	if (dirp2 == NULL) {
		char* comando = new char[510];

		sprintf(comando, "mkdir %s", ruta);
		std::system(comando);
		cout << "Se ha creado el directorio de salida en la ruta " << ruta
				<< endl;
	} else {
		cout << "Ya existia el directorio de salida en la ruta " << ruta
				<< endl;
	}
	this->strOutputPath = ruta;
}

void OutputWriter::guardarImagenes(vector<cv::Mat> imgs, map<int, string> names){
	for (int i = 0; i < imgs.size(); ++i) {
		char* nombre;
		cv::Mat img = imgs[i];
		sprintf(nombre, "%s/%s", strOutputPath, names[i]);
		const string ruta = strOutputPath+"/"+to_string(i)+".png";
		cv::imwrite(ruta,img);
	}
}

void OutputWriter::guardarResultados(int N,
		map<int, vector<float> > errors_map,
		map<int, vector<cv::Point2f> > rep_map,
		map<int, vector<cv::Point2f> > kps_map,
		map<int, vector<cv::Point2f> > points_map,
		map<int, vector<int> > track_id_map,
		map<int, cv::Point3d> mps,
		int cal_1, int cal_2, map<int, string> names,
		map<int, vector<float> > radios) {
	ofstream myfile;
	myfile.open(strOutputPath+"/Reproyecciones.csv");
	if (myfile.is_open()) {

		//Escribe encabezados
		myfile << "track_id,label,x,y,z";
		for (int i = 0; i < N; ++i){
			myfile << ",img_name_"<<i<<",x_"<<i<<",y_"<<i<<",r_"<<i<<",xrep_"<<i<<",yrep_"<<i<<",error_"<<i<<",x1cm_"<<i<<",y1cm_"<<i;
		}
		myfile << endl;

		//Itera sobre los mappoints para generar una fila para cada uno
		for(std::map<int,cv::Point3d>::iterator iter = mps.begin(); iter != mps.end(); ++iter)
		{
			//columna "track_id"
			int track_id =  iter->first;
			myfile << track_id;

			//columna "label"
			if(track_id==cal_1) myfile << ",cal_1";
			else if(track_id==cal_2) myfile << ",cal_2";
			else myfile << ",baya";

			//columnas "x","y" y "z"
			cv::Point3d p = iter->second;
			myfile << ","<<p.x<< ","<<p.y<< ","<<p.z;

			//columnas por frame
			for (int frame_index = 0; frame_index < N; ++frame_index){
				//verifica si el map point es visible en el frame
				vector<int> tracks_del_frame = track_id_map[frame_index];
				int index_del_track_en_el_frame = -1;
				for (int kp_index = 0; kp_index < tracks_del_frame.size(); ++kp_index){
					if(tracks_del_frame[kp_index] == track_id)
						index_del_track_en_el_frame =  kp_index;
				}

				//si es visible...
				if(index_del_track_en_el_frame>-1){
					//columna "img_name_"
					myfile << ","<< names[frame_index];
					//columnas "x_" e "y_"
					cv::Point2f observacion = kps_map[frame_index][index_del_track_en_el_frame];
					myfile << ","<<observacion.x<< ","<<observacion.y;
					//columna "r_"
					myfile << ","<< radios[frame_index][index_del_track_en_el_frame];
					//columnas "x_rep_" e "y_rep_"
					cv::Point2f reproyeccion = rep_map[frame_index][index_del_track_en_el_frame];
					myfile << ","<<reproyeccion.x<< ","<<reproyeccion.y;
					//columna "error_"
					myfile << ","<< errors_map[frame_index][index_del_track_en_el_frame];
					//columnas "x_1cm_" e "y_1cm_"
					cv::Point2f point1cm = points_map[frame_index][index_del_track_en_el_frame];
					myfile << ","<<point1cm.x<< ","<<point1cm.y;
				}

				//sino...
				else {
					myfile << ",NULL"; //columna "img_name_"
					myfile << ",NULL,NULL"; //columnas "x_" e "y_"
					myfile << ",NULL"; //columna "r_"
					myfile << ",NULL,NULL"; //columnas "x_rep_" e "y_rep_"
					myfile << ",NULL"; //columna "error_"
					myfile << ",NULL,NULL"; //columnas "x_1cm_" e "y_1cm_"
				}
			}
			//fin de fila
			myfile << endl;
		}
	}
	myfile.close();
}



