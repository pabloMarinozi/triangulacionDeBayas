//============================================================================
// Name        : main.cpp
// Author      : Pablo
// Version     :
// Copyright   : Hecho para Óptima.
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <Initializer.h>
#include <InputReader.h>
#include <MapCreationDTO.h>
#include <MapManager.h>
#include <OutputWriter.h>
#include <colormod.h>
#include <InitializationStrategies.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <iostream>
#include <numeric>
//# define M_PI 3.14159265358979323846

using namespace std;
using namespace ORB_SLAM2;
typedef pair<int, int> Match;
class InputParser;
void scaleMap(MapManager* mpMapManager, InputReader* mpInputReader);
void fillMap(InitializationStrategy* mpInitializationStrategy, int numFrames, MapManager* mpMapManager, InputReader* mpInputReader);

int main(int argc, char **argv) {
	Color::Modifier red(Color::FG_RED);
	Color::Modifier yellow(Color::FG_YELLOW);
	Color::Modifier green(Color::FG_GREEN);
	Color::Modifier def(Color::FG_DEFAULT); 
	InputParser* inputParser = new InputParser(argc, argv);
	inputParser->checkArgs();

	if(!inputParser->calibFlag || !inputParser->outputFlag || !inputParser->detectionsFlag){
		cout<<endl<<red<<"LOS ARGUMENTOS '-c', '-d' y '-o' SON OBLIGATORIOS."<<endl;
		return 0;
	}
	

	//Rellenar variables con info del archivo
	InputReader* mpInputReader = new InputReader(inputParser);
	//return 0;
	if (mpInputReader->error){
		cout<<red<<"HUBO UN ERROR EN LA LECTURA DE LOS ARCHIVOS DE ENTRADA."<<endl;
		return -1;
	}
	cv::Mat mK = mpInputReader->GetK();
	//mpInputReader->showRadios();
	//return 0;


	
	int numFrames = mpInputReader->GetNumFrames();
	if (numFrames < 2) {
		cout<<red<< "NO HAY SUFICIENTES FRAMES PARA INICIALIZAR EL MAPA." << endl;
		return 0;
	}


	//Inicializar Mapa
	float distancia_inicializacion = mpInputReader->GetInitDist();
	cout<<endl<<distancia_inicializacion<<endl;
	InitializationStrategy* mpInitializationStrategy =
		FactoryInitializationStrategies::Get()->CreateInitializationStrategy(1, mpInputReader, distancia_inicializacion);

	MapManager* mpMapManager = mpInitializationStrategy->initialize();
	if(!mpMapManager) return -1;


	//Iterar sobre los frames para procesar las observaciones
	fillMap(mpInitializationStrategy, numFrames, mpMapManager, mpInputReader);

	//Calibrar Mapa en centimetros
	float scaleFactor = -1.0;
	if(inputParser->scaleFlag) scaleMap(mpMapManager, mpInputReader);
	else cout<<endl<<yellow<<"EL MAPA NO SERÁ ESCALADO PORQUE NO SE HA PASADO EL ARGUMENTO '-x'."<<def<<endl;


	//Generar salidas
	int f0 = mpInitializationStrategy->f0; int f1 = mpInitializationStrategy->f1;
	OutputWriter* mpOutputWriter = new OutputWriter(inputParser, mpInputReader,f0,f1);
	mpOutputWriter->calcularReproyecciones(mpMapManager, mpInputReader, inputParser->scaleFlag);
	string filename = "/Reproyecciones.csv";
	mpOutputWriter->guardarTriangulacion(filename, inputParser->scaleFlag);
	bool graficarNoVisibles = true;
	if(inputParser->imagesFlag) mpOutputWriter->graficarReproyecciones(mpInputReader, graficarNoVisibles, false);
	cout<< endl<<green<< "SALIDAS GUARDADAS EN " << mpOutputWriter->getStrOutputPath()<< endl;

	return 0;
}

void scaleMap(MapManager* mpMapManager, InputReader* mpInputReader){
	Color::Modifier yellow(Color::FG_YELLOW);
	Color::Modifier def(Color::FG_DEFAULT);	
	Color::Modifier green(Color::FG_GREEN); 
	string x = "-x";
	float distancia_calibracion = mpInputReader->getScaleDistance();
	float scaleFactor = mpMapManager->GetScaleFactor(distancia_calibracion);
	if (scaleFactor > 0){
		mpMapManager->ScaleMap(scaleFactor);
		cout<<endl<<green<<"EL MAPA FUE ESCALADO EXITOSAMENTE. "<<def<<endl;
	} 
}

void fillMap(InitializationStrategy* mpInitializationStrategy, int numFrames, MapManager* mpMapManager, InputReader* mpInputReader)
{
	int f0 = mpInitializationStrategy->f0;
	int f1 = mpInitializationStrategy->f1;
	vector<int> kfBefore,kfBetween,kfAfter,kfRestantes;
	for (int i = 0; i < numFrames; i++) {
		if (i < f0) kfBefore.push_back(i);
		if (i > f0 && i < f1) kfBetween.push_back(i);
		if (i > f1) kfAfter.push_back(i);
	}
	kfRestantes.reserve(kfBefore.size() + kfBetween.size() + kfAfter.size());
	kfRestantes.insert(kfRestantes.end(),kfBetween.begin(),kfBetween.end());
	kfRestantes.insert(kfRestantes.end(),kfAfter.begin(),kfAfter.end());
	kfRestantes.insert(kfRestantes.end(),kfBefore.rbegin(),kfBefore.rend());
	for (auto i : kfRestantes) {
		mpMapManager->CreateNewKeyFrame(i, mpInputReader);
	}
}

/////////////////////VIEJO MAIN///////////////////////////////////////////////////////7
//Realizar Reproyecciones para verificar Resultados


/*

	map<int, map<int, float> > errors_map;  //r
	map<int, map<int, cv::Point2f> > rep_map; //r //i
	map<int, map<int, cv::Point2f> > normal_points_map; //e
	map<int, vector<float> > vols_rep_map; //v
	map<int, vector<float> > vols_real_map; //v
	map<int, map<int, float> > radios = mpInputReader->getRadios(); //r
	map<int, string> all_img_names = mpInputReader->getImgNames(); //i
	map<int, string> img_names; //i
	map<int, cv::Mat> imgs; //i
	vector<long unsigned int> allKfIds = mpMapManager->GetAllKeyFramesId(); 


	for (auto i : allKfIds) {
				//if(!mpMapManager->CheckKF(i)) continue;
		map<int, cv::Point2f> reprojections =
		mpMapManager->ReproyectAllMapPointsOnKeyFrame(i);
			//cout<<"Ya reproyectó"<<endl;
		map<int, cv::Point2f> normal_points =
		mpMapManager->CreatePointsOnNormalPlane(i, 1);
			//cout<<"Ya pasò la normal"<<endl;
		map<int, cv::Point2f> kpsInput = mpInputReader->GetPoints(i);
		vector<int> tracks = mpInputReader->GetTrackIds(i);
		map<int, float> errors;
		vector<float> vols_rep;
		vector<float> vols_real;
		string imgname = imagesPath+"/"+to_string(i)+".jpg";//mpInputReader->GetImageName(i);
		cout<<imgname<<endl;

		cv::Mat img = cv::imread(imgname);
		bool observado;
		for (auto const& tuple : reprojections){
			//for (int j = 0; j < kps.size(); ++j) {

			int track = tuple.first;
			cv::Point2f rep = tuple.second;
			cv::Point2f kp, normal;
			if(kpsInput.count(track) > 0){
				observado = true;
				kp = kpsInput[track];
				normal = normal_points[track];
				//dibuja la etiqueta
				cv::circle(img, kp, 2, cv::Scalar(255, 0, 0), 2);
				cv::putText(img, to_string(track), kp, 0, 0.5,
					cv::Scalar(255, 0, 0));
			}else observado =false;

			if (rep.x >= 0 && rep.y >= 0) {
				if(observado){
					//dibuja la reproyección en verde
					float dist = cv::norm(rep - kp);
					errors[track] = dist;
					cv::circle(img, rep, 3, cv::Scalar(0, 255, 0), 2);
					cv::putText(img, to_string(track), rep, 0, 0.5,
					cv::Scalar(0, 255, 0));


					//calcula volumenes
					float pixeles_en_1cm_rep = cv::norm(normal - rep);
					float pixeles_en_1cm_real = cv::norm(normal - kp);
					float radio_en_cm_rep = radios[i][track] / pixeles_en_1cm_rep;
					float radio_en_cm_real = radios[i][track] / pixeles_en_1cm_real;
					float vol_rep = (4 * M_PI * radio_en_cm_rep * radio_en_cm_rep
						* radio_en_cm_rep) / 3;
					float vol_real = (4 * M_PI * radio_en_cm_real * radio_en_cm_real
						* radio_en_cm_real) / 3;
					vols_rep.push_back(vol_rep);
					vols_real.push_back(vol_real);
					cv::circle(img, normal, 2, cv::Scalar(0, 0, 255), 2);
					cv::putText(img, to_string(track), normal, 0, 0.5,
						cv::Scalar(0, 0, 255));
				}else{
					//dibuja la reproyección en amarillo
					cv::circle(img, rep, 3, cv::Scalar(255, 255, 0), 2);
					cv::putText(img, to_string(track), rep, 0, 0.5, cv::Scalar(255, 255, 0));
				}
			} 


		}
		errors_map[i] = errors;
		rep_map[i] = reprojections;
		normal_points_map[i] = normal_points;
		vols_real_map[i] = vols_real;
		vols_rep_map[i] = vols_rep;
		imgs[i]=img;
		img_names[i]=all_img_names[i];
	}
	rep_acumulator.push_back(rep_map);
	kfIds_acumulator.push_back(allKfIds);

	//Generar Salidas
	map<int, cv::Point3d> mps = mpMapManager->GetAllMapPoints();
	map<int, vector<cv::Point2f> > kps = mpInputReader->getKps();
	map<int, vector<int> > track_ids = mpInputReader->getTrackIds();
	string str1 = string(outputFolder); string str2 = string(bundlesPath);
	//int f0 = mpInitializationStrategy->f0; int f1 = mpInitializationStrategy->f1; 
	
	mpOutputWriter->guardarImagenes(imgs, img_names);
	if(mpInitializationStrategy->has_Outliers())
		mpOutputWriter->guardarOutliers(mpInitializationStrategy->outliers_prop);
	map<int, string> labels = mpInputReader->getLabels();
	int cal_1 = mpInputReader->getTrackCal1(), cal_2 =
	mpInputReader->getTrackCal2();
	mpOutputWriter->guardarResultados(allKfIds, errors_map, rep_map, kps,
		normal_points_map, track_ids, mps, cal_1, cal_2, img_names, radios,
		vols_rep_map, vols_real_map, labels);
	cout << "SALIDAS GUARDADAS EN " << mpOutputWriter->getStrOutputPath()
	<< endl;



	// Generar Dispersion Reproyecciones
	/*int frame = rand() % numFrames;
	string imgname = mpInputReader->GetImageName(frame);
	vector<int> visibleTracks = mpInputReader->GetTrackIds(frame);
	map<int, cv::Point2f> kps = mpInputReader->GetPoints(frame);
	for (int i = 0; i < visibleTracks.size(); ++i) {
		int t = visibleTracks[i];
		cv::Mat img = cv::imread(imgname);
		//Dibuja las reproyecciones
		for (int num_rec = 0; num_rec < rep_acumulator.size(); ++num_rec) {
			tie(num,f0,f1) = mvPairs[num_rec];
			map<int, map<int, cv::Point2f> > rep_map = rep_acumulator[num_rec];
			if(rep_map.count(frame) != 0){
				map<int, cv::Point2f> all_rep = rep_map[frame];
				cv::Point2f pt = all_rep[t];
				cv::circle(img, pt, 3, cv::Scalar(0, 255, 0), 2);
				cv::putText(img, to_string(f0)+"_"+to_string(f1), pt, 0, 0.3, cv::Scalar(0, 0, 255));
			}
		}
		//Dibuja la etiqueta
		cv::Point2f kp = kps[t];
		cv::circle(img, kp, 2, cv::Scalar(255, 0, 0), 2);
		cv::putText(img, to_string(t), kp, 0, 1, cv::Scalar(255, 0, 0));
		//Guarda la imagen
		string ruta = string(outputFolder)+"/t"+to_string(frame)+"_b"+to_string(t)+".png";
		cv::imwrite(ruta,img);
	}*/

	
