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
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <OutputWriter.h>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <math.h>
//# define M_PI 3.14159265358979323846

using namespace std;
using namespace ORB_SLAM2;

int main(int argc, char **argv) {

	cout << "argc: " << argc << endl;
	if (argc != 4) {
		cerr << endl << "Uso: \n"
				"./TrianguladoCentroBayas\n"
				"ruta_calibracion\n"
				"ruta_bundles\n"
				"distancia_calibracion_en_cm" << endl;
		return 1;
	}

	//Rellenar variables con info del archivo
	InputReader* mpInputReader = new InputReader(argv[1], argv[2]);
	float distancia_calibracion = atof(argv[3]);
	if (mpInputReader->error)
		return -1;
	cv::Mat mK = mpInputReader->GetK();
	vector<int> bordes = mpInputReader->GetImageBounds(mK);
	int numFrames = mpInputReader->GetNumFrames();
	if (numFrames < 2) {
		cout << "No hay suficientes frames para inicializar el mapa" << endl;
		return 0;
	}
	vector<int> mvIniMatches = mpInputReader->GetInitialMatches();
	int f0 = mpInputReader->getFrame0(), f1 = mpInputReader->getFrame1();
	vector<cv::KeyPoint> kpsUn1 = mpInputReader->GetUndistortedKPs(f0, mK);
	vector<cv::KeyPoint> kpsUn2 = mpInputReader->GetUndistortedKPs(f1, mK);

	//Inicializar Mapa
	MapManager* mpMapManager = new MapManager();
	std::vector<cv::Point3f> mvIniP3D;
	vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
	cv::Mat Rcw; // Current Camera Rotation
	cv::Mat tcw; // Current Camera Translation
	cout << "Comenzando inicialización con frames " << f0 << " y " << f1
			<< endl;
	Initializer* mpInitializer = new Initializer(mK, kpsUn1, kpsUn2, 1.0,
			100000);
	if (mpInitializer->Initialize(mvIniMatches, Rcw, tcw, mvIniP3D,
			vbTriangulated)) {
		// Set KeyFrame Poses
		cv::Mat Tcw1 = cv::Mat::eye(4, 4, CV_32F);
		cv::Mat Tcw2 = cv::Mat::eye(4, 4, CV_32F);
		Rcw.copyTo(Tcw2.rowRange(0, 3).colRange(0, 3));
		tcw.copyTo(Tcw2.rowRange(0, 3).col(3));
		//DTO creation
		MapCreationDTO dto = MapCreationDTO();
		dto.setBordes(bordes);
		dto.setK(mK);
		dto.setMvIniMatches(mvIniMatches);
		dto.setMvIniP3D(mvIniP3D);
		dto.setMvKeys1(mpInputReader->GetKPs(f0));
		dto.setMvKeys2(mpInputReader->GetKPs(f1));
		dto.setMvKeysUn1(kpsUn1);
		dto.setMvKeysUn2(kpsUn2);
		dto.setTcw1(Tcw1);
		dto.setTcw2(Tcw2);
		dto.setMvTracks(mpInputReader->GetTrackIds(f1));
		dto.setFrame0(mpInputReader->getFrame0());
		dto.setFrame1(mpInputReader->getFrame1());
		mpMapManager->CreateInitialMapMonocular(dto);
	}

	//Calibrar Mapa en centimetros
	float scaleFactor = mpMapManager->GetScaleFactor(distancia_calibracion,
			mpInputReader);
	if (scaleFactor > 0)
		mpMapManager->ScaleMap(scaleFactor);

	//Iterar sobre los frames para procesar las observaciones
	vector<int> kfRestantes = mpInputReader->GetNotInitialFrames();
	for (auto i : kfRestantes) {
		mpMapManager->CreateNewKeyFrame(i, mpInputReader);
		scaleFactor = mpMapManager->GetScaleFactor(distancia_calibracion,
				mpInputReader);
		if (scaleFactor > 0)
			mpMapManager->ScaleMap(scaleFactor);
//		int cal_1=mpInputReader->getTrackCal1(), cal_2 =mpInputReader->getTrackCal2();
//		cout<<mpMapManager->GetDistanceCal1Cal2(cal_1,cal_2)<<endl;
	}

	//Realizar Reproyecciones para verificar Resultados
	map<int, vector<float> > errors_map;
	map<int, vector<cv::Point2f> > rep_map;
	map<int, vector<cv::Point2f> > normal_points_map;
	map<int, vector<float> > vols_rep_map;
	map<int, vector<float> > vols_real_map;
	map<int, vector<float> > radios = mpInputReader->getRadios();
	vector<cv::Mat> imgs;
	for (int i = 0; i < numFrames; ++i) {
		vector<cv::Point2f> reprojections =
				mpMapManager->ReproyectAllMapPointsOnKeyFrame(i);
		vector<cv::Point2f> normal_points =
				mpMapManager->CreatePointsOnNormalPlane(i, 1);
		vector<cv::Point2f> kps = mpInputReader->GetPoints(i);
		vector<int> tracks = mpInputReader->GetTrackIds(i);
		vector<float> errors;
		vector<float> vols_rep;
		vector<float> vols_real;
		string imgname = mpInputReader->GetImageName(i);
		cv::Mat img = cv::imread(imgname);
		for (int j = 0; j < kps.size(); ++j) {
			int track = tracks[j];
			if (reprojections[j].x >= 0 && reprojections[j].y >= 0) {
				float dist = cv::norm(reprojections[j] - kps[j]);
				errors.push_back(dist);
				cv::circle(img, reprojections[j], 3, cv::Scalar(0, 255, 0), 2);

				//calcula volumenes
				float pixeles_en_1cm_rep = cv::norm(
						normal_points[j] - reprojections[j]);
				float pixeles_en_1cm_real = cv::norm(normal_points[j] - kps[j]);
				float radio_en_cm_rep = radios[i][j] / pixeles_en_1cm_rep;
				float radio_en_cm_real = radios[i][j] / pixeles_en_1cm_real;
				float vol_rep = (4 * M_PI * radio_en_cm_rep * radio_en_cm_rep
						* radio_en_cm_rep) / 3;
				float vol_real = (4 * M_PI * radio_en_cm_real * radio_en_cm_real
						* radio_en_cm_real) / 3;
				vols_rep.push_back(vol_rep);
				vols_real.push_back(vol_real);

				cv::circle(img, normal_points[j], 2, cv::Scalar(0, 0, 255), 2);
				cv::putText(img, to_string(track), normal_points[j], 0, 0.5,
						cv::Scalar(0, 0, 255));
			} else
				errors.push_back(-1);
			cv::circle(img, kps[j], 2, cv::Scalar(255, 0, 0), 2);

			cv::putText(img, to_string(track), kps[j], 0, 0.5,
					cv::Scalar(255, 0, 0));

		}
		errors_map[i] = errors;
		rep_map[i] = reprojections;
		normal_points_map[i] = normal_points;
		vols_real_map[i] = vols_real;
		vols_rep_map[i] = vols_rep;
		imgs.push_back(img);
	}

	//Generar Salidas
	map<int, cv::Point3d> mps = mpMapManager->GetAllMapPoints();
	map<int, vector<cv::Point2f> > kps = mpInputReader->getKps();
	map<int, vector<int> > track_ids = mpInputReader->getTrackIds();
	map<int, string> img_names = mpInputReader->getImgNames();
	OutputWriter* mpOutputWriter = new OutputWriter("output", string(argv[2]));
	mpOutputWriter->guardarImagenes(imgs, img_names);
	map<int, string> labels = mpInputReader->getLabels();
	int cal_1 = mpInputReader->getTrackCal1(), cal_2 =
			mpInputReader->getTrackCal2();
	mpOutputWriter->guardarResultados(numFrames, errors_map, rep_map, kps,
			normal_points_map, track_ids, mps, cal_1, cal_2, img_names, radios,
			vols_rep_map, vols_real_map, labels);
	cout << "SALIDAS GUARDADAS EN " << mpOutputWriter->getStrOutputPath()
			<< endl;
	return 0;
}

