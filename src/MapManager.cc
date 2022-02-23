#include <Converter2D-3D.h>
#include <InputReader.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapCreationDTO.h>
#include <MapManager.h>
#include <MapPoint.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core.hpp>
#include <Optimizer.h>
#include <stddef.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <utility>

using namespace std;
using namespace ORB_SLAM2;

MapManager::MapManager() {
}

void MapManager::CreateInitialMapMonocular(MapCreationDTO dto) {
	mpMap = new ORB_SLAM2::Map();
	// Create KeyFrames
	KeyFrame* pKFini = new KeyFrame(mpMap, dto.getBordes(), dto.getMvKeys1(),
			dto.getMvKeysUn1(), dto.getK());
	KeyFrame* pKFcur = new KeyFrame(mpMap, dto.getBordes(), dto.getMvKeys2(),
			dto.getMvKeysUn2(), dto.getK());
	pKFini->mnId = dto.getFrame0();
	pKFcur->mnId = dto.getFrame1();
	pKFini->SetPose(dto.getTcw1());
	pKFcur->SetPose(dto.getTcw2());

	// Update motion model
	cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat mRcw = dto.getTcw1().rowRange(0, 3).colRange(0, 3);
	cv::Mat mRwc = mRcw.t();
	cv::Mat mtcw = dto.getTcw1().rowRange(0, 3).col(3);
	mRwc.copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
	mtcw.copyTo(LastTwc.rowRange(0, 3).col(3));
	mVelocity = dto.getTcw2() * LastTwc;

	// Insert KFs in the map
	mpMap->AddKeyFrame(pKFini);
	mpMap->AddKeyFrame(pKFcur);

	// Create MapPoints and asscoiate to keyframes
	vector<int> mvIniMatches = dto.getMvIniMatches();
	vector<cv::Point3f> mvIniP3D = dto.getMvIniP3D();
	vector<int> mvIds = dto.getMvTracks();
	for (size_t i = 0; i < mvIniMatches.size(); i++) {
		if (mvIniMatches[i] < 0){
			cout<<"El MapPoint con id "<< mvIds[i] << " no pudo ser creado por no estar visible en ambos frames."<<endl;
			continue;
		}


		//Create MapPoint.
		cv::Mat worldPos(mvIniP3D[i]);

		MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);
		if(mvIniP3D[i].x==0.f && mvIniP3D[i].y==0.f && mvIniP3D[i].z==0.f){
			pMP->SetBadFlag();
			cout<<"El MapPoint con id "<< mvIds[i] << " es defectuoso."<<endl;
		}

		pMP->mnId = mvIds[i];

		pKFini->AddMapPoint(pMP, i);
		pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

		pMP->AddObservation(pKFini, i);
		pMP->AddObservation(pKFcur, mvIniMatches[i]);

		//pMP->ComputeDistinctiveDescriptors();
		pMP->UpdateNormalAndDepth();

		//Add to Map
		mpMap->AddMapPoint(pMP);
	}

	// Update Connections
	pKFini->UpdateConnections();
	pKFcur->UpdateConnections();

	// Bundle Adjustment
	cout << "New Map created with " << mpMap->MapPointsInMap() << " points from frames "<<dto.getFrame0()<<" and "<<dto.getFrame1()
			<< endl;
	bool mbStopGBA = false;
	Optimizer::GlobalBundleAdjustemnt(mpMap, 20000, &mbStopGBA, pKFcur->mnId,
			false);

	mnLastKeyFrameId = pKFcur->mnId;
	mpLastKeyFrame = pKFcur;

	//    mvpLocalKeyFrames.push_back(pKFcur);
	//    mvpLocalKeyFrames.push_back(pKFini);
	//    mvpLocalMapPoints=mpMap->GetAllMapPoints();
	//    mpReferenceKF = pKFcur;
	//    mCurrentFrame.mpReferenceKF = pKFcur;

	//    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	mpMap->mvpKeyFrameOrigins.push_back(pKFini);

}

ORB_SLAM2::KeyFrame* BuscarKF(long unsigned int kfId,
		vector<ORB_SLAM2::KeyFrame*> allKFs) {
	vector<ORB_SLAM2::KeyFrame*>::iterator kf;
	for (kf = allKFs.begin(); kf != allKFs.end(); ++kf) {
		if ((*kf)->mnId == kfId) {
			return *kf;
		}
	}
	return static_cast<ORB_SLAM2::KeyFrame*>(NULL);
}

ORB_SLAM2::MapPoint* BuscarMP(long unsigned int mpId,
		vector<ORB_SLAM2::MapPoint*> allMPs) {
	vector<ORB_SLAM2::MapPoint*>::iterator mp;
	for (mp = allMPs.begin(); mp != allMPs.end(); ++mp) {
		if ((*mp)->mnId == mpId) {
			return *mp;
		}
	}
	return static_cast<ORB_SLAM2::MapPoint*>(NULL);
}

void MapManager::CreateNewKeyFrame(int id, InputReader* mpInputReader) {
	cout << "INICIANDO INSERCIÓN DEL KEYFRAME " << id << ".\n\n";

	//Recupera KFs anteriores
	vector<KeyFrame*> allKFs = mpMap->GetAllKeyFrames();
	vector<int> idsAnteriores;
	vector<ORB_SLAM2::KeyFrame*>::iterator kf;
	for (kf = allKFs.begin(); kf != allKFs.end(); ++kf) {
		idsAnteriores.push_back((*kf)->mnId);
	}

	//Crea el objeto KF
	cv::Mat mK = mpInputReader->GetK();
	vector<int> bordes = mpInputReader->GetImageBounds(mK);
	vector<cv::KeyPoint> mvKeys = mpInputReader->GetKPs(id);
	KeyFrame* pKF = new KeyFrame(mpMap, bordes, mvKeys,
			mpInputReader->GetUndistortedKPs(id, mK), mK);

	//Estima una posición inicial en base a la posición del último KF del mapa
	const cv::Mat Tcw = mpLastKeyFrame->GetPose();
	pKF->SetPose(Tcw);
	pKF->mnId = id;
	mpLastKeyFrame = pKF;

	mpMap->AddKeyFrame(pKF);

	//Puebla las observaciones en base a los matcheos
	vector<bool> tieneMatchPrevio = vector<bool>(mvKeys.size(),false);
	//tieneMatchPrevio.reserve(mvKeys.size());
	bool sonTodosTrue = false;
	vector<MapPoint*> allMPs = mpMap->GetAllMapPoints();

	for(auto idAnterior : idsAnteriores){
		cout<<"Buscando matcheos con el frame "<<idAnterior<<"..."<<endl;
		//mientras queden keypoints sin matchear o queden keyframes previos por analizar
		KeyFrame* kfPrevio = BuscarKF(idAnterior, allKFs);
		vector<int> mvMatches = mpInputReader->GetMatches(id, idAnterior);
		vector<int> tracks = mpInputReader->GetTrackIds(id);

		for (int index = 0; index < mvMatches.size(); index++) {
			if (tieneMatchPrevio[index])
				continue;
			if (mvMatches[index] >= 0) {
				MapPoint* pMP = BuscarMP(tracks[index],allMPs);
				if(pMP && !pMP->isBad())
					cout<<"Ya existía el mapPoint "<<tracks[index]<<". Añadiendo observación."<<endl;

				if (!pMP || pMP->isBad()) {    //es el primer matcheo, hay que crear el mapPoint
					cv::KeyPoint kp1 = kfPrevio->mvKeys[mvMatches[index]];
					cv::KeyPoint kp2 = mvKeys[index];
					Converter* converter = new Converter();
					cout<<"Se intentará triangular el MapPoint con id"<<tracks[index]<<endl;
					cv::Mat x3D = converter->TriangularMapPoint(
							make_pair(kfPrevio, kp1.pt),
							make_pair(pKF, kp2.pt));

					if(!x3D.empty()){
						MapPoint* pMP_viejo = BuscarMP(tracks[index],allMPs);
						if(pMP && pMP->isBad()) mpMap->EraseMapPoint(pMP_viejo);

						cout<<"Se generó un nuevo mapPoint con id "<<tracks[index]<<endl;
						pMP = new ORB_SLAM2::MapPoint(x3D, pKF, mpMap);

						pMP->mnId = tracks[index];
						mpMap->AddMapPoint(pMP);

						//Actualiza las observaciones de kfs anteriores
						vector<int> indexes = mpInputReader->GetIndexInKfs(idsAnteriores,tracks[index]);
						for (int i = 0; i < indexes.size(); i++) {
							if (indexes[i]>-1) {
								KeyFrame* kf = BuscarKF(idsAnteriores[i],allKFs);
								kf->AddMapPoint(pMP,indexes[i]);
								pMP->AddObservation(kf,indexes[i]);
								cout<<"Ahora el kf "<<kf->mnId<<" observa al mapPoint "<<pMP->mnId<<endl;
							}
						}

					}
				}

				if (!pMP || pMP->isBad()) {    //Falló la triangulación
					cout << "Falló la triangulación del mapPoint"<< tracks[index]
							<< " con los frames " << id << " y " << idAnterior << endl;
					continue;
				}

				pKF->AddMapPoint(pMP, index);
				pMP->AddObservation(pKF, index);
				pMP->UpdateNormalAndDepth();

				tieneMatchPrevio[index] = true;
			}
		}
		sonTodosTrue = std::find(begin(tieneMatchPrevio), end(tieneMatchPrevio),
				false) == end(tieneMatchPrevio);
		if(sonTodosTrue){
			cout<<"Ya todos los matcheos fueron analizados"<<endl;
			break;
		}
	}

	pKF->UpdateConnections();

	//Acomoda todos los objetos 3D en su lugar con un nuevo bundle adjustment
	bool mbStopGBA = false;
	Optimizer::GlobalBundleAdjustemnt(mpMap, 20000, &mbStopGBA, pKF->mnId, false);

	cout << "KEYFRAME " << id << " AÑADIDO CON ÉXITO.\n\n";

}

vector<cv::Point2f> MapManager::ReproyectAllMapPointsOnKeyFrame(int id) {
	vector<KeyFrame*> allKFs = mpMap->GetAllKeyFrames();
	cout<<"Reproyectando MapPoints sobre el Keyframe "<<id<<endl;
	KeyFrame* kf = BuscarKF(id, allKFs);
	Converter* converter = new Converter();
	return converter->ReprojectAllMapPointsOnKeyFrame(kf);
}

map<int, cv::Point3d> MapManager::GetAllMapPoints() {
	map<int, cv::Point3d> mps;
	vector<MapPoint*> allMps = mpMap->GetAllMapPoints();
	for (int i = 0; i < allMps.size(); ++i) {
		MapPoint* mp = allMps[i];
		cv::Mat pos = mp->GetWorldPos();
		mps[mp->mnId] = cv::Point3d(
				pos.at<float>(0),
				pos.at<float>(1),
				pos.at<float>(2));
	}
	return mps;
}

float distanciaPuntoPunto(cv::Mat punto1, cv::Mat punto2) {
	return sqrt(
			pow(punto1.at<float>(0) - punto2.at<float>(0), 2)
					+ pow(punto1.at<float>(1) - punto2.at<float>(1), 2)
					+ pow(punto1.at<float>(2) - punto2.at<float>(2), 2));
}



float MapManager::GetScaleFactor(float dist_cm, InputReader* inputReader) {
	int cal_1_id = inputReader->getTrackCal1();
	int cal_2_id = inputReader->getTrackCal2();
	vector<MapPoint*> vpAllMapPoints = mpMap->GetAllMapPoints();
	MapPoint* cal_1 = BuscarMP(cal_1_id, vpAllMapPoints);
	MapPoint* cal_2 = BuscarMP(cal_2_id, vpAllMapPoints);
	if(cal_1 && !cal_1->isBad() && cal_2 && !cal_2->isBad()){
		float dist_uni = distanciaPuntoPunto(cal_1->GetWorldPos(),
				cal_2->GetWorldPos());
		return dist_uni / dist_cm;
	}else{
		cout <<"No se pudo escalar el mapa porque alguno de los"
				" puntos de calibración aún no fue bien triangulado."<<endl;
		return -1;
	}
}

void MapManager::ScaleMap(float scaleFactor) {

	float invScaleFactor = 1.0f / scaleFactor;

	// Scale cameras
	vector<KeyFrame*> vpAllKeyFrames = mpMap->GetAllKeyFrames();
	for (size_t iMP = 0; iMP < vpAllKeyFrames.size(); iMP++) {
		if (vpAllKeyFrames[iMP]) {
			KeyFrame* pKFcur = vpAllKeyFrames[iMP];
			cv::Mat Tc2w = pKFcur->GetPose();
			Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)
					* invScaleFactor;
			pKFcur->SetPose(Tc2w);
		}
	}

	// Scale points
	vector<MapPoint*> vpAllMapPoints = mpMap->GetAllMapPoints();
	for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
		if (vpAllMapPoints[iMP]) {
			MapPoint* pMP = vpAllMapPoints[iMP];
			cv::Mat newWorldPos = pMP->GetWorldPos() * invScaleFactor;
			pMP->SetWorldPos(newWorldPos);
		}
	}
}

cv::Mat calcularLineaVisionCamara(cv::Mat R){
	R = R.t();
	cv::Mat vectorSinRotar = cv::Mat(3,1,CV_32F,0.0);
	vectorSinRotar.at<float>(0,0) = 0;
	vectorSinRotar.at<float>(1,0) = 0; //el vector sin rotar es el eje z
	vectorSinRotar.at<float>(2,0) = 1;
	return R*vectorSinRotar;
}

//Devuelve el producto vectorial entre dos vectores de 3 dimensiones v1 y v2
cv::Vec3f prodVect(cv::Vec3f v1, cv::Vec3f v2){
	cv::Vec3f result;
	result[0] = v1[1]*v2[2]-v1[2]*v2[1];
	result[1] = v1[2]*v2[0]-v1[0]*v2[2];
	result[2] = v1[0]*v2[1]-v1[1]*v2[0];
	return result;
}

float distanciaLineaPunto(cv::Mat linea, cv::Mat t, cv::Mat p){
	//vector director de la recta
//	cout<<"linea: "<<linea<<endl;
	cv::Vec3f director;
	director[0] = linea.at<float>(0);
	director[1] = linea.at<float>(1);
	director[2] = linea.at<float>(2);
	//vector tp
	cv::Vec3f vect;
	vect[0] = t.at<float>(0) - p.at<float>(0);
	vect[1] = t.at<float>(1) - p.at<float>(1);
	vect[2] = t.at<float>(2) - p.at<float>(2);
	cv::Vec3f prod = prodVect(vect,director);
	float norm_vec = norm(prod);
	norm_vec = norm_vec/norm(director);

	return norm_vec;
}

cv::Vec3f encontrarPuntoDistancia(cv::Vec3f  director, cv::Vec3f  punto, float distancia){

	float lambda = distancia/norm(director);

	//obtiene el punto mediante la ecuacion de la recta
	return punto + lambda*director;
}

cv::Mat encontrarPuntoDistancia(cv::Mat  director, cv::Mat  punto, float distancia){
	cv::Vec3f res = encontrarPuntoDistancia(
			cv::Vec3f(director.at<float>(0),director.at<float>(1),director.at<float>(2)),
					cv::Vec3f(punto.at<float>(0),punto.at<float>(1),punto.at<float>(2)),
							distancia);
	cv::Mat res_mat = cv::Mat(1,3, CV_32F, 0.0);
	res_mat.at<float>(0) = res[0];
	res_mat.at<float>(1) = res[1];
	res_mat.at<float>(2) = res[2];
	return res_mat;
}

/*calcula los puntos que pertenecen tanto a un plano como a una recta que lo atraviesa.
 los planos y las rectas estan en las filas de las matrices que se pasan como parámetro*/
cv::Mat calcularInterseccionPlanoRecta(cv::Mat lineaVision, cv::Mat t, cv::Mat planoNormal){
	cv::Mat interseccion = cv::Mat(1,3, CV_32F, 0.0);
	//recupero datos
	float a = planoNormal.at<float>(0);
	float b = planoNormal.at<float>(1);
	float c = planoNormal.at<float>(2);
	float d = planoNormal.at<float>(3);
	float x = t.at<float>(0);
	float y = t.at<float>(1);
	float z = t.at<float>(2);
	float v1 = lineaVision.at<float>(0);
	float v2 = lineaVision.at<float>(1);
	float v3 = lineaVision.at<float>(2);

	//calculo la constante k de la ecuacion de la recta para el punto intersección
	float k = (-a*x-b*y-c*z-d)/(v1*a+v2*b+v3*c);
	//guardo las coordenadas de la intersección en la matriz
	interseccion.at<float>(0) = x+k*v1;
	interseccion.at<float>(1) = y+k*v2;
	interseccion.at<float>(2) = z+k*v3;

	return interseccion;

}

cv::Mat calcularPlanoNormal(cv::Mat rectaVision, cv::Mat Punto){
	cv::Mat planoNormal = cv::Mat(1, 4, CV_32F, 0.0);
	cv::Vec3d director;
	cv::Vec3d posicionCamara;
	//los coeficientes A,B,C del plano se corresponden a las componentes del vector director de la recta normal
	float a = planoNormal.at<float>(0) = director[0] = rectaVision.at<float>(0);
	float b = planoNormal.at<float>(1) = director[1] = rectaVision.at<float>(1);
	float c = planoNormal.at<float>(2) = director[2] = rectaVision.at<float>(2);
	//calculo el coeficiente D del plano segun la distancia a la que se encuentra del punto
	planoNormal.at<float>(3) = -a*Punto.at<float>(0)-b*Punto.at<float>(1)-c*Punto.at<float>(2);

	return planoNormal;
}


vector<cv::Point2f> MapManager::CreatePointsOnNormalPlane(int id, float distance) {
	vector<KeyFrame*> allKFs = mpMap->GetAllKeyFrames();
	KeyFrame* kf = BuscarKF(id, allKFs);
	cv::Mat Rcw = kf->GetRotation();
//	cout<<"R: "<<Rcw<<endl;
	cv::Mat tcw = kf->GetTranslation();
//	cout<<"t: "<<tcw<<endl;
	cv::Mat lineaVision = calcularLineaVisionCamara(Rcw.clone());
//	cout<<"linea: "<<lineaVision<<endl;
	Converter* converter = new Converter();

	vector<MapPoint*> centros = kf->GetMapPointMatches();
	vector<cv::Point2f> points;
	for(auto centro: centros){
		if(!centro) continue;
		cv::Mat pos = centro->GetWorldPos();
		cv::Mat planoNormal = calcularPlanoNormal(lineaVision,pos);
//		cout<<"plano: "<<planoNormal<<endl;
		cv::Mat interseccion = calcularInterseccionPlanoRecta(lineaVision,tcw,planoNormal);
//		cout<<"interseccion: "<<interseccion<<endl;
////		cout<<"P: "<<pos<<endl;
//		float distPuntoCamara = distanciaPuntoPunto(tcw,pos);
////		cout<<"TP: "<<distPuntoCamara<<endl;
//		float distPuntoLineaVision = distanciaLineaPunto(lineaVision,tcw,pos);
////		cout<<"PQ: "<<distPuntoLineaVision<<endl;
//		float distPlanoCamara = sqrt(pow(distPuntoCamara,2)-pow(distPuntoLineaVision,2));
////		cout<<"TQ: "<<distPlanoCamara<<endl;
//		cv::Mat puntoPlano =  encontrarPuntoDistancia(lineaVision,tcw,distPlanoCamara);
////		cout<<"Q: "<<puntoPlano<<endl;
		cv::Mat pos_puntoPlano_vec = cv::Mat(3,1, CV_32F, 0.0);
		pos_puntoPlano_vec.at<float>(0) = interseccion.at<float>(0) - pos.at<float>(0);
		pos_puntoPlano_vec.at<float>(1) = interseccion.at<float>(1) - pos.at<float>(1);
		pos_puntoPlano_vec.at<float>(2) = interseccion.at<float>(2) - pos.at<float>(2);
//		cout<<"P1P2: "<<pos_puntoPlano_vec<<endl;

//		cout<<"producto punto: "<<lineaVision.dot(pos_puntoPlano_vec)<<endl;
		cv::Mat punto1cm = encontrarPuntoDistancia(pos_puntoPlano_vec,pos,distance);
//		cout<<"distancia: "<<norm(punto1cm-pos.t())<<endl;
//		cout<<"p2: "<<punto1cm<<endl;
		cv::Point2f punto1cm_2d = converter->ReproyectarPunto(Rcw,tcw,kf->mK,punto1cm);
		points.push_back(punto1cm_2d);
	}
	return points;
}

float MapManager::GetDistanceCal1Cal2(int cal_1_id, int cal_2_id) {
	vector<MapPoint*> vpAllMapPoints = mpMap->GetAllMapPoints();
	MapPoint* cal_1 = BuscarMP(cal_1_id, vpAllMapPoints);
	MapPoint* cal_2 = BuscarMP(cal_2_id, vpAllMapPoints);
	return distanciaPuntoPunto(cal_1->GetWorldPos(),
			cal_2->GetWorldPos());
}
