/*
 * InitializationStrategies.cc
 *
 *  Created on: 27 oct. 2022
 *      Author: pablo
 */

#include "InitializationStrategies.h"
#include <iostream>
#include <fstream>
typedef pair<int, int> Match;

//////////////////////////////////////////Implementación de la clase InitializationStrategy///////////////////////////////////////////////////////////
void InitializationStrategy::createInitialMap(){
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
	dto.setFrame0(f0);
	dto.setFrame1(f1);
	mpMapManager->CreateInitialMapMonocular(dto);
}




//////////////////////////////////////////Implementación de la clase MostMatches_InitializationStrategy///////////////////////////////////////////////////////////
MostMatches_InitializationStrategy::MostMatches_InitializationStrategy(InputReader* mpInputReader, float dist){
	mvPairs = mpInputReader->GetInitialPairsFromMostMatches(dist);
	initialized = false;
	mK = mpInputReader->GetK();
	bordes = mpInputReader->GetImageBounds(mK);
	this->mpInputReader = mpInputReader;
}

MapManager* MostMatches_InitializationStrategy::initialize(){
	int best = 0;
	int num, f0, f1;

	while(mvPairs.size()>best && !initialized){

		tie(num,f0,f1) = mvPairs[best];

		kpsUn1 = mpInputReader->GetUndistortedKPs(f0, mK);
		kpsUn2 = mpInputReader->GetUndistortedKPs(f1, mK);
		cout <<endl<<endl<< "=========================================================="<<endl;
		cout << "Comenzando inicialización con frames " << f0 << " y " << f1 << endl
		<< num <<" matcheos"<<endl
		<< "=========================================================="<< endl<<endl<<endl;
		mpInitializer = new Initializer(mK, kpsUn1, kpsUn2, 1.0, 100000);
		mvIniMatches = mpInputReader->GetMatches(f0,f1);

		initialized = mpInitializer->Initialize(mvIniMatches, Rcw, tcw, mvIniP3D,vbTriangulated);

		if (!initialized){
			cout << "LA RECONSTRUCCIÓN FALLÓ. Se intentará con el próximo par de frames."<<endl;
			best++;
		}
	}

	if(initialized){
		this->f0 = f0;
		this->f1 = f1;
		createInitialMap();
		
		}
	else{
		cout << "LA RECONSTRUCCIÓN NO FUE POSIBLE CON NINGÚN PAR DE FRAMES"<<endl;
		return static_cast<MapManager*>(NULL);
	} 

	return mpMapManager;
}

//////////////////////////////////////////Implementación de la clase EveryPair_InitializationStrategy///////////////////////////////////////////////////////////

EveryPair_InitializationStrategy::EveryPair_InitializationStrategy(InputReader* mpInputReader,float dist){
	mvPairs = mpInputReader->GetInitialPairsFromMostMatches(dist);
	initialized = false;
	mK = mpInputReader->GetK();
	bordes = mpInputReader->GetImageBounds(mK);
	this->mpInputReader = mpInputReader;
}

MapManager* EveryPair_InitializationStrategy::initialize(){
	int best = 0;
	float best_score = 0.0;
	int num, f0, f1;
	map<int, vector<bool> > track_inliers_map;
	cout << "Pares a probar: "<<mvPairs.size()<<endl;

	//Calcula proporción de outliers
	for(int i = 0; i<mvPairs.size(); i++){
		//calcula inliers con el par de frames
		float current_score;
		vector<bool> current_inliers;
		tie(num,f0,f1) = mvPairs[i];
		kpsUn1 = mpInputReader->GetUndistortedKPs(f0, mK);
		kpsUn2 = mpInputReader->GetUndistortedKPs(f1, mK);
		cout << i<<"/"<<mvPairs.size()<<"   Calculando inliers con frames " << f0 << " y " << f1 << endl;
		mpInitializer = new Initializer(mK, kpsUn1, kpsUn2, 1.0, 200);
		mvIniMatches = mpInputReader->GetMatches(f0,f1);
		tie(current_score, current_inliers) = mpInitializer->GetRansacInliers(mvIniMatches);
		if(current_score > best_score){
			best = i; best_score = current_score;
		} 
		
		//Guarda cuáles tracks fueron inliers y cuáles no
		//inliers_por_frame.open("output_outliers/inliers_por_frame.txt");
		vector<int> tracks;
		for (auto const& x : mvIniMatches){
		    int trackId = x.first;
		    tracks.push_back(trackId);
		}
		assert (current_inliers.size()==tracks.size());
		// if (inliers_por_frame.is_open()){
		// 	inliers_por_frame<<"Inicialización con frames "<<f0<<" y "<<f1<<endl;
		// 	inliers_por_frame<<"score: "<<current_score<<endl;
		// 	inliers_por_frame<<"inliers: "<<endl;
		// } 
		
		for(int j=0; j<tracks.size(); j++){
			int t = tracks[j];
			bool inlier = current_inliers[j];
			track_inliers_map[t].push_back(inlier);
			//if (inliers_por_frame.is_open()) 
			//	inliers_por_frame << "track "<<t<<": "<<inlier<<endl;
		}
		//inliers_por_frame<<endl;
		//inliers_por_frame.close();
	}

	//Determina los outliers (proporcion > 10%)
	
	vector<int> outliers;
	
	for (auto const& x : track_inliers_map){
	    int trackId = x.first;
	    auto count = std::count(x.second.begin(), x.second.end(), false);
	    float prop = float(count)/x.second.size()*100;
	    this->outliers_prop[trackId] = prop;
	    if(prop>7.5) outliers.push_back(trackId);
	}



	int intento=0;
	while(mvPairs.size()>intento && !initialized){

		tie(num,f0,f1) = mvPairs[intento];

		kpsUn1 = mpInputReader->GetUndistortedKPs(f0, mK);
		kpsUn2 = mpInputReader->GetUndistortedKPs(f1, mK);
		cout <<endl<<endl<< "=========================================================="<<endl;
		cout << "Comenzando inicialización con frames " << f0 << " y " << f1 << endl
		<< "=========================================================="<< endl<<endl<<endl;
		mpInitializer = new Initializer(mK, kpsUn1, kpsUn2, 1.0, 100000);
		mvIniMatches = mpInputReader->GetMatchesSinOutliers(f0,f1,outliers);

		initialized = mpInitializer->Initialize(mvIniMatches, Rcw, tcw, mvIniP3D,vbTriangulated);

		if (!initialized){
			cout << "LA RECONSTRUCCIÓN FALLÓ. Se intentará con el próximo par de frames."<<endl;
			intento++;
		}
	}

	if(initialized){
		this->f0 = f0;
		this->f1 = f1;
		createInitialMap();
		mpMapManager->outliers = outliers;
		
	}else{
		cout << "LA RECONSTRUCCIÓN NO FUE POSIBLE CON NINGÚN PAR DE FRAMES"<<endl;
		return static_cast<MapManager*>(NULL);
	} 

	return mpMapManager;
}

//////////////////////////////////////////Implementación de la clase FactoryInitializationStrategies///////////////////////////////////////////////////////////
void FactoryInitializationStrategies::Register(const int tipoMetodo, CreateInitializationStrategyFn pfnCreate)
{
    m_FactoryMap[tipoMetodo] = pfnCreate;
}

FactoryInitializationStrategies::FactoryInitializationStrategies(){
	Register(1, &MostMatches_InitializationStrategy::Create);
	Register(2, &EveryPair_InitializationStrategy::Create);
}

InitializationStrategy *FactoryInitializationStrategies::CreateInitializationStrategy(const int tipoMetodo, InputReader* mpInputReader,float dist)
{
	FactoryMap::iterator it = m_FactoryMap.find(tipoMetodo);
	if( it != m_FactoryMap.end() )
		return it->second(mpInputReader, dist);
	return NULL;
}