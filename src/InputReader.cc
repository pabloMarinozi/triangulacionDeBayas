//============================================================================
// Name        : InputReader.cpp
// Author      : Pablo
// Version     :
// Copyright   : Hecho para Óptima.
// Description : Lee las entradas al algoritmo
//============================================================================

#include <InputReader.h>
#include <colormod.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <algorithm>
#include <random>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
typedef pair<int, int> Match;

vector<string> getCols(string str) {
	vector<string> cols;
	std::size_t a, b;
	a = str.find(',', 0);
	cols.push_back(str.substr(0, a));
	bool frenar = false;
	while (!frenar) {
		b = str.find(',', a + 1);
		if (b == std::string::npos)
			frenar = true;
		cols.push_back(str.substr(a + 1, b - a - 1));
		//str = str.substr(b);
		a = b;
	}
	return cols;
}

std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

InputParser::InputParser (int &argc, char **argv){
    for (int i=1; i < argc; ++i)
        this->tokens.push_back(std::string(argv[i]));
}

std::string InputParser::getCmdOption(std::string option) {
    std::vector<std::string>::iterator itr;
    itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
    if (itr != this->tokens.end() && ++itr != this->tokens.end()){
        return *itr;
    }
    static std::string empty_string("");
    return empty_string;
}

/// @author iain
bool InputParser::cmdOptionExists(std::string option) {
    return std::find(this->tokens.begin(), this->tokens.end(), option)
           != this->tokens.end();
}

string c = "-c"; string o = "-o"; string d = "-d"; string i = "-i"; string x = "-x"; string imd = "--init_distance";

void InputParser::checkArgs(){
	Color::Modifier red(Color::FG_RED);
	Color::Modifier yellow(Color::FG_YELLOW);
	Color::Modifier def(Color::FG_DEFAULT);     	
	if(!cmdOptionExists(c)){
		cout<<red<<"No se ha pasado el argumento '-c' con la ruta al archivo de calibración."<<def<<endl;
		calibFlag = false;
	}else calibFlag = true;
	if(!cmdOptionExists(o)){
		cout<<red<<"No se ha pasado el argumento '-o' con la ruta de salida."<<def<<endl;
		outputFlag = false;
	}else outputFlag = true;
	if(!cmdOptionExists(d)){
		cout<<yellow<<"No se ha pasado el argumento '-d' con la ruta del archivo con los bundles de las bayas."<<def<<endl;
		detectionsFlag = false;
	}else detectionsFlag = true;
	if(!cmdOptionExists(i)){
		cout<<yellow<<"No se ha pasado el argumento '-i' con la ruta a la carpeta de imágenes."<<def<<endl;
		imagesFlag = false;
	}else imagesFlag = true;
	if(!cmdOptionExists(x)){
		cout<<yellow<<"No se ha pasado el argumento '-x' con la distancia en cm usada para calibrar"<<def<<endl;
		scaleFlag = false;
	}else scaleFlag = true;
	if(!cmdOptionExists(imd)){
		cout<<yellow<<"No se ha pasado el argumento '--init_distance' con la distancia mínima entre frames inicializadores"<<def<<endl;
		initFlag = false;
	}else initFlag = true;
}

InputReader::InputReader(InputParser* input){
	calibPath = input->getCmdOption(c);
	if(input->detectionsFlag) parseDetectionsFile(input->getCmdOption(d));
	if(input->imagesFlag) imagesPath = input->getCmdOption(i);
	cout << "imagesPath " << imagesPath;
	if(input->initFlag) init_min_dist = stoi(input->getCmdOption(imd));
	if(input->scaleFlag) scaleDistance = stof(input->getCmdOption(x));
}

void InputReader::parseDetectionsFile(string detectionMatchesPath){
	std::string str;
	ifstream matchesFile;
	matchesFile.open(detectionMatchesPath);
	track_cal_1 = -1;
	track_cal_2 = -1;
	track_val_1 = 0;
	track_val_2 = 0;
	error = false;
	int numcolsperframe = 4;
	getline(matchesFile, str); //se deshace del encabezado
	while (getline(matchesFile, str)) {
		vector<string> cols = getCols(str);
		if (cols.size() % numcolsperframe != 3) {
			cout << "La cantidad de columnas de " << detectionMatchesPath
					<< " no coincide con la cantidad de frames indicada.";
			error = true;
			break;
		} else if (atoi(cols[2].c_str()) < 2) {
			cout << "Se ignorará el bundle con id " << cols[0]
					<< " debido a que no tiene suficientes observaciones"
					<< endl;
			continue;
		} else {
			numFrames = (cols.size() - 3) / numcolsperframe;
			int id_track = atoi(cols[0].c_str());
			labels[id_track] = cols[1];
			if (cols[1] == "cal_1") track_cal_1 = id_track;
			else if (cols[1] == "cal_2") track_cal_2 = id_track;
			if (cols[1] == "val_1") track_val_1 = id_track;
			else if (cols[1] == "val_2") track_val_2 = id_track;
			for (int i = 0; i < numFrames; i++) {
				if (cols[i * numcolsperframe + 3] != "NULL") {
					img_names[i] = cols[i * numcolsperframe + 3];
					cv::Point2f kp = cv::Point2f(
							atof(cols[i * numcolsperframe + 4].c_str()),
							atof(cols[i * numcolsperframe + 5].c_str()));
					kps[i].push_back(kp);
					radios[i][id_track] =
							atof(cols[i * numcolsperframe + 6].c_str());
					track_ids[i].push_back(id_track);
				}
			}
			allTracks.push_back(id_track);
		}
	}
}

void InputReader::showRadios(){
	for(auto const& imap: radios){
		cout << "Frame "<<imap.first<<endl<<endl;
		map<int, float> radios_frame = imap.second;
		for(auto const& imap2: radios_frame){
			cout << imap2.first <<"	"<<imap2.second<<endl;
		}
	}

}

cv::Mat InputReader::GetK() {
	cv::FileStorage fSettings(calibPath, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];
	cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
	K.at<float>(0, 0) = fx;
	K.at<float>(1, 1) = fy;
	K.at<float>(0, 2) = cx;
	K.at<float>(1, 2) = cy;

	return K;
}

float InputReader::GetFocalDistance() {
	cv::FileStorage fSettings(calibPath, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	return (fx+fy)/2;
}


vector<int> InputReader::GetImageBounds(cv::Mat K) {
	int mnMinX, mnMinY, mnMaxX, mnMaxY;
	cv::FileStorage fSettings(calibPath, cv::FileStorage::READ);
	cv::Mat DistCoef(4, 1, CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if (k3 != 0) {
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	}

	int cols = fSettings["Camera.width"];
	int rows = fSettings["Camera.heigth"];

	if (DistCoef.at<float>(0) != 0.0) {
		cv::Mat mat(4, 2, CV_32F);
		mat.at<float>(0, 0) = 0.0;
		mat.at<float>(0, 1) = 0.0;
		mat.at<float>(1, 0) = cols;
		mat.at<float>(1, 1) = 0.0;
		mat.at<float>(2, 0) = 0.0;
		mat.at<float>(2, 1) = rows;
		mat.at<float>(3, 0) = cols;
		mat.at<float>(3, 1) = rows;

		// Undistort corners
		mat = mat.reshape(2);
		cv::undistortPoints(mat, mat, K, DistCoef, cv::Mat(), K);
		mat = mat.reshape(1);

		mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
		mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
		mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
		mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));

	} else {
		mnMinX = 0.0f;
		mnMaxX = cols;
		mnMinY = 0.0f;
		mnMaxY = rows;
	}
	return vector<int> { mnMinX, mnMinY, mnMaxX, mnMaxY };
}

vector<cv::KeyPoint> InputReader::GetKPs(int frameId) {
	vector<cv::KeyPoint> mvKeys;
	vector<cv::Point2f> points = kps[frameId];
	cv::KeyPoint::convert(points, mvKeys);
	return mvKeys;
}

vector<int> InputReader::GetTrackIds(int frameId) {
	return track_ids[frameId];
}

std::string dirnameOf(const std::string& fname) {
	size_t pos = fname.find_last_of("/");
	return (std::string::npos == pos) ? "" : fname.substr(0, pos);
}



/**
 * Get observations of the same 3D Points on both frames.
 *
 * @params frameId1, frameId2 ids of the frames whose matches are being looked for.
 * @return a map whith trackId as keys and matches as values 
 */
map<int, Match> InputReader::GetMatches(int frameId1, int frameId2) {
	map<int, Match> matches;
	vector<int> tracks1 = track_ids[frameId1];
	vector<int> tracks2 = track_ids[frameId2];
	for (int i1 = 0; i1 < tracks1.size(); ++i1) {
		int match_index = -1, trackId = tracks1[i1];
		for (int i2 = 0; i2 < tracks2.size(); ++i2) {
			if (trackId == tracks2[i2]) {
				match_index = i2;
				break;
			}
		}
		if(match_index>0) matches[trackId] = make_pair(i1, match_index);
	}
	return matches;
}

map<int, Match> InputReader::GetMatchesSinOutliers(int frameId1, int frameId2, vector<int> outliers) {
	map<int, Match> matches;
	vector<int> tracks1 = track_ids[frameId1];
	vector<int> tracks2 = track_ids[frameId2];
	for (int i1 = 0; i1 < tracks1.size(); ++i1) {
		int match_index = -1, trackId = tracks1[i1];
		if (std::find(outliers.begin(), outliers.end(), trackId) != outliers.end())
			continue;
		for (int i2 = 0; i2 < tracks2.size(); ++i2) {
			if (trackId == tracks2[i2]) {
				match_index = i2;
				break;
			}
		}
		if(match_index>=0) matches[trackId] = make_pair(i1, match_index);
	}
	return matches;
}


map<int, cv::Point2f> InputReader::GetPoints(int frameId) {
	map<int, cv::Point2f> mKps;
	vector<cv::Point2f> vKps = kps[frameId];
	vector<int> tracks = track_ids[frameId];
	for (int j = 0; j < vKps.size(); ++j) {
		int t  = tracks[j];
		mKps[t] = vKps[j];
	}
	return mKps;
}

int InputReader::GetNumFrames() {
	return numFrames;
}

int InputReader::GetInitDist() {
	return init_min_dist;
}

string InputReader::GetImageName(int frameId) {
	string full_path = imagesPath + "/" + img_names[frameId];
	return full_path;
}

vector<cv::KeyPoint> InputReader::GetUndistortedKPs(int frameId, cv::Mat mK) {

	//Lee los coeficientes de distorsión de los archivos
	cv::Mat DistCoef(4, 1, CV_32F);
	cv::FileStorage fSettings(calibPath, cv::FileStorage::READ);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if (k3 != 0) {
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	}

	//Obtiene los kp
	vector<cv::KeyPoint> mvKeys = GetKPs(frameId);
	vector<cv::KeyPoint> mvKeysUn;
	if (DistCoef.at<float>(0) == 0.0) {
		mvKeysUn = mvKeys;
		return mvKeysUn;
	}

	// Fill matrix with points
	int N = mvKeys.size();
	cv::Mat mat(N, 2, CV_32F);
	for (int i = 0; i < N; i++) {
		mat.at<float>(i, 0) = mvKeys[i].pt.x;
		mat.at<float>(i, 1) = mvKeys[i].pt.y;
	}

	// Undistort points
	mat = mat.reshape(2);
	cv::undistortPoints(mat, mat, mK, DistCoef, cv::Mat(), mK);
	mat = mat.reshape(1);

	// Fill undistorted keypoint vector
	mvKeysUn.resize(N);
	for (int i = 0; i < N; i++) {
		cv::KeyPoint kp = mvKeys[i];
		kp.pt.x = mat.at<float>(i, 0);
		kp.pt.y = mat.at<float>(i, 1);
		mvKeysUn[i] = kp;
	}
	return mvKeysUn;
}

bool sortdesc(const tuple<int, int, int>& a,
              const tuple<int, int, int>& b)
{
    return (get<0>(a) > get<0>(b));
}



vector<tuple<int,int,int> > InputReader::GetInitialPairsFromMostMatches(float dist) {
	vector<tuple<int,int,int> > pairs;

	//int kf1, kf2, max = 0;
	for (int i = 0; i < numFrames; i++) {
		for (int j = 0; j < numFrames; j++) {
			if (i >= j | abs(i-j) != dist)
				continue;
			map<int, Match> matches = GetMatches(i, j);
			int num = matches.size();
			if (num<7) continue;
			//cout << num <<" "<<i<<" "<<j <<endl;
			pairs.push_back(make_tuple(num,i,j));
			}
	}	
	sort(pairs.begin(), pairs.end(), sortdesc);
	return pairs;	
}

vector<tuple<int,int,int> > InputReader::GetInitialPairsFromLeastMatches() {
	vector<tuple<int,int,int> > pairs;
	float dist = 15;

	//int kf1, kf2, max = 0;
	for (int i = 0; i < numFrames; i++) {
		for (int j = 0; j < numFrames; j++) {
			if (i >= j | abs(i-j) < dist)
				continue;
			map<int, Match> matches = GetMatches(i, j);
			int num = matches.size();
			if (num<7) continue;
			//cout << num <<" "<<i<<" "<<j <<endl;
			pairs.push_back(make_tuple(num,i,j));
			}
	}	
	sort(pairs.begin(), pairs.end());//, sortdesc);
	return pairs;	
}

vector<tuple<int,int,int> > InputReader::GetInitialPairsFromQuartiles() {
	vector<tuple<int,int,int> > pairs;
	int q = numFrames/4;
	int q1 = q;
	int q3 = 3*q;
	int index = 0; 

	while(q1!=q3){
		pairs.push_back(make_tuple(index,q1,q3));
		index++; q1++; q3--;
	}
	return pairs;	
}
	

vector<int> InputReader::GetNotInitialFrames(int n=0) {
	//if (frame0 < 0 || frame1 < 0)
	//	vector<int> m = GetInitialMatches();
	vector<int> f;
	for (int i = 0; i < numFrames; i++) {
		if (i != frame0 && i != frame1)
			f.push_back(i);
	}
	if(n>0){
		vector<int> out;
	    size_t nelems = n;

		std::sample(f.begin(),f.end(),
	        std::back_inserter(out), n,
	        std::mt19937{std::random_device{}()});
		return out;
	}else{
		return f;
	}
}



vector<int> InputReader::GetIndexInKfs(vector<int> kfs, int track) {
	vector<int> indexes;
	for (auto id : kfs) {
		vector<int> tracksKf = track_ids[id];
		int index = -1;
		for (int i = 0; i < tracksKf.size(); i++) {
			if (tracksKf[i] == track) {
				index = i;
				break;
			}
		}
		indexes.push_back(index);
	}
	return indexes;
}
