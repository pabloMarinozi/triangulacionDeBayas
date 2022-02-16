//============================================================================
// Name        : InputReader.cpp
// Author      : Pablo
// Version     :
// Copyright   : Hecho para Óptima.
// Description : Lee las entradas al algoritmo
//============================================================================

#include <InputReader.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

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

InputReader::InputReader(const string &strSettingPath,
		const string &strMatchesPath) :
		strSettingPath(strSettingPath), strMatchesPath(strMatchesPath) {
	std::string str;
	ifstream myfile;
	myfile.open(strMatchesPath);
	error = false;
	int numcolsperframe = 4;
	getline(myfile, str); //se deshace del encabezado
	while (getline(myfile, str)) {
		vector<string> cols = getCols(str);
		if (cols.size() % numcolsperframe != 3) {
			cout << "La cantidad de columnas de " << strMatchesPath
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
			if (cols[1] == "cal_1")
				track_cal_1 = id_track;
			else if (cols[1] == "cal_2")
				track_cal_2 = id_track;
			for (int i = 0; i < numFrames; i++) {
				if (cols[i * numcolsperframe + 3] != "NULL") {
					img_names[i] = cols[i * numcolsperframe + 3];
					cv::Point2f kp = cv::Point2f(
							atof(cols[i * numcolsperframe + 4].c_str()),
							atof(cols[i * numcolsperframe + 5].c_str()));
					kps[i].push_back(kp);
					radios[i].push_back(
							atof(cols[i * numcolsperframe + 6].c_str()));
					track_ids[i].push_back(id_track);
				}
			}
		}
	}
	frame0 = -1;
	frame1 = -1;

}

cv::Mat InputReader::GetK() {
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
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

vector<int> InputReader::GetImageBounds(cv::Mat K) {
	int mnMinX, mnMinY, mnMaxX, mnMaxY;
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
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

vector<int> InputReader::GetMatches(int frameId1, int frameId2) {
	vector<int> matches;
	vector<int> tracks1 = track_ids[frameId1];
	vector<int> tracks2 = track_ids[frameId2];
	for (int i1 = 0; i1 < tracks1.size(); ++i1) {
		int match_index = -1;
		for (int i2 = 0; i2 < tracks2.size(); ++i2) {
			if (tracks1[i1] == tracks2[i2]) {
				match_index = i2;
				break;
			}
		}
		matches.push_back(match_index);
	}
	return matches;
}

vector<cv::Point2f> InputReader::GetPoints(int frameId) {
	return kps[frameId];
}

int InputReader::GetNumFrames() {
	return numFrames;
}

string InputReader::GetImageName(int frameId) {
	string full_path = dirnameOf(strMatchesPath) + "/" + img_names[frameId];
	return full_path;
}

vector<cv::KeyPoint> InputReader::GetUndistortedKPs(int frameId, cv::Mat mK) {

	//Lee los coeficientes de distorsión de los archivos
	cv::Mat DistCoef(4, 1, CV_32F);
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
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

vector<int> InputReader::GetInitialMatches() {
	int kf1, kf2, max = 0;
	for (int i = 0; i < numFrames; i++) {
		for (int j = 0; j < numFrames; j++) {
			if (i == j)
				continue;
			vector<int> matches = GetMatches(i, j);
			int num = 0;
			for (int k = 0; k < matches.size(); k++) {
				if (matches[k] != -1)
					num++;
			}
			if (num > max) {
				kf1 = i;
				kf2 = j;
				max = num;
			}
		}
	}
	frame0 = kf1;
	frame1 = kf2;
	return GetMatches(frame0, frame1);

}

vector<int> InputReader::GetNotInitialFrames() {
	if (frame0 < 0 || frame1 < 0)
		vector<int> m = GetInitialMatches();
	vector<int> f;
	for (int i = 0; i < numFrames; i++) {
		if (i != frame0 && i != frame1)
			f.push_back(i);
	}
	return f;
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
