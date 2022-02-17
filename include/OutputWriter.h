#ifndef OUTPUTWRITER_H
#define OUTPUTWRITER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <map>
#include <string>
#include <vector>

using namespace std;

class Map;

class OutputWriter {
public:
	OutputWriter(string strOutputPath, string strMatchesPath);
	const string& getStrOutputPath() const {return strOutputPath;}
	void guardarImagenes(vector<cv::Mat> imgs, map<int, string> names);
	void guardarResultados(int N, map<int, vector<float> > errors_map,
			map<int, vector<cv::Point2f> > rep_map,
			map<int, vector<cv::Point2f> > kps,
			map<int, vector<cv::Point2f> > points_map,
			map<int, vector<int> > track_ids, map<int, cv::Point3d> mps,
			int cal_1, int cal_2, map<int, string> names,
			map<int, vector<float> > radios,
			map<int, vector<float> > vols_rep_map,
			map<int, vector<float> > vols_real_map,
			map<int, string> labels);

protected:
	string strOutputPath;

};
#endif // OUTPUTWRITER_H;

