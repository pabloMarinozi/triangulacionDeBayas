#ifndef INPUTREADER_H
#define INPUTREADER_H

#include <opencv2/core/types.hpp>
#include <map>
#include <string>
#include <vector>


using namespace std;

class Map;

class InputReader {
public:
	InputReader(const string &strSettingPath, const string &strMatchesPaths);
	int GetNumFrames();
	vector<int> GetImageBounds(cv::Mat K);
	cv::Mat GetK();
	vector<int> GetInitialMatches();
	vector<int> GetNotInitialFrames();
	vector<int> GetMatches(int frameId1, int frameId2);
	vector<cv::Point2f> GetPoints(int frameId);
	vector<cv::KeyPoint> GetKPs(int frameId);
	vector<cv::KeyPoint> GetUndistortedKPs(int frameId, cv::Mat K);
	vector<int> GetTrackIds(int frameId);
	string GetImageName(int frameId);
	const map<int, vector<cv::Point2f> >& getKps() const {return kps;}
	void setKps(const map<int, vector<cv::Point2f> >& kps) {this->kps = kps;}
	const map<int, vector<int> >& getTrackIds() const {return track_ids;}
	void setTrackIds(const map<int, vector<int> >& trackIds) {track_ids = trackIds;}
	int getTrackCal1() const {return track_cal_1;}
	void setTrackCal1(int trackCal1) {track_cal_1 = trackCal1;}
	int getTrackCal2() const {return track_cal_2;}
	void setTrackCal2(int trackCal2) {track_cal_2 = trackCal2;}
	const map<int, string>& getImgNames() const {return img_names;}
	vector<int> GetIndexInKfs(vector<int>kfs, int track);
	int getFrame0() const {return frame0;}
	int getFrame1() const {return frame1;}
	const map<int, vector<float> >& getRadios() const {return radios;}
	const map<int, string>& getLabels() const {return labels;}

	bool error;

protected:
	const string strSettingPath;
	const string strMatchesPath;
	int numFrames;
	map<int, vector<cv::Point2f> > kps;
	map<int, vector<int> > track_ids;
	map<int, string> img_names;
	map<int, string> labels;
	map<int, vector<float> > radios;

	int track_cal_1;
	int track_cal_2;
	int frame0;
	int frame1;

};
#endif // INPUTREADER_H;

