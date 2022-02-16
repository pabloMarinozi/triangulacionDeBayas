#ifndef MAPCREATIONDTO_H
#define MAPCREATIONDTO_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <vector>


class MapCreationDTO{
public:
	cv::Mat Tcw1;
	cv::Mat Tcw2;
	std::vector<cv::KeyPoint> mvKeys1;
	std::vector<cv::KeyPoint> mvKeys2;
	std::vector<cv::KeyPoint> mvKeysUn1;
	std::vector<cv::KeyPoint> mvKeysUn2;
	std::vector<int> mvIniMatches;
	std::vector<cv::Point3f> mvIniP3D;
	std::vector<int> mvTracks;
	std::vector<int> bordes;
	cv::Mat mK;
	int frame0;
	int frame1;

	const std::vector<int>& getBordes() const {
		return bordes;
	}

	void setBordes(const std::vector<int>& bordes) {
		this->bordes = bordes;
	}

	const cv::Mat& getK() const {
		return mK;
	}

	void setK(const cv::Mat& k) {
		mK = k;
	}

	const std::vector<int>& getMvIniMatches() const {
		return mvIniMatches;
	}

	void setMvIniMatches(const std::vector<int>& mvIniMatches) {
		this->mvIniMatches = mvIniMatches;
	}

	const std::vector<cv::Point3f>& getMvIniP3D() const {
		return mvIniP3D;
	}

	void setMvIniP3D(const std::vector<cv::Point3f>& mvIniP3D) {
		this->mvIniP3D = mvIniP3D;
	}

	const std::vector<cv::KeyPoint>& getMvKeys1() const {
		return mvKeys1;
	}

	void setMvKeys1(const std::vector<cv::KeyPoint>& mvKeys1) {
		this->mvKeys1 = mvKeys1;
	}

	const std::vector<cv::KeyPoint>& getMvKeys2() const {
		return mvKeys2;
	}

	void setMvKeys2(const std::vector<cv::KeyPoint>& mvKeys2) {
		this->mvKeys2 = mvKeys2;
	}

	const cv::Mat& getTcw1() const {
		return Tcw1;
	}

	void setTcw1(const cv::Mat& tcw1) {
		Tcw1 = tcw1;
	}

	const cv::Mat& getTcw2() const {
		return Tcw2;
	}

	void setTcw2(const cv::Mat& tcw2) {
		Tcw2 = tcw2;
	}

	const std::vector<cv::KeyPoint>& getMvKeysUn1() const {
		return mvKeysUn1;
	}

	void setMvKeysUn1(const std::vector<cv::KeyPoint>& mvKeysUn1) {
		this->mvKeysUn1 = mvKeysUn1;
	}

	const std::vector<cv::KeyPoint>& getMvKeysUn2() const {
		return mvKeysUn2;
	}

	void setMvKeysUn2(const std::vector<cv::KeyPoint>& mvKeysUn2) {
		this->mvKeysUn2 = mvKeysUn2;
	}

	const std::vector<int>& getMvTracks() const {
		return mvTracks;
	}

	void setMvTracks(const std::vector<int>& mvTracks) {
		this->mvTracks = mvTracks;
	}

	int getFrame0() const {
		return frame0;
	}

	void setFrame0(int frame0) {
		this->frame0 = frame0;
	}

	int getFrame1() const {
		return frame1;
	}

	void setFrame1(int frame1) {
		this->frame1 = frame1;
	}
};
#endif // MAPCREATIONDTO_H
