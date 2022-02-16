#ifndef CONVERTER2D-3D_H
#define CONVERTER2D-3D_H

#include <KeyFrame.h>
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

using namespace ORB_SLAM2;
using namespace std;
class Map;

class Converter {
public:
	Converter();
	cv::Mat TriangularMapPoint(pair<KeyFrame*, cv::Point2f> kp1,
			pair<KeyFrame*, cv::Point2f> kp2);
	vector<cv::Point2f> ReprojectAllMapPointsOnKeyFrame(KeyFrame* kf);
	cv::Point2f ReprojectMapPointOnKeyFrame(MapPoint* hp, KeyFrame* kf);
	cv::Point2f ReproyectarPunto(cv::Mat R, cv::Mat t, cv::Mat K, cv::Mat p3d);

protected:
	cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
	cv::Mat ComputeF12(ORB_SLAM2::KeyFrame *&pKF1, ORB_SLAM2::KeyFrame *&pKF2);

};
#endif // CONVERTER2D-3D_H
