#ifndef MAPMANAGER_H
#define MAPMANAGER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <map>
#include <vector>

class InputReader;
class MapCreationDTO;
namespace ORB_SLAM2 {
class KeyFrame;
class Map;
} /* namespace ORB_SLAM2 */

using namespace ORB_SLAM2;
using namespace std;
class Map;

class MapManager {
public:
	MapManager();
	void CreateInitialMapMonocular(MapCreationDTO dto);
	void CreateNewKeyFrame(int id, InputReader* inputReader);
	map<int, cv::Point2f>  ReproyectAllMapPointsOnKeyFrame(int id);
	map<int, cv::Point3d> GetAllMapPoints();
  map<int, cv::Point3d> GetAllCameras();
  vector<long unsigned int> GetAllKeyFramesId();
	void ScaleMap(float scaleFactor);
	float GetScaleFactor(float dist);
	float GetDistanceCal1Cal2(int id1, int id2);
	map<int, cv::Point2f> CreatePointsOnNormalPlane(int id, float distance);
  map<int, cv::Point2f> GetPoints(int frameId);

	ORB_SLAM2::Map* mpMap;
	unsigned long int mnLastKeyFrameId;
	KeyFrame* mpLastKeyFrame;
	cv::Mat mVelocity; //Motion Model
  vector<int> outliers;

};
#endif // MAPMANAGER_H
