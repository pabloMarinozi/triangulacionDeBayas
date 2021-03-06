/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <MapPoint.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <stddef.h>
#include <map>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

namespace ORB_SLAM2 {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame {
public:
	KeyFrame(Map *pMap, std::vector<int> bordes,
			std::vector<cv::KeyPoint> mvKeys,
			std::vector<cv::KeyPoint> mvKeysUn, cv::Mat mK);

	// Pose functions
	void SetPose(const cv::Mat &Tcw);
	cv::Mat GetPose();
	cv::Mat GetPoseInverse();
	cv::Mat GetCameraCenter();
	cv::Mat GetStereoCenter();
	cv::Mat GetRotation();
	cv::Mat GetTranslation();

	// Bag of Words Representation
	void ComputeBoW();

	// Covisibility graph functions
	void AddConnection(KeyFrame* pKF, const int &weight);
	void EraseConnection(KeyFrame* pKF);
	void UpdateConnections();
	void UpdateBestCovisibles();
	std::set<KeyFrame *> GetConnectedKeyFrames();
	std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
	std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
	std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
	int GetWeight(KeyFrame* pKF);

	// Spanning tree functions
	void AddChild(KeyFrame* pKF);
	void EraseChild(KeyFrame* pKF);
	void ChangeParent(KeyFrame* pKF);
	std::set<KeyFrame*> GetChilds();
	KeyFrame* GetParent();
	bool hasChild(KeyFrame* pKF);

	// Loop Edges
	void AddLoopEdge(KeyFrame* pKF);
	std::set<KeyFrame*> GetLoopEdges();

	// MapPoint observation functions
	void AddMapPoint(MapPoint* pMP, const size_t &idx);
	void EraseMapPointMatch(const size_t &idx);
	void EraseMapPointMatch(MapPoint* pMP);
	void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
	std::set<MapPoint*> GetMapPoints();
	std::vector<MapPoint*> GetMapPointMatches();
	int TrackedMapPoints(const int &minObs);
	MapPoint* GetMapPoint(const size_t &idx);

	// KeyPoint functions
	std::vector<size_t> GetFeaturesInArea(const float &x, const float &y,
			const float &r) const;
	cv::Mat UnprojectStereo(int i);

	// Image
	bool IsInImage(const float &x, const float &y) const;

	// Enable/Disable bad flag changes
	void SetNotErase();
	void SetErase();

	// Set/check bad flag
	void SetBadFlag();
	bool isBad();

	// Compute Scene Depth (q=2 median). Used in monocular.
	float ComputeSceneMedianDepth(const int q);

	static bool weightComp(int a, int b) {
		return a > b;
	}

	static bool lId(KeyFrame* pKF1, KeyFrame* pKF2) {
		return pKF1->mnId < pKF2->mnId;
	}

	/**
	 * Reconstruye las observaciones de los puntos observados por el keyframe.
	 *
	 * Usado exclusivamente en la serialización, para reconstruir datos redundantes.
	 *
	 * Los keyframes registran los puntos obervados, y éstos registran los keyframes que los observan.
	 * Sólo los primeros se serializan, los segundos se reconstruyen con este método.
	 *
	 * Invocado sólo por Serializer::mapLoad
	 */
	void buildObservations();

	const cv::Mat getImGray() const {
		return img.clone();
	}

	// The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

	static long unsigned int nNextId;
	long unsigned int mnId;
//	const long unsigned int mnFrameId;
//
//	const double mTimeStamp;
	bool mbNotErase;

	// Variables used by the tracking
	long unsigned int mnTrackReferenceForFrame;
	long unsigned int mnFuseTargetForKF;

	// Variables used by the local mapping
	long unsigned int mnBALocalForKF;
	long unsigned int mnBAFixedForKF;

	// Variables used by loop closing
	cv::Mat mTcwGBA;
	cv::Mat mTcwBefGBA;
	long unsigned int mnBAGlobalForKF;

	// Calibration parameters
	const float fx, fy, cx, cy, invfx, invfy; // mbf, mb, mThDepth;

	// Number of KeyPoints
	const int N;

	// KeyPoints, stereo coordinate and descriptors (all associated by an index)
	std::vector<cv::KeyPoint> mvKeys;
	std::vector<cv::KeyPoint> mvKeysUn;

	// Image bounds and calibration
	const int mnMinX;
	const int mnMinY;
	const int mnMaxX;
	const int mnMaxY;
	const cv::Mat mK;

	// The following variables need to be accessed trough a mutex to be thread safe.
protected:

	// SE3 Pose and camera center
	cv::Mat Tcw;
	cv::Mat Twc;
	cv::Mat Ow;

	cv::Mat Cw; // Stereo middel point. Only for visualization

	//imagen en escala de grises
	cv::Mat img;

	// MapPoints associated to keypoints
	std::vector<MapPoint*> mvpMapPoints;

	std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
	std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
	std::vector<int> mvOrderedWeights;

	// Pose relative to parent (this is computed when bad flag is activated)
	cv::Mat mTcp;

	// Spanning Tree and Loop Edges
	bool mbFirstConnection;
	KeyFrame* mpParent;
	std::set<KeyFrame*> mspChildrens;
	std::set<KeyFrame*> mspLoopEdges;

	// Bad flags

	bool mbToBeErased;
	bool mbBad;
	float mHalfBaseline; // Only for visualization

	Map* mpMap;

	std::mutex mMutexPose;
	std::mutex mMutexConnections;
	std::mutex mMutexFeatures;

};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
