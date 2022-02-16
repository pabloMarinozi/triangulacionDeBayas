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

#include <ext/type_traits.h>
#include <KeyFrame.h>
#include <Map.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <stddef.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <set>
#include <utility>
#include <vector>

using namespace std;

namespace ORB_SLAM2 {

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame(Map *pMap, std::vector<int> bordes,
		std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn,
		cv::Mat mK) :
		mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(
				0), mnBAGlobalForKF(0), N(mvKeys.size()), mvKeys(mvKeys), mvKeysUn(
				mvKeysUn), mnMinX(bordes[0]), mnMinY(bordes[1]), mnMaxX(
				bordes[2]), mnMaxY(bordes[3]), mK(mK), mbFirstConnection(true), mpParent(
				NULL), mbNotErase(false), mHalfBaseline(0), mbToBeErased(false), mbBad(
				false), mpMap(pMap), fx(mK.at<float>(0, 0)), fy(
				mK.at<float>(1, 1)), cx(mK.at<float>(0, 2)), cy(
				mK.at<float>(1, 2)), invfx(1.0f / fx), invfy(1.0f / fy) {
	mnId = nNextId++;
	mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

}

//void KeyFrame::ComputeBoW()
//{
//    if(mBowVec.empty() || mFeatVec.empty())
//    {
//        std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
//        // Feature std::vector associate features with nodes in the 4th level (from leaves up)
//        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
//        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
//    }
//}

void KeyFrame::SetPose(const cv::Mat &Tcw_) {
	//unique_lock<mutex> lock(mMutexPose);
	Tcw_.copyTo(Tcw);
	cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
	cv::Mat Rwc = Rcw.t();
	Ow = -Rwc * tcw;

	Twc = cv::Mat::eye(4, 4, Tcw.type());
	Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
	Ow.copyTo(Twc.rowRange(0, 3).col(3));
	cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
	Cw = Twc * center;
}

cv::Mat KeyFrame::GetPose() {
	//unique_lock<mutex> lock(mMutexPose);
	return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse() {
	//unique_lock<mutex> lock(mMutexPose);
	return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter() {
	//unique_lock<mutex> lock(mMutexPose);
	return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter() {
	//unique_lock<mutex> lock(mMutexPose);
	return Cw.clone();
}

cv::Mat KeyFrame::GetRotation() {
	//unique_lock<mutex> lock(mMutexPose);
	return Tcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation() {
	//unique_lock<mutex> lock(mMutexPose);
	return Tcw.rowRange(0, 3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
	{
		//unique_lock<mutex> lock(mMutexConnections);
		if (!mConnectedKeyFrameWeights.count(pKF))
			mConnectedKeyFrameWeights[pKF] = weight;
		else if (mConnectedKeyFrameWeights[pKF] != weight)
			mConnectedKeyFrameWeights[pKF] = weight;
		else
			return;
	}

	UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
	//unique_lock<mutex> lock(mMutexConnections);
	std::vector<pair<int, KeyFrame*> > vPairs;
	vPairs.reserve(mConnectedKeyFrameWeights.size());
	for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(),
			mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
		vPairs.push_back(make_pair(mit->second, mit->first));

	sort(vPairs.begin(), vPairs.end());
	list<KeyFrame*> lKFs;
	list<int> lWs;
	for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
		lKFs.push_front(vPairs[i].second);
		lWs.push_front(vPairs[i].first);
	}

	mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(lKFs.begin(),
			lKFs.end());
	mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames() {
	//unique_lock<mutex> lock(mMutexConnections);
	set<KeyFrame*> s;
	for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin();
			mit != mConnectedKeyFrameWeights.end(); mit++)
		s.insert(mit->first);
	return s;
}

std::vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames() {
	//unique_lock<mutex> lock(mMutexConnections);
	return mvpOrderedConnectedKeyFrames;
}

std::vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
	//unique_lock<mutex> lock(mMutexConnections);
	if ((int) mvpOrderedConnectedKeyFrames.size() < N)
		return mvpOrderedConnectedKeyFrames;
	else
		return std::vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),
				mvpOrderedConnectedKeyFrames.begin() + N);

}

std::vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w) {
	//unique_lock<mutex> lock(mMutexConnections);

	if (mvpOrderedConnectedKeyFrames.empty())
		return std::vector<KeyFrame*>();

	std::vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),
			mvOrderedWeights.end(), w, KeyFrame::weightComp);
	if (it == mvOrderedWeights.end())
		return std::vector<KeyFrame*>();
	else {
		int n = it - mvOrderedWeights.begin();
		return std::vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),
				mvpOrderedConnectedKeyFrames.begin() + n);
	}
}

int KeyFrame::GetWeight(KeyFrame *pKF) {
	//unique_lock<mutex> lock(mMutexConnections);
	if (mConnectedKeyFrameWeights.count(pKF))
		return mConnectedKeyFrameWeights[pKF];
	else
		return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
	//unique_lock<mutex> lock(mMutexFeatures);
	mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx) {
	//unique_lock<mutex> lock(mMutexFeatures);
	mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP) {
	int idx = pMP->GetIndexInKeyFrame(this);
	if (idx >= 0)
		mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP) {
	mvpMapPoints[idx] = pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints() {
	//unique_lock<mutex> lock(mMutexFeatures);
	std::set<MapPoint*> s;
	for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
		if (!mvpMapPoints[i])
			continue;
		MapPoint* pMP = mvpMapPoints[i];
		if (!pMP->isBad())
			s.insert(pMP);
	}
	return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
	//unique_lock<mutex> lock(mMutexFeatures);

	int nPoints = 0;
	const bool bCheckObs = minObs > 0;
	for (int i = 0; i < N; i++) {
		MapPoint* pMP = mvpMapPoints[i];
		if (pMP) {
			if (!pMP->isBad()) {
				if (bCheckObs) {
					if (mvpMapPoints[i]->Observations() >= minObs)
						nPoints++;
				} else
					nPoints++;
			}
		}
	}

	return nPoints;
}

std::vector<MapPoint*> KeyFrame::GetMapPointMatches() {
	//unique_lock<mutex> lock(mMutexFeatures);
	return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx) {
	//unique_lock<mutex> lock(mMutexFeatures);
	return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections() {
	std::map<KeyFrame*, int> KFcounter;

	std::vector<MapPoint*> vpMP;

	{
		//unique_lock<mutex> lockMPs(mMutexFeatures);
		vpMP = mvpMapPoints;
	}

	//For all map points in keyframe check in which other keyframes are they seen
	//Increase counter for those keyframes
	for (std::vector<MapPoint*>::iterator vit = vpMP.begin(), vend = vpMP.end();
			vit != vend; vit++) {
		MapPoint* pMP = *vit;

		if (!pMP)
			continue;

		if (pMP->isBad())
			continue;

		map<KeyFrame*, size_t> observations = pMP->GetObservations();

		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend =
				observations.end(); mit != mend; mit++) {
			if (mit->first->mnId == mnId)
				continue;
			KFcounter[mit->first]++;
		}
	}

	// This should not happen
	if (KFcounter.empty())
		return;

	//If the counter is greater than threshold add connection
	//In case no keyframe counter is over threshold add the one with maximum counter
	int nmax = 0;
	KeyFrame* pKFmax = NULL;
	int th = 15;

	std::vector<pair<int, KeyFrame*> > vPairs;
	vPairs.reserve(KFcounter.size());
	for (map<KeyFrame*, int>::iterator mit = KFcounter.begin(), mend =
			KFcounter.end(); mit != mend; mit++) {
		if (mit->second > nmax) {
			nmax = mit->second;
			pKFmax = mit->first;
		}
		if (mit->second >= th) {
			vPairs.push_back(make_pair(mit->second, mit->first));
			(mit->first)->AddConnection(this, mit->second);
		}
	}

	if (vPairs.empty()) {
		vPairs.push_back(make_pair(nmax, pKFmax));
		pKFmax->AddConnection(this, nmax);
	}

	sort(vPairs.begin(), vPairs.end());
	list<KeyFrame*> lKFs;
	list<int> lWs;
	for (size_t i = 0; i < vPairs.size(); i++) {
		lKFs.push_front(vPairs[i].second);
		lWs.push_front(vPairs[i].first);
	}

	{
		//unique_lock<mutex> lockCon(mMutexConnections);

		// mspConnectedKeyFrames = spConnectedKeyFrames;
		mConnectedKeyFrameWeights = KFcounter;
		mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(lKFs.begin(),
				lKFs.end());
		mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

		if (mbFirstConnection && mnId != 0) {
			mpParent = mvpOrderedConnectedKeyFrames.front();
			mpParent->AddChild(this);
			mbFirstConnection = false;
		}

	}
}

void KeyFrame::AddChild(KeyFrame *pKF) {
	//unique_lock<mutex> lockCon(mMutexConnections);
	mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF) {
	//unique_lock<mutex> lockCon(mMutexConnections);
	mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF) {
	//unique_lock<mutex> lockCon(mMutexConnections);
	mpParent = pKF;
	pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds() {
	//unique_lock<mutex> lockCon(mMutexConnections);
	return mspChildrens;
}

KeyFrame* KeyFrame::GetParent() {
	//unique_lock<mutex> lockCon(mMutexConnections);
	return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF) {
	//unique_lock<mutex> lockCon(mMutexConnections);
	return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
	//unique_lock<mutex> lockCon(mMutexConnections);
	mbNotErase = true;
	mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges() {
	//unique_lock<mutex> lockCon(mMutexConnections);
	return mspLoopEdges;
}

void KeyFrame::SetNotErase() {
	//unique_lock<mutex> lock(mMutexConnections);
	mbNotErase = true;
}

void KeyFrame::SetErase() {
	{
		//unique_lock<mutex> lock(mMutexConnections);
		if (mspLoopEdges.empty()) {
			mbNotErase = false;
		}
	}

	if (mbToBeErased) {
		cout << "se ha decidido borrar el keyframe " << mnId << endl;
		SetBadFlag();
	}
}

void KeyFrame::SetBadFlag() {
	{
		//unique_lock<mutex> lock(mMutexConnections);
		if (mnId == 0)
			return;
		else if (mbNotErase) {
			mbToBeErased = true;
			return;
		}
	}

	for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(),
			mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
		mit->first->EraseConnection(this);

	for (size_t i = 0; i < mvpMapPoints.size(); i++)
		if (mvpMapPoints[i])
			mvpMapPoints[i]->EraseObservation(this);
	{
		//unique_lock<mutex> lock(mMutexConnections);
		//unique_lock<mutex> lock1(mMutexFeatures);

		mConnectedKeyFrameWeights.clear();
		mvpOrderedConnectedKeyFrames.clear();

		// Update Spanning Tree
		set<KeyFrame*> sParentCandidates;
		sParentCandidates.insert(mpParent);

		// Assign at each iteration one children with a parent (the pair with highest covisibility weight)
		// Include that children as new parent candidate for the rest
		while (!mspChildrens.empty()) {
			bool bContinue = false;

			int max = -1;
			KeyFrame* pC;
			KeyFrame* pP;

			for (set<KeyFrame*>::iterator sit = mspChildrens.begin(), send =
					mspChildrens.end(); sit != send; sit++) {
				KeyFrame* pKF = *sit;
				if (pKF->isBad())
					continue;

				// Check if a parent candidate is connected to the keyframe
				std::vector<KeyFrame*> vpConnected =
						pKF->GetVectorCovisibleKeyFrames();
				for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
					for (set<KeyFrame*>::iterator spcit =
							sParentCandidates.begin(), spcend =
							sParentCandidates.end(); spcit != spcend; spcit++) {
						if (vpConnected[i]->mnId == (*spcit)->mnId) {
							int w = pKF->GetWeight(vpConnected[i]);
							if (w > max) {
								pC = pKF;
								pP = vpConnected[i];
								max = w;
								bContinue = true;
							}
						}
					}
				}
			}

			if (bContinue) {
				pC->ChangeParent(pP);
				sParentCandidates.insert(pC);
				mspChildrens.erase(pC);
			} else
				break;
		}

		// If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
		if (!mspChildrens.empty())
			for (set<KeyFrame*>::iterator sit = mspChildrens.begin();
					sit != mspChildrens.end(); sit++) {
				(*sit)->ChangeParent(mpParent);
			}

		if (mpParent) {
			mpParent->EraseChild(this);
			mTcp = Tcw * mpParent->GetPoseInverse();
		}
		mbBad = true;
	}

	mpMap->EraseKeyFrame(this);
}

bool KeyFrame::isBad() {
	//unique_lock<mutex> lock(mMutexConnections);
	return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF) {
	bool bUpdate = false;
	{
		//unique_lock<mutex> lock(mMutexConnections);
		if (mConnectedKeyFrameWeights.count(pKF)) {
			mConnectedKeyFrameWeights.erase(pKF);
			bUpdate = true;
		}
	}

	if (bUpdate)
		UpdateBestCovisibles();
}

bool KeyFrame::IsInImage(const float &x, const float &y) const {
	return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
	std::vector<MapPoint*> vpMapPoints;
	cv::Mat Tcw_;
	{
		//unique_lock<mutex> lock(mMutexFeatures);
		//unique_lock<mutex> lock2(mMutexPose);
		vpMapPoints = mvpMapPoints;
		Tcw_ = Tcw.clone();
	}

	std::vector<float> vDepths;
	vDepths.reserve(N);
	cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
	Rcw2 = Rcw2.t();
	float zcw = Tcw_.at<float>(2, 3);
	for (int i = 0; i < N; i++) {
		if (mvpMapPoints[i]) {
			MapPoint* pMP = mvpMapPoints[i];
			cv::Mat x3Dw = pMP->GetWorldPos();
			float z = Rcw2.dot(x3Dw) + zcw;
			vDepths.push_back(z);
		}
	}

	sort(vDepths.begin(), vDepths.end());

	return vDepths[(vDepths.size() - 1) / q];
}

} //namespace ORB_SLAM
