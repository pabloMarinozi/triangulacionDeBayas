#include <Converter2D-3D.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <stddef.h>
#include <algorithm>
#include <iostream>
#include <utility>

Converter::Converter() {
}

vector<cv::Point2f> Converter::ReprojectAllMapPointsOnKeyFrame(KeyFrame* kf){
	vector<MapPoint*> visibles = kf->GetMapPointMatches();
	vector<cv::Point2f> rep;
	for (int i = 0; i < visibles.size(); ++i) {

		MapPoint* pMP = visibles[i];
		if(pMP){
			rep.push_back(ReprojectMapPointOnKeyFrame(pMP,kf));
		}
		else{

			rep.push_back(cv::Point2f(-1,-1));
		}
	}
	return rep;
}



cv::Point2f Converter::ReprojectMapPointOnKeyFrame(ORB_SLAM2::MapPoint* hp, ORB_SLAM2::KeyFrame* kf){
	cv::Mat p3d = hp->GetWorldPos().t();
	cv::Mat R = kf->GetRotation();
	cv::Mat t = kf->GetTranslation();
	cv::Mat K = kf->mK;
	return ReproyectarPunto(R,t,K,p3d);
}

cv::Point2f Converter::ReproyectarPunto(cv::Mat R, cv::Mat t, cv::Mat K, cv::Mat p3d){
	float fx = K.at<float>(0,0);
	float fy = K.at<float>(1,1);
	float cx = K.at<float>(0,2);
	float cy = K.at<float>(1,2);

	float z = R.row(2).dot(p3d) + t.at<float>(2);
	float invz1 = 1.0 / z;
	float x = R.row(0).dot(p3d) + t.at<float>(0);
	float y = R.row(1).dot(p3d) + t.at<float>(1);
	float u = fx * x * invz1 + cx;
	float v = fy * y * invz1 + cy;

	return cv::Point2f(u,v);
}

cv::Mat Converter::TriangularMapPoint(pair<KeyFrame*, cv::Point2f> kp1,
		pair<KeyFrame*, cv::Point2f> kp2) {

	try {
		//Recupera ambos keyframes
		ORB_SLAM2::KeyFrame* pKF1 = kp1.first;
		ORB_SLAM2::KeyFrame* pKF2 = kp2.first;
		cv::Point p1 = kp1.second;
		cv::Point p2 = kp2.second;

		//Recupera los datos de los keyframes que necesita para triangular
		cv::Mat Rcw1 = pKF1->GetRotation();
		cv::Mat Rwc1 = Rcw1.t();
		cv::Mat tcw1 = pKF1->GetTranslation();
		cv::Mat Tcw1(3, 4, CV_32F);
		Rcw1.copyTo(Tcw1.colRange(0, 3));
		tcw1.copyTo(Tcw1.col(3));
		cv::Mat Ow1 = pKF1->GetCameraCenter();
		cv::Mat K1 = pKF1->mK;

		const float &fx1 = K1.at<float>(0, 0);
		const float &fy1 = K1.at<float>(1, 1);
		const float &cx1 = K1.at<float>(0, 2);
		const float &cy1 = K1.at<float>(1, 2);
		const float &invfx1 = 1 / fx1;
		const float &invfy1 = 1 / fy1;

		//const float ratioFactor = 1.5f*pKF1->mfScaleFactor;

		cv::Mat Rcw2 = pKF2->GetRotation();
		cv::Mat Rwc2 = Rcw2.t();
		cv::Mat tcw2 = pKF2->GetTranslation();
		cv::Mat Tcw2(3, 4, CV_32F);
		Rcw2.copyTo(Tcw2.colRange(0, 3));
		tcw2.copyTo(Tcw2.col(3));

		cv::Mat K2 = pKF2->mK;

		const float &fx2 = K2.at<float>(0, 0);
		const float &fy2 = K2.at<float>(1, 1);
		const float &cx2 = K2.at<float>(0, 2);
		const float &cy2 = K2.at<float>(1, 2);
		const float &invfx2 = 1 / fx2;
		const float &invfy2 = 1 / fy2;

		// Check first that baseline is not too short
		cv::Mat Ow2 = pKF2->GetCameraCenter();
		cv::Mat vBaseline = Ow2 - Ow1;
		const float baseline = cv::norm(vBaseline);

		/*if(!mbMonocular)
		 {
		 if(baseline<pKF2->mb)
		 continue;
		 }
		 else*/
		{
			const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
			const float ratioBaselineDepth = baseline / medianDepthKF2;

			if (ratioBaselineDepth < 0.01) {
				cout
						<< "Falló la triangulación porque el ratioBaselineDepth < 0.01"
						<< endl;
				return cv::Mat();
			}
		}

		// Compute Fundamental Matrix
		cv::Mat F12 = ComputeF12(pKF1, pKF2);

		/*const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
		 const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];


		 const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
		 const float kp2_ur = pKF2->mvuRight[idx2];
		 */

		//seteo los booleanos en false porque son frames monoculares
		bool bStereo1 = false;
		bool bStereo2 = false;

		// Check parallax between rays
		cv::Mat xn1 =
				(cv::Mat_<float>(3, 1) << (kp1.second.x - cx1) * invfx1, (kp1.second.y
						- cy1) * invfy1, 1.0);
		cv::Mat xn2 =
				(cv::Mat_<float>(3, 1) << (kp2.second.x - cx2) * invfx2, (kp2.second.y
						- cy2) * invfy2, 1.0);

		cv::Mat ray1 = Rwc1 * xn1;
		cv::Mat ray2 = Rwc2 * xn2;
		const float cosParallaxRays = ray1.dot(ray2)
				/ (cv::norm(ray1) * cv::norm(ray2));

		float cosParallaxStereo = cosParallaxRays + 1;
		float cosParallaxStereo1 = cosParallaxStereo;
		float cosParallaxStereo2 = cosParallaxStereo;

		cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

		cv::Mat x3D;
		if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0
				&& (bStereo1 || bStereo2 || cosParallaxRays < 0.9998)) {
			// Linear Triangulation Method
			cv::Mat A(4, 4, CV_32F);
			A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
			A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
			A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
			A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

			cv::Mat w, u, vt;
			cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

			x3D = vt.row(3).t();

			if (x3D.at<float>(3) == 0)
				cout << "la ultima componente es 0" << endl;
			// Euclidean coordinates
			x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

		}

		cv::Mat x3Dt = x3D.t();

		//Check triangulation in front of cameras
		float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
		if (z1 <= 0) {
			cout << "\n\n\n LA TRIANGULACIÓN FALLÓ";
			cout << " porque el MapPoint quedó ubicado detrás del primer keyframe\n"
					<< endl;
			return cv::Mat();
		}

		float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
		if (z2 <= 0) {
			cout << "\n\n\n LA TRIANGULACIÓN FALLÓ";
			cout << " porque el MapPoint quedó ubicado detrás del primer keyframe\n"
					<< endl;
			return cv::Mat();
		}

		//Check reprojection error in first keyframe
		//const float &sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
		const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
		const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
		const float invz1 = 1.0 / z1;

		if (!bStereo1) {
			float u1 = fx1 * x1 * invz1 + cx1;
			float v1 = fy1 * y1 * invz1 + cy1;
			float errX1 = u1 - kp1.second.x;
			float errY1 = v1 - kp1.second.y;
			cout << "error1: " << errX1 << "," << errY1 << endl;
			//if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
			//continue;
		} else {
			float u1 = fx1 * x1 * invz1 + cx1;
			//float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
			float v1 = fy1 * y1 * invz1 + cy1;
			float errX1 = u1 - kp1.second.x;
			float errY1 = v1 - kp1.second.y;
			cout << "error2: " << errX1 << "," << errY1 << endl;
			//float errX1_r = u1_r - kp1_ur;
			//if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
			//continue;
		}

		//Check reprojection error in second keyframe
		//const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
		const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
		const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
		const float invz2 = 1.0 / z2;
		if (!bStereo2) {
			float u2 = fx2 * x2 * invz2 + cx2;
			float v2 = fy2 * y2 * invz2 + cy2;
			float errX2 = u2 - kp2.second.x;
			float errY2 = v2 - kp2.second.y;
			cout << "error: " << errX2 << "," << errY2 << endl;
			cout << "punto: " << u2 << ", " << v2 << endl;
			//if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
			//continue;
		} else {
			float u2 = fx2 * x2 * invz2 + cx2;
			//float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
			float v2 = fy2 * y2 * invz2 + cy2;
			float errX2 = u2 - kp2.second.x;
			float errY2 = v2 - kp2.second.y;
			cout << "error: " << errX2 << ", " << errY2 << endl;
			cout << "punto: " << u2 << ", " << v2 << endl;
			//float errX2_r = u2_r - kp2_ur;
			//if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
			//continue;
		}

		//Check scale consistency
		cv::Mat normal1 = x3D - Ow1;
		float dist1 = cv::norm(normal1);

		cv::Mat normal2 = x3D - Ow2;
		float dist2 = cv::norm(normal2);

		/*if (dist1 == 0 || dist2 == 0)
		 continue;

		 const float ratioDist = dist2 / dist1;
		 //const float ratioOctave = pKF1->mvScaleFactors[kp1.octave]
		 / pKF2->mvScaleFactors[kp2.octave];

		 if(fabs(ratioDist-ratioOctave)>ratioFactor)
		 continue;
		 if (ratioDist * ratioFactor < ratioOctave
		 || ratioDist > ratioOctave * ratioFactor)
		 continue;*/
		cout << "TRIANGULACION EXITOSA!" << endl;
		// Triangulation is succesfull
		return x3D;
	} catch (cv::Exception e) {
		cout << e.what() << endl;
		return cv::Mat();
	}
}

cv::Mat Converter::ComputeF12(ORB_SLAM2::KeyFrame *&pKF1,
		ORB_SLAM2::KeyFrame *&pKF2) {
	cv::Mat R1w = pKF1->GetRotation();
	cv::Mat t1w = pKF1->GetTranslation();
	cv::Mat R2w = pKF2->GetRotation();
	cv::Mat t2w = pKF2->GetTranslation();

	cv::Mat R12 = R1w * R2w.t();
	cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

	cv::Mat t12x = SkewSymmetricMatrix(t12);

	const cv::Mat &K1 = pKF1->mK;
	const cv::Mat &K2 = pKF2->mK;

	return K1.t().inv() * t12x * R12 * K2.inv();
}

cv::Mat Converter::SkewSymmetricMatrix(const cv::Mat &v) {
	return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1), v.at<
			float>(2), 0, -v.at<float>(0), -v.at<float>(1), v.at<float>(0), 0);
}


