#pragma once
#include "LetsGo.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "opencv2/opencv.hpp"

typedef struct SlamKeyframeCameraData
{
	int id_kf;
	int id_frame;

	cv::Mat camMat;
	cv::Mat camRot;

	cv::Point3d camTrans;
	cv::Point3d camRotEuler;

	FVector camTransConv;
	FRotator camRotEulerConv;

}SlamCameraData;

typedef struct SlamFrameCameraData
{
	int id_frame;

	cv::Mat camMat;
	cv::Mat camRot;

	cv::Point3d camTrans;
	cv::Point3d camRotEuler;

	FVector camTransConv;
	FRotator camRotEulerConv;

}SlamFrameCameraData;

typedef struct SlamLandmarkData
{
	int id_kf;
	int id_frame;

	cv::Point3d pos_w;

	FVector posConv;

	bool isOutlier;
}SlamLandmarkData;

class SlamDatabase
{
public:
	std::string dataFilePath;
	float mapScale;
	bool isReadXYZForExp;
	int numKeyframeCamera = 0;
	int numFrameCamera = 0;
	int numLandmark = 0;
	std::vector<SlamKeyframeCameraData*> keyframeCamDataList;
	std::vector<SlamFrameCameraData*> frameCamDataList;
	std::vector<SlamLandmarkData*> landmarkDataList;

	int curFrameNum = 0;
	std::vector<SlamKeyframeCameraData*> curFrameKFs;
	std::vector<SlamFrameCameraData*> curFrameCams;
	std::vector<SlamLandmarkData*> curFrameLandmarks;
	std::vector<FVector> curFrameInlierLandmarks;

public:
	SlamDatabase(std::string _dataFilePath, float _mapScale, bool _isReadXYZForExp);
	void InitDataForXYZ();
	void InitData();

	// frame initialization
	void initCurFrameData(int frameNum);

	// rotation to euler
	bool isRotationMatrix(cv::Mat &R);
	bool closeEnough(const float& a, const float& b, const float& epsilon);
	cv::Point3d rotMatToEulerAngle(cv::Mat &R);

	// coordinate conversion
	static FVector convertCoord(FVector trans);
	static FRotator convertCoord(FRotator rot);
};

