#pragma once
#include "LetsGo.h"
#include <iostream>
#include <fstream>
#include <string>
#include <io.h>
#include "opencv2/opencv.hpp"
#include "CubemapMgr.h"
#include "SphericalProjectorDatabase.h"

typedef struct CubemapImgStruct
{
	cv::VideoCapture VCap;
	cv::Size sz;

	cv::Mat sourceImg;
	cv::Mat panoramaImg;

	cv::Mat projectedImgs[6];
	FVector recentProjBox[8];
}CubemapImgStruct;

class CubemapProjector
{
public:
	std::string dataFilePath;
	int panoramaWidth;
	int panoramaHeight;

	class ACubemapMgr* cubemapMgr;
	CubemapImgStruct* imageData;
	SphericalProjectorDatabase* sphericalDB;


public:
	CubemapProjector(ACubemapMgr* _cubemapMgr, std::string _dataFilePath, int _panoramaWidth, int _panoramaHeight);

	// initialization
	bool loadFile();
	void initFrame(int frameNum);
	bool readFrame();
	void doProcessing();
};
