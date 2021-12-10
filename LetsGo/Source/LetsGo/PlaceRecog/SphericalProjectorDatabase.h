#pragma once
#include "LetsGo.h"
#include "opencv2/opencv.hpp"

class SphericalProjectorDatabase
{

public:
	SphericalProjectorDatabase(cv::Size _panoramaSize);


public:
	cv::Size panoramaSize;


public:
	void createPorjectedImg(cv::Mat* img, cv::Mat* panoImg, FVector* rects_w);

	FVector convertCoord_WtoC(FVector posW);
	FVector convertCoord_CtoW(FVector posC);


	std::pair<float, float> dirVecToLonLat(FVector dirVec);
	cv::Point2f LonLatToPanoAxis(std::pair<float, float> LonLat);
};

