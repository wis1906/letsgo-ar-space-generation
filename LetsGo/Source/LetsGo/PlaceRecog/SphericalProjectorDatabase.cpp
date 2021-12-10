#include "SphericalProjectorDatabase.h"


SphericalProjectorDatabase::SphericalProjectorDatabase(cv::Size _panoramaSize)
{
	panoramaSize = _panoramaSize;

}

void SphericalProjectorDatabase::createPorjectedImg(cv::Mat* img, cv::Mat* panoImg, FVector* rects_w)
{
	//카메라 로컬 좌표계로 좌표계변환
	FVector rects_l[4];
	for (int i = 0; i < 4; i++)
		rects_l[i] = convertCoord_WtoC(rects_w[i]);


	//이미지 평면 크기 정의
	FVector rects_l_center = (rects_l[0] + rects_l[1] + rects_l[2] + rects_l[3]) / 4;
	double szQuarter = panoramaSize.width / 4;
	double stdDist = szQuarter / 2;
	double dist = sqrt(rects_l_center.X * rects_l_center.X + rects_l_center.Y * rects_l_center.Y + rects_l_center.Z * rects_l_center.Z);
	std::pair<double, double> wh;
	wh.first = FVector::Dist(rects_l[3], rects_l[2]) * (stdDist / dist);
	wh.second = FVector::Dist(rects_l[3], rects_l[0])* (stdDist / dist);
	if (wh.first > panoramaSize.width / 2)
	{
		double prevW = wh.first;
		wh.first = panoramaSize.width / 2;
		wh.second = wh.second * (wh.first / prevW);
	}
	if (wh.second > panoramaSize.width / 2)
	{
		double prevH = wh.second;
		wh.second = panoramaSize.width / 2;
		wh.second = wh.second * (wh.second / prevH);
	}
	cv::Size sz((int)wh.first, (int)wh.second);

	//이미지 픽셀 투영
	FVector originPoint = rects_l[3];
	FVector dirX = rects_l[2] - rects_l[3];
	FVector dirY = rects_l[0] - rects_l[3];
	*img = cv::Mat(sz, CV_8UC3, cv::Scalar(0, 0, 0));
	for (int i = 0; i < sz.height; i++)
	{
		for (int j = 0; j < sz.width; j++)
		{
			FVector dir = originPoint + (dirX * j / sz.width) + (dirY * i / sz.height);

			std::pair<float, float> LonLat = dirVecToLonLat(dir);
			cv::Point2f xy = LonLatToPanoAxis(LonLat);

			int idx = j + (i * sz.width);
			int xy_idx = (int)xy.x + ((int)xy.y * panoramaSize.width);

			//img->at<cv::Vec3b>(cv::Point(j, i)) = panoImg->at<cv::Vec3b>(cv::Point(xy.x, xy.y));
			img->data[idx * 3 + 0] = panoImg->data[xy_idx * 3 + 0];
			img->data[idx * 3 + 1] = panoImg->data[xy_idx * 3 + 1];
			img->data[idx * 3 + 2] = panoImg->data[xy_idx * 3 + 2];
		}
	}

	//cv::imshow("test", *img);
}



FVector SphericalProjectorDatabase::convertCoord_WtoC(FVector posW)
{
	FTransform camTransform;
	camTransform.SetLocation(GET_MAINST->slamMgr->curFrameCameraPos);
	camTransform.SetRotation(GET_MAINST->slamMgr->curFrameCameraRot.Quaternion());

	FTransform pointTransform;
	pointTransform.SetLocation(posW);
	FTransform pointTransformRel = pointTransform.GetRelativeTransform(camTransform);

	return pointTransformRel.GetLocation();
}
FVector SphericalProjectorDatabase::convertCoord_CtoW(FVector posC)
{
	FTransform camTransform;
	camTransform.SetLocation(GET_MAINST->slamMgr->curFrameCameraPos);
	camTransform.SetRotation(GET_MAINST->slamMgr->curFrameCameraRot.Quaternion());

	FTransform pointTransform;
	pointTransform.SetLocation(posC);
	FTransform pointTransformWorld = camTransform * pointTransform;

	return pointTransformWorld.GetLocation();
}


std::pair<float, float> SphericalProjectorDatabase::dirVecToLonLat(FVector dirVec)
{
	/*
	float dv = sqrt(dirVec.X * dirVec.X + dirVec.Y * dirVec.Y + dirVec.Z * dirVec.Z);
	dirVec.X = dirVec.X / dv;
	dirVec.Y = dirVec.Y / dv;
	dirVec.Z = dirVec.Z / dv;
	*/
	dirVec.Normalize();

	std::pair<float, float> LonLat(FMath::Atan2(dirVec.Y, dirVec.X), FMath::Acos(dirVec.Z));
	//LonLat.first = atan2(dirVec.Y, dirVec.X);
	//LonLat.second = acos(dirVec.Z);
	return LonLat;
}
cv::Point2f SphericalProjectorDatabase::LonLatToPanoAxis(std::pair<float, float> LonLat)
{
	float _x = 0.5 + 0.5*(LonLat.first / PI);
	float _y = (LonLat.second / PI);

	cv::Point2f xy;
	xy.x = _x * (panoramaSize.width - 1);
	xy.y = _y * (panoramaSize.height - 1);
	/*
	if (xy.x < 0)
		xy.x = 0;
	if (xy.x >= panoramaSize.width)
	xy.x = panoramaSize.width - 1;
	if (xy.y < 0)
		xy.y = 0;
	if (xy.y >= panoramaSize.height)
		xy.y = panoramaSize.height - 1;
	*/
	return xy;
}
