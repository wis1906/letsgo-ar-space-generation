#include "CubemapProjector.h"

using namespace std;
using namespace cv;

CubemapProjector::CubemapProjector(ACubemapMgr* _cubemapMgr, std::string _dataFilePath, int _panoramaWidth, int _panoramaHeight) :
	cubemapMgr(_cubemapMgr), dataFilePath(_dataFilePath), panoramaWidth(_panoramaWidth), panoramaHeight(_panoramaHeight)
{
	sphericalDB = new SphericalProjectorDatabase(cv::Size(_panoramaWidth, _panoramaHeight));
	imageData = new CubemapImgStruct();
	imageData->sz = Size((int)panoramaWidth / 4, (int)panoramaHeight / 2);


	//Load File
	if (!loadFile())
	{
		PRINTF("Input file not found.");
		return;
	}
}

bool CubemapProjector::loadFile()
{
	//Load video
	imageData->VCap = VideoCapture(dataFilePath);
	if (!imageData->VCap.isOpened())
	{
		PRINTF("Can't load video file in CubemapProjector::loadFile()");
		return false;
	}
	else
	{
		//Adjust Width and Height of Video
		imageData->VCap.set(CAP_PROP_FRAME_WIDTH, panoramaWidth);
		imageData->VCap.set(CAP_PROP_FRAME_HEIGHT, panoramaHeight);
	}

	return true;
}

void CubemapProjector::initFrame(int frameNum)
{
	imageData->VCap.set(cv::CAP_PROP_POS_FRAMES, frameNum);
}

bool CubemapProjector::readFrame()
{
	bool isGet = imageData->VCap.read(imageData->sourceImg);
	if (!isGet)
		return false;

	doProcessing();

	return true;
}

// Processing
void CubemapProjector::doProcessing()
{
	//Get panorama image
	resize(imageData->sourceImg, imageData->panoramaImg, Size(panoramaWidth, panoramaHeight));

	if (!cubemapMgr->needSkyBox())
	{
		for (int i = 0; i < 8; i++)
			imageData->recentProjBox[i] = GET_MAINST->bbMgr->bestBoxRects[i];

		PRINTF("%f", imageData->recentProjBox[0].X);

		for (int planeIdx = 0; planeIdx < 6; planeIdx++)
		{
			//박스 평면 4모서리 좌표 가져오기
			FVector rects_w[4];
			for (int i = 0; i < 4; i++)
				rects_w[i] = imageData->recentProjBox[GET_MAINST->bbMgr->bestBoxPlaneStats[planeIdx]->cornerIndices[i]];

			sphericalDB->createPorjectedImg(&imageData->projectedImgs[planeIdx], &imageData->panoramaImg, rects_w);
		}
	}
	else
	{
		//카메라 좌표계에서의 원점 정육면체 생성
		imageData->recentProjBox[0] = sphericalDB->convertCoord_CtoW(FVector(-10000, -10000, -10000));
		imageData->recentProjBox[1] = sphericalDB->convertCoord_CtoW(FVector(10000, -10000, -10000));
		imageData->recentProjBox[2] = sphericalDB->convertCoord_CtoW(FVector(10000, 10000, -10000));
		imageData->recentProjBox[3] = sphericalDB->convertCoord_CtoW(FVector(-10000, 10000, -10000));
		imageData->recentProjBox[4] = sphericalDB->convertCoord_CtoW(FVector(-10000, -10000, 10000));
		imageData->recentProjBox[5] = sphericalDB->convertCoord_CtoW(FVector(10000, -10000, 10000));
		imageData->recentProjBox[6] = sphericalDB->convertCoord_CtoW(FVector(10000, 10000, 10000));
		imageData->recentProjBox[7] = sphericalDB->convertCoord_CtoW(FVector(-10000, 10000, 10000));

		for (int planeIdx = 0; planeIdx < 6; planeIdx++)
		{
			//박스 평면 4모서리 좌표 가져오기
			FVector rects_w[4];
			for (int i = 0; i < 4; i++)
				rects_w[i] = imageData->recentProjBox[GET_MAINST->bbMgr->planePointIndices[planeIdx][i]];

			sphericalDB->createPorjectedImg(&imageData->projectedImgs[planeIdx], &imageData->panoramaImg, rects_w);
		}

		cubemapMgr->updateTextureTransforms();
	}
}