#include "SlamMgr.h"

using namespace std;
using namespace cv;

ASlamMgr::ASlamMgr()
{
	PrimaryActorTick.bCanEverTick = true;

	GET_MAINST->slamMgr = this;
	FString path = GET_MAINST->slamDataFilePath;
	GET_MAINST->slamDB = new SlamDatabase(TCHAR_TO_UTF8(*path), mainST::getInst()->slamMapScale, mainST::getInst()->isReadXYZForExp);
}

void ASlamMgr::BeginPlay()
{
	Super::BeginPlay();
	slamDB = mainST::getInst()->slamDB;
}

void ASlamMgr::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ASlamMgr::initCurFrameData(int frameNum)
{
	slamDB->initCurFrameData(frameNum);
}

void ASlamMgr::drawMap(bool isDrawLandmark, bool isDrawCamTr)
{
	if (isDrawLandmark)
	{
		//draw landmark
		for (int i = 0; i < slamDB->curFrameLandmarks.size(); i++)
		{
			if (slamDB->curFrameLandmarks[i]->isOutlier)
				drawLandmark(slamDB->curFrameLandmarks[i]->posConv, true);
			else
				drawLandmark(slamDB->curFrameLandmarks[i]->posConv, false);
		}
	}

	//draw frame camera
	initBPPrevCamPos();
	for (int i = 0; i < slamDB->curFrameCams.size(); i++)
	{
		//curFrameCameraPos = slamDB->curFrameCams[i]->camTransConv;
		//curFrameCameraRot = slamDB->curFrameCams[i]->camRotEulerConv;

		if (isDrawCamTr)
		{
			drawFrameCamera(slamDB->curFrameCams[i]->camTransConv);
		}
	}
	//draw keyframe camera
	for (int i = 0; i < slamDB->curFrameKFs.size(); i++)
	{
		if (isDrawCamTr)
		{
			drawKeyframeCamera(slamDB->curFrameKFs[i]->camTransConv);
		}
	}
}

void ASlamMgr::drawCurCamera()
{
	int frameNum = slamDB->curFrameNum;

	if (frameNum >= slamDB->frameCamDataList.size())
		return;

	//draw frame camera
	curFrameCameraPos = slamDB->frameCamDataList[frameNum]->camTransConv;
	curFrameCameraRot = slamDB->frameCamDataList[frameNum]->camRotEulerConv;

	drawFrameCamera(slamDB->frameCamDataList[frameNum]->camTransConv);
}

bool ASlamMgr::isKeyframe(int frameNum)
{
	for (int i = 0; i < slamDB->keyframeCamDataList.size(); i++)
	{
		if (slamDB->keyframeCamDataList[i]->id_frame == frameNum)
			return true;
	}
	return false;
}

int ASlamMgr::getLastKFNum()
{
	int maxFrameNum = 0;
	for (int i = 0; i < slamDB->keyframeCamDataList.size(); i++)
	{
		if (slamDB->keyframeCamDataList[i]->id_frame > maxFrameNum)
			maxFrameNum = slamDB->keyframeCamDataList[i]->id_frame;
	}

	return maxFrameNum;
}

bool ASlamMgr::getMainST_isReadXYZForExp()
{
	return mainST::getInst()->isReadXYZForExp;
}

void ASlamMgr::setSegmentWeightValAndSaveName(float w_o, float w_p)
{
	FString dataPath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/exp_xyz/outputs/";

	int num_o = (int)(w_o * 10);
	int num_p = (int)(w_p * 10);

	FString boxPath = dataPath;
	boxPath.Append(FString::FromInt(num_o));
	boxPath.Append("_");
	boxPath.Append(FString::FromInt(num_p));
	boxPath.Append("_proposed_bounding_box.txt");

	FString clusterPath = dataPath;
	clusterPath.Append(FString::FromInt(num_o));
	clusterPath.Append("_");
	clusterPath.Append(FString::FromInt(num_p));
	clusterPath.Append("_proposed_clustering_box.txt");

	GET_MAINST->clusteringToleranceWeight = w_o;
	GET_MAINST->clusteringDistFilterWeight = w_p;
	GET_MAINST->saveBoxDataPath = boxPath;
	GET_MAINST->saveClusteringDataPath = clusterPath;



}

void ASlamMgr::setSegmentDistValAndSaveName(float d_o)
{
	FString dataPath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/exp_xyz/outputs/";

	int num_d = (int)(d_o * 100);

	FString boxPath = dataPath;
	boxPath.Append(FString::FromInt(num_d));
	boxPath.Append("_proposed_bounding_box.txt");

	FString clusterPath = dataPath;
	clusterPath.Append(FString::FromInt(num_d));
	clusterPath.Append("_proposed_clustering_box.txt");

	GET_MAINST->clusteringToleranceMult = d_o;
	GET_MAINST->saveBoxDataPath = boxPath;
	GET_MAINST->saveClusteringDataPath = clusterPath;
}
