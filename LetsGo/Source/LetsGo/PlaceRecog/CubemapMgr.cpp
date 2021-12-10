#include "CubemapMgr.h"

using namespace std;
using namespace cv;

ACubemapMgr::ACubemapMgr()
{
	PrimaryActorTick.bCanEverTick = true;

	FString path = mainST::getInst()->cubemapDataFilePath;
	cubeProjector = new CubemapProjector(this, TCHAR_TO_UTF8(*path), mainST::getInst()->panoramaWidth, mainST::getInst()->panoramaHeight);
}

void ACubemapMgr::BeginPlay()
{
	Super::BeginPlay();

	cv::Size sz = cubeProjector->imageData->sz;
	TextureArr.Init(UTexture2D::CreateTransient(sz.width, sz.height), 6);
}

void ACubemapMgr::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ACubemapMgr::initFrame(int frameNum)
{
	if (frameNum > 0)
		cubeProjector->initFrame(frameNum);
}

bool ACubemapMgr::getFrame()
{
	bool isPlaying = cubeProjector->readFrame();
	if (!isPlaying)
		return false;

	//Update Texture
	updateTextureAll();

	//Update Segmented box textures
	updateAllSegmentedBoxTextures();
	return true;
}
void ACubemapMgr::updateTextureTransforms()
{
	for (int planeIdx = 0; planeIdx < 6; planeIdx++)
	{
		//평면 4점 위치 획득
		FVector rects_w[4];
		for (int i = 0; i < 4; i++)
		{
			FVector P;
			if (!needSkyBox())
				P = GET_MAINST->bbMgr->bestBoxRects[GET_MAINST->bbMgr->planePointIndices[planeIdx][i]];
			else
				P = cubeProjector->imageData->recentProjBox[GET_MAINST->bbMgr->planePointIndices[planeIdx][i]];

			rects_w[i] = P;

		}

		//평면 속성 획득
		FVector center = (rects_w[0] + rects_w[1] + rects_w[2] + rects_w[3]) / 4;
		float width = FVector::Dist(rects_w[3], rects_w[2]);
		float height = FVector::Dist(rects_w[3], rects_w[0]);
		FVector vecRight = -(rects_w[0] - rects_w[1]);
		FVector vecUp = -(rects_w[3] - rects_w[0]);
		FVector vecForward = FVector::CrossProduct(vecRight, vecUp);
		vecRight.Normalize(); vecUp.Normalize(); vecForward.Normalize();
		FMatrix rotMatrix(vecRight, vecUp, vecForward, FVector::ZeroVector);

		//월드 플레인 Transform 획득
		FTransform tr;
		tr.SetLocation(center);
		tr.SetScale3D(FVector(width / 100, height / 100, 1));
		tr.SetRotation(rotMatrix.Rotator().Quaternion());

		//월드 플레인 트랜스폼 업데이트
		updateWorldPlaneTransform(planeIdx, tr);
	}

}
void ACubemapMgr::updateTextureAll()
{
	FVector camPose = GET_MAINST->slamMgr->curFrameCameraPos;

	for (int planeIdx = 0; planeIdx < 6; planeIdx++)
	{
		//텍스쳐 업데이트
		cv::Size sz = cubeProjector->imageData->projectedImgs[planeIdx].size();
		TextureArr[planeIdx] = UTexture2D::CreateTransient(sz.width, sz.height);
		TextureArr[planeIdx]->Filter = TextureFilter::TF_Nearest;
		TextureArr[planeIdx]->UpdateResource();
		updateTextureAtOneTime(sz, &cubeProjector->imageData->projectedImgs[planeIdx], TextureArr[planeIdx]);

		//월드 플레인 업데이트
		updateWorldPlaneTexture(planeIdx);
	}

}
void ACubemapMgr::updateTextureAtOneTime(Size sz, Mat* image, UTexture2D* texture)
{
	CData.Init(FColor(0, 0, 0, 255), sz.width * sz.height);
	if (image->data)
	{
		// Copy Mat data to Data array
		for (int y = 0; y < sz.height; y++)
		{
			for (int x = 0; x < sz.width; x++)
			{
				int i = x + (y * sz.width);
				CData[i].B = image->data[i * 3 + 0];
				CData[i].G = image->data[i * 3 + 1];
				CData[i].R = image->data[i * 3 + 2];
			}
		}

		FTexture2DMipMap& Mip = texture->PlatformData->Mips[0];
		void* pData = Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(pData, CData.GetData(), CData.Num() * 4);
		Mip.BulkData.Unlock();
		texture->UpdateResource();
	}
}

UTexture2D* ACubemapMgr::getTextureForRects(TArray<FVector>& rects)
{
	if (rects.Num() != 4)
	{
		PRINTF("ACubemapMgr::getTextureForRects --> rects size is not 4.");
		return NULL;
	}

	cv::Mat img;
	FVector rects_w[4];
	for (int i = 0; i < 4; i++)
	{
		rects_w[i] = rects[i];
		//PRINTF("%f %f %f", rects_w[i].X, rects_w[i].Y, rects_w[i].Z);
	}

	//이미지 생성
	cubeProjector->sphericalDB->createPorjectedImg(&img, &cubeProjector->imageData->panoramaImg, rects_w);

	//텍스쳐 업데이트
	cv::Size sz = img.size();

	if (sz.width < 1 || sz.height < 1)
		return NULL;

	UTexture2D* texture = UTexture2D::CreateTransient(sz.width, sz.height);
	texture->Filter = TextureFilter::TF_Nearest;
	texture->UpdateResource();
	updateTextureAtOneTime(sz, &img, texture);

	return texture;
}


bool ACubemapMgr::needSkyBox()
{
	if (GET_MAINST->bbMgr->bestBoxPlaneStats.size() != 0 && !GET_MAINST->isCubemapSkyboxMode)
		return false;

	return true;
}