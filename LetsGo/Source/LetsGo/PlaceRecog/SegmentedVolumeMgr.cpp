#include "SegmentedVolumeMgr.h"

using namespace std;
using namespace cv;

// Sets default values
ASegmentedVolumeMgr::ASegmentedVolumeMgr()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	SegVolume = new SegmentedVolume();
}

// Called when the game starts or when spawned
void ASegmentedVolumeMgr::BeginPlay()
{
	Super::BeginPlay();
	slamDB = GET_MAINST->slamDB;

	segVolThread = new FSegmentedVolumeThread(this, slamDB, SegVolume);
	currentRunningThread = FRunnableThread::Create(segVolThread, TEXT("Segmented Volume Thread"));
}

// Called every frame
void ASegmentedVolumeMgr::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (segVolThread->bClusterDone)
	{
		segVolThread->bClusterDone = false;
		bClusterDone = true;

		if (GET_MAINST->isReadXYZForExp)
			saveSegmentedBoxData();

		if (segVolThread->clusteringBoxes.size() > 0)
		{
			if (!GET_MAINST->isReadXYZForExp)
				makeUE4DMPlanes();

			redrawDebugWorld();

		}
	}
	else
	{
		if (!GET_MAINST->isReadXYZForExp)
			updateBoxPlaneTypes();
	}
}

void ASegmentedVolumeMgr::EndPlay(EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	if (currentRunningThread && segVolThread)
	{
		currentRunningThread->Suspend(true);
		segVolThread->bStopThread = true;
		currentRunningThread->Suspend(false);
		currentRunningThread->Kill(false);
		currentRunningThread->WaitForCompletion();
		//delete segVolThread;
	}
}

//#########################################
// Clustering
//#########################################
void ASegmentedVolumeMgr::clusterPointCloud()
{
	// get regions dist

	// thread process start
	segVolThread->bClusterStart = true;

	// draw until process is done
	drawClusteringBoxes(true);
}

void ASegmentedVolumeMgr::makeUE4DMPlanes()
{
	pushAllItem();
	for (int boxIdx = 0; boxIdx < segVolThread->clusteringBoxPlaneStats.size(); boxIdx++)
	{
		for (int i = 0; i < 6; i++)
		{
			BoxPlaneStat* st = segVolThread->clusteringBoxPlaneStats[boxIdx][i];

			//평면 4점 위치 획득
			TArray<FVector> rects_w;
			for (int k = 0; k < 4; k++)
			{
				FVector P = segVolThread->clusteringBoxes[boxIdx][st->cornerIndices[k]];
				rects_w.Add(P);
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

			//월드 플레인 업데이트
			if (st->planeType == SEG_NOT_VISIBLE)
				popAndInitItem(tr, rects_w, false);
			else
				popAndInitItem(tr, rects_w, true);

		}
	}
}

void ASegmentedVolumeMgr::updateBoxPlaneTypes()
{
	segVolThread->setBoxPlaneTypes();

	for (int boxIdx = 0; boxIdx < segVolThread->clusteringBoxPlaneStats.size(); boxIdx++)
	{
		for (int i = 0; i < 6; i++)
		{
			BoxPlaneStat* st = segVolThread->clusteringBoxPlaneStats[boxIdx][i];

			int idx = (6 * boxIdx) + i;

			//월드 플레인 업데이트
			if (st->planeType == SEG_NOT_VISIBLE)
				updateItemVisibility(idx, false);
			else
				updateItemVisibility(idx, true);

		}
	}

}

void ASegmentedVolumeMgr::drawClusteringBoxes(bool isTransparent)
{
	for (int i = 0; i < segVolThread->clusteringBoxes.size(); i++)
	{
		FVector box[8];
		for (int j = 0; j < 8; j++)
			box[j] = segVolThread->clusteringBoxes[i][j];

		if (isTransparent)
		{
			FColor c(0, 255, 0, 50);
			GET_MAINST->drawer->drawWholeBox(box, c, c, true, 3, 3);
		}
		else
		{
			GET_MAINST->drawer->drawWholeBox(box, FColor::Green, FColor::Green, true, 3, 3);
		}
	}
	for (int i = 0; i < segVolThread->clusteringBoxPlaneStats.size(); i++)
	{
		for (int j = 0; j < 6; j++)
		{
			BoxPlaneStat* st = segVolThread->clusteringBoxPlaneStats[i][j];

			if (st->planeType == SEG_VISIBLE)
			{
				//평면 4점 위치 획득
				FVector rects_w[4];
				for (int k = 0; k < 4; k++)
					rects_w[k] = segVolThread->clusteringBoxes[i][st->cornerIndices[k]];

				GET_MAINST->drawer->drawLine(rects_w[0], rects_w[1], FColor::Blue, 3);
				GET_MAINST->drawer->drawLine(rects_w[1], rects_w[2], FColor::Blue, 3);
				GET_MAINST->drawer->drawLine(rects_w[2], rects_w[3], FColor::Blue, 3);
				GET_MAINST->drawer->drawLine(rects_w[3], rects_w[0], FColor::Blue, 3);
			}
		}
	}
}

void ASegmentedVolumeMgr::redrawDebugWorld()
{
	FlushPersistentDebugLines(GetWorld());
	GET_MAINST->slamMgr->drawMap(true, true);
	GET_MAINST->bbMgr->drawBestBox();
	drawClusteringBoxes(false);
}


//
void ASegmentedVolumeMgr::saveSegmentedBoxData()
{
	std::string path = TCHAR_TO_UTF8(*GET_MAINST->saveClusteringDataPath);
	float slamMapScale = GET_MAINST->slamMapScale;

	std::fstream dataFile;

	//open data file
	dataFile.open(path, ios::out);

	for (int boxIdx = 0; boxIdx < segVolThread->clusteringBoxes.size(); boxIdx++)
	{
		for (int i = 0; i < 8; i++)
		{
			FVector rect = segVolThread->clusteringBoxes[boxIdx][i] / slamMapScale;
			dataFile << rect.X << " " << rect.Y << " " << rect.Z << std::endl;
		}

		dataFile << "\n" << std::endl;
	}
	dataFile << "\n" << std::endl;

	dataFile.close();

	PRINTF("Segmented Box Datafile is saved.");
}