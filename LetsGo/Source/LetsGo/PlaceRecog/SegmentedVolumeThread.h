#pragma once
#include "LetsGo.h"
#include "HAL/Runnable.h"
#include "SlamDatabase.h"
#include "SegmentedVolume.h"
#include "opencv2/opencv.hpp"

class FRunnableThread;
class ASegmentedVolumeMgr;

class FSegmentedVolumeThread : public FRunnable
{
	
public:
	FSegmentedVolumeThread(ASegmentedVolumeMgr* _segVolMgr, SlamDatabase* _slamDB, SegmentedVolume* _SegVolume);

public:
	SlamDatabase* slamDB;
	SegmentedVolume* SegVolume;
	ASegmentedVolumeMgr* segVolMgr;

	Eigen::Matrix3f convertCoordPlaneMat;
	Eigen::Vector3f convertCoordplaneCenter;
	Eigen::Vector3f boxProjHalfSize;

	std::vector<std::vector<FVector>> clusteringBoxes;
	std::vector< std::vector<BoxPlaneStat*>> clusteringBoxPlaneStats;

	bool bClusterStart;
	bool bClusterDone;
	bool bStopThread;

public:
	///Thread Function
	virtual bool Init();
	virtual uint32 Run();
	virtual void Stop();

public:
	/// Clustering
	UFUNCTION(BlueprintCallable, Category = SegmentVolume) void clusterPointCloud();
	void setBoxPlaneTypes();
	bool getBoundingBoxOrientedCoord();
	Eigen::Vector3f convertCoordPoint(Eigen::Vector3f point);
	Eigen::Vector3f reconvertCoordPoint(Eigen::Vector3f point);

	/// ±‚≈∏
	double getdistOfTwoPoint(FVector a, FVector b);
	Eigen::Vector3f FVec2EigenVec(FVector vec);
	FVector EigenVec2FVec(Eigen::Vector3f  vec);
};
