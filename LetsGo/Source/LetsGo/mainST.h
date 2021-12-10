#pragma once
#include "LetsGo.h"
#include "PlaceRecog/SlamDatabase.h"
#include "PlaceRecog/SlamMgr.h"
#include "PlaceRecog/BoundingBoxMgr.h"
#include "PlaceRecog/DrawMgr.h"

class mainST
{
public:
	mainST();

private:
	static mainST* instance;
public:
	static mainST* getInst();
	static void destroyInst();

public:
	//File path
	static FString saveBoxDataPath;
	static FString saveClusteringDataPath;
	static FString slamDataFilePath;
	static FString cubemapDataFilePath;
	//Mode
	static bool isReadXYZForExp;
	static bool isCubemapSkyboxMode;
	//Param - SLAM
	static float slamMapScale;
	//Param - Bounding box
	static double boundingBoxStdDevMult;
	static int boundingBoxRotOptNum;
	static int boundingBoxRotIterPerOpt;
	static double boundingBoxRotDegreePerOpt;
	static double boundingBoxTransOptLimitDist;
	//Param - Clustering
	static int clusteringClusterSizeMin;
	static float clusteringToleranceMult;
	static float clusteringDistFilterWeight;
	static float clusteringToleranceWeight;
	//Param - Cube map
	static int panoramaWidth;
	static int panoramaHeight;

	//Referencing
	static SlamDatabase* slamDB;
	static class ASlamMgr* slamMgr;
	static class ABoundingBoxMgr* bbMgr;
	static class ADrawMgr* drawer;
};
