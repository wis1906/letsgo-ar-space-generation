#include "mainST.h"

mainST* mainST::instance = nullptr;
///--------------Parameters------------------
//File path
FString mainST::saveBoxDataPath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/exp_xyz/proposed_bounding_box.txt";
FString mainST::saveClusteringDataPath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/exp_xyz/proposed_clustering_box.txt";
FString mainST::slamDataFilePath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/exp_xyz/BoilerRoom_origin.xyz";
//FString mainST::slamDataFilePath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/ecl_lab_5/map.cdb";
FString mainST::cubemapDataFilePath = "E:/Unreal_Project/4.19.2/LetsGo/inputs/ecl_lab_5/video.mp4";

//Mode
bool mainST::isReadXYZForExp = true;				//실험을 위해 맨 마지막 프레임만 분석하는 모드
bool mainST::isCubemapSkyboxMode = false;				//다이나믹 텍스처를 스카이박스 형태로만 만드는 모드

//Param - SLAM
float mainST::slamMapScale = 200.0f;

//Param - Bounding box
double mainST::boundingBoxStdDevMult = 0.7;			//KDTree에서 StdDev의 가중치 / default = 0.7;
int mainST::boundingBoxRotOptNum = 5;				//bounding box의 rotation optimization에서 최적화 실행 횟수 / default = 5;
int mainST::boundingBoxRotIterPerOpt = 10;			//bounding box의 rotation optimization에서 한 번의 최적화 때 반복 교정할 횟수 / default = 10;
double mainST::boundingBoxRotDegreePerOpt = 2.0;		//bounding box의 rotation optimization에서 한 번의 최적화 때 교정할 박스 최대각도 / default = 2.0;
double mainST::boundingBoxTransOptLimitDist = 0.3;		//bounding box의 translation optimization에서 최대로 줄일 사이즈 한계치(1에 가까울수록 많이 줄일 수 있음) / default = 0.3;

//Param - Clustering
int mainST::clusteringClusterSizeMin = 10;			//clustering의 클러스터 최소 사이즈 조건 / default = 10;
float mainST::clusteringToleranceMult = 0.25;			//clustering의 Radius search에서 최초 기준 범위(region_dist / clusteringToleranceMult) / default = 0.25;
float mainST::clusteringDistFilterWeight = 0.3;			//clustering의 Radius search에서 최근 탐색 거리 반영 비율(w_p) / default = 0.3;
float mainST::clusteringToleranceWeight = 0.8;			//clustering의 Radius search에서 최초 기준거리 반영 비율(w_o) / default = 0.6;

//Param - Cube map
int mainST::panoramaWidth = 1920;
int mainST::panoramaHeight = 960;
///--------------------------------------------


///--------------Referencing-------------------
SlamDatabase* mainST::slamDB = nullptr;
ASlamMgr* mainST::slamMgr = nullptr;
ABoundingBoxMgr* mainST::bbMgr = nullptr;
ADrawMgr* mainST::drawer = nullptr;
///--------------------------------------------

mainST::mainST() {}
mainST* mainST::getInst()
{
	if (!instance)
	{
		instance = new mainST;
	}
	return instance;
}
void mainST::destroyInst()
{
	if (!instance)
	{
		return;
	}
	delete instance;
	instance = nullptr;
}
