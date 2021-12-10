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
bool mainST::isReadXYZForExp = true;				//������ ���� �� ������ �����Ӹ� �м��ϴ� ���
bool mainST::isCubemapSkyboxMode = false;				//���̳��� �ؽ�ó�� ��ī�̹ڽ� ���·θ� ����� ���

//Param - SLAM
float mainST::slamMapScale = 200.0f;

//Param - Bounding box
double mainST::boundingBoxStdDevMult = 0.7;			//KDTree���� StdDev�� ����ġ / default = 0.7;
int mainST::boundingBoxRotOptNum = 5;				//bounding box�� rotation optimization���� ����ȭ ���� Ƚ�� / default = 5;
int mainST::boundingBoxRotIterPerOpt = 10;			//bounding box�� rotation optimization���� �� ���� ����ȭ �� �ݺ� ������ Ƚ�� / default = 10;
double mainST::boundingBoxRotDegreePerOpt = 2.0;		//bounding box�� rotation optimization���� �� ���� ����ȭ �� ������ �ڽ� �ִ밢�� / default = 2.0;
double mainST::boundingBoxTransOptLimitDist = 0.3;		//bounding box�� translation optimization���� �ִ�� ���� ������ �Ѱ�ġ(1�� �������� ���� ���� �� ����) / default = 0.3;

//Param - Clustering
int mainST::clusteringClusterSizeMin = 10;			//clustering�� Ŭ������ �ּ� ������ ���� / default = 10;
float mainST::clusteringToleranceMult = 0.25;			//clustering�� Radius search���� ���� ���� ����(region_dist / clusteringToleranceMult) / default = 0.25;
float mainST::clusteringDistFilterWeight = 0.3;			//clustering�� Radius search���� �ֱ� Ž�� �Ÿ� �ݿ� ����(w_p) / default = 0.3;
float mainST::clusteringToleranceWeight = 0.8;			//clustering�� Radius search���� ���� ���ذŸ� �ݿ� ����(w_o) / default = 0.6;

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
