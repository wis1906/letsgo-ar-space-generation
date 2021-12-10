#pragma once

#include "LetsGo.h"
#include "GameFramework/Actor.h"

#include "SlamDatabase.h"
#include "opencv2/opencv.hpp"

#include "ApproxMVBB/ComputeApproxMVBB.hpp"
#include "ApproxMVBB/Config/Config.hpp"
#include "ApproxMVBB/Common/CPUTimer.hpp"
#include "ApproxMVBB/KdTree.hpp"

#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"

//#pragma warning(disable:4541)
#include "pcl/features/moment_of_inertia_estimation.h"

#include "BoundingBoxMgr.generated.h"

ApproxMVBB_DEFINE_MATRIX_TYPES;
ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

enum planeTypeEnum
{
	PLANE_CEILING,
	PLANE_FLOOR,
	PLANE_WALL,
	SEG_VISIBLE,
	SEG_NOT_VISIBLE
};

typedef struct BoxPlaneStat
{
	int boxIdx;
	int cornerIndices[4];
	double score;
	double depth;
	planeTypeEnum planeType;

}BoxPlaneStat;

UCLASS()
class LETSGO_API ABoundingBoxMgr : public AActor
{
	GENERATED_BODY()

public:
	ABoundingBoxMgr();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

public:

	// 평면 모서리 좌표 정리
	int planePointIndices[6][4] = { { 0, 1, 5, 4 },{ 1, 2, 6, 5 },{ 2, 3, 7, 6 },{ 3, 0, 4, 7 },{ 3, 2, 1, 0 },{ 4, 5, 6, 7 } };
	int planeDepthIndices[6][2] = { {1, 2}, { 0,1 }, { 1,2 }, { 0,1 }, { 0,4 }, { 4,0 } };

	class SlamDatabase* slamDB;

	// rects order is,
	// rects[0] = (min, min, min)
	// rects[1] = (max, min, min)
	// rects[2] = (max, max, min)
	// rects[3] = (min, max, min)
	// rects[4] = (min, min, max)
	// rects[5] = (max, min, max)
	// rects[6] = (max, max, max)
	// rects[7] = (min, max, max)

	FVector curBoxCenter;
	FVector curBoxRects[8];
	double curBoxScoreArr[6] = { 0, };
	double curBoxScoreSum = 0;

	FVector bestBoxCenter;
	FVector bestBoxRects[8];
	double bestBoxScoreArr[6] = { 0, };
	double bestBoxScoreSum = 0;

	int groundPlaneIdx = 0;
	std::vector<BoxPlaneStat*> bestBoxPlaneStats;


public:
	/// 이상점 제거
	//Outlier Filtering
	UFUNCTION(BlueprintCallable, Category = BoudingBox)void doKdTree();

	/// OBB 검출
	//Compute ORBB
	UFUNCTION(BlueprintCallable, Category = BoudingBox) void drawBoxFromPoints();

	//Object Builder
	UFUNCTION(BlueprintCallable, Category = BoudingBox) void buildObjectFromPoints();
	double buildObjectFromPointsForJustRad(bool isRoll);
	double trimRadian(double rad);

	//PCL Box Builder
	UFUNCTION(BlueprintCallable, Category = BoudingBox) void buildPCLBoxFromPoints();

	/// OBB 교정
	//Box Optimization
	UFUNCTION(BlueprintCallable, Category = BoudingBox) bool optimizeBox(bool justDrawForExp);
	void optimizeBoxRot(std::vector<FVector>& pointList, bool isDrawBox);
	double optimizePlaneRot(std::vector<FVector>& pointList, double boxDepth, Eigen::Vector3f rect_1, Eigen::Vector3f rect_2,
		Eigen::Vector3f rect_3, Eigen::Vector3f rect_4, int moveRight, int moveUp);
	double optimizePlaneTrans(std::vector<FVector>& pointList, double boxDepth, int rectIdx_1, int rectIdx_2, int rectIdx_3, int rectIdx_4, int moveRight, int moveUp);

	/// 구조체 생성
	//Struct Generator
	void makeBoxPlaneStats();
	void setBoxPlaneTypes();


	//Debug Drawing
	UFUNCTION(BlueprintCallable, Category = BoudingBox) void drawBestBox();

	//Make Real Plane in UE4 World
	void makeBestBoxPlaneInUE4();
	UFUNCTION(BlueprintImplementableEvent, Category = BoudingBox) void destroyAllPlaneInUE4();
	UFUNCTION(BlueprintImplementableEvent, Category = BoudingBox) void makePlaneInUE4(FTransform tr, int id);


	/// 기타
	//Else
	UFUNCTION(BlueprintCallable, Category = BoudingBox) TArray<FVector> getBestBoxRects();
	UFUNCTION(BlueprintCallable, Category = BoudingBox) void saveBestBoxData();
	double getdistOfTwoPoint(FVector a, FVector b);
	void shuffleFVecList(std::vector<FVector>& vec);
	Eigen::Vector3f FVec2EigenVec(FVector vec);
	FVector EigenVec2FVec(Eigen::Vector3f  vec);
};

typedef struct MyPoint
{
	//Special point type with id
	ApproxMVBB_DEFINE_MATRIX_TYPES;
	Vector3* m_p;
	int m_id;
}MyPoint;

typedef struct MyPointGetter
{
	// Special point getter
	ApproxMVBB_DEFINE_MATRIX_TYPES;
	static const Vector3& get(const MyPoint& p)
	{
		return *(p.m_p);
	}
	static Vector3& get(MyPoint& p)
	{
		return *(p.m_p);
	}
}MyPointGetter;

USTRUCT(Atomic, BlueprintType)
struct FPlanePoint
{
	GENERATED_BODY()
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FVector EntryPoint;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		TArray<FVector> RoutePoint;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FVector ExitPoint;
};