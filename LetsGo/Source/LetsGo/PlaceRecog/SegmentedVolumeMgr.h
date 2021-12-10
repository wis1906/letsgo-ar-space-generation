#pragma once
#include "LetsGo.h"
//core
#include "SlamDatabase.h"
#include "SegmentedVolume.h"
#include "SegmentedVolumeThread.h"
#include "BoundingBoxMgr.h"
//thread
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
//ue4
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "SegmentedVolumeMgr.generated.h"

UCLASS()
class LETSGO_API ASegmentedVolumeMgr : public AActor
{
	GENERATED_BODY()

public:
	ASegmentedVolumeMgr();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void EndPlay(EEndPlayReason::Type EndPlayReason);

public:
	class SlamDatabase* slamDB;
	class SegmentedVolume* SegVolume;

	class FSegmentedVolumeThread* segVolThread = nullptr;
	FRunnableThread* currentRunningThread = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) bool bClusterDone = false;

public:
	/// Clustering
	UFUNCTION(BlueprintCallable, Category = SegmentVolume) void clusterPointCloud();
	void makeUE4DMPlanes();
	void updateBoxPlaneTypes();

	///drawing
	UFUNCTION(BlueprintCallable, Category = SegmentVolume) void drawClusteringBoxes(bool isTransparent);
	UFUNCTION(BlueprintCallable, Category = SegmentVolume) void redrawDebugWorld();

	///BP
	UFUNCTION(BlueprintImplementableEvent, Category = SegmentVolume) void pushAllItem();
	UFUNCTION(BlueprintImplementableEvent, Category = SegmentVolume) void popAndInitItem(FTransform tr, const TArray<FVector>& rects, bool isVisible);
	UFUNCTION(BlueprintImplementableEvent, Category = SegmentVolume) void updateItemVisibility(int idx, bool isVisible);

	///Data IO for Exp
	void saveSegmentedBoxData();
};
