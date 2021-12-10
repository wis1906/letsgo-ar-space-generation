#pragma once

#include "LetsGo.h"
#include "GameFramework/Actor.h"
#include "SlamDatabase.h"
#include "opencv2/opencv.hpp"
#include "SlamMgr.generated.h"

UCLASS()
class LETSGO_API ASlamMgr : public AActor
{
	GENERATED_BODY()

public:
	ASlamMgr();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

public:
	//SLAM Property
	class SlamDatabase* slamDB;

	UPROPERTY(BlueprintReadWrite, Category = SLAM) FVector curFrameCameraPos;
	UPROPERTY(BlueprintReadWrite, Category = SLAM) FRotator curFrameCameraRot;

public:
	//SLAM Function
	UFUNCTION(BlueprintCallable, Category = SLAM) void initCurFrameData(int frameNum);
	UFUNCTION(BlueprintCallable, Category = SLAM) void drawMap(bool isDrawLandmark, bool isDrawCamTr);
	UFUNCTION(BlueprintCallable, Category = SLAM) void drawCurCamera();
	UFUNCTION(BlueprintCallable, Category = SLAM) bool isKeyframe(int frameNum);
	UFUNCTION(BlueprintCallable, Category = SLAM) int getLastKFNum();
	UFUNCTION(BlueprintImplementableEvent, Category = SLAM) void drawLandmark(FVector pos, bool isOutlier);
	UFUNCTION(BlueprintImplementableEvent, Category = SLAM) void drawFrameCamera(FVector pos);
	UFUNCTION(BlueprintImplementableEvent, Category = SLAM) void drawKeyframeCamera(FVector pos);
	UFUNCTION(BlueprintImplementableEvent, Category = SLAM) void initBPPrevCamPos();

	//Mainst-Blueprint Aceesible Function
	UFUNCTION(BlueprintCallable, Category = SLAM) bool getMainST_isReadXYZForExp();

	UFUNCTION(BlueprintCallable, Category = EXP) void setSegmentWeightValAndSaveName(float w_o, float w_p);
	UFUNCTION(BlueprintCallable, Category = EXP) void setSegmentDistValAndSaveName(float d_o);
};
