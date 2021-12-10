#pragma once
#include "LetsGo.h"
#include "GameFramework/Actor.h"
#include "Runtime/Engine/Classes/Engine/Texture2D.h"
#include "CubemapProjector.h"
#include "opencv2/opencv.hpp"
#include "CubemapMgr.generated.h"

UCLASS()
class LETSGO_API ACubemapMgr : public AActor
{
	GENERATED_BODY()

public:
	ACubemapMgr();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

public:
	//Cubemap Property
	class CubemapProjector* cubeProjector;
	// The current texture array
	TArray<FColor> CData;
	// Image textures
	UPROPERTY(BlueprintReadOnly, Category = Cubemap) TArray<UTexture2D*> TextureArr;

public:
	///Init & Read Frame
	UFUNCTION(BlueprintCallable, Category = Cubemap)
		void initFrame(int frameNum);
	UFUNCTION(BlueprintCallable, Category = Cubemap)
		bool getFrame();

	///For Srrounding planes Texture Update
	void updateTextureAll();
	void updateTextureAtOneTime(cv::Size size, cv::Mat* image, UTexture2D* texture);

	///For Surrounding planes
	UFUNCTION(BlueprintCallable, Category = Cubemap)
		void updateTextureTransforms();
	UFUNCTION(BlueprintImplementableEvent, Category = Cubemap)
		void updateWorldPlaneTransform(int idx, FTransform tr);
	UFUNCTION(BlueprintImplementableEvent, Category = Cubemap)
		void updateWorldPlaneTexture(int idx);

	///For segmented boxes texture
	UFUNCTION(BlueprintImplementableEvent, Category = Cubemap)
		void updateAllSegmentedBoxTextures();
	UFUNCTION(BlueprintCallable, Category = Cubemap)
		UTexture2D* getTextureForRects(UPARAM(ref) TArray<FVector>& rects);

	///Else
		bool needSkyBox();
};
