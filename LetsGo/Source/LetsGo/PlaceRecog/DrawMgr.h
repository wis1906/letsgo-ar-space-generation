// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "LetsGo.h"
#include "GameFramework/Actor.h"
#include "DrawMgr.generated.h"

UCLASS()
class LETSGO_API ADrawMgr : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADrawMgr();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

public:
	//Debug Drawing
	void drawWholeBox(FVector* rects, FColor pointC, FColor lineC, bool drawGrid, float pointSize = 3, float lineSize = 10);
	UFUNCTION(BlueprintImplementableEvent, Category = BoudingBox) void drawPoint(FVector pos, FColor c, float size = 3);
	UFUNCTION(BlueprintImplementableEvent, Category = BoudingBox) void drawLine(FVector startPos, FVector destPos, FColor c, float size = 10);
	
};
