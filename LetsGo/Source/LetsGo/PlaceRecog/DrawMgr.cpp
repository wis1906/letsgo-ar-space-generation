// Fill out your copyright notice in the Description page of Project Settings.

#include "DrawMgr.h"


// Sets default values
ADrawMgr::ADrawMgr()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	GET_MAINST->drawer = this;
}

// Called when the game starts or when spawned
void ADrawMgr::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void ADrawMgr::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

//Draw Whole Box from points
// rects order is,
// rects[0] = (min, min, min)
// rects[1] = (max, min, min)
// rects[2] = (max, max, min)
// rects[3] = (min, max, min)
// rects[4] = (min, min, max)
// rects[5] = (max, min, max)
// rects[6] = (max, max, max)
// rects[7] = (min, max, max)
void ADrawMgr::drawWholeBox(FVector* rects, FColor pointC, FColor lineC, bool drawGrid, float pointSize, float lineSize)
{
	//Draw Box Point
	for (int i = 0; i < 8; i++)
		drawPoint(rects[i], pointC, pointSize);

	//Draw Box Line
	FColor c = lineC;

	//bottom lines
	drawLine(rects[0], rects[1], c, lineSize);
	drawLine(rects[1], rects[2], c, lineSize);
	drawLine(rects[2], rects[3], c, lineSize);
	drawLine(rects[3], rects[0], c, lineSize);

	//top lines
	drawLine(rects[4], rects[5], c, lineSize);
	drawLine(rects[5], rects[6], c, lineSize);
	drawLine(rects[6], rects[7], c, lineSize);
	drawLine(rects[7], rects[4], c, lineSize);

	//middle lines
	drawLine(rects[0], rects[4], c, lineSize);
	drawLine(rects[1], rects[5], c, lineSize);
	drawLine(rects[2], rects[6], c, lineSize);
	drawLine(rects[3], rects[7], c, lineSize);

	//ground grid
	if (drawGrid)
	{
		int slice = 8;
		//bottom
		for (int i = 0; i < slice; i++)
		{
			drawLine((rects[0] * i + rects[3] * (slice - i)) / slice, (rects[1] * i + rects[2] * (slice - i)) / slice, c, lineSize / 2);
			drawLine((rects[0] * i + rects[1] * (slice - i)) / slice, (rects[3] * i + rects[2] * (slice - i)) / slice, c, lineSize / 2);
		}
		//top
		for (int i = 0; i < slice; i++)
		{
			drawLine((rects[4] * i + rects[7] * (slice - i)) / slice, (rects[5] * i + rects[6] * (slice - i)) / slice, c, lineSize / 2);
			drawLine((rects[4] * i + rects[5] * (slice - i)) / slice, (rects[7] * i + rects[6] * (slice - i)) / slice, c, lineSize / 2);
		}
		//side1
		for (int i = 0; i < slice; i++)
		{
			drawLine((rects[0] * i + rects[4] * (slice - i)) / slice, (rects[3] * i + rects[7] * (slice - i)) / slice, c, lineSize / 2);
			drawLine((rects[0] * i + rects[3] * (slice - i)) / slice, (rects[4] * i + rects[7] * (slice - i)) / slice, c, lineSize / 2);
		}
		//side2
		for (int i = 0; i < slice; i++)
		{
			drawLine((rects[3] * i + rects[7] * (slice - i)) / slice, (rects[2] * i + rects[6] * (slice - i)) / slice, c, lineSize / 2);
			drawLine((rects[3] * i + rects[2] * (slice - i)) / slice, (rects[7] * i + rects[6] * (slice - i)) / slice, c, lineSize / 2);
		}
		//side3
		for (int i = 0; i < slice; i++)
		{
			drawLine((rects[1] * i + rects[5] * (slice - i)) / slice, (rects[2] * i + rects[6] * (slice - i)) / slice, c, lineSize / 2);
			drawLine((rects[1] * i + rects[2] * (slice - i)) / slice, (rects[5] * i + rects[6] * (slice - i)) / slice, c, lineSize / 2);
		}
		//side4
		for (int i = 0; i < slice; i++)
		{
			drawLine((rects[0] * i + rects[4] * (slice - i)) / slice, (rects[1] * i + rects[5] * (slice - i)) / slice, c, lineSize / 2);
			drawLine((rects[0] * i + rects[1] * (slice - i)) / slice, (rects[4] * i + rects[5] * (slice - i)) / slice, c, lineSize / 2);
		}
	}
}

