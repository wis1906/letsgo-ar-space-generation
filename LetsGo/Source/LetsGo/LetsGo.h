#pragma once

#include "EngineMinimal.h"
#include <iostream>
#include "mainST.h"

#define PRINTF(txt,...) UE_LOG(LetsGo, Warning, TEXT(txt), __VA_ARGS__)
#define GET_MAINST mainST::getInst()

DECLARE_LOG_CATEGORY_EXTERN(LetsGo, Log, All);