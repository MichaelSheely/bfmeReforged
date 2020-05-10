// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "bfmeReforgedGameMode.h"
#include "bfmeReforgedPlayerController.h"
#include "bfmeReforgedCharacter.h"
#include "UObject/ConstructorHelpers.h"

AbfmeReforgedGameMode::AbfmeReforgedGameMode()
{
	// use our custom PlayerController class
	PlayerControllerClass = AbfmeReforgedPlayerController::StaticClass();

	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/TopDownCPP/Blueprints/TopDownCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}