// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "bfmeReforgedPlayerController.generated.h"

UCLASS()
class AbfmeReforgedPlayerController : public APlayerController
{
  GENERATED_BODY()

public:
  AbfmeReforgedPlayerController();

protected:
  /** True if the controlled character should navigate to the mouse cursor. */
  uint32 bMoveToMouseCursor : 1;

  // Begin PlayerController interface
  virtual void PlayerTick(float DeltaTime) override;
  virtual void SetupInputComponent() override;
  // End PlayerController interface

  /** Calculates the path the player should take to the current
   *  mouse cursor location. */
  void ComputePathToCursor();

  /** Navigate player to the current touch location. */
  void MoveToTouchLocation(const ETouchIndex::Type FingerIndex, const FVector Location);

  /** Navigate player to the given world location. */
  void SetNewMoveDestination(const FVector DestLocation);

  /** Input handlers for SetDestination action. */
  void OnSetDestinationPressed();
  void OnSetDestinationReleased();

  TArray<FVector> current_path;
  uint32 moving : 1;

  struct TurningRadiusAwarePathPlan {
    // We do not include all waypoints as we will likely have to recompute
    // later ones.  We plan the initial steps and begin executing them, and
    // we will eventually compute more.
    TArray<FVector> initial_waypoints;
    FVector final_destination;
  };
  TurningRadiusAwarePathPlan path_plan;
};

