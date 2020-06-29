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

  struct TurningRadiusAwarePathPlan {
    // We do not include all waypoints as we will likely have to recompute
    // later ones.  We plan the initial steps and begin executing them, and
    // we will eventually compute more.
    TArray<FVector> initial_waypoints;
    TArray<FVector> forward_vectors;
    // Could add an array of timestamps (based on unit speed profile) which
    // determine when the unit is allowed to reach each of the waypoints
    // above.  Need to sync with reforged team to determine overall unit
    // movement implementation strategy.
    FVector final_destination;
  };

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
  // Returns true if bath computed successfully, otherwise false.
  bool ComputeTurningRadiusAwarePathTo(FVector destination);

  /** Navigate player to the current touch location. */
  void MoveToTouchLocation(const ETouchIndex::Type FingerIndex, FVector Location);

  /** Navigate player to the given world location. */
  void SetNewMoveDestination(const FVector& DestLocation);

  /** Input handlers for SetDestination action. */
  void OnSetDestinationPressed();
  void OnSetDestinationReleased();

  // TArray<FVector> current_path;
  uint32 moving : 1;
  uint32 marked_circles : 1;
  float turning_radius = 200.0f;
  float actor_speed = 1.0f;

  TurningRadiusAwarePathPlan path_plan;
};

