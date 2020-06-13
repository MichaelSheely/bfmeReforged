// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "bfmeReforgedPlayerController.h"
#include "Runtime/Engine/Classes/Components/DecalComponent.h"
#include "bfmeReforgedCharacter.h"
#include "Engine/World.h"

// Probably don't need all of these, remove superfluous ones:
#include "EngineGlobals.h"
#include "TimerManager.h"
#include "Engine/Engine.h"
#include "AITypes.h"
#include "AISystem.h"
#include "BrainComponent.h"
#include "Navigation/PathFollowingComponent.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Blueprint/AIAsyncTaskBlueprintProxy.h"
#include "Animation/AnimInstance.h"
#include "Math/UnrealMathUtility.h"
#include "NavigationPath.h"
#include "NavigationData.h"
#include "NavigationSystem.h"
#include "Logging/MessageLog.h"
// END.

AbfmeReforgedPlayerController::AbfmeReforgedPlayerController()
{
  bShowMouseCursor = true;
  moving = false;
  DefaultMouseCursor = EMouseCursor::Crosshairs;
}

void AbfmeReforgedPlayerController::PlayerTick(float DeltaTime)
{
  Super::PlayerTick(DeltaTime);

  // keep updating the destination every tick while desired
  if (bMoveToMouseCursor)
  {
    ComputePathToCursor();
    moving = true;
  }

  if (!moving) return;

  float const distance_to_final_dest = FVector::Dist(
      path_plan.final_destination, GetPawn()->GetActorLocation());
  if (distance_to_final_dest < 300.0f)
  {
    UE_LOG(LogTemp, Log, TEXT("Close enough to destination. Stopping movement."));
    moving = false;
    return;
  }

  if (path_plan.initial_waypoints.Num() == 0)
  {
    // Need to compute more waypoints.
  }

  float const distance_to_current_waypoint = FVector::Dist(
      path_plan.initial_waypoints.Last(), GetPawn()->GetActorLocation());

  if (FMath::RandRange(0, 100) == 0)
  {
    UE_LOG(LogTemp, Log, TEXT("Distance to current waypoint is %f. Close enough? %d"),
        distance_to_current_waypoint, (distance_to_current_waypoint < 300.0f));
  }

  if (distance_to_current_waypoint < 300.0f) {
    FVector waypoint = path_plan.initial_waypoints.Pop();
    UE_LOG(LogTemp, Log, TEXT("Close enough to waypoint (%f, %f, %f), popped."),
        waypoint.X, waypoint.Y, waypoint.Z);
  } else {
    SetNewMoveDestination(path_plan.initial_waypoints.Last());
  }
}

void AbfmeReforgedPlayerController::SetupInputComponent()
{
  // set up gameplay key bindings
  Super::SetupInputComponent();

  InputComponent->BindAction("SetDestination", IE_Pressed, this, &AbfmeReforgedPlayerController::OnSetDestinationPressed);
  InputComponent->BindAction("SetDestination", IE_Released, this, &AbfmeReforgedPlayerController::OnSetDestinationReleased);

  // support touch devices 
  InputComponent->BindTouch(EInputEvent::IE_Pressed, this, &AbfmeReforgedPlayerController::MoveToTouchLocation);
  InputComponent->BindTouch(EInputEvent::IE_Repeat, this, &AbfmeReforgedPlayerController::MoveToTouchLocation);

}

UPathFollowingComponent* InitNavigationControl(AController& Controller)
{
  AAIController* AsAIController = Cast<AAIController>(&Controller);
  UPathFollowingComponent* PathFollowingComp = nullptr;

  if (AsAIController)
  {
    PathFollowingComp = AsAIController->GetPathFollowingComponent();
  }
  else
  {
    PathFollowingComp = Controller.FindComponentByClass<UPathFollowingComponent>();
    if (PathFollowingComp == nullptr)
    {
      PathFollowingComp = NewObject<UPathFollowingComponent>(&Controller);
      PathFollowingComp->RegisterComponentWithWorld(Controller.GetWorld());
      PathFollowingComp->Initialize();
    }
  }

  return PathFollowingComp;
}

void MoveWithTurnRadius(AController* Controller, const FVector& GoalLocation, bool* still_moving)
{
  UNavigationSystemV1* NavSys = Controller ? FNavigationSystem::GetCurrent<UNavigationSystemV1>(Controller->GetWorld()) : nullptr;
  if (NavSys == nullptr || Controller == nullptr || Controller->GetPawn() == nullptr)
  {
    UE_LOG(LogNavigation, Warning, TEXT("UNavigationSystemV1::SimpleMoveToActor called for NavSys:%s Controller:%s controlling Pawn:%s (if any of these is None then there's your problem"),
      *GetNameSafe(NavSys), *GetNameSafe(Controller), Controller ? *GetNameSafe(Controller->GetPawn()) : TEXT("NULL"));
    *still_moving = false;
    return;
  }

  // See https://www.gamasutra.com/view/feature/131505/toward_more_realistic_pathfinding.php?page=3
  // for various improvements to handling obstacles in the turning radius aware path.
  UPathFollowingComponent* PFollowComp = InitNavigationControl(*Controller);

  if (PFollowComp == nullptr)
  {
    UE_LOG(LogNavigation, Warning, TEXT("Null PFollowComp"));
    *still_moving = false;
    return;
  }

  if (!PFollowComp->IsPathFollowingAllowed())
  {
    UE_LOG(LogNavigation, Warning, TEXT("PathFollowing not allowed."));
    *still_moving = false;
    return;
  }

  const bool bAlreadyAtGoal = PFollowComp->HasReached(GoalLocation, EPathFollowingReachMode::OverlapAgent);

  // script source, keep only one move request at time
  if (PFollowComp->GetStatus() != EPathFollowingStatus::Idle)
  {
    PFollowComp->AbortMove(*NavSys, FPathFollowingResultFlags::ForcedScript | FPathFollowingResultFlags::NewRequest
      , FAIRequestID::AnyRequest, bAlreadyAtGoal ? EPathFollowingVelocityMode::Reset : EPathFollowingVelocityMode::Keep);
    // UE_LOG(LogTemp, Log, TEXT("Abort move due to status not idle"));
    // *still_moving = false;
  }

  // script source, keep only one move request at time
  if (PFollowComp->GetStatus() != EPathFollowingStatus::Idle)
  {
    PFollowComp->AbortMove(*NavSys, FPathFollowingResultFlags::ForcedScript | FPathFollowingResultFlags::NewRequest);
    // UE_LOG(LogTemp, Log, TEXT("Abort move due to status not idle 2"));
    // *still_moving = false;
  }

  if (bAlreadyAtGoal)
  {
    PFollowComp->RequestMoveWithImmediateFinish(EPathFollowingResult::Success);
    UE_LOG(LogTemp, Log, TEXT("Abort move due to already at goal"));
    *still_moving = false;
  }
  else
  {
    const FVector AgentNavLocation = Controller->GetNavAgentLocation();
    const ANavigationData* NavData = NavSys->GetNavDataForProps(Controller->GetNavAgentPropertiesRef(), AgentNavLocation);
    if (NavData)
    {
      FPathFindingQuery Query(Controller, *NavData, AgentNavLocation, GoalLocation);
      FNavAgentProperties navigation_properties;
      // TODO: consider setting navigation_properties.AgentRadius.
      // Something like this may be interesting / useful:
      // https://www.gamedev.net/tutorials/programming/general-and-gameplay-programming/creating-a-movement-component-for-an-rts-in-ue4-r4019/
      FPathFindingResult Result = NavSys->FindPathSync(navigation_properties, Query);
      if (Result.IsSuccessful())
      {
        int point_idx = 0;
        for (auto& path_point : Result.Path->GetPathPoints()) {
          // path_point.Location.X -= 100;
          // path_point.Location.Y -= 100;
          // float x = path_point.Location.X;
          // float y = path_point.Location.Y;
          // float z = path_point.Location.Z;
          // FString point_debug = FString::Printf(TEXT("Point %d (%f,%f,%f)"),
          //     point_idx, x, y, z);
          // UE_LOG(LogTemp, Log, TEXT("%s"), *point_debug);
          // ++point_idx;
        }
        // UE_LOG(LogTemp, Log, TEXT("Path %s"), *(Result.Path->GetDescription()));

        // FVector waypoint;
        // waypoint.X = -200.0;
        // waypoint.Y = -670.8;
        // waypoint.Z = 1.0;
        // TArray<FVector> points;
        // points.Push(waypoint);
        // points.Push(GoalLocation);
        // FAIMoveRequest move_request = FAIMoveRequest(GoalLocation);
        // move_request.SetUsePathfinding(false);
        // PFollowComp->RequestMove(move_request, Result.Path);
        PFollowComp->RequestMove(FAIMoveRequest(GoalLocation), Result.Path);
      }
      else if (PFollowComp->GetStatus() != EPathFollowingStatus::Idle)
      {
        PFollowComp->RequestMoveWithImmediateFinish(EPathFollowingResult::Invalid);
        UE_LOG(LogTemp, Log, TEXT("Abort move due to status idle 3"));
        *still_moving = false;
      }
    }
  }
  // switch (PFollowComp->GetStatus()) {
  //   case EPathFollowingStatus::Idle:
  //     UE_LOG(LogTemp, Log, TEXT("PFollowComp->GetStatus() is Idle."));
  //   case EPathFollowingStatus::Waiting:
  //     UE_LOG(LogTemp, Log, TEXT("PFollowComp->GetStatus() is Waiting."));
  //   case EPathFollowingStatus::Paused:
  //     UE_LOG(LogTemp, Log, TEXT("PFollowComp->GetStatus() is Paused."));
  //   case EPathFollowingStatus::Moving:
  //     UE_LOG(LogTemp, Log, TEXT("PFollowComp->GetStatus() is Moving."));
  // }
}

void AbfmeReforgedPlayerController::ComputePathToCursor()
{
  // Trace to see what is under the mouse cursor
  FHitResult Hit;
  GetHitResultUnderCursor(ECC_Visibility, false, Hit);

  // FString display = TEXT("Running MoveToMouseCursor.");
  // GEngine->AddOnScreenDebugMessage(-1, 1.0, FColor::Red, display);

  // If we hit something, calculate path with turning radius to that point.
  if (Hit.bBlockingHit)
  {
    // Clear out existing waypoints (if any).
    path_plan.initial_waypoints.Empty();
    path_plan.final_destination = Hit.ImpactPoint;
    FVector waypoint;
    waypoint.X = -200.0;
    waypoint.Y = -670.8;
    waypoint.Z = 1.0;
    // Push path points in reverse order.
    // current_path.Push(Hit.ImpactPoint);
    // current_path.Push(waypoint);
    path_plan.initial_waypoints.Push(Hit.ImpactPoint);
    path_plan.initial_waypoints.Push(waypoint);
    UE_LOG(LogTemp, Log, TEXT(
          "Set one waypoint (%f,%f,%f), on the path to (%f,%f,%f)."),
        waypoint.X, waypoint.Y, waypoint.Z,
        Hit.ImpactPoint.X, Hit.ImpactPoint.Y, Hit.ImpactPoint.Z);
    // SetNewMoveDestination(Hit.ImpactPoint);
  }
}

void AbfmeReforgedPlayerController::MoveToTouchLocation(const ETouchIndex::Type FingerIndex, const FVector Location)
{
  FVector2D ScreenSpaceLocation(Location);

  // Trace to see what is under the touch location
  FHitResult HitResult;
  GetHitResultAtScreenPosition(ScreenSpaceLocation, CurrentClickTraceChannel, true, HitResult);
  if (HitResult.bBlockingHit)
  {
    // We hit something, move there
    SetNewMoveDestination(HitResult.ImpactPoint);
  }
}

void AbfmeReforgedPlayerController::SetNewMoveDestination(const FVector DestLocation)
{
  // FString display = TEXT("Running SetNewMoveDestination.");
  // GEngine->AddOnScreenDebugMessage(-1, 1.0, FColor::Red, display);
  APawn* const MyPawn = GetPawn();
  if (MyPawn)
  {
    float const Distance = FVector::Dist(DestLocation, MyPawn->GetActorLocation());

    // FVector pawn_loc = MyPawn->GetActorLocation();
    // FString float_str = FString::SanitizeFloat(runtime);
    // FString display = TEXT("Running time: ");
    // display += float_str;
    // GEngine->AddOnScreenDebugMessage(-1, 1.0, FColor::Red, display);

    // We need to issue move command only if far enough in order for walk animation to play correctly
    if ((Distance > 120.0f))
    {
      bool still_moving = true;
      MoveWithTurnRadius(this, DestLocation, &still_moving);
      moving = still_moving;
    }
  }
}

void AbfmeReforgedPlayerController::OnSetDestinationPressed()
{
  // set flag to keep updating destination until released
  bMoveToMouseCursor = true;
}

void AbfmeReforgedPlayerController::OnSetDestinationReleased()
{
  FString display = TEXT("Clearing bMoveToMouseCursor flag.");
  GEngine->AddOnScreenDebugMessage(-1, 1.0, FColor::Red, display);
  // clear flag to indicate we should stop updating the destination
  bMoveToMouseCursor = false;
}
