// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "bfmeReforgedPlayerController.h"
#include "GenericPlatform/GenericPlatformMath.h"
#include "Runtime/Engine/Classes/Components/DecalComponent.h"
#include "MarkerActor.h"
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

constexpr float kCloseEnoughToDestination = 280.0f;

AbfmeReforgedPlayerController::AbfmeReforgedPlayerController()
{
  bShowMouseCursor = true;
  moving = false;
  marked_circles = false;
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

  APawn* const pawn = GetPawn();
  if (!pawn)
  {
    UE_LOG(LogTemp, Log, TEXT("Unexpected null pawn when trying to move character."));
    return;
  }
  float const distance_to_final_dest = FVector::Dist(
      path_plan.final_destination, pawn->GetActorLocation());
  if (distance_to_final_dest < kCloseEnoughToDestination)
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
      path_plan.initial_waypoints.Last(), pawn->GetActorLocation());

  if (FMath::RandRange(0, 100) == 0)
  {
    UE_LOG(LogTemp, Log, TEXT("Distance to current waypoint is %f. Close enough? %d"),
        distance_to_current_waypoint,
        (distance_to_current_waypoint < kCloseEnoughToDestination));
  }

  if (distance_to_current_waypoint < kCloseEnoughToDestination) {
    FVector waypoint = path_plan.initial_waypoints.Pop();
    UE_LOG(LogTemp, Log, TEXT("Close enough to waypoint (%f, %f, %f), popped."),
        waypoint.X, waypoint.Y, waypoint.Z);
  } else {
    // SetNewMoveDestination(path_plan.initial_waypoints.Last());
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
    TArray<FVector> initial_waypoints =
      ComputeTurningRadiusAwarePathTo(path_plan.final_destination);

    // FVector waypoint;
    // waypoint.X = -200.0;
    // waypoint.Y = -670.8;
    // waypoint.Z = 1.0;
    // Push path points in reverse order.
    // current_path.Push(Hit.ImpactPoint);
    // current_path.Push(waypoint);
    for (const FVector& waypoint : initial_waypoints)
    {
      path_plan.initial_waypoints.Push(waypoint);
    }
    // path_plan.initial_waypoints.Push(Hit.ImpactPoint);
    // path_plan.initial_waypoints.Push(waypoint);
    // UE_LOG(LogTemp, Log, TEXT(
    //       "Set one waypoint (%f,%f,%f), on the path to (%f,%f,%f)."),
    //     waypoint.X, waypoint.Y, waypoint.Z,
    //     Hit.ImpactPoint.X, Hit.ImpactPoint.Y, Hit.ImpactPoint.Z);
  }
}

TArray<FVector> TurningRadiusWaypointsAlongCircle(
    FVector circle_center, FVector actor_loc, FVector destination)
{
  // TODO(msheely): Implement me.
  TArray<FVector> waypoints;
  waypoints.Push(destination);
  return waypoints;
}

TArray<FVector> AbfmeReforgedPlayerController::ComputeTurningRadiusAwarePathTo(
    FVector destination)
{
  // Note Z axis is currently ignored. All pathfinding is done on a grid.
  // This is because (even for flying units) we can consider the map as
  // a 2D space in terms of pathfinding. Flying units will be able to
  // ignore obstacles on the map.
  APawn* const pawn = GetPawn();
  if (!pawn)
  {
    UE_LOG(LogTemp, Log, TEXT("Unexpected null pawn when trying to move character."));
    return TArray<FVector>();
  }
  FVector actor_loc = pawn->GetActorLocation();
  FVector actor_forward_vec = pawn->GetActorForwardVector();
  UE_LOG(LogTemp, Log, TEXT(
        "Unit at loc (%f,%f,%f) has forward vec (%f,%f,%f)."),
        actor_loc.X, actor_loc.Y, actor_loc.Z,
        actor_forward_vec.X, actor_forward_vec.Y, actor_forward_vec.Z);
  // From the actor's perspective, there would be two circles tangent to the
  // unit on their left and right with a radius equal to their turning radius.
  // To find the center of the circle to the left of the unit, we use apply
  // the rotation matrix [[0 -1] [1 0]] (a 90 degree clockwise rotation) to
  // the unit's forward vector.  The matrix multiplication results in:
  FVector left_circle_center_displacement(
      -1 * actor_forward_vec.Y, actor_forward_vec.X, 0);
  FVector right_circle_center_displacement(
      -1 * left_circle_center_displacement.X,
      -1 * left_circle_center_displacement.Y, 0);
  // TODO(msheely): If unit it not moving, set turning radius to zero?
  // Ensure that the displacement vectors have magnitude equal to the turning
  // radius of the unit (in the plane).
  float scaling_factor = turning_radius / FGenericPlatformMath::Sqrt(
      (left_circle_center_displacement.X * left_circle_center_displacement.X) +
      (left_circle_center_displacement.Y * left_circle_center_displacement.Y));
  left_circle_center_displacement.X *= scaling_factor;
  left_circle_center_displacement.Y *= scaling_factor;
  right_circle_center_displacement.X *= scaling_factor;
  right_circle_center_displacement.Y *= scaling_factor;
  // Compute the center of the left and right circles.
  FVector left_circle_center(
      actor_loc.X + left_circle_center_displacement.X,
      actor_loc.Y + left_circle_center_displacement.Y,
      actor_loc.Z + left_circle_center_displacement.Z);
  FVector right_circle_center(
      actor_loc.X + right_circle_center_displacement.X,
      actor_loc.Y + right_circle_center_displacement.Y,
      actor_loc.Z + right_circle_center_displacement.Z);
  if (!marked_circles) {
    // Mark the circle centers on the map for debugging.
    UE_LOG(LogTemp, Log, TEXT(
          "Unit sf=%f, lcc=(%f,%f,%f) rcc=(%f,%f,%f)."),
        scaling_factor, left_circle_center.X, left_circle_center.Y, 0,
        right_circle_center.X, right_circle_center.Y, 0);
    FRotator rotation(0, 0, 0);
    FActorSpawnParameters params;
    params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    GetWorld()->SpawnActor<AMarkerActor>(left_circle_center, rotation, params);
    GetWorld()->SpawnActor<AMarkerActor>(right_circle_center, rotation, params);
    // Only mark the circle centers once, even if mouse is held for more than
    // a single tick.
    marked_circles = true;
  }

  float dist_left_center =
    FVector::Dist(left_circle_center, pawn->GetActorLocation());
  float dist_right_center =
    FVector::Dist(right_circle_center, pawn->GetActorLocation());
  if (dist_left_center < turning_radius || dist_right_center < turning_radius)
  {
    // We are close enough to the destination we just need to turn and move
    // slightly forward.  We will implement this later as it should be the
    // simple case.  Using the turning radius is the more interesting case.
    // For now, just set the final destination and use default pathfinding.
    TArray<FVector> final_dest;
    final_dest.Push(destination);
    return final_dest;
  }
  if (dist_left_center < dist_right_center) {
    return TurningRadiusWaypointsAlongCircle(
        left_circle_center, actor_loc, destination);
  } else {
    return TurningRadiusWaypointsAlongCircle(
        right_circle_center, actor_loc, destination);
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

void MoveWithTurnRadius(
    AController* Controller, const FVector& GoalLocation, bool* still_moving)
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
        FAIMoveRequest move_request(GoalLocation);
        move_request.SetAcceptanceRadius(10.0f);
        PFollowComp->RequestMove(move_request, Result.Path);
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

void AbfmeReforgedPlayerController::SetNewMoveDestination(const FVector DestLocation)
{
  // FString display = TEXT("Running SetNewMoveDestination.");
  // GEngine->AddOnScreenDebugMessage(-1, 1.0, FColor::Red, display);
  APawn* const MyPawn = GetPawn();
  if (MyPawn)
  {
    // float const Distance = FVector::Dist(DestLocation, MyPawn->GetActorLocation());

    // FVector pawn_loc = MyPawn->GetActorLocation();
    // FString float_str = FString::SanitizeFloat(runtime);
    // FString display = TEXT("Running time: ");
    // display += float_str;
    // GEngine->AddOnScreenDebugMessage(-1, 1.0, FColor::Red, display);

    // We need to issue move command only if far enough in order for walk animation to play correctly
    // if ((Distance > 120.0f))
    {
      bool still_moving = true;
      MoveWithTurnRadius(this, DestLocation, &still_moving);
      // moving = still_moving;
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
