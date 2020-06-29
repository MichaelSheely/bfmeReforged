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
#include "Engine/EngineTypes.h"
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
  // If set to true, we do not mark circles on the map.
  marked_circles = true;
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
  // float const distance_to_final_dest = FVector::Dist(
  //     path_plan.final_destination, pawn->GetActorLocation());
  // if (distance_to_final_dest < kCloseEnoughToDestination)
  // {
  //   UE_LOG(LogTemp, Log, TEXT("Close enough to destination. Stopping movement."));
  //   moving = false;
  //   return;
  // }

  if (path_plan.initial_waypoints.Num() == 0)
  {
    UE_LOG(LogTemp, Log, TEXT("Arrived at destination. Stopping movement."));
    moving = false;  // We made it!
    return;
    // Need to compute more waypoints.
  }

  float const distance_to_current_waypoint = FVector::Dist(
      path_plan.initial_waypoints.Last(), pawn->GetActorLocation());

  // if (FMath::RandRange(0, 100) == 0)
  // {
  //   UE_LOG(LogTemp, Log, TEXT("Distance to current waypoint is %f. Close enough? %d"),
  //       distance_to_current_waypoint,
  //       (distance_to_current_waypoint < kCloseEnoughToDestination));
  // }

  SetNewMoveDestination(path_plan.initial_waypoints.Last());

  // if (distance_to_current_waypoint < kCloseEnoughToDestination) {
  //   FVector waypoint = path_plan.initial_waypoints.Pop();
  //   UE_LOG(LogTemp, Log, TEXT("Close enough to waypoint (%f, %f, %f), popped."),
  //       waypoint.X, waypoint.Y, waypoint.Z);
  // } else {
  //   SetNewMoveDestination(path_plan.initial_waypoints.Last());
  // }
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
    UE_LOG(LogTemp, Log, TEXT("Move to (%f,%f,%f) requested."),
        path_plan.final_destination.X,
        path_plan.final_destination.Y,
        path_plan.final_destination.Z);
    bool success = ComputeTurningRadiusAwarePathTo(
        path_plan.final_destination);
    marked_circles = true;
    if (!success)
    {
      UE_LOG(LogTemp, Log, TEXT("Failed to construct turning radius path."));
    }
    // FVector waypoint;
    // waypoint.X = -200.0;
    // waypoint.Y = -670.8;
    // waypoint.Z = 1.0;
    // Push path points in reverse order.
    // current_path.Push(Hit.ImpactPoint);
    // current_path.Push(waypoint);
    // path_plan.initial_waypoints.Push(Hit.ImpactPoint);
    // path_plan.initial_waypoints.Push(waypoint);
    // UE_LOG(LogTemp, Log, TEXT(
    //       "Set one waypoint (%f,%f,%f), on the path to (%f,%f,%f)."),
    //     waypoint.X, waypoint.Y, waypoint.Z,
    //     Hit.ImpactPoint.X, Hit.ImpactPoint.Y, Hit.ImpactPoint.Z);
  }
}

FVector PositiveHeadingTangentToCircleAtPoint(
    const FVector& center, const FVector& tangent_point)
{
  if (center.Y == tangent_point.Y)
  {
    // The tangent line for this point would be perfectly vertical so the slope
    // would be undefined.  Thus, we special case this by returning the
    // appropriate heading vector directly.
    return tangent_point.X > center.X ? FVector(0, 1, 0) : FVector(0, -1, 0);
  }
  float tangent_line_slope =
    (center.Y - tangent_point.Y) / (tangent_point.X - center.X);
  // Define a lambda for the equation of the line (using point slope form)
  // which is perpendicular to the circle at the specified tangent point.
  auto y = [tangent_point, tangent_line_slope](float x)
  {
    return tangent_line_slope * (x - tangent_point.X) + tangent_point.Y;
  };
  // Could be any positive value (we will scale to unit vector size).
  // Move forward along X and use equation of the line to determine change in Y.
  float delta_x = 1;
  float delta_y = y(tangent_point.X + delta_x) - tangent_point.Y;
  FVector orientation_vector(delta_x, delta_y, 0);
  float magnitude = orientation_vector.Size();
  orientation_vector /= magnitude;
  // In a left handed coordinate system, we want to move in the direction of
  // increasing X only when we are between -pi and 0 radians. So if the
  // tangent point's Y value indicates that we are between 0 and pi radians,
  // we should invert the orientation vector calculated above which was computed
  // based on dX > 0 (the value of X increasing).
  if (tangent_point.Y > center.Y)
  {
    orientation_vector *= -1;
  }
  return orientation_vector;
}


void InterpolateArcBetween(
    const FVector& center, float radius, const FVector& actor_loc,
    const FVector& final_point_on_circle,
    AbfmeReforgedPlayerController::TurningRadiusAwarePathPlan* path_plan,
    bool is_left_circle, float speed)
{
  float actor_theta = FGenericPlatformMath::Atan2(
      actor_loc.Y - center.Y, actor_loc.X - center.X);
  float dest_theta = FGenericPlatformMath::Atan2(
      final_point_on_circle.Y - center.Y, final_point_on_circle.X - center.X);
  // The step size of the arc length is the speed of the actor.
  float d_theta = speed / radius;
  UE_LOG(LogTemp, Log, TEXT("Actor (%f,%f), last circle waypoint (%f,%f)"
        "cirlce center (%f,%f), speed %f, radius %f"),
      actor_loc.X, actor_loc.Y, final_point_on_circle.X,
      final_point_on_circle.Y, center.X, center.Y, speed, radius);
  UE_LOG(LogTemp, Log, TEXT("Actor theta %f, waypoint theta %f, d_theta %f."),
      actor_theta, dest_theta, d_theta);
  // This will track the actor's progress along the arc of the turning radius
  // circle, starting at the actor's angle and ending at the final point on
  // the circle -- after which the actor will leave the circle and follow the
  // straight line path to their destination (as their heading will be such
  // that they do not need to turn any further.
  float theta = actor_theta;
  // Consider consolidating the branches and have the only difference be
  // the condition for when the while loop ends and how to increment, since
  // the remainder (setting position and orientation) is the same.
  if (is_left_circle)
  {
    // Moving left around the circle implies that actor_theta > dest_theta.
    // Thus, we will move in the negative direction until reaching the
    // final_point_on_circle, which occurs at angle dest_theta.
    while (theta > dest_theta)
    {
      theta -= d_theta;
      FVector arc_point(
          radius * FGenericPlatformMath::Cos(theta) + center.X,
          radius * FGenericPlatformMath::Sin(theta) + center.Y,
          actor_loc.Z);
      path_plan->initial_waypoints.Push(arc_point);
      path_plan->forward_vectors.Push(
          -1 * PositiveHeadingTangentToCircleAtPoint(center, arc_point));
    }
  }
  else
  {
    while (theta < dest_theta)
    {
      theta += d_theta;
      FVector arc_point(
          radius * FGenericPlatformMath::Cos(theta) + center.X,
          radius * FGenericPlatformMath::Sin(theta) + center.Y,
          actor_loc.Z);
      path_plan->initial_waypoints.Push(arc_point);
      path_plan->forward_vectors.Push(
          PositiveHeadingTangentToCircleAtPoint(center, arc_point));
    }
  }
}

void InterpolateLineSegmentBetween(
    const FVector& p1, const FVector& p2, float speed, float actor_z_loc,
    AbfmeReforgedPlayerController::TurningRadiusAwarePathPlan* path_plan)
{
  float delta_y = p2.Y - p1.Y;
  float delta_x = p2.X - p1.X;
  float distance = FGenericPlatformMath::Sqrt(delta_y*delta_y + delta_x*delta_x);
  float dx = speed * FGenericPlatformMath::Cos(
      FGenericPlatformMath::Atan2(delta_y, delta_x));
  float dy = speed * FGenericPlatformMath::Sin(
      FGenericPlatformMath::Atan2(delta_y, delta_x));
  int steps_needed = static_cast<int>(distance / speed);
  // Since it is travel along a line segment, the actor will use the same
  // orientation throughout the travel. Get a unit vector in the direction of
  // the line.
  FVector orientation = (p2 - p1) / (p2 - p1).Size();
  for (int i = 0; i < steps_needed; ++i)
  {
    FVector point(p1.X + i * dx, p1.Y + i * dy, actor_z_loc);
    path_plan->initial_waypoints.Push(point);
    path_plan->forward_vectors.Push(orientation);
  }
}

bool SetPathPlanArcAndLine(
    const FVector& circle_center, float radius, const FVector& actor_loc,
    const FVector& final_point_on_circle, const FVector& destination,
    AbfmeReforgedPlayerController::TurningRadiusAwarePathPlan* path_plan,
    bool is_left_circle, float speed, UWorld* world, bool marked)
{
  UE_LOG(LogTemp, Log, TEXT("Final circle waypoint = (%f,%f,%f)."),
      final_point_on_circle.X, final_point_on_circle.Y, final_point_on_circle.Z);
  InterpolateArcBetween(circle_center, radius, actor_loc,
      final_point_on_circle, path_plan, is_left_circle, speed);
  int arc_waypoints = path_plan->initial_waypoints.Num();
  UE_LOG(LogTemp, Log, TEXT("There were %d arc waypoints."), arc_waypoints);
  if (arc_waypoints > 0)
  {
    UE_LOG(LogTemp, Log, TEXT("Final arc waypoint was (%f, %f, %f)."),
        path_plan->initial_waypoints.Last().X,
        path_plan->initial_waypoints.Last().Y,
        path_plan->initial_waypoints.Last().Z);
  }
  InterpolateLineSegmentBetween(
      final_point_on_circle, destination, speed, actor_loc.Z, path_plan);
  path_plan->initial_waypoints.Push(destination);
  // This is silly, let's put them in the path in the correct order initially.
  TArray<FVector> waypoints = path_plan->initial_waypoints;
  TArray<FVector> forwards = path_plan->forward_vectors;
  bool update_headings = false;
  if (update_headings && (waypoints.Num() != forwards.Num()))
  {
    // Consider bundling the location and orientation into a single struct
    // so we can statically eliminate this "should never happen" error.
    UE_LOG(LogTemp, Log,
        TEXT("Found %d headings and %d waypoints. These should be the same."),
        waypoints.Num(), forwards.Num());
    path_plan->initial_waypoints.Empty();
    path_plan->forward_vectors.Empty();
    return false;
  }
  path_plan->initial_waypoints.Empty();
  path_plan->forward_vectors.Empty();
  for (int i = 0; i < waypoints.Num(); ++i)
  {
    if (!marked) {
      FRotator rotation(0,0,0);
      FActorSpawnParameters params;
      params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
      world->SpawnActor<AMarkerActor>(waypoints[waypoints.Num() - i - 1], rotation, params);
    }
    path_plan->initial_waypoints.Push(waypoints[waypoints.Num() - i - 1]);
    if (update_headings) {
      path_plan->forward_vectors.Push(forwards[forwards.Num() - i - 1]);
    }
  }
  return true;
}

void FindSlopesTangentToCircle(
    const FVector& circle_center, float radius,
    const FVector& destination, float* m1, float* m2)
{
  float px = destination.X;
  float py = destination.Y;
  float cx = circle_center.X;
  float cy = circle_center.Y;
  float R = radius;
  UE_LOG(LogTemp, Log, TEXT("px %f py %f cx %f cy %f R %f"),
      px, py, cx, cy, R);
  float A = 8*cx*px - 4*px*px - 4*cx*cx + 4*R*R;
  float B = 8*cx*cy - 8*cx*py - 8*cy*px + 8*px*py;
  float C = 8*cy*py - 4*cy*cy - 4*py*py + 4*R*R;
  UE_LOG(LogTemp, Log, TEXT("A=%f, B=%f, C=%f"), A, B, C);
  *m1 = ( -B + FGenericPlatformMath::Sqrt(B*B - 4*A*C) ) / (2*A);
  *m2 = ( -B - FGenericPlatformMath::Sqrt(B*B - 4*A*C) ) / (2*A);
}

void TangentIntersectionOfCircleAndLine(
    const FVector& circle_center, float circle_radius,
    const FVector& point_on_line, float tangent_line_slope,
    FVector* tangent_intersection)
{
  float px = point_on_line.X;
  float py = point_on_line.Y;
  float cx = circle_center.X;
  float cy = circle_center.Y;
  float m = tangent_line_slope;
  float R = circle_radius;
  float A = m*m + 1;
  float B = 2*m*py - 2*m*m*px - 2*m*cy - 2*cx;
  // We do not need to calculate C because our choice of `slope`
  // ensures that the line is tangent to the circle, meaning there
  // will be only one intersection, which means that there is only
  // one real valued solution to the equation, which means that
  // the discriminant of the quadratic is zero.
  // float C = cx*cx - R*R + (py - m*px - cy)*(py - m*px - cy);
  // The quadratic formula simplifies immensely when the discriminant is zero:
  tangent_intersection->X = -B / ( 2 * A );
  tangent_intersection->Y = m * ( tangent_intersection->X - px ) + py;
}

enum TangentPointOrdinal {
  TAN_INTERSECT_1,
  TAN_INTERSECT_2,
};

void FindTangentIntersections(
    const FVector& circle_center, float radius, const FVector& destination,
    FVector* tan_intersect_1, FVector* tan_intersect_2)
{
  float m1, m2;
  FindSlopesTangentToCircle(circle_center, radius, destination, &m1, &m2);
  UE_LOG(LogTemp, Log, TEXT("Slopes tangent to circle = { %f, %f }."), m1, m2);
  TangentIntersectionOfCircleAndLine(
      circle_center, radius, destination, m1, tan_intersect_1);
  TangentIntersectionOfCircleAndLine(
      circle_center, radius, destination, m2, tan_intersect_2);
}

// Starting at the angle (from zero) made by the actor, determines which of the
// two tangent line intersections would be encountered first as the positive
// value of the angle increases.  In UE4's left handed coordinate system, this
// means rotation clockwise from the actor's angle.
TangentPointOrdinal DetermineFirstIntersectionPointEncountered(
    const FVector& actor_loc, const FVector& tan_intersect_1,
    const FVector& tan_intersect_2, const FVector& circle_center)
{
  float theta_actor = FGenericPlatformMath::Atan2(
      actor_loc.Y - circle_center.Y,
      actor_loc.X - circle_center.X);
  float theta_1 = FGenericPlatformMath::Atan2(
      tan_intersect_1.Y - circle_center.Y,
      tan_intersect_1.X - circle_center.X);
  float theta_2 = FGenericPlatformMath::Atan2(
      tan_intersect_2.Y - circle_center.Y,
      tan_intersect_2.X - circle_center.X);
  UE_LOG(LogTemp, Log, TEXT(
        "tanx1 (%f,%f) has theta %f, tanx2 (%f,%f) has theta %f"),
      tan_intersect_1.X, tan_intersect_1.Y, theta_1,
      tan_intersect_2.X, tan_intersect_2.Y, theta_2);
  UE_LOG(LogTemp, Log, TEXT("actor (%f,%f) has theta %f"),
      actor_loc.X, actor_loc.Y, theta_actor);
  if ((theta_actor < theta_1 && theta_actor < theta_2) ||
      (theta_actor > theta_1 && theta_actor > theta_2))
  {
    UE_LOG(LogTemp, Log, TEXT("Actor proj not inside intersections, picked %s"),
          theta_1 < theta_2 ? TEXT("TANX_1") : TEXT("TANX_2"));
    return theta_1 < theta_2 ? TAN_INTERSECT_1 : TAN_INTERSECT_2;
  }
  // This means that the player is located in between the two tangent line
  // intersections.
  UE_LOG(LogTemp, Log, TEXT("Actor proj between intersections, picked %s"),
        theta_1 > theta_2 ? TEXT("TANX_1") : TEXT("TANX_2"));
  return theta_1 > theta_2 ? TAN_INTERSECT_1 : TAN_INTERSECT_2;
}

FVector SelectAppropriateWaypoint(
    const FVector& actor_loc,
    const FVector& tan_intersect_1, const FVector& tan_intersect_2,
    const FVector& circle_center, bool is_left_circle)
{
  TangentPointOrdinal first_intersection =
    DetermineFirstIntersectionPointEncountered(
        actor_loc, tan_intersect_1, tan_intersect_2, circle_center);
  // If right hand circle, the actor would move along the circle in the positive
  // direction, so we would return the first intersection encountered (moving
  // positively). If left hand circle, this is reversed, as the first point
  // the actor will encountered will be the second one we encountered (because
  // we were rotating in the opposite direction).
  if (is_left_circle ^ (first_intersection == TAN_INTERSECT_1))
  {
    return tan_intersect_1;
  }
  return tan_intersect_2;
}

FVector SelectWaypointOnCircle(
    const FVector& actor_loc, const FVector& circle_center,
    float radius, const FVector& destination, bool is_left_circle)
{
  FVector tan_intersect_1, tan_intersect_2;
  FindTangentIntersections(
      circle_center, radius, destination,
      &tan_intersect_1, &tan_intersect_2);
  return SelectAppropriateWaypoint(actor_loc, tan_intersect_1, tan_intersect_2,
      circle_center, is_left_circle);
}

bool AbfmeReforgedPlayerController::ComputeTurningRadiusAwarePathTo(
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
    return false;
  }
  FVector actor_loc = pawn->GetActorLocation();
  // Only move in the 2D plane, for now.
  destination.Z = actor_loc.Z;
  FVector actor_forward_vec = pawn->GetActorForwardVector();
  UE_LOG(LogTemp, Log, TEXT(
        "Unit at loc (%f,%f,%f) has forward vec (%f,%f,%f)."),
        actor_loc.X, actor_loc.Y, actor_loc.Z,
        actor_forward_vec.X, actor_forward_vec.Y, actor_forward_vec.Z);
  // From the actor's perspective, there would be two circles tangent to the
  // unit on their left and right with a radius equal to their turning radius.
  // To find the center of the circle to the left of the unit, we use apply
  // the rotation matrix [[0 -1] [1 0]] (a 90 degree clockwise rotation) to
  // the unit's forward vector.  Note that because UE4 uses a left handed
  // coordinate system, we swap the left and right if we are looking down
  // on the game.  This matrix multiplication results in:
  FVector right_circle_center_displacement(
      -1 * actor_forward_vec.Y, actor_forward_vec.X, 0);
  FVector left_circle_center_displacement(
      -1 * right_circle_center_displacement.X,
      -1 * right_circle_center_displacement.Y, 0);
  // Ensure that the displacement vectors have magnitude equal to the turning
  // radius of the unit (in the plane).
  // TODO(msheely): If unit it not moving, set turning radius to zero?
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
  UE_LOG(LogTemp, Log, TEXT(
        "Unit sf=%f, lcc=(%f,%f) rcc=(%f,%f)."),
      scaling_factor, left_circle_center.X, left_circle_center.Y,
      right_circle_center.X, right_circle_center.Y);
  if (false) {
    // Mark the circle centers on the map for debugging.
    FRotator rotation(0, 0, 0);
    FActorSpawnParameters params;
    params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    GetWorld()->SpawnActor<AMarkerActor>(left_circle_center, rotation, params);
    GetWorld()->SpawnActor<AMarkerActor>(right_circle_center, rotation, params);
    // Only mark the circle centers once, even if mouse is held for more than
    // a single tick.
    marked_circles = true;
  }
  // Check which circle has a center closer to our final destination.
  // That is the circle around which the unit should turn when travelling.
  float dist_left_center = FVector::Dist(left_circle_center, destination);
  float dist_right_center = FVector::Dist(right_circle_center, destination);
  if (dist_left_center < turning_radius || dist_right_center < turning_radius)
  {
    UE_LOG(LogTemp, Log, TEXT("Destination within turning radius."));
    // We are close enough to the destination we just need to turn and move
    // slightly forward.  We will implement this later as it should be the
    // simple case.  Using the turning radius is the more interesting case.
    // For now, just set the final destination and use default pathfinding.
    InterpolateLineSegmentBetween(
        actor_loc, destination, actor_speed, actor_loc.Z, &path_plan);
    // This is silly, let's put them in the path in the correct order initially.
    TArray<FVector> waypoints = path_plan.initial_waypoints;
    path_plan.initial_waypoints.Empty();
    for (int i = 0; i < waypoints.Num(); ++i)
    {
      path_plan.initial_waypoints.Push(waypoints[waypoints.Num() - i - 1]);
    }
    // TODO: Set forward_vectors to final destination.
    // Consider edge case of speed * tick_duration << turning radius,
    // in which case there could be a situation where rotation + directly
    // moving to the destination may violate the actor's move abilities.
    return true;
  }
  if (dist_left_center < dist_right_center) {
    UE_LOG(LogTemp, Log, TEXT("Picked left circle (closer center)."));
    FVector final_point_on_circle = SelectWaypointOnCircle(
        actor_loc, left_circle_center, turning_radius,
        destination, /*is_left_circle=*/true);
    return SetPathPlanArcAndLine(
        left_circle_center, turning_radius, actor_loc, final_point_on_circle,
        destination, &path_plan, /*is_left_circle=*/true, actor_speed,
        GetWorld(), marked_circles);
  } else {
    UE_LOG(LogTemp, Log, TEXT("Picked right circle (closer center)."));
    FVector final_point_on_circle = SelectWaypointOnCircle(
        actor_loc, right_circle_center, turning_radius,
        destination, /*is_left_circle=*/false);
    return SetPathPlanArcAndLine(
        right_circle_center, turning_radius, actor_loc,
        final_point_on_circle, destination, &path_plan,
        /*is_left_circle=*/false, actor_speed, GetWorld(), marked_circles);
  }
}

void AbfmeReforgedPlayerController::MoveToTouchLocation(
    const ETouchIndex::Type FingerIndex, FVector Location)
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

void AbfmeReforgedPlayerController::SetNewMoveDestination(const FVector& DestLocation)
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

    // We need to issue move command only if far enough in order for walk
    // animation to play correctly
    // if ((Distance > 120.0f))
    {
      // bool still_moving = true;
      // MoveWithTurnRadius(this, DestLocation, &still_moving);
      // moving = still_moving;
      if (path_plan.initial_waypoints.Num() == 0)
      {
        UE_LOG(LogTemp, Log, TEXT("SetNewMoveDestination called when path "
              "plan was empty. Should never happen."));
        // We made it.
        moving = false;
      }
      // Just for testing purposes, until we tie speeds to TickRate, we will
      // only have the next point pop off a certain percentage of the time. This
      // will ensure it is slow enough to see what is going on while debugging.
      // if (FMath::RandRange(0, 100) == 0)
      if (true)
      {
        // Move to the next waypoint.
        FVector waypoint = path_plan.initial_waypoints.Pop();
        // FVector orientation = path_plan.forward_vectors.Pop();
        // UE_LOG(LogTemp, Log, TEXT("Moving to waypoint (%f,%f,%f)"),
        //     waypoint.X, waypoint.Y, 0);
        bool check_for_collisions_on_path = false;
        FHitResult hit_result;
        MyPawn->SetActorLocation(waypoint, check_for_collisions_on_path,
            &hit_result, ETeleportType::TeleportPhysics);
        // MyPawn->SetActorRotation(orientation.Rotation());
      }
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
