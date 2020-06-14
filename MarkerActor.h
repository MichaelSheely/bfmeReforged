#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MarkerActor.generated.h"

UCLASS()
class BFMEREFORGED_API AMarkerActor : public AActor
{
  GENERATED_BODY()

public:
  // Sets default values for this actor's properties
  AMarkerActor();

protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

public:
  // Called every frame
  virtual void Tick(float DeltaTime) override;

  UPROPERTY(EditAnywhere)
    UStaticMeshComponent* mesh;

};
