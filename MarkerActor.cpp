#include "MarkerActor.h"

// Sets default values
AMarkerActor::AMarkerActor()
{
  // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
  mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MarkerMesh"));
  RootComponent = mesh;
  static ConstructorHelpers::FObjectFinder<UStaticMesh> BaseMeshAsset(
      TEXT("StaticMesh'/Game/StarterContent/Props/SM_PillarFrame.SM_PillarFrame'"));
  if (BaseMeshAsset.Object)
  {
    mesh->SetStaticMesh(BaseMeshAsset.Object);
  }
}

// Called when the game starts or when spawned
void AMarkerActor::BeginPlay()
{
  Super::BeginPlay();

}

// Called every frame
void AMarkerActor::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);
}

