// Fill out your copyright notice in the Description page of Project Settings.
#pragma once
#include "FunctionLibrary/OHCollisionUtils.h"

#include "Component/OHMovementComponent.h"
#include "FunctionLibrary/OHCombatUtils.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "GameFramework/Character.h"
#include "Kismet/GameplayStatics.h"

#pragma region CollisionDetection

#pragma region AABB

bool UOHCollisionUtils::AABB_Overlap(const FVector& CenterA, const FVector& ExtentsA, const FVector& CenterB,
                                     const FVector& ExtentsB) {
    const float DeltaX = FMath::Abs(CenterA.X - CenterB.X);
    const float DeltaY = FMath::Abs(CenterA.Y - CenterB.Y);
    const float DeltaZ = FMath::Abs(CenterA.Z - CenterB.Z);

    const float TotalX = ExtentsA.X + ExtentsB.X;
    const float TotalY = ExtentsA.Y + ExtentsB.Y;
    const float TotalZ = ExtentsA.Z + ExtentsB.Z;

    return (DeltaX <= TotalX) && (DeltaY <= TotalY) && (DeltaZ <= TotalZ);
}

FVector UOHCollisionUtils::ComputeAABBDistanceOrPenetration(const FVector& CenterA, const FVector& ExtentsA,
                                                            const FVector& CenterB, const FVector& ExtentsB) {
    FVector Delta = CenterB - CenterA;
    FVector TotalExtents = ExtentsA + ExtentsB;
    FVector AbsDelta = Delta.GetAbs();

    // Positive value = penetration depth, negative = gap
    FVector Overlap = TotalExtents - AbsDelta;

    return Overlap;
}

FVector UOHCollisionUtils::ClosestPointOnAABB(const FVector& Point, const FVector& BoxCenter, const FVector& Extents) {
    FVector Local = Point - BoxCenter;
    FVector Clamped = Local;

    Clamped.X = FMath::Clamp(Clamped.X, -Extents.X, Extents.X);
    Clamped.Y = FMath::Clamp(Clamped.Y, -Extents.Y, Extents.Y);
    Clamped.Z = FMath::Clamp(Clamped.Z, -Extents.Z, Extents.Z);

    return BoxCenter + Clamped;
}

FVector UOHCollisionUtils::SupportAABB(const FVector& Direction, const FVector& Center, const FVector& Extents) {
    return Center + FVector(Direction.X >= 0 ? Extents.X : -Extents.X, Direction.Y >= 0 ? Extents.Y : -Extents.Y,
                            Direction.Z >= 0 ? Extents.Z : -Extents.Z);
}
#pragma endregion

#pragma region SAT

bool UOHCollisionUtils::SAT_BoxIntersect(const FVector& CenterA, const FRotator& RotationA, const FVector& ExtentsA,
                                         const FVector& CenterB, const FRotator& RotationB, const FVector& ExtentsB) {
    const FVector AxesA[3] = {RotationA.RotateVector(FVector::ForwardVector),
                              RotationA.RotateVector(FVector::RightVector), RotationA.RotateVector(FVector::UpVector)};

    const FVector AxesB[3] = {RotationB.RotateVector(FVector::ForwardVector),
                              RotationB.RotateVector(FVector::RightVector), RotationB.RotateVector(FVector::UpVector)};

    const FVector T = CenterB - CenterA;

    float R[3][3], AbsR[3][3];
    constexpr float EPSILON = 1e-5f;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = FVector::DotProduct(AxesA[i], AxesB[j]);
            AbsR[i][j] = FMath::Abs(R[i][j]) + EPSILON;
        }
    }

    for (int i = 0; i < 3; ++i) {
        float ra = ExtentsA.Component(i);
        float rb = ExtentsB.X * AbsR[i][0] + ExtentsB.Y * AbsR[i][1] + ExtentsB.Z * AbsR[i][2];
        float proj = FMath::Abs(FVector::DotProduct(T, AxesA[i]));
        if (proj > ra + rb) {
            return false;
        }
    }

    for (int i = 0; i < 3; ++i) {
        float ra = ExtentsA.X * AbsR[0][i] + ExtentsA.Y * AbsR[1][i] + ExtentsA.Z * AbsR[2][i];
        float rb = ExtentsB.Component(i);
        float proj = FMath::Abs(FVector::DotProduct(T, AxesB[i]));
        if (proj > ra + rb) {
            return false;
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            FVector Axis = FVector::CrossProduct(AxesA[i], AxesB[j]);
            if (!Axis.Normalize()) {
                continue;
            }

            float ra = ExtentsA.X * FMath::Abs(FVector::DotProduct(Axis, AxesA[0])) +
                       ExtentsA.Y * FMath::Abs(FVector::DotProduct(Axis, AxesA[1])) +
                       ExtentsA.Z * FMath::Abs(FVector::DotProduct(Axis, AxesA[2]));

            float rb = ExtentsB.X * FMath::Abs(FVector::DotProduct(Axis, AxesB[0])) +
                       ExtentsB.Y * FMath::Abs(FVector::DotProduct(Axis, AxesB[1])) +
                       ExtentsB.Z * FMath::Abs(FVector::DotProduct(Axis, AxesB[2]));

            float proj = FMath::Abs(FVector::DotProduct(T, Axis));
            if (proj > ra + rb) {
                return false;
            }
        }
    }

    return true; // No separating axis found
}

#pragma endregion

#pragma region OBB
bool UOHCollisionUtils::IsPointInOBB(const FVector& Point, const FVector& BoxCenter, const FRotator& BoxRotation,
                                     const FVector& BoxExtents) {
    const FTransform BoxTransform(BoxRotation, BoxCenter);
    const FVector LocalPoint = BoxTransform.InverseTransformPosition(Point);

    return FMath::Abs(LocalPoint.X) <= BoxExtents.X && FMath::Abs(LocalPoint.Y) <= BoxExtents.Y &&
           FMath::Abs(LocalPoint.Z) <= BoxExtents.Z;
}

FVector UOHCollisionUtils::SupportOBB(const FVector& Direction, const FVector& BoxCenter, const FRotator& BoxRotation,
                                      const FVector& BoxExtents) {
    const FVector Axes[3] = {BoxRotation.RotateVector(FVector::ForwardVector),
                             BoxRotation.RotateVector(FVector::RightVector),
                             BoxRotation.RotateVector(FVector::UpVector)};

    FVector Support = BoxCenter;

    for (int i = 0; i < 3; ++i) {
        float Sign = FMath::Sign(FVector::DotProduct(Direction, Axes[i]));
        Support += Axes[i] * (Sign * BoxExtents.Component(i));
    }

    return Support;
}

FVector UOHCollisionUtils::ClosestPointOnOBB(const FVector& Point, const FVector& BoxCenter,
                                             const FRotator& BoxRotation, const FVector& BoxExtents) {
    // Transform to local box space
    FTransform BoxTransform(BoxRotation, BoxCenter);
    FVector LocalPoint = BoxTransform.InverseTransformPosition(Point);

    // Clamp point to box extents
    FVector ClampedLocalPoint = LocalPoint;
    ClampedLocalPoint.X = FMath::Clamp(ClampedLocalPoint.X, -BoxExtents.X, BoxExtents.X);
    ClampedLocalPoint.Y = FMath::Clamp(ClampedLocalPoint.Y, -BoxExtents.Y, BoxExtents.Y);
    ClampedLocalPoint.Z = FMath::Clamp(ClampedLocalPoint.Z, -BoxExtents.Z, BoxExtents.Z);

    // Transform back to world space
    return BoxTransform.TransformPosition(ClampedLocalPoint);
}
#pragma endregion

#pragma region Minkowski
TArray<FVector> UOHCollisionUtils::ComputeMinkowskiSum(const TArray<FVector>& ShapeA, const TArray<FVector>& ShapeB) {
    TArray<FVector> Result;
    for (const FVector& A : ShapeA) {
        for (const FVector& B : ShapeB) {
            Result.Add(A + B);
        }
    }
    return Result;
}

TArray<FVector> UOHCollisionUtils::ComputeMinkowskiDifference(const TArray<FVector>& ShapeA,
                                                              const TArray<FVector>& ShapeB) {
    TArray<FVector> Result;
    for (const FVector& A : ShapeA) {
        for (const FVector& B : ShapeB) {
            Result.Add(A - B);
        }
    }
    return Result;
}

#pragma endregion

#pragma region GJK

bool UOHCollisionUtils::GJK_Intersect(const TArray<FVector>& ShapeA, const TArray<FVector>& ShapeB, int32 MaxIterations,
                                      float Tolerance) {
    if (ShapeA.Num() == 0 || ShapeB.Num() == 0) {
        return false;
    }

    // Support function for Minkowski Difference
    auto Support = [&](const FVector& Dir) -> FVector {
        FVector A = GetFarthestPointInDirection(ShapeA, Dir);
        FVector B = GetFarthestPointInDirection(ShapeB, -Dir);
        return A - B;
    };

    // Initial search direction
    FVector Dir = FVector::ForwardVector;

    // Start with a single point in the direction
    TArray<FVector> Simplex;
    Simplex.Add(Support(Dir));
    Dir = -Simplex[0]; // Search back toward origin

    for (int32 Iter = 0; Iter < MaxIterations; ++Iter) {
        const FVector NewPoint = Support(Dir);
        if (FVector::DotProduct(NewPoint, Dir) < 0.f) {
            return false; // No collision: point is outside
        }

        Simplex.Add(NewPoint);

        if (ProcessSimplex(Simplex, Dir)) {
            return true; // Collision confirmed: origin in simplex
        }
    }

    // Failed to converge
    return false;
}

bool UOHCollisionUtils::GJK_Intersect(const TArray<FVector>& ShapeA, const TArray<FVector>& ShapeB,
                                      TArray<FVector>& OutSimplex, int32 MaxIterations, float Tolerance) {
    OutSimplex.Reset();

    if (ShapeA.Num() == 0 || ShapeB.Num() == 0) {
        return false;
    }

    auto Support = [&](const FVector& Dir) -> FVector {
        FVector A = GetFarthestPointInDirection(ShapeA, Dir);
        FVector B = GetFarthestPointInDirection(ShapeB, -Dir);
        return A - B;
    };

    FVector Dir = FVector::ForwardVector;

    TArray<FVector> Simplex;
    Simplex.Add(Support(Dir));
    Dir = -Simplex[0];

    for (int32 Iter = 0; Iter < MaxIterations; ++Iter) {
        FVector NewPoint = Support(Dir);
        if (FVector::DotProduct(NewPoint, Dir) < 0.f) {
            return false;
        }

        Simplex.Add(NewPoint);

        if (ProcessSimplex(Simplex, Dir)) {
            OutSimplex = Simplex; // Pass final simplex to caller
            return true;
        }
    }

    return false;
}

bool UOHCollisionUtils::GJK_Intersect(const TArray<FVector>& ShapeA, const TArray<FVector>& ShapeB,
                                      TArray<FVector>& OutSimplex, TArray<FVector>& OutSupportA,
                                      TArray<FVector>& OutSupportB, int32 MaxIterations, float Tolerance) {
    OutSimplex.Reset();
    OutSupportA.Reset();
    OutSupportB.Reset();

    if (ShapeA.Num() == 0 || ShapeB.Num() == 0) {
        return false;
    }

    auto Support = [&](const FVector& Dir) -> FVector {
        FVector A = GetFarthestPointInDirection(ShapeA, Dir);
        FVector B = GetFarthestPointInDirection(ShapeB, -Dir);
        OutSupportA.Add(A);
        OutSupportB.Add(B);
        return A - B;
    };

    FVector Dir = FVector::ForwardVector;
    TArray<FVector> Simplex;
    Simplex.Add(Support(Dir));
    Dir = -Simplex[0];

    for (int32 Iter = 0; Iter < MaxIterations; ++Iter) {
        FVector NewPoint = Support(Dir);
        if (FVector::DotProduct(NewPoint, Dir) < 0.f) {
            return false;
        }

        Simplex.Add(NewPoint);

        if (ProcessSimplex(Simplex, Dir)) {
            OutSimplex = Simplex;
            return true;
        }
    }

    return false;
}

FVector UOHCollisionUtils::GetFarthestPointInDirection(const TArray<FVector>& Points, const FVector& Direction) {
    if (Points.Num() == 0) {
        return FVector::ZeroVector;
    }

    float MaxDot = -FLT_MAX;
    FVector FarthestPoint = Points[0];

    for (const FVector& P : Points) {
        float Dot = FVector::DotProduct(P, Direction);
        if (Dot > MaxDot) {
            MaxDot = Dot;
            FarthestPoint = P;
        }
    }

    return FarthestPoint;
}

FVector UOHCollisionUtils::TripleCrossProduct(const FVector& A, const FVector& B, const FVector& C) {
    return FVector::CrossProduct(FVector::CrossProduct(A, B), C);
}

bool UOHCollisionUtils::ProcessSimplex(TArray<FVector>& Simplex, FVector& OutDirection) {
    switch (Simplex.Num()) {
    case 2: // Line
    {
        const FVector A = Simplex[1];
        const FVector B = Simplex[0];
        const FVector AB = B - A;
        const FVector AO = -A;

        OutDirection = TripleCrossProduct(AB, AO, AB);
        return false;
    }

    case 3: // Triangle
    {
        const FVector A = Simplex[2];
        const FVector B = Simplex[1];
        const FVector C = Simplex[0];

        const FVector AB = B - A;
        const FVector AC = C - A;
        const FVector AO = -A;
        const FVector ABC = FVector::CrossProduct(AB, AC);

        if (FVector::DotProduct(FVector::CrossProduct(ABC, AC), AO) > 0) {
            Simplex = {A, C};
            OutDirection = TripleCrossProduct(AC, AO, AC);
            return false;
        }
        if (FVector::DotProduct(FVector::CrossProduct(AB, ABC), AO) > 0) {
            Simplex = {A, B};
            OutDirection = TripleCrossProduct(AB, AO, AB);
            return false;
        }
        OutDirection = ABC * (FVector::DotProduct(ABC, AO) > 0 ? 1.f : -1.f);
        return false;
    }

    case 4: // Tetrahedron
    {
        const FVector A = Simplex[3];
        const FVector B = Simplex[2];
        const FVector C = Simplex[1];
        const FVector D = Simplex[0];

        const FVector AO = -A;

        const FVector AB = B - A;
        const FVector AC = C - A;
        const FVector AD = D - A;

        const FVector ABC = FVector::CrossProduct(AB, AC);
        const FVector ACD = FVector::CrossProduct(AC, AD);
        const FVector ADB = FVector::CrossProduct(AD, AB);

        if (FVector::DotProduct(ABC, AO) > 0) {
            Simplex = {A, B, C};
            OutDirection = ABC;
            return false;
        }
        if (FVector::DotProduct(ACD, AO) > 0) {
            Simplex = {A, C, D};
            OutDirection = ACD;
            return false;
        }
        if (FVector::DotProduct(ADB, AO) > 0) {
            Simplex = {A, D, B};
            OutDirection = ADB;
            return false;
        }

        return true; // Origin is within tetrahedron
    }

    default:
        return false;
    }
}

#pragma endregion

#pragma region EPA

bool UOHCollisionUtils::EPA_PenetrationDepth(const TArray<FVector>& ShapeA, const TArray<FVector>& ShapeB,
                                             const TArray<FVector>& InitialSimplex, FVector& OutNormal,
                                             float& OutPenetrationDepth, int32 MaxIterations, float Tolerance) {
    if (InitialSimplex.Num() < 4) {
        OutNormal = FVector::ZeroVector;
        OutPenetrationDepth = 0.f;
        return false;
    }

    auto Support = [&](const FVector& Dir) -> FVector {
        FVector A = GetFarthestPointInDirection(ShapeA, Dir);
        FVector B = GetFarthestPointInDirection(ShapeB, -Dir);
        return A - B;
    };

    TArray<FVector> Vertices = InitialSimplex;

    // Faces stored as triplets of vertex indices
    TArray<TTuple<int32, int32, int32>> Faces = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};

    for (int Iter = 0; Iter < MaxIterations; ++Iter) {
        int32 ClosestFaceIndex = -1;
        float MinDistance = FLT_MAX;
        FVector ClosestNormal = FVector::ZeroVector;

        // Find the face closest to the origin
        for (int32 i = 0; i < Faces.Num(); ++i) {
            const auto& [A, B, C] = Faces[i];
            const FVector& PA = Vertices[A];
            const FVector& PB = Vertices[B];
            const FVector& PC = Vertices[C];

            FVector Normal = FVector::CrossProduct(PB - PA, PC - PA).GetSafeNormal();
            float Distance = FVector::DotProduct(Normal, PA);

            if (Distance >= 0 && Distance < MinDistance) {
                MinDistance = Distance;
                ClosestFaceIndex = i;
                ClosestNormal = Normal;
            }
        }

        if (ClosestFaceIndex == -1) {
            break;
        }

        FVector NewPoint = Support(ClosestNormal);
        float DistanceToNewPoint = FVector::DotProduct(ClosestNormal, NewPoint);

        // Convergence check
        if (DistanceToNewPoint - MinDistance < Tolerance) {
            OutNormal = ClosestNormal;
            OutPenetrationDepth = MinDistance;
            return true;
        }

        // Add new point
        int32 NewIndex = Vertices.Add(NewPoint);

        // Rebuild visible faces
        TArray<TTuple<int32, int32, int32>> NewFaces;

        for (const auto& Face : Faces) {
            const auto& [A, B, C] = Face;
            const FVector& PA = Vertices[A];
            const FVector& Normal = FVector::CrossProduct(Vertices[B] - PA, Vertices[C] - PA).GetSafeNormal();

            if (FVector::DotProduct(Normal, PA - NewPoint) < 0) {
                // Face visible to new point — create 3 new faces around edges
                NewFaces.Add({A, B, NewIndex});
                NewFaces.Add({B, C, NewIndex});
                NewFaces.Add({C, A, NewIndex});
            } else {
                NewFaces.Add(Face);
            }
        }

        Faces = MoveTemp(NewFaces);
    }

    // Failed to converge
    OutNormal = FVector::ZeroVector;
    OutPenetrationDepth = 0.f;
    return false;
}
#pragma endregion

#pragma endregion

#pragma region ShapeGeneration

TArray<FVector> UOHCollisionUtils::GenerateBoxHull(const FVector& Center, const FVector& Extents,
                                                   const FRotator& Rotation) {
    const FTransform BoxTransform(Rotation, Center);

    const FVector LocalCorners[8] = {
        FVector(-Extents.X, -Extents.Y, -Extents.Z), FVector(Extents.X, -Extents.Y, -Extents.Z),
        FVector(Extents.X, Extents.Y, -Extents.Z),   FVector(-Extents.X, Extents.Y, -Extents.Z),
        FVector(-Extents.X, -Extents.Y, Extents.Z),  FVector(Extents.X, -Extents.Y, Extents.Z),
        FVector(Extents.X, Extents.Y, Extents.Z),    FVector(-Extents.X, Extents.Y, Extents.Z)};

    TArray<FVector> WorldCorners;
    for (int32 i = 0; i < 8; ++i) {
        WorldCorners.Add(BoxTransform.TransformPosition(LocalCorners[i]));
    }

    return WorldCorners;
}

TArray<FVector> UOHCollisionUtils::GenerateSphereHull(const FVector& Center, float Radius, int32 Subdivisions) {
    TArray<FVector> Points;
    const int32 LatitudeSteps = FMath::Max(2, Subdivisions * 2);
    const int32 LongitudeSteps = FMath::Max(3, Subdivisions * 4);

    for (int32 Lat = 0; Lat <= LatitudeSteps; ++Lat) {
        float Theta = PI * Lat / LatitudeSteps;
        float SinTheta = FMath::Sin(Theta);
        float CosTheta = FMath::Cos(Theta);

        for (int32 Lon = 0; Lon < LongitudeSteps; ++Lon) {
            float Phi = 2.f * PI * Lon / LongitudeSteps;
            float SinPhi = FMath::Sin(Phi);
            float CosPhi = FMath::Cos(Phi);

            FVector Point(Radius * SinTheta * CosPhi, Radius * SinTheta * SinPhi, Radius * CosTheta);

            Points.Add(Center + Point);
        }
    }

    return Points;
}

TArray<FVector> UOHCollisionUtils::GenerateCapsuleHull(const FVector& Center, const FRotator& Rotation,
                                                       float HalfHeight, float Radius, int32 HemisphereResolution,
                                                       int32 RingResolution) {
    TArray<FVector> Points;

    // Clamp resolution
    HemisphereResolution = FMath::Max(2, HemisphereResolution);
    RingResolution = FMath::Max(4, RingResolution);

    const FVector Up = Rotation.RotateVector(FVector::UpVector);
    const FVector Right = Rotation.RotateVector(FVector::RightVector);
    const FVector Forward = Rotation.RotateVector(FVector::ForwardVector);

    // Generate hemisphere arcs (top and bottom)
    for (int32 LatStep = 0; LatStep <= HemisphereResolution; ++LatStep) {
        float Theta = (PI / 2.0f) * (static_cast<float>(LatStep) / HemisphereResolution);
        float Y = FMath::Cos(Theta);
        float R = FMath::Sin(Theta);

        for (int32 LonStep = 0; LonStep < RingResolution; ++LonStep) {
            float Phi = (2.0f * PI * LonStep) / RingResolution;
            float X = FMath::Cos(Phi);
            float Z = FMath::Sin(Phi);

            FVector Local = FVector(X * R, Z * R, Y) * Radius;

            // Top hemisphere
            FVector Top = Center + (Up * HalfHeight) + Rotation.RotateVector(Local);
            Points.Add(Top);

            // Bottom hemisphere (inverted Z)
            FVector Bottom = Center - (Up * HalfHeight) + Rotation.RotateVector(FVector(X * R, Z * R, -Y) * Radius);
            Points.Add(Bottom);
        }
    }

    // Generate equator ring
    for (int32 i = 0; i < RingResolution; ++i) {
        float Angle = (2.0f * PI * i) / RingResolution;
        FVector Local = FVector(FMath::Cos(Angle), FMath::Sin(Angle), 0.f) * Radius;
        FVector RingPoint = Center + Rotation.RotateVector(Local);
        Points.Add(RingPoint);
    }

    return Points;
}

TArray<FVector> UOHCollisionUtils::ComputeConvexHull(const TArray<FVector>& InputPoints) {
    TArray<FVector> Result;
    ComputeConvexHull_Internal(InputPoints, Result);
    return Result;
}

void UOHCollisionUtils::ComputeConvexHull_Internal(const TArray<FVector>& InPoints, TArray<FVector>& OutHullVertices,
                                                   int32 MaxIterations, float Epsilon) {
    if (InPoints.Num() < 4) {
        return;
    }

    OutHullVertices.Empty();

    // Step 1: Find extreme X
    int32 MinX = 0, MaxX = 0;
    for (int32 i = 1; i < InPoints.Num(); ++i) {
        if (InPoints[i].X < InPoints[MinX].X) {
            MinX = i;
        }
        if (InPoints[i].X > InPoints[MaxX].X) {
            MaxX = i;
        }
    }
    FVector A = InPoints[MinX];
    FVector B = InPoints[MaxX];

    // Step 2: Farthest from line AB
    int32 MaxDistIdx = -1;
    float MaxArea = -1.f;
    for (int32 i = 0; i < InPoints.Num(); ++i) {
        if (i == MinX || i == MaxX) {
            continue;
        }
        FVector AB = B - A;
        FVector AP = InPoints[i] - A;
        float Area = FVector::CrossProduct(AB, AP).Size();
        if (Area > MaxArea) {
            MaxArea = Area;
            MaxDistIdx = i;
        }
    }
    if (MaxDistIdx == -1) {
        return;
    }
    FVector C = InPoints[MaxDistIdx];

    // Step 3: Farthest from triangle ABC
    FVector Normal = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
    int32 MaxDepthIdx = -1;
    float MaxDist = -1.f;
    for (int32 i = 0; i < InPoints.Num(); ++i) {
        if (i == MinX || i == MaxX || i == MaxDistIdx) {
            continue;
        }
        float Dist = FMath::Abs(FVector::DotProduct(InPoints[i] - A, Normal));
        if (Dist > MaxDist) {
            MaxDist = Dist;
            MaxDepthIdx = i;
        }
    }
    if (MaxDepthIdx == -1) {
        return;
    }
    FVector D = InPoints[MaxDepthIdx];

    // Add the tetrahedron corners as the basic convex hull
    OutHullVertices.Add(A);
    OutHullVertices.Add(B);
    OutHullVertices.Add(C);
    OutHullVertices.Add(D);

    // Can stop here Short of full face triangulation later
}

FVector UOHCollisionUtils::ComputeAverageContactPoint(const TArray<FVector>& SupportA,
                                                      const TArray<FVector>& SupportB) {
    FVector Sum = FVector::ZeroVector;
    int32 Count = FMath::Min(SupportA.Num(), SupportB.Num());

    for (int32 i = 0; i < Count; ++i) {
        Sum += (SupportA[i] + SupportB[i]) * 0.5f;
    }

    return (Count > 0) ? (Sum / Count) : FVector::ZeroVector;
}

#pragma endregion

#pragma region MeshData

TArray<FVector> UOHCollisionUtils::GetStaticMeshComponentVertices(UStaticMeshComponent* StaticComp, bool bWorldSpace) {
    TArray<FVector> Points;
    if (!StaticComp)
        return Points;

    UStaticMesh* StaticMesh = StaticComp->GetStaticMesh();
    if (!StaticMesh || !StaticMesh->GetRenderData() || StaticMesh->GetRenderData()->LODResources.Num() == 0)
        return Points;

    const FStaticMeshLODResources& LODResource = StaticMesh->GetRenderData()->LODResources[0];
    const FPositionVertexBuffer& VertexBuffer = LODResource.VertexBuffers.PositionVertexBuffer;

    for (uint32 i = 0; i < VertexBuffer.GetNumVertices(); ++i) {
        FVector LocalPos = FVector(VertexBuffer.VertexPosition(i));
        Points.Add(bWorldSpace ? StaticComp->GetComponentTransform().TransformPosition(LocalPos) : LocalPos);
    }
    return Points;
}

TArray<FVector> UOHCollisionUtils::GetSkeletalMeshComponentVertices(USkeletalMeshComponent* SkelComp,
                                                                    bool bWorldSpace) {
    TArray<FVector> Points;
    if (!SkelComp)
        return Points;

    USkeletalMesh* SkelMesh = SkelComp->GetSkeletalMeshAsset();
    if (!SkelMesh)
        return Points;

    const FSkeletalMeshRenderData* RenderData = SkelComp->GetSkeletalMeshRenderData();
    if (!RenderData || RenderData->LODRenderData.Num() == 0)
        return Points;

    const FSkeletalMeshLODRenderData& LODData = RenderData->LODRenderData[0];
    const FPositionVertexBuffer& VertexBuffer = LODData.StaticVertexBuffers.PositionVertexBuffer;

    for (uint32 i = 0; i < VertexBuffer.GetNumVertices(); ++i) {
        FVector LocalPos = FVector(VertexBuffer.VertexPosition(i));
        Points.Add(bWorldSpace ? SkelComp->GetComponentTransform().TransformPosition(LocalPos) : LocalPos);
    }
    return Points;
}

TArray<FVector> UOHCollisionUtils::GetSplineMeshComponentVertices(USplineMeshComponent* SplineComp, bool bWorldSpace) {
    TArray<FVector> Points;
    if (!SplineComp)
        return Points;

    const int32 NumSamples = 20;

    FVector StartPos = SplineComp->GetStartPosition();
    FVector StartTangent = SplineComp->GetStartTangent();
    FVector EndPos = SplineComp->GetEndPosition();
    FVector EndTangent = SplineComp->GetEndTangent();

    for (int32 i = 0; i <= NumSamples; ++i) {
        float T = static_cast<float>(i) / static_cast<float>(NumSamples);
        FVector Point = FMath::CubicInterp(StartPos, StartTangent, EndPos, EndTangent, T);
        Points.Add(bWorldSpace ? SplineComp->GetComponentTransform().TransformPosition(Point) : Point);
    }
    return Points;
}

TArray<FVector> UOHCollisionUtils::GetMeshComponentVertices(UMeshComponent* MeshComp, bool bWorldSpace) {
    if (UStaticMeshComponent* StaticComp = Cast<UStaticMeshComponent>(MeshComp))
        return GetStaticMeshComponentVertices(StaticComp, bWorldSpace);

    if (USkeletalMeshComponent* SkelComp = Cast<USkeletalMeshComponent>(MeshComp))
        return GetSkeletalMeshComponentVertices(SkelComp, bWorldSpace);

    if (USplineMeshComponent* SplineComp = Cast<USplineMeshComponent>(MeshComp))
        return GetSplineMeshComponentVertices(SplineComp, bWorldSpace);

    // Add more engine types here if needed (all must be Engine module).

    return {};
}

void UOHCollisionUtils::GetMeshComponentAABB(UMeshComponent* MeshComp, FVector& OutCenter, FVector& OutExtents,
                                             bool bWorldSpace) {
    TArray<FVector> Points = GetMeshComponentVertices(MeshComp, bWorldSpace);
    if (Points.Num() == 0) {
        OutCenter = FVector::ZeroVector;
        OutExtents = FVector::ZeroVector;
        return;
    }

    FVector Min = Points[0];
    FVector Max = Points[0];
    for (const FVector& Pt : Points) {
        Min = Min.ComponentMin(Pt);
        Max = Max.ComponentMax(Pt);
    }

    OutCenter = (Min + Max) * 0.5f;
    OutExtents = (Max - Min) * 0.5f;
}

void UOHCollisionUtils::GetMeshComponentOBB(UMeshComponent* MeshComp, FVector& OutCenter, FRotator& OutRotation,
                                            FVector& OutExtents, bool bWorldSpace) {
    TArray<FVector> Points = GetMeshComponentVertices(MeshComp, bWorldSpace);
    if (Points.Num() == 0) {
        OutCenter = FVector::ZeroVector;
        OutRotation = FRotator::ZeroRotator;
        OutExtents = FVector::ZeroVector;
        return;
    }

    // Use component rotation for OBB axes
    FTransform MeshTransform = MeshComp->GetComponentTransform();
    FTransform OBBTransform = bWorldSpace ? MeshTransform : FTransform::Identity;
    FTransform OBBInv = OBBTransform.Inverse();

    FVector Min(FLT_MAX), Max(-FLT_MAX);
    for (const FVector& Pt : Points) {
        FVector LocalPt = OBBInv.TransformPosition(Pt);
        Min = Min.ComponentMin(LocalPt);
        Max = Max.ComponentMax(LocalPt);
    }

    FVector OBBLocalCenter = (Min + Max) * 0.5f;
    OutCenter = OBBTransform.TransformPosition(OBBLocalCenter);
    OutRotation = OBBTransform.GetRotation().Rotator();
    OutExtents = (Max - Min) * 0.5f;
}

#pragma endregion

#pragma region Debug

void UOHCollisionUtils::DebugDrawSATBox(const UObject* WorldContextObject, const FVector& Center,
                                        const FRotator& Rotation, const FVector& Extents, const FColor Color,
                                        const float Duration, const float LineThickness) {
    if (!WorldContextObject) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    const FTransform BoxTransform(Rotation, Center);

    const FVector LocalCorners[8] = {
        FVector(-Extents.X, -Extents.Y, -Extents.Z), FVector(Extents.X, -Extents.Y, -Extents.Z),
        FVector(Extents.X, Extents.Y, -Extents.Z),   FVector(-Extents.X, Extents.Y, -Extents.Z),
        FVector(-Extents.X, -Extents.Y, Extents.Z),  FVector(Extents.X, -Extents.Y, Extents.Z),
        FVector(Extents.X, Extents.Y, Extents.Z),    FVector(-Extents.X, Extents.Y, Extents.Z)};

    FVector WorldCorners[8];
    for (int i = 0; i < 8; ++i) {
        WorldCorners[i] = BoxTransform.TransformPosition(LocalCorners[i]);
    }

    // Bottom square
    DrawDebugLine(World, WorldCorners[0], WorldCorners[1], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[1], WorldCorners[2], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[2], WorldCorners[3], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[3], WorldCorners[0], Color, false, Duration, 0, LineThickness);

    // Top square
    DrawDebugLine(World, WorldCorners[4], WorldCorners[5], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[5], WorldCorners[6], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[6], WorldCorners[7], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[7], WorldCorners[4], Color, false, Duration, 0, LineThickness);

    // Vertical edges
    for (int i = 0; i < 4; ++i) {
        DrawDebugLine(World, WorldCorners[i], WorldCorners[i + 4], Color, false, Duration, 0, LineThickness);
    }
}

void UOHCollisionUtils::DebugDrawAABB(const UObject* WorldContextObject, const FVector& Center, const FVector& Extents,
                                      const FColor Color, float Duration, float LineThickness) {
    if (!WorldContextObject) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    DrawDebugBox(World, Center, Extents, FQuat::Identity, Color, false, Duration, 0, LineThickness);

    DrawDebugString(World, Center + FVector(0, 0, Extents.Z + 10.f), TEXT("AABB"), nullptr, Color, Duration);
}

void UOHCollisionUtils::DrawDebugOBB(const UObject* WorldContextObject, const FVector& Center, const FRotator& Rotation,
                                     const FVector& Extents, const FColor Color, float Duration, float LineThickness) {
    if (!WorldContextObject) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    FTransform BoxTransform(Rotation, Center);

    const FVector LocalCorners[8] = {
        FVector(-Extents.X, -Extents.Y, -Extents.Z), FVector(Extents.X, -Extents.Y, -Extents.Z),
        FVector(Extents.X, Extents.Y, -Extents.Z),   FVector(-Extents.X, Extents.Y, -Extents.Z),
        FVector(-Extents.X, -Extents.Y, Extents.Z),  FVector(Extents.X, -Extents.Y, Extents.Z),
        FVector(Extents.X, Extents.Y, Extents.Z),    FVector(-Extents.X, Extents.Y, Extents.Z)};

    FVector WorldCorners[8];
    for (int i = 0; i < 8; ++i) {
        WorldCorners[i] = BoxTransform.TransformPosition(LocalCorners[i]);
    }

    // Bottom
    DrawDebugLine(World, WorldCorners[0], WorldCorners[1], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[1], WorldCorners[2], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[2], WorldCorners[3], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[3], WorldCorners[0], Color, false, Duration, 0, LineThickness);

    // Top
    DrawDebugLine(World, WorldCorners[4], WorldCorners[5], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[5], WorldCorners[6], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[6], WorldCorners[7], Color, false, Duration, 0, LineThickness);
    DrawDebugLine(World, WorldCorners[7], WorldCorners[4], Color, false, Duration, 0, LineThickness);

    // Vertical edges
    for (int i = 0; i < 4; ++i) {
        DrawDebugLine(World, WorldCorners[i], WorldCorners[i + 4], Color, false, Duration, 0, LineThickness);
    }
}

void UOHCollisionUtils::DrawDebugSimplex(const UObject* WorldContextObject, const TArray<FVector>& SimplexPoints,
                                         const FColor Color, float Duration, float PointSize, float LineThickness) {
    if (!WorldContextObject || SimplexPoints.Num() == 0) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    // Draw each point
    for (int32 i = 0; i < SimplexPoints.Num(); ++i) {
        DrawDebugSphere(World, SimplexPoints[i], PointSize, 8, Color, false, Duration);
        DrawDebugString(World, SimplexPoints[i] + FVector(0, 0, 10), FString::Printf(TEXT("P%d"), i), nullptr, Color,
                        Duration, false);
    }

    // Draw lines (depends on simplex size)
    if (SimplexPoints.Num() == 2) {
        DrawDebugLine(World, SimplexPoints[0], SimplexPoints[1], Color, false, Duration, 0, LineThickness);
    } else if (SimplexPoints.Num() == 3) {
        DrawDebugLine(World, SimplexPoints[0], SimplexPoints[1], Color, false, Duration, 0, LineThickness);
        DrawDebugLine(World, SimplexPoints[1], SimplexPoints[2], Color, false, Duration, 0, LineThickness);
        DrawDebugLine(World, SimplexPoints[2], SimplexPoints[0], Color, false, Duration, 0, LineThickness);
    } else if (SimplexPoints.Num() == 4) {
        // Tetrahedron: draw all edges
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                DrawDebugLine(World, SimplexPoints[i], SimplexPoints[j], Color, false, Duration, 0, LineThickness);
            }
        }
    }
}

void UOHCollisionUtils::DrawDebugPointCloud(const UObject* WorldContextObject, const TArray<FVector>& Points,
                                            const FColor Color, float PointSize, float Duration) {
    if (!WorldContextObject || Points.Num() == 0) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    for (int32 i = 0; i < Points.Num(); ++i) {
        DrawDebugPoint(World, Points[i], PointSize, Color, false, Duration);
    }
}

void UOHCollisionUtils::DrawDebugConvexHull(const UObject* WorldContextObject, const TArray<FVector>& HullPoints,
                                            const FColor Color, float PointSize, float LineThickness, float Duration,
                                            bool bDrawEdgesIfSafe) {
    if (!WorldContextObject || HullPoints.Num() == 0) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    const int32 Count = HullPoints.Num();

    // Draw labeled points
    for (int32 i = 0; i < Count; ++i) {
        DrawDebugSphere(World, HullPoints[i], PointSize, 8, Color, false, Duration);
        DrawDebugString(World, HullPoints[i] + FVector(0, 0, PointSize * 1.5f), FString::Printf(TEXT("H%d"), i),
                        nullptr, Color, Duration);
    }

    if (!bDrawEdgesIfSafe) {
        return;
    }

    // Draw valid edge connections (only for small sets)
    if (Count == 4) {
        // Tetrahedron — 6 edges
        static const int32 Indices[][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

        for (const auto& Pair : Indices) {
            DrawDebugLine(World, HullPoints[Pair[0]], HullPoints[Pair[1]], Color, false, Duration, 0, LineThickness);
        }
    } else if (Count <= 8) {
        // Small hull — draw full mesh
        for (int32 i = 0; i < Count; ++i) {
            for (int32 j = i + 1; j < Count; ++j) {
                DrawDebugLine(World, HullPoints[i], HullPoints[j], Color, false, Duration, 0, LineThickness);
            }
        }
    }
    // Else: skip drawing pairwise connections
}

void UOHCollisionUtils::DrawDebugConvexMeshFromFaces(const UObject* WorldContextObject, const TArray<FVector>& Vertices,
                                                     const TArray<int32>& Indices, const FColor Color,
                                                     float LineThickness, float Duration) {
    if (!WorldContextObject || Vertices.Num() == 0 || Indices.Num() == 0) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    for (int32 i = 0; i + 2 < Indices.Num(); i += 3) {
        const FVector& A = Vertices[Indices[i]];
        const FVector& B = Vertices[Indices[i + 1]];
        const FVector& C = Vertices[Indices[i + 2]];

        DrawDebugLine(World, A, B, Color, false, Duration, 0, LineThickness);
        DrawDebugLine(World, B, C, Color, false, Duration, 0, LineThickness);
        DrawDebugLine(World, C, A, Color, false, Duration, 0, LineThickness);

        // Optional: face normal
        FVector FaceCenter = (A + B + C) / 3.0f;
        FVector Normal = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
        DrawDebugLine(World, FaceCenter, FaceCenter + Normal * 10.0f, FColor::Cyan, false, Duration, 0, 1.0f);
    }
}

void UOHCollisionUtils::ComputeConvexHullWithIndices(const TArray<FVector>& InputPoints, TArray<FVector>& OutVertices,
                                                     TArray<int32>& OutTriangleIndices) {
    OutVertices.Empty();
    OutTriangleIndices.Empty();

    if (InputPoints.Num() < 4) {
        return;
    }

    // Step 1: Remove duplicates (optional)
    TSet<FVector> UniquePoints(InputPoints);
    TArray<FVector> Points = UniquePoints.Array();

    // Step 2: Generate triangle faces from all point triplets (brute force)
    constexpr float MinArea = 1e-4f;

    for (int32 i = 0; i < Points.Num(); ++i) {
        for (int32 j = i + 1; j < Points.Num(); ++j) {
            for (int32 k = j + 1; k < Points.Num(); ++k) {
                const FVector& A = Points[i];
                const FVector& B = Points[j];
                const FVector& C = Points[k];

                const FVector Normal = FVector::CrossProduct(B - A, C - A);
                if (Normal.SizeSquared() < MinArea) {
                    continue; // Skip degenerate triangle
                }

                bool bValid = true;
                const FVector PlaneNormal = Normal.GetSafeNormal();
                const float PlaneD = FVector::DotProduct(PlaneNormal, A);

                for (int32 p = 0; p < Points.Num(); ++p) {
                    if (p == i || p == j || p == k) {
                        continue;
                    }

                    const float Side = FVector::DotProduct(PlaneNormal, Points[p]) - PlaneD;
                    if (Side > KINDA_SMALL_NUMBER) {
                        bValid = false;
                        break;
                    }
                }

                if (bValid) {
                    // Store unique vertex indices
                    int32 BaseIndex = OutVertices.Num();
                    OutVertices.Add(A);
                    OutVertices.Add(B);
                    OutVertices.Add(C);

                    OutTriangleIndices.Add(BaseIndex);
                    OutTriangleIndices.Add(BaseIndex + 1);
                    OutTriangleIndices.Add(BaseIndex + 2);
                }
            }
        }
    }
}

/*
void UOHCollisionUtils::ComputeConvexHullWithIndices(
    const TArray<FVector>& InputPoints,
    TArray<FVector>& OutVertices,
    TArray<int32>& OutTriangleIndices)
{
    OutVertices.Empty();
    OutTriangleIndices.Empty();

    if (InputPoints.Num() < 4) return;

    // Step 1: Find extremes on X
    int32 MinX = 0, MaxX = 0;
    for (int32 i = 1; i < InputPoints.Num(); ++i)
    {
        if (InputPoints[i].X < InputPoints[MinX].X) MinX = i;
        if (InputPoints[i].X > InputPoints[MaxX].X) MaxX = i;
    }
    FVector A = InputPoints[MinX];
    FVector B = InputPoints[MaxX];

    // Step 2: Farthest from AB
    int32 MaxAreaIdx = -1;
    float MaxArea = -1.f;
    for (int32 i = 0; i < InputPoints.Num(); ++i)
    {
        if (i == MinX || i == MaxX) continue;
        FVector AreaVec = FVector::CrossProduct(B - A, InputPoints[i] - A);
        float Area = AreaVec.Size();
        if (Area > MaxArea)
        {
            MaxArea = Area;
            MaxAreaIdx = i;
        }
    }
    if (MaxAreaIdx == -1) return;
    FVector C = InputPoints[MaxAreaIdx];

    // Step 3: Farthest from triangle plane
    FVector Normal = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
    int32 MaxDepthIdx = -1;
    float MaxDist = -1.f;
    for (int32 i = 0; i < InputPoints.Num(); ++i)
    {
        if (i == MinX || i == MaxX || i == MaxAreaIdx) continue;
        float Dist = FMath::Abs(FVector::DotProduct(InputPoints[i] - A, Normal));
        if (Dist > MaxDist)
        {
            MaxDist = Dist;
            MaxDepthIdx = i;
        }
    }
    if (MaxDepthIdx == -1) return;
    FVector D = InputPoints[MaxDepthIdx];

    // Add tetrahedron vertices
    OutVertices.Add(A); // 0
    OutVertices.Add(B); // 1
    OutVertices.Add(C); // 2
    OutVertices.Add(D); // 3

    // Build 4 triangle faces (wound outward)
    OutTriangleIndices.Append({0, 1, 2}); // ABC
    OutTriangleIndices.Append({0, 3, 1}); // ADB
    OutTriangleIndices.Append({0, 2, 3}); // ACD
    OutTriangleIndices.Append({1, 3, 2}); // BDC
}
*/
void UOHCollisionUtils::DebugConvexIntersectionCheck(const UObject* WorldContextObject, const TArray<FVector>& PointsA,
                                                     const TArray<FVector>& PointsB, const FColor ColorA,
                                                     const FColor ColorB, float Duration) {
    if (!WorldContextObject || PointsA.Num() == 0 || PointsB.Num() == 0) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    // Step 1: Compute convex hulls
    TArray<FVector> HullA = ComputeConvexHull(PointsA);
    TArray<FVector> HullB = ComputeConvexHull(PointsB);

    // Step 2: GJK intersection
    bool bOverlaps = GJK_Intersect(HullA, HullB);

    // Step 3: EPA (if overlap)
    FVector Normal;
    float Depth = 0.f;
    bool bEPA = false;
    if (bOverlaps) {
        // Try EPA using last simplex from GJK (or fall back to best guess tetrahedron)
        // For now, just use first 4 from hull
        TArray<FVector> InitialSimplex;
        for (int32 i = 0; i < 4 && i < HullA.Num(); ++i) {
            InitialSimplex.Add(HullA[i] - HullB[i % HullB.Num()]); // crude difference
        }

        bEPA = EPA_PenetrationDepth(HullA, HullB, InitialSimplex, Normal, Depth);
    }

    // Step 4: Draw hulls
    DrawDebugConvexHull(WorldContextObject, HullA, ColorA, 6.0f, 1.5f, Duration);
    DrawDebugConvexHull(WorldContextObject, HullB, ColorB, 6.0f, 1.5f, Duration);

    // Step 5: Draw penetration vector if found
    if (bEPA && Depth > 0.f) {
        FVector Midpoint = (HullA[0] + HullB[0]) * 0.5f;
        DrawDebugLine(World, Midpoint, Midpoint + Normal * Depth, FColor::Red, false, Duration, 0, 3.0f);
        DrawDebugString(World, Midpoint + Normal * Depth * 0.5f, FString::Printf(TEXT("Penetration: %.2f"), Depth),
                        nullptr, FColor::Red, Duration);
    } else {
        DrawDebugString(World, (HullA[0] + HullB[0]) * 0.5f, bOverlaps ? TEXT("Overlap Detected") : TEXT("No Overlap"),
                        nullptr, bOverlaps ? FColor::Green : FColor::White, Duration);
    }
}

void UOHCollisionUtils::DrawConvexHullFromMotionHistory(const UObject* WorldContextObject,
                                                        const TArray<FVector>& HistoryPoints, const FColor HullColor,
                                                        float Duration) {
    if (!WorldContextObject || HistoryPoints.Num() < 4) {
        return;
    }

    TArray<FVector> Hull;
    TArray<int32> Indices;
    ComputeConvexHullWithIndices(HistoryPoints, Hull, Indices);

    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::DrawDebugStrikeArc(const UObject* WorldContextObject, const FVector& Origin,
                                           const FVector& Forward, float Radius, float ArcAngleDegrees, FColor Color,
                                           float Duration, bool bPersistentLines, int32 Segments) {
    if (!WorldContextObject || Radius <= 0.f || Segments <= 1) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    const FVector ForwardFlat = FVector(Forward.X, Forward.Y, 0.f).GetSafeNormal();
    const float HalfAngleRad = FMath::DegreesToRadians(ArcAngleDegrees * 0.5f);
    const float AngleStep = (2.f * HalfAngleRad) / Segments;

    // Start direction
    const FRotator StartRot = ForwardFlat.Rotation().Add(0.f, -ArcAngleDegrees * 0.5f, 0.f);
    FVector LastPoint = Origin + StartRot.Vector() * Radius;

    for (int32 i = 1; i <= Segments; ++i) {
        const float Angle = -HalfAngleRad + i * AngleStep;
        const FVector Dir = ForwardFlat.RotateAngleAxis(FMath::RadiansToDegrees(Angle), FVector::UpVector);
        const FVector Point = Origin + Dir * Radius;

        DrawDebugLine(World, LastPoint, Point, Color, bPersistentLines, Duration, 0, 1.5f);
        LastPoint = Point;
    }

    // Optional center line
    DrawDebugLine(World, Origin, Origin + ForwardFlat * Radius, Color, bPersistentLines, Duration, 0, 2.0f);
}

void UOHCollisionUtils::DrawDebugCone(const UObject* WorldContextObject, const FVector& Origin,
                                      const FVector& Direction, float Length, float ConeAngleDegrees,
                                      const FColor Color, int32 Segments, float Duration, bool bPersistent) {
    if (!WorldContextObject) {
        return;
    }
    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    const float AngleRad = FMath::DegreesToRadians(ConeAngleDegrees * 0.5f);
    const FVector Forward = Direction.GetSafeNormal();
    const FVector Right = FVector::CrossProduct(Forward, FVector::UpVector).GetSafeNormal();
    const FVector Up = FVector::CrossProduct(Right, Forward);

    for (int32 i = 0; i < Segments; ++i) {
        float Theta = i * (2 * PI / Segments);
        FVector LocalDir = FMath::Cos(AngleRad) * Forward +
                           FMath::Sin(AngleRad) * (FMath::Cos(Theta) * Right + FMath::Sin(Theta) * Up);
        FVector End = Origin + LocalDir * Length;

        DrawDebugLine(World, Origin, End, Color, bPersistent, Duration, 0, 1.0f);
    }
}

void UOHCollisionUtils::DrawDebugTrajectory(const UObject* WorldContextObject, const FVector& Start,
                                            const FVector& Velocity, float StepSize, int32 MaxSteps, const FColor Color,
                                            float Duration, bool bPersistent, FVector Gravity) {
    if (!WorldContextObject) {
        return;
    }
    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    FVector Pos = Start;
    FVector Vel = Velocity;

    for (int32 i = 0; i < MaxSteps; ++i) {
        FVector NextVel = Vel + Gravity * StepSize;
        FVector NextPos = Pos + Vel * StepSize;

        DrawDebugLine(World, Pos, NextPos, Color, bPersistent, Duration, 0, 1.5f);

        Pos = NextPos;
        Vel = NextVel;
    }
}

void UOHCollisionUtils::DrawDebugConvexSweep(const UObject* WorldContextObject, const TArray<FVector>& ShapePoints,
                                             const FVector& Velocity, float DeltaTime, const FColor StartColor,
                                             const FColor EndColor, float Duration) {
    if (!WorldContextObject || ShapePoints.Num() == 0) {
        return;
    }

    TArray<FVector> OffsetPoints;
    for (const FVector& P : ShapePoints) {
        OffsetPoints.Add(P + Velocity * DeltaTime);
    }

    DrawDebugConvexHull(WorldContextObject, ShapePoints, StartColor, 6.f, 1.0f, Duration, true);
    DrawDebugConvexHull(WorldContextObject, OffsetPoints, EndColor, 6.f, 1.0f, Duration, true);
}

void UOHCollisionUtils::DrawDebugConvexFaceNormals(const UObject* WorldContextObject, const TArray<FVector>& Vertices,
                                                   const TArray<int32>& Indices, float NormalLength, FColor Color,
                                                   float Duration, float Thickness) {
    if (!WorldContextObject || Vertices.Num() == 0 || Indices.Num() % 3 != 0) {
        return;
    }

    UWorld* World = WorldContextObject->GetWorld();
    if (!World) {
        return;
    }

    for (int32 i = 0; i < Indices.Num(); i += 3) {
        const FVector& A = Vertices[Indices[i]];
        const FVector& B = Vertices[Indices[i + 1]];
        const FVector& C = Vertices[Indices[i + 2]];

        const FVector FaceCenter = (A + B + C) / 3.f;
        const FVector Normal = FVector::CrossProduct(B - A, C - A).GetSafeNormal();

        DrawDebugLine(World, FaceCenter, FaceCenter + Normal * NormalLength, Color, false, Duration, 0, Thickness);
        DrawDebugPoint(World, FaceCenter, 6.f, Color, false, Duration);
    }
}

#pragma endregion

#pragma region Tests

void UOHCollisionUtils::GenerateConvexHullFromBoxBounds(const UObject* WorldContextObject, const AActor* SourceActor,
                                                        const FColor HullColor, float Duration) {
    if (!WorldContextObject || !SourceActor) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    FBox Bounds = SourceActor->GetComponentsBoundingBox();
    FVector Center = Bounds.GetCenter();
    FVector Extents = Bounds.GetExtent();
    FRotator Rotation = SourceActor->GetActorRotation();

    TArray<FVector> BoxPoints = GenerateBoxHull(Center, Extents, Rotation);
    TArray<FVector> Hull;
    TArray<int32> Indices;

    ComputeConvexHullWithIndices(BoxPoints, Hull, Indices);
    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::GenerateConvexHullFromSockets(const UObject* WorldContextObject,
                                                      USkeletalMeshComponent* MeshComp,
                                                      const TArray<FName>& SocketNames, const FColor HullColor,
                                                      float Duration) {
    if (!WorldContextObject || !MeshComp || SocketNames.Num() == 0) {
        return;
    }

    TArray<FVector> Points;
    for (const FName& Socket : SocketNames) {
        if (MeshComp->DoesSocketExist(Socket)) {
            Points.Add(MeshComp->GetSocketLocation(Socket));
        }
    }

    if (Points.Num() < 4) {
        return;
    }

    TArray<FVector> Hull;
    TArray<int32> Indices;

    ComputeConvexHullWithIndices(Points, Hull, Indices);
    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::GenerateConvexHullFromImpactPoints(const UObject* WorldContextObject,
                                                           const TArray<FVector>& ImpactPoints, const FColor HullColor,
                                                           float Duration) {
    if (!WorldContextObject || ImpactPoints.Num() < 4) {
        return;
    }

    TArray<FVector> Hull;
    TArray<int32> Indices;

    ComputeConvexHullWithIndices(ImpactPoints, Hull, Indices);
    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::GenerateConvexHullFromMotionTrail(const UObject* WorldContextObject,
                                                          USceneComponent* TargetComponent, int32 NumSamples,
                                                          float SimulatedDuration, const FColor HullColor,
                                                          float Duration) {
    if (!WorldContextObject || !TargetComponent || NumSamples < 2) {
        return;
    }

    TArray<FVector> TrailPoints = SampleMotionPoints(TargetComponent, NumSamples, SimulatedDuration);
    if (TrailPoints.Num() < 4) {
        return;
    }

    TArray<FVector> Hull;
    TArray<int32> Indices;

    ComputeConvexHullWithIndices(TrailPoints, Hull, Indices);
    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::GenerateConvexHullFromBoneCluster(const UObject* WorldContextObject,
                                                          USkeletalMeshComponent* MeshComp,
                                                          const TArray<FName>& BoneOrSocketNames,
                                                          const FColor HullColor, float Duration) {
    if (!WorldContextObject || !MeshComp || BoneOrSocketNames.Num() < 4) {
        return;
    }

    TArray<FVector> Positions;
    for (const FName& Name : BoneOrSocketNames) {
        if (MeshComp->DoesSocketExist(Name)) {
            Positions.Add(MeshComp->GetSocketLocation(Name));
        } else if (MeshComp->GetBoneIndex(Name) != INDEX_NONE) {
            Positions.Add(MeshComp->GetBoneLocation(Name));
        }
    }

    if (Positions.Num() < 4) {
        return;
    }

    TArray<FVector> Hull;
    TArray<int32> Indices;

    ComputeConvexHullWithIndices(Positions, Hull, Indices);
    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::GenerateConvexHullFromBodyPartChain(const UObject* WorldContextObject,
                                                            USkeletalMeshComponent* MeshComp, EOHBodyPart BodyPart,
                                                            const FColor HullColor, float Duration) {
    if (!WorldContextObject || !MeshComp) {
        return;
    }

    const EOHSkeletalBone RootBone = UOHSkeletalPhysicsUtils::GetRootBoneInBodyPartFromMesh(MeshComp, BodyPart);
    const EOHSkeletalBone EndBone = UOHSkeletalPhysicsUtils::GetEndBoneInBodyPartFromMesh(MeshComp, BodyPart);
    const TArray<FName> Chain = UOHSkeletalPhysicsUtils::GetBonesInChain(EndBone);

    TArray<FVector> Points;
    for (const FName& BoneName : Chain) {
        if (MeshComp->DoesSocketExist(BoneName)) {
            Points.Add(MeshComp->GetSocketLocation(BoneName));
        }
    }

    if (Points.Num() < 4) {
        return;
    }

    TArray<FVector> Hull;
    TArray<int32> Indices;
    ComputeConvexHullWithIndices(Points, Hull, Indices);
    DrawDebugConvexMeshFromFaces(WorldContextObject, Hull, Indices, HullColor, 1.5f, Duration);
}

void UOHCollisionUtils::RunStrikePredictionDebug(const UObject* WorldContextObject, AActor* Attacker, AActor* Target,
                                                 float ArcAngle, float Radius, float PredictTime) {
    if (!Attacker || !Target || !WorldContextObject) {
        return;
    }

    FVector Origin = Attacker->GetActorLocation();
    FVector Forward = Attacker->GetActorForwardVector();
    FVector TargetFuture = Target->GetActorLocation() + Target->GetVelocity() * PredictTime;

    DrawDebugStrikeArc(WorldContextObject, Origin, Forward, Radius, ArcAngle, FColor::Red, 2.0f);

    DrawDebugSphere(GEngine->GetWorldFromContextObjectChecked(WorldContextObject), TargetFuture, 10.f, 12,
                    FColor::Green, false, 2.0f);

    bool bInArc = IsTargetWithinStrikeArc(Origin, Forward, TargetFuture, ArcAngle, Radius);

    DrawDebugString(GEngine->GetWorldFromContextObjectChecked(WorldContextObject), TargetFuture + FVector(0, 0, 20),
                    bInArc ? TEXT("HIT") : TEXT("MISS"), nullptr, bInArc ? FColor::Green : FColor::White, 2.0f);
}

void UOHCollisionUtils::RunStrikePredictionLive(const UObject* WorldContextObject, AActor* Attacker, AActor* Target,
                                                float Radius, float ArcAngle, float PredictTime, bool bShowTrajectory,
                                                bool bShowArc, bool bShowResult) {
    if (!Attacker || !Target || !WorldContextObject) {
        return;
    }

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World) {
        return;
    }

    const FVector Origin = Attacker->GetActorLocation();
    const FVector Forward = Attacker->GetActorForwardVector();
    const FVector TargetVel = Target->GetVelocity();
    const FVector TargetFuture = Target->GetActorLocation() + TargetVel * PredictTime;

    if (bShowTrajectory) {
        DrawDebugTrajectory(WorldContextObject, Target->GetActorLocation(), TargetVel, 0.05f, 10, FColor::Blue, 1.5f);
    }

    if (bShowArc) {
        DrawDebugStrikeArc(WorldContextObject, Origin, Forward, Radius, ArcAngle, FColor::Red, 1.5f);
    }

    const bool bWillHit = IsTargetWithinStrikeArc(Origin, Forward, TargetFuture, ArcAngle, Radius);

    if (bShowResult) {
        const FVector LabelPos = TargetFuture + FVector(0, 0, 40);
        const FString Label = bWillHit ? TEXT("PREDICTED HIT") : TEXT("MISS");
        const FColor LabelColor = bWillHit ? FColor::Green : FColor::White;
        DrawDebugString(World, LabelPos, Label, nullptr, LabelColor, 1.5f, false);
    }
}

bool UOHCollisionUtils::ShouldDodgePredictedStrike(AActor* Attacker, AActor* SelfActor, float ArcAngleDegrees,
                                                   float Radius, float PredictTime) {
    if (!Attacker || !SelfActor) {
        return false;
    }

    const FVector Origin = Attacker->GetActorLocation();
    const FVector Forward = Attacker->GetActorForwardVector();
    const FVector FuturePos = SelfActor->GetActorLocation() + SelfActor->GetVelocity() * PredictTime;

    return IsTargetWithinStrikeArc(Origin, Forward, FuturePos, ArcAngleDegrees, Radius);
}

#pragma endregion

#pragma region Sampling

TArray<FVector> UOHCollisionUtils::SampleMotionPoints(USceneComponent* TargetComponent, int32 NumSamples,
                                                      float DurationSeconds) {
    TArray<FVector> Samples;
    if (!TargetComponent || NumSamples < 2 || DurationSeconds <= 0.f) {
        return Samples;
    }

    const float StepTime = DurationSeconds / (NumSamples - 1);
    const FVector Start = TargetComponent->GetComponentLocation();
    const FVector End = Start + TargetComponent->GetForwardVector() * 100.f; // simulate forward arc

    for (int32 i = 0; i < NumSamples; ++i) {
        float Alpha = static_cast<float>(i) / (NumSamples - 1);
        FVector Sample = FMath::Lerp(Start, End, Alpha);
        Sample += FVector(0.f, FMath::Sin(Alpha * PI) * 30.f, 0.f); // add curve
        Samples.Add(Sample);
    }

    return Samples;
}

void UOHCollisionUtils::UpdateMotionHistory(TArray<FVector>& HistoryBuffer, const FVector& NewSample, float MaxDistance,
                                            int32 MaxSamples) {
    if (HistoryBuffer.Num() > 0) {
        if (FVector::DistSquared(HistoryBuffer.Last(), NewSample) < FMath::Square(1.f)) {
            return; // Skip near-duplicates
        }
    }

    HistoryBuffer.Add(NewSample);

    // Clamp size
    while (HistoryBuffer.Num() > MaxSamples) {
        HistoryBuffer.RemoveAt(0);
    }

    // Clamp distance
    if (HistoryBuffer.Num() >= 2) {
        while (FVector::Dist(HistoryBuffer[0], HistoryBuffer.Last()) > MaxDistance && HistoryBuffer.Num() > 2) {
            HistoryBuffer.RemoveAt(0);
        }
    }
}

float UOHCollisionUtils::ComputeContactConfidence(const FHitResult& Hit, const FOHBoneData& BoneData) {
    if (!Hit.bBlockingHit || !BoneData.IsValid())
        return 0.f;

    const FVector ToImpact = (Hit.ImpactPoint - BoneData.GetCurrentPosition()).GetSafeNormal();
    const FVector Velocity = BoneData.GetBodyLinearVelocity().GetSafeNormal();

    const float ForwardDot = FVector::DotProduct(ToImpact, Velocity);
    const float NormalDot = FVector::DotProduct(Velocity, -Hit.ImpactNormal);

    const float Combined = (ForwardDot + NormalDot) * 0.5f;
    return FMath::Clamp(Combined, 0.f, 1.f);
}
#pragma endregion

#pragma region Detection

bool UOHCollisionUtils::GetDirectionToClosestCharacter(AActor* OriginActor, FVector& OutDirection,
                                                       ACharacter*& OutClosestCharacter) {
    OutDirection = FVector::ZeroVector;
    OutClosestCharacter = nullptr;

    if (!OriginActor) {
        return false;
    }

    FVector OriginLocation = OriginActor->GetActorLocation();
    float ClosestDistSq = FLT_MAX;

    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), FoundActors);

    for (AActor* Actor : FoundActors) {
        if (Actor == OriginActor) {
            continue;
        }

        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character) {
            continue;
        }

        float DistSq = FVector::DistSquared(OriginLocation, Character->GetActorLocation());
        if (DistSq < ClosestDistSq) {
            ClosestDistSq = DistSq;
            OutClosestCharacter = Character;
        }
    }

    if (OutClosestCharacter) {
        FVector TargetLoc = OutClosestCharacter->GetActorLocation();
        OutDirection = (TargetLoc - OriginLocation).GetSafeNormal();
        return true;
    }

    return false;
}

TArray<ACharacter*> UOHCollisionUtils::GetCharactersInRange(AActor* OriginActor, float Radius) {
    TArray<ACharacter*> CharactersInRange;

    if (!OriginActor || Radius <= 0.f) {
        return CharactersInRange;
    }

    FVector OriginLocation = OriginActor->GetActorLocation();
    float RadiusSq = Radius * Radius;

    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), FoundActors);

    for (AActor* Actor : FoundActors) {
        if (Actor == OriginActor) {
            continue;
        }

        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character) {
            continue;
        }

        float DistSq = FVector::DistSquared(OriginLocation, Character->GetActorLocation());
        if (DistSq <= RadiusSq) {
            CharactersInRange.Add(Character);
        }
    }

    return CharactersInRange;
}

bool UOHCollisionUtils::GetClosestCharacterInRange(AActor* OriginActor, float Radius,
                                                   ACharacter*& OutClosestCharacter) {
    OutClosestCharacter = nullptr;

    if (!OriginActor || Radius <= 0.f) {
        return false;
    }

    FVector OriginLocation = OriginActor->GetActorLocation();
    float ClosestDistSq = Radius * Radius;

    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), FoundActors);

    for (AActor* Actor : FoundActors) {
        if (Actor == OriginActor) {
            continue;
        }

        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character) {
            continue;
        }

        float DistSq = FVector::DistSquared(OriginLocation, Character->GetActorLocation());
        if (DistSq <= ClosestDistSq) {
            if (!OutClosestCharacter ||
                DistSq < FVector::DistSquared(OriginLocation, OutClosestCharacter->GetActorLocation())) {
                OutClosestCharacter = Character;
                ClosestDistSq = DistSq; // Update the closest distance found so far
            }
        }
    }

    return OutClosestCharacter != nullptr;
}

bool UOHCollisionUtils::GetClosestCharacterInTraceRadius(AActor* OriginActor, float Radius,
                                                         ACharacter*& OutClosestCharacter,
                                                         ECollisionChannel TraceChannel) {
    OutClosestCharacter = nullptr;

    if (!OriginActor || Radius <= 0.f) {
        return false;
    }

    UWorld* World = OriginActor->GetWorld();
    if (!World) {
        return false;
    }

    const FVector Origin = OriginActor->GetActorLocation();

    TArray<FOverlapResult> Overlaps;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(OriginActor);

    FCollisionShape Sphere = FCollisionShape::MakeSphere(Radius);
    bool bHasHit = World->OverlapMultiByChannel(Overlaps, Origin, FQuat::Identity, TraceChannel, Sphere, Params);

    if (!bHasHit) {
        return false;
    }

    float ClosestDistSq = Radius * Radius;

    for (const FOverlapResult& Result : Overlaps) {
        ACharacter* Character = Cast<ACharacter>(Result.GetActor());
        if (!Character || Character == OriginActor) {
            continue;
        }

        const float DistSq = FVector::DistSquared(Origin, Character->GetActorLocation());
        if (DistSq <= ClosestDistSq) {
            OutClosestCharacter = Character;
            ClosestDistSq = DistSq;
        }
    }

    return OutClosestCharacter != nullptr;
}

bool UOHCollisionUtils::GetDirectionToClosestCharacterInRadiusTrace(AActor* OriginActor, float Radius,
                                                                    FVector& OutDirection,
                                                                    ACharacter*& OutClosestCharacter,
                                                                    ECollisionChannel TraceChannel) {
    OutDirection = FVector::ZeroVector;
    OutClosestCharacter = nullptr;

    if (!OriginActor || Radius <= 0.f) {
        return false;
    }

    UWorld* World = OriginActor->GetWorld();
    if (!World) {
        return false;
    }

    const FVector Origin = OriginActor->GetActorLocation();

    TArray<FOverlapResult> Overlaps;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(OriginActor);

    FCollisionShape Sphere = FCollisionShape::MakeSphere(Radius);
    bool bHasHit = World->OverlapMultiByChannel(Overlaps, Origin, FQuat::Identity, TraceChannel, Sphere, Params);

    if (!bHasHit) {
        return false;
    }

    float ClosestDistSq = Radius * Radius;

    for (const FOverlapResult& Result : Overlaps) {
        ACharacter* Character = Cast<ACharacter>(Result.GetActor());
        if (!Character || Character == OriginActor) {
            continue;
        }

        const float DistSq = FVector::DistSquared(Origin, Character->GetActorLocation());
        if (DistSq <= ClosestDistSq) {
            OutClosestCharacter = Character;
            ClosestDistSq = DistSq;
        }
    }

    if (OutClosestCharacter) {
        OutDirection = (OutClosestCharacter->GetActorLocation() - Origin).GetSafeNormal();
        return true;
    }

    return false;
}

bool UOHCollisionUtils::GetDirectionAndHitToClosestCharacterInRadiusTrace(
    AActor* OriginActor, float Radius, FVector& OutDirection, ACharacter*& OutClosestCharacter, FHitResult& OutHit,
    ECollisionChannel OverlapChannel, ECollisionChannel TraceChannel, bool bRequireLineOfSight, bool bDrawDebug,
    FColor DebugColor, float DebugDuration) {
    OutDirection = FVector::ZeroVector;
    OutClosestCharacter = nullptr;
    OutHit = FHitResult();

    if (!OriginActor || Radius <= 0.f) {
        return false;
    }

    UWorld* World = OriginActor->GetWorld();
    if (!World) {
        return false;
    }

    const FVector Origin = OriginActor->GetActorLocation();
    FCollisionShape Sphere = FCollisionShape::MakeSphere(Radius);

    TArray<FOverlapResult> Overlaps;
    FCollisionQueryParams OverlapParams;
    OverlapParams.AddIgnoredActor(OriginActor);

    const bool bHasOverlap =
        World->OverlapMultiByChannel(Overlaps, Origin, FQuat::Identity, OverlapChannel, Sphere, OverlapParams);

    if (bDrawDebug) {
        DrawDebugSphere(World, Origin, Radius, 16, DebugColor, false, DebugDuration);
    }

    if (!bHasOverlap) {
        return false;
    }

    float ClosestDistSq = Radius * Radius;

    for (const FOverlapResult& Result : Overlaps) {
        ACharacter* Candidate = Cast<ACharacter>(Result.GetActor());
        if (!Candidate || Candidate == OriginActor) {
            continue;
        }

        const FVector TargetLocation = Candidate->GetActorLocation();
        const float DistSq = FVector::DistSquared(Origin, TargetLocation);

        if (DistSq < ClosestDistSq) {
            FCollisionQueryParams TraceParams;
            TraceParams.AddIgnoredActor(OriginActor);
            TraceParams.AddIgnoredActor(Candidate);
            TraceParams.bTraceComplex = true;

            FHitResult TraceHit;
            bool bHit = World->LineTraceSingleByChannel(TraceHit, Origin, TargetLocation, TraceChannel, TraceParams);

            const bool bLOSOK = !bRequireLineOfSight || (bHit && TraceHit.GetActor() == Candidate);

            if (bDrawDebug) {
                DrawDebugLine(World, Origin, TargetLocation, bLOSOK ? DebugColor : FColor::Red, false, DebugDuration, 0,
                              1.5f);
            }

            if (bLOSOK) {
                OutClosestCharacter = Candidate;
                OutHit = TraceHit;
                ClosestDistSq = DistSq;
            }
        }
    }

    if (!OutClosestCharacter) {
        return false;
    }

    OutDirection = (OutClosestCharacter->GetActorLocation() - Origin).GetSafeNormal();
    return true;
}

bool UOHCollisionUtils::GetDirectionToClosestCharacterUsingAABB(AActor* OriginActor, float Radius,
                                                                FVector& OutDirection,
                                                                ACharacter*& OutClosestCharacter) {
    OutDirection = FVector::ZeroVector;
    OutClosestCharacter = nullptr;

    if (!OriginActor || Radius <= 0.f) {
        return false;
    }

    FVector OriginCenter = OriginActor->GetActorLocation();
    FVector OriginExtents = FVector(Radius); // Create a cubic volume as AABB around origin

    TArray<AActor*> AllCharacters;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), AllCharacters);

    float ClosestDistSq = Radius * Radius;

    for (AActor* Actor : AllCharacters) {
        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || Character == OriginActor) {
            continue;
        }

        USkeletalMeshComponent* Mesh = Character->GetMesh();
        if (!Mesh) {
            continue;
        }

        FVector CharCenter = Mesh->Bounds.Origin;
        FVector CharExtents = Mesh->Bounds.BoxExtent;

        // Use your custom AABB overlap test
        if (!AABB_Overlap(OriginCenter, OriginExtents, CharCenter, CharExtents)) {
            continue;
        }

        const float DistSq = FVector::DistSquared(OriginCenter, CharCenter);
        if (DistSq < ClosestDistSq) {
            ClosestDistSq = DistSq;
            OutClosestCharacter = Character;
        }
    }

    if (!OutClosestCharacter) {
        return false;
    }

    OutDirection = (OutClosestCharacter->GetActorLocation() - OriginCenter).GetSafeNormal();
    return true;
}

bool UOHCollisionUtils::GetClosestCharacterUsingOBBIntersection(AActor* OriginActor, FVector Extents, FRotator Rotation,
                                                                ACharacter*& OutClosestCharacter) {
    OutClosestCharacter = nullptr;
    if (!OriginActor) {
        return false;
    }

    const FVector Origin = OriginActor->GetActorLocation();
    float ClosestDistSq = FLT_MAX;

    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), Found);

    for (AActor* Actor : Found) {
        if (Actor == OriginActor) {
            continue;
        }

        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || !Character->GetMesh()) {
            continue;
        }

        const FVector TargetCenter = Character->GetMesh()->Bounds.Origin;
        const FVector TargetExtents = Character->GetMesh()->Bounds.BoxExtent;
        const FRotator TargetRotation = Character->GetActorRotation();

        if (SAT_BoxIntersect(Origin, Rotation, Extents, TargetCenter, TargetRotation, TargetExtents)) {
            const float DistSq = FVector::DistSquared(Origin, TargetCenter);
            if (DistSq < ClosestDistSq) {
                ClosestDistSq = DistSq;
                OutClosestCharacter = Character;
            }
        }
    }

    return OutClosestCharacter != nullptr;
}

bool UOHCollisionUtils::GetClosestCharacterUsingGJKIntersection(const TArray<FVector>& AttackHull, AActor* OriginActor,
                                                                ACharacter*& OutClosestCharacter) {
    OutClosestCharacter = nullptr;
    if (!OriginActor || AttackHull.Num() == 0) {
        return false;
    }

    UWorld* World = OriginActor->GetWorld();
    if (!World) {
        return false;
    }

    float ClosestDistSq = FLT_MAX;
    const FVector Origin = OriginActor->GetActorLocation();

    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(World, ACharacter::StaticClass(), Found);

    for (AActor* Actor : Found) {
        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || Character == OriginActor || !Character->GetMesh()) {
            continue;
        }

        TArray<FVector> TargetHull = GenerateBoxHull(
            Character->GetMesh()->Bounds.Origin, Character->GetMesh()->Bounds.BoxExtent, Character->GetActorRotation());

        if (GJK_Intersect(AttackHull, TargetHull)) {
            const float DistSq = FVector::DistSquared(Origin, Character->GetActorLocation());
            if (DistSq < ClosestDistSq) {
                ClosestDistSq = DistSq;
                OutClosestCharacter = Character;
            }
        }
    }

    return OutClosestCharacter != nullptr;
}

bool UOHCollisionUtils::DetectTargetUsingAABB(AActor* OriginActor, float Radius, ACharacter*& OutClosestCharacter,
                                              FVector& OutDirection) {
    OutDirection = FVector::ZeroVector;
    OutClosestCharacter = nullptr;

    if (!OriginActor) {
        return false;
    }

    FVector OriginCenter = OriginActor->GetActorLocation();
    FVector OriginExtents = FVector(Radius);
    float ClosestDistSq = Radius * Radius;

    TArray<AActor*> AllChars;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), AllChars);

    for (AActor* Actor : AllChars) {
        if (Actor == OriginActor) {
            continue;
        }
        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || !Character->GetMesh()) {
            continue;
        }

        FVector CharCenter = Character->GetMesh()->Bounds.Origin;
        FVector CharExtents = Character->GetMesh()->Bounds.BoxExtent;

        if (!AABB_Overlap(OriginCenter, OriginExtents, CharCenter, CharExtents)) {
            continue;
        }

        float DistSq = FVector::DistSquared(OriginCenter, CharCenter);
        if (DistSq < ClosestDistSq) {
            ClosestDistSq = DistSq;
            OutClosestCharacter = Character;
        }
    }

    if (OutClosestCharacter) {
        OutDirection = (OutClosestCharacter->GetActorLocation() - OriginCenter).GetSafeNormal();
        return true;
    }

    return false;
}

bool UOHCollisionUtils::DetectTargetUsingOBB(AActor* OriginActor, const FVector& OriginExtents,
                                             const FRotator& OriginRotation, ACharacter*& OutClosestCharacter,
                                             FVector& OutDirection) {
    OutClosestCharacter = nullptr;
    OutDirection = FVector::ZeroVector;

    if (!OriginActor) {
        return false;
    }

    FVector OriginCenter = OriginActor->GetActorLocation();
    float ClosestDistSq = FLT_MAX;

    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), Found);

    for (AActor* Actor : Found) {
        if (Actor == OriginActor) {
            continue;
        }
        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || !Character->GetMesh()) {
            continue;
        }

        FVector TargetCenter = Character->GetMesh()->Bounds.Origin;
        FVector TargetExtents = Character->GetMesh()->Bounds.BoxExtent;
        FRotator TargetRotation = Character->GetActorRotation();

        if (SAT_BoxIntersect(OriginCenter, OriginRotation, OriginExtents, TargetCenter, TargetRotation,
                             TargetExtents)) {
            float DistSq = FVector::DistSquared(OriginCenter, TargetCenter);
            if (DistSq < ClosestDistSq) {
                ClosestDistSq = DistSq;
                OutClosestCharacter = Character;
            }
        }
    }

    if (OutClosestCharacter) {
        OutDirection = (OutClosestCharacter->GetActorLocation() - OriginCenter).GetSafeNormal();
        return true;
    }

    return false;
}

bool UOHCollisionUtils::DetectTargetUsingGJK(const TArray<FVector>& AttackHull, AActor* OriginActor,
                                             ACharacter*& OutClosestCharacter, FVector& OutDirection) {
    OutClosestCharacter = nullptr;
    OutDirection = FVector::ZeroVector;
    if (!OriginActor || AttackHull.Num() == 0) {
        return false;
    }

    const FVector Origin = OriginActor->GetActorLocation();
    float ClosestDistSq = FLT_MAX;

    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), Found);

    for (AActor* Actor : Found) {
        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || Character == OriginActor || !Character->GetMesh()) {
            continue;
        }

        TArray<FVector> TargetHull = GenerateBoxHull(
            Character->GetMesh()->Bounds.Origin, Character->GetMesh()->Bounds.BoxExtent, Character->GetActorRotation());

        if (GJK_Intersect(AttackHull, TargetHull)) {
            float DistSq = FVector::DistSquared(Origin, Character->GetActorLocation());
            if (DistSq < ClosestDistSq) {
                ClosestDistSq = DistSq;
                OutClosestCharacter = Character;
            }
        }
    }

    if (OutClosestCharacter) {
        OutDirection = (OutClosestCharacter->GetActorLocation() - Origin).GetSafeNormal();
        return true;
    }

    return false;
}

bool UOHCollisionUtils::DetectTargetWithEPA(const TArray<FVector>& AttackHull, AActor* OriginActor,
                                            ACharacter*& OutClosestCharacter, FVector& OutNormal,
                                            float& OutPenetrationDepth) {
    OutClosestCharacter = nullptr;
    OutNormal = FVector::ZeroVector;
    OutPenetrationDepth = 0.f;

    if (!OriginActor || AttackHull.Num() == 0) {
        return false;
    }

    const FVector Origin = OriginActor->GetActorLocation();
    float ClosestDistSq = FLT_MAX;

    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(OriginActor->GetWorld(), ACharacter::StaticClass(), Found);

    for (AActor* Actor : Found) {
        ACharacter* Character = Cast<ACharacter>(Actor);
        if (!Character || Character == OriginActor || !Character->GetMesh()) {
            continue;
        }

        TArray<FVector> TargetHull = GenerateBoxHull(
            Character->GetMesh()->Bounds.Origin, Character->GetMesh()->Bounds.BoxExtent, Character->GetActorRotation());

        TArray<FVector> Simplex;
        if (GJK_Intersect(AttackHull, TargetHull) && ProcessSimplex(Simplex, OutNormal)) {
            FVector Normal;
            float Depth = 0.f;
            if (EPA_PenetrationDepth(AttackHull, TargetHull, Simplex, Normal, Depth)) {
                float DistSq = FVector::DistSquared(Origin, Character->GetActorLocation());
                if (DistSq < ClosestDistSq) {
                    OutClosestCharacter = Character;
                    OutNormal = Normal;
                    OutPenetrationDepth = Depth;
                    ClosestDistSq = DistSq;
                }
            }
        }
    }

    return OutClosestCharacter != nullptr;
}

bool UOHCollisionUtils::DetectTargetFromTrailHull(USceneComponent* TrailComponent, float Duration, int32 NumSamples,
                                                  AActor* OriginActor, ACharacter*& OutClosestCharacter,
                                                  FVector& OutDirection) {
    OutClosestCharacter = nullptr;
    OutDirection = FVector::ZeroVector;

    if (!TrailComponent || !OriginActor) {
        return false;
    }

    // Sample trail motion
    TArray<FVector> TrailPoints;
    for (int32 i = 0; i < NumSamples; ++i) {
        float TimeOffset = (i / static_cast<float>(NumSamples)) * Duration;
        FVector SamplePos = TrailComponent->GetComponentLocation(); // In real code: cache transform history
        TrailPoints.Add(SamplePos);
    }

    TArray<FVector> AttackHull = ComputeConvexHull(TrailPoints);
    if (AttackHull.Num() == 0) {
        return false;
    }

    return DetectTargetUsingGJK(AttackHull, OriginActor, OutClosestCharacter, OutDirection);
}

#pragma endregion

#pragma region PhysicsHelpers

bool UOHCollisionUtils::ComputeAABBSeparationImpulse(const FVector& CenterA, const FVector& ExtentsA,
                                                     const FVector& CenterB, const FVector& ExtentsB,
                                                     FVector& OutImpulse) {
    OutImpulse = FVector::ZeroVector;

    FVector Delta = CenterB - CenterA;
    FVector TotalExtents = ExtentsA + ExtentsB;
    FVector AbsDelta = Delta.GetAbs();
    FVector Overlap = TotalExtents - AbsDelta;

    // If all axes overlap, we have penetration
    if (Overlap.X > 0.f && Overlap.Y > 0.f && Overlap.Z > 0.f) {
        // Choose smallest axis of penetration
        if (Overlap.X < Overlap.Y && Overlap.X < Overlap.Z) {
            OutImpulse = FVector(FMath::Sign(Delta.X) * Overlap.X, 0.f, 0.f);
        } else if (Overlap.Y < Overlap.Z) {
            OutImpulse = FVector(0.f, FMath::Sign(Delta.Y) * Overlap.Y, 0.f);
        } else {
            OutImpulse = FVector(0.f, 0.f, FMath::Sign(Delta.Z) * Overlap.Z);
        }
        return true;
    }

    return false;
}

bool UOHCollisionUtils::ComputeEPAPenetrationCorrection(const TArray<FVector>& ShapeA, const TArray<FVector>& ShapeB,
                                                        FVector& OutCorrectionNormal, float& OutPenetrationDepth) {
    TArray<FVector> Simplex;
    if (!GJK_Intersect(ShapeA, ShapeB, 30, 0.001f)) {
        return false;
    }

    // Generate an initial simplex (needed for EPA)
    FVector Dummy;
    ProcessSimplex(Simplex, Dummy);

    return EPA_PenetrationDepth(ShapeA, ShapeB, Simplex, OutCorrectionNormal, OutPenetrationDepth, 30, 0.001f);
}

void UOHCollisionUtils::ApplyImpulseToBone(USkeletalMeshComponent* Mesh, FName BoneName, const FVector& Impulse,
                                           bool bVelChange) {
    if (!Mesh || !Mesh->IsSimulatingPhysics(BoneName)) {
        return;
    }

    Mesh->AddImpulseToAllBodiesBelow(Impulse, BoneName, bVelChange);
}

#pragma endregion

#pragma region Physics_Extensions

void UOHCollisionUtils::ApplySafeAngularTorque(USkeletalMeshComponent* Mesh, FName BoneName,
                                               const FRotator& TargetRotation, float TorqueStrength) {
    if (!Mesh || !Mesh->IsSimulatingPhysics(BoneName)) {
        return;
    }

    FBodyInstance* Body = Mesh->GetBodyInstance(BoneName);
    if (!Body) {
        return;
    }

    const FQuat CurrentQuat = Body->GetUnrealWorldTransform().GetRotation();
    const FQuat TargetQuat = TargetRotation.Quaternion();
    const FQuat DeltaQuat = TargetQuat * CurrentQuat.Inverse();

    FVector Axis;
    float Angle;
    DeltaQuat.ToAxisAndAngle(Axis, Angle);

    // Clamp extreme values
    Angle = FMath::Clamp(FMath::UnwindRadians(Angle), -PI, PI);

    const FVector Torque = Axis * Angle * TorqueStrength;
    Body->AddTorqueInRadians(Torque, true);
}

void UOHCollisionUtils::ApplySafeSpringForce(USkeletalMeshComponent* Mesh, FName BoneName,
                                             const FVector& TargetPosition, float SpringStrength, float Damping) {
    if (!Mesh || !Mesh->IsSimulatingPhysics(BoneName)) {
        return;
    }

    FBodyInstance* Body = Mesh->GetBodyInstance(BoneName);
    if (!Body) {
        return;
    }

    const FVector Current = Body->GetUnrealWorldTransform().GetLocation();
    const FVector Velocity = Body->GetUnrealWorldVelocity();

    const FVector Spring = (TargetPosition - Current) * SpringStrength;
    const FVector DampingForce = -Velocity * Damping;

    Body->AddForce(Spring + DampingForce);
}

void UOHCollisionUtils::ApplyLookAtTorque(USkeletalMeshComponent* Mesh, FName BoneName, const FVector& TargetLocation,
                                          float TorqueStrength) {
    if (!Mesh || !Mesh->IsSimulatingPhysics(BoneName)) {
        return;
    }

    FBodyInstance* Body = Mesh->GetBodyInstance(BoneName);
    if (!Body) {
        return;
    }

    const FTransform BodyTransform = Body->GetUnrealWorldTransform();
    const FVector Forward = BodyTransform.GetRotation().GetForwardVector();
    const FVector ToTarget = (TargetLocation - BodyTransform.GetLocation()).GetSafeNormal();

    if (ToTarget.IsNearlyZero()) {
        return;
    }

    const FQuat TargetQuat = FRotationMatrix::MakeFromX(ToTarget).ToQuat();
    const FQuat DeltaQuat = TargetQuat * BodyTransform.GetRotation().Inverse();

    FVector Axis;
    float Angle;
    DeltaQuat.ToAxisAndAngle(Axis, Angle);
    Angle = FMath::UnwindRadians(Angle);

    const FVector Torque = Axis * Angle * TorqueStrength;
    Body->AddTorqueInRadians(Torque, true);
}

void UOHCollisionUtils::AdjustPhysicalStiffness(UPhysicalAnimationComponent* PhysAnim, FName BoneName,
                                                float HitMagnitude, float MaxExpectedForce, float MinStiffness,
                                                float MaxStiffness) {
    if (!PhysAnim) {
        return;
    }

    const float Alpha = FMath::Clamp(HitMagnitude / MaxExpectedForce, 0.f, 1.f);
    const float Stiffness = FMath::Lerp(MinStiffness, MaxStiffness, Alpha);

    FPhysicalAnimationData Data;
    Data.OrientationStrength = Stiffness;
    Data.PositionStrength = Stiffness;
    Data.VelocityStrength = Stiffness * 0.1f;

    PhysAnim->ApplyPhysicalAnimationSettings(BoneName, Data);
}

#pragma endregion

#pragma region Prediction

FVector UOHCollisionUtils::PredictFuturePosition(const FVector& StartPosition, const FVector& Velocity, float Time) {
    return StartPosition + Velocity * Time;
}

float UOHCollisionUtils::PredictTimeToCollision(const FVector& PositionA, const FVector& VelocityA,
                                                const FVector& PositionB, const FVector& VelocityB,
                                                float CombinedRadius) {
    const FVector RelPos = PositionB - PositionA;
    const FVector RelVel = VelocityB - VelocityA;

    const float A = RelVel.SizeSquared();
    if (A < KINDA_SMALL_NUMBER) {
        return -1.f; // Not moving relative to each other
    }

    const float B = FVector::DotProduct(RelPos, RelVel);
    const float C = RelPos.SizeSquared() - FMath::Square(CombinedRadius);

    const float Discriminant = FMath::Square(B) - A * C;
    if (Discriminant < 0.f) {
        return -1.f; // No real roots: miss
    }

    const float T = (-B - FMath::Sqrt(Discriminant)) / A;
    return (T >= 0.f) ? T : -1.f;
}

bool UOHCollisionUtils::PredictAABBOverlap(const FVector& CenterA, const FVector& ExtentsA, const FVector& VelocityA,
                                           const FVector& CenterB, const FVector& ExtentsB, const FVector& VelocityB,
                                           float DeltaTime) {
    FVector FutureA = CenterA + VelocityA * DeltaTime;
    FVector FutureB = CenterB + VelocityB * DeltaTime;

    return AABB_Overlap(FutureA, ExtentsA, FutureB, ExtentsB);
}

FVector UOHCollisionUtils::ComputeInterceptPoint(const FVector& ChaserPosition, float ChaserSpeed,
                                                 const FVector& TargetPosition, const FVector& TargetVelocity) {
    const FVector ToTarget = TargetPosition - ChaserPosition;
    const float Distance = ToTarget.Size();

    if (ChaserSpeed <= 0.f || TargetVelocity.IsNearlyZero()) {
        return TargetPosition;
    }

    const float RelativeSpeed = ChaserSpeed;
    const float T = Distance / RelativeSpeed;

    return TargetPosition + TargetVelocity * T;
}

bool UOHCollisionUtils::PredictConvexSweepOverlap(const TArray<FVector>& ShapeA, const FVector& VelocityA,
                                                  const TArray<FVector>& ShapeB, float DeltaTime) {
    if (ShapeA.Num() == 0 || ShapeB.Num() == 0) {
        return false;
    }

    TArray<FVector> SweptShapeA;
    for (const FVector& P : ShapeA) {
        SweptShapeA.Add(P + VelocityA * DeltaTime);
    }

    return GJK_Intersect(SweptShapeA, ShapeB);
}

FVector UOHCollisionUtils::ComputeLeadAimDirection(const FVector& Origin, const FVector& TargetLocation,
                                                   const FVector& TargetVelocity, float ProjectileSpeed) {
    const FVector ToTarget = TargetLocation - Origin;
    const float Distance = ToTarget.Size();

    if (ProjectileSpeed <= 0.f || TargetVelocity.IsNearlyZero()) {
        return ToTarget.GetSafeNormal();
    }

    // Time to impact assuming linear travel
    float T = Distance / ProjectileSpeed;

    // Projected future location
    FVector FutureTarget = TargetLocation + TargetVelocity * T;

    return (FutureTarget - Origin).GetSafeNormal();
}

bool UOHCollisionUtils::IsTargetWithinCone(const FVector& Origin, const FVector& Direction,
                                           const FVector& TargetLocation, float ConeAngleDegrees, float MaxDistance) {
    const FVector ToTarget = TargetLocation - Origin;
    const float Distance = ToTarget.Size();
    if (Distance > MaxDistance) {
        return false;
    }

    const float CosAngle = FMath::Cos(FMath::DegreesToRadians(ConeAngleDegrees * 0.5f));
    const float Dot = FVector::DotProduct(ToTarget.GetSafeNormal(), Direction.GetSafeNormal());

    return Dot >= CosAngle;
}

bool UOHCollisionUtils::IsTargetWithinStrikeArc(const FVector& Origin, const FVector& Forward,
                                                const FVector& TargetLocation, float ArcAngleDegrees, float Radius) {
    FVector FlatForward = Forward;
    FVector FlatToTarget = TargetLocation - Origin;

    FlatForward.Z = 0.f;
    FlatToTarget.Z = 0.f;

    if (FlatToTarget.SizeSquared() > Radius * Radius) {
        return false;
    }

    const float Dot = FVector::DotProduct(FlatForward.GetSafeNormal(), FlatToTarget.GetSafeNormal());
    const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));

    return AngleDeg <= ArcAngleDegrees * 0.5f;
}

FStrikePredictionResult UOHCollisionUtils::EvaluateStrikePrediction(AActor* Attacker, AActor* Target, float Radius,
                                                                    float ArcAngleDegrees, float PredictTime) {
    FStrikePredictionResult Result;

    if (!Attacker || !Target) {
        return Result;
    }

    const FVector Origin = Attacker->GetActorLocation();
    const FVector Forward = Attacker->GetActorForwardVector();
    const FVector VelocityTarget = Target->GetVelocity();
    const FVector FutureTarget = Target->GetActorLocation() + VelocityTarget * PredictTime;

    const float TimeToHit = PredictTime;
    const float Distance = FVector::Dist(Origin, FutureTarget);
    const bool bHit = IsTargetWithinStrikeArc(Origin, Forward, FutureTarget, ArcAngleDegrees, Radius);

    Result.bWillHit = bHit;
    Result.PredictedImpactPoint = FutureTarget;
    Result.TimeUntilImpact = TimeToHit;
    Result.DistanceAtImpact = Distance;
    Result.AttackerForward = Forward;
    Result.FutureTargetPosition = FutureTarget;
    Result.DirectionToTarget = (FutureTarget - Origin).GetSafeNormal();

    return Result;
}

#pragma endregion

#pragma region Animation

bool UOHCollisionUtils::SweepBoneChainWithMetrics(USkeletalMeshComponent* MeshComp, const TArray<FName>& BoneNames,
                                                  AActor* SelfActor, TArray<FStrikeContactMetrics>& OutContacts,
                                                  TArray<ACharacter*>& OutHitCharacters, bool bUseEPA, bool bDrawDebug,
                                                  float DeltaTime) {
    OutContacts.Empty();
    OutHitCharacters.Empty();

    if (!MeshComp || !SelfActor || BoneNames.Num() < 2) {
        return false;
    }

    // Build convex hull from bone positions
    TArray<FVector> HullPoints;
    for (const FName& BoneName : BoneNames) {
        HullPoints.Add(MeshComp->GetBoneLocation(BoneName));
    }

    TArray<FVector> ConvexHull = ComputeConvexHull(HullPoints);
    if (ConvexHull.Num() < 4) {
        return false;
    }

    bool bHit = false;

    // Find all target characters in the world
    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(SelfActor->GetWorld(), ACharacter::StaticClass(), Found);

    for (AActor* Actor : Found) {
        ACharacter* Target = Cast<ACharacter>(Actor);
        if (!Target || Target == SelfActor || !Target->GetMesh()) {
            continue;
        }

        const FBoxSphereBounds Bounds = Target->GetMesh()->Bounds;
        TArray<FVector> TargetHull = GenerateBoxHull(Bounds.Origin, Bounds.BoxExtent, Target->GetActorRotation());

        if (GJK_Intersect(ConvexHull, TargetHull)) {
            bHit = true;
            OutHitCharacters.Add(Target);

            // Default values
            FVector ContactNormal = FVector::ZeroVector;
            float PenetrationDepth = 1.f;

            if (bUseEPA) {
                TArray<FVector> Simplex;
                if (ProcessSimplex(Simplex, ContactNormal)) {
                    EPA_PenetrationDepth(ConvexHull, TargetHull, Simplex, ContactNormal, PenetrationDepth);
                }
            }

            // Striking bone info
            const FName StrikingBone = BoneNames.Last();
            const FVector BoneLocation = MeshComp->GetBoneLocation(StrikingBone);
            const FVector BoneVelocity = UOHCombatUtils::GetBoneVelocitySafe(MeshComp, StrikingBone, DeltaTime);

            // Build metrics struct
            FStrikeContactMetrics Metrics;
            Metrics.HitActor = Target;
            Metrics.StrikingBone = StrikingBone;
            Metrics.ContactLocation = BoneLocation;
            Metrics.ContactNormal = ContactNormal.GetSafeNormal();
            Metrics.Velocity = BoneVelocity;

            const float Dot = FVector::DotProduct(Metrics.ContactNormal, BoneVelocity.GetSafeNormal());
            Metrics.VelocityDotNormal = Dot;
            Metrics.ImpactAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
            Metrics.EstimatedForce = BoneVelocity.Size() * PenetrationDepth * FMath::Abs(Dot);

            OutContacts.Add(Metrics);
        }

        if (bDrawDebug) {
            DrawDebugConvexHull(SelfActor, ConvexHull, FColor::Red, 6.f, 0.25f);
        }
    }

    return bHit;
}

#pragma endregion

#pragma region MovementPush
<<<<<<< HEAD FVector UOHCollisionUtils::CalculateElasticCollisionResponse(const FVector& Velocity1, const FVector& Velocity2,
                                                               float Mass1, float Mass2, const FVector& CollisionNormal,
                                                               float Restitution) {
    // Calculate relative velocity
    FVector RelativeVelocity = Velocity1 - Velocity2;

    // Calculate relative velocity along collision normal
    float VelocityAlongNormal = FVector::DotProduct(RelativeVelocity, CollisionNormal);

    // Don't resolve if velocities are separating
    if (VelocityAlongNormal > 0)
        return FVector::ZeroVector;

    // Calculate impulse scalar
    float ImpulseScalar = -(1 + Restitution) * VelocityAlongNormal;
    ImpulseScalar /= (1 / Mass1 + 1 / Mass2);

    // Apply impulse to get new velocity
    FVector Impulse = ImpulseScalar * CollisionNormal;
    FVector NewVelocity1 = Velocity1 + (Impulse / Mass1);

    return NewVelocity1 - Velocity1; // Return velocity change
}

void UOHCollisionUtils::ResolveCharacterCollision(ACharacter* Character1, ACharacter* Character2, float PushStrength) {
    if (!Character1 || !Character2)
        return;

    // Get movement components
    UOHMovementComponent* MoveComp1 = Character1->FindComponentByClass<UOHMovementComponent>();
    UOHMovementComponent* MoveComp2 = Character2->FindComponentByClass<UOHMovementComponent>();

    if (!MoveComp1 || !MoveComp2)
        return;

    == == == = FVector UOHCollisionUtils::CalculateElasticCollisionResponse(
                 const FVector& Velocity1, const FVector& Velocity2, float Mass1, float Mass2,
                 const FVector& CollisionNormal, float Restitution) {
        // Calculate relative velocity
        FVector RelativeVelocity = Velocity1 - Velocity2;

        // Calculate relative velocity along collision normal
        float VelocityAlongNormal = FVector::DotProduct(RelativeVelocity, CollisionNormal);

        // Don't resolve if velocities are separating
        if (VelocityAlongNormal > 0)
            return FVector::ZeroVector;

        // Calculate impulse scalar
        float ImpulseScalar = -(1 + Restitution) * VelocityAlongNormal;
        ImpulseScalar /= (1 / Mass1 + 1 / Mass2);

        // Apply impulse to get new velocity
        FVector Impulse = ImpulseScalar * CollisionNormal;
        FVector NewVelocity1 = Velocity1 + (Impulse / Mass1);

        return NewVelocity1 - Velocity1; // Return velocity change
    }

    void UOHCollisionUtils::ResolveCharacterCollision(ACharacter * Character1, ACharacter * Character2,
                                                      float PushStrength) {
        if (!Character1 || !Character2)
            return;

        // Get movement components
        UOHMovementComponent* MoveComp1 = Character1->FindComponentByClass<UOHMovementComponent>();
        UOHMovementComponent* MoveComp2 = Character2->FindComponentByClass<UOHMovementComponent>();

        if (!MoveComp1 || !MoveComp2)
            return;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        // Calculate collision normal (from char1 to char2)
        FVector Pos1 = Character1->GetActorLocation();
        FVector Pos2 = Character2->GetActorLocation();
        FVector CollisionNormal = (Pos2 - Pos1).GetSafeNormal2D();
<<<<<<< HEAD

        == == == =
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                     // Get velocities and masses
            FVector Vel1 = Character1->GetVelocity();
        FVector Vel2 = Character2->GetVelocity();
        float Mass1 = MoveComp1->MovementConfig.CharacterMass;
        float Mass2 = MoveComp2->MovementConfig.CharacterMass;
<<<<<<< HEAD

        // Calculate elastic collision response
        FVector Response1 = CalculateElasticCollisionResponse(Vel1, Vel2, Mass1, Mass2, CollisionNormal, 0.3f);

        FVector Response2 = CalculateElasticCollisionResponse(Vel2, Vel1, Mass2, Mass1, -CollisionNormal, 0.3f);

        // Apply responses
        // MoveComp1->ApplyCollisionPushback(Response1 * PushStrength);
        // MoveComp2->ApplyCollisionPushback(Response2 * PushStrength);
        == == == =

                     // Calculate elastic collision response
            FVector Response1 = CalculateElasticCollisionResponse(Vel1, Vel2, Mass1, Mass2, CollisionNormal, 0.3f);

        FVector Response2 = CalculateElasticCollisionResponse(Vel2, Vel1, Mass2, Mass1, -CollisionNormal, 0.3f);

        // Apply responses
        // MoveComp1->ApplyCollisionPushback(Response1 * PushStrength);
        // MoveComp2->ApplyCollisionPushback(Response2 * PushStrength);
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    }

#pragma endregion