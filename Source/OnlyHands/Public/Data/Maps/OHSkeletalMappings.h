#pragma once
#include "Component/OHPhysicsManager.h"
#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "UObject/NoExportTypes.h"

namespace OHSkeletalMappings {
// ==========================================
// Core String Conversions
// ==========================================
extern const TMap<EOHSkeletalBone, FName> SkeletalBoneToFNameMap;
extern const TMap<FName, EOHSkeletalBone> FNameToSkeletalBoneMap;
extern const TMap<EOHSkeletalBone, FName> PrimaryBoneToFNameMap;

// ==========================================
// COMPLETE ENUM COMBINATIONS - ALL POSSIBLE MAPPINGS
// ==========================================

// =========================
// EOHSkeletalBone <-> EOHBodyZone
// =========================
// One-to-One
extern const TMap<EOHSkeletalBone, EOHBodyZone> BoneToZoneMap;
extern const TMap<EOHBodyZone, EOHSkeletalBone> ZoneToBoneMap;

// One-to-Many
extern const TMap<EOHSkeletalBone, TArray<EOHBodyZone>> BoneToZonesMap;
extern const TMap<EOHBodyZone, TArray<EOHSkeletalBone>> ZoneToBonesMap;

// Primary Variants
extern const TMap<EOHSkeletalBone, EOHBodyZone> PrimaryBoneToZoneMap;
extern const TMap<EOHBodyZone, EOHSkeletalBone> ZoneToPrimaryBoneMap;
extern const TMap<EOHSkeletalBone, TArray<EOHBodyZone>> PrimaryBoneToZonesMap;
extern const TMap<EOHBodyZone, TArray<EOHSkeletalBone>> ZoneToPrimaryBonesMap;

// =========================
// EOHSkeletalBone <-> EOHBodyPart
// =========================
// One-to-One
extern const TMap<EOHSkeletalBone, EOHBodyPart> BoneToBodyPartMap;
extern const TMap<EOHBodyPart, EOHSkeletalBone> BodyPartToBoneMap;

// One-to-Many
extern const TMap<EOHSkeletalBone, TArray<EOHBodyPart>> BoneToBodyPartsMap;
extern const TMap<EOHBodyPart, TArray<EOHSkeletalBone>> BodyPartToBonesMap;

// Primary Variants
extern const TMap<EOHSkeletalBone, EOHBodyPart> PrimaryBoneToBodyPartMap;
extern const TMap<EOHBodyPart, EOHSkeletalBone> BodyPartToPrimaryBoneMap;
extern const TMap<EOHSkeletalBone, TArray<EOHBodyPart>>
    PrimaryBoneToBodyPartsMap;
extern const TMap<EOHBodyPart, TArray<EOHSkeletalBone>>
    BodyPartToPrimaryBonesMap;

// =========================
// EOHSkeletalBone <-> EOHFunctionalBoneGroup
// =========================
// One-to-One
extern const TMap<EOHSkeletalBone, EOHFunctionalBoneGroup>
    BoneToFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, EOHSkeletalBone>
    FunctionalGroupToBoneMap;

// One-to-Many
extern const TMap<EOHSkeletalBone, TArray<EOHFunctionalBoneGroup>>
    BoneToFunctionalGroupsMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHSkeletalBone>>
    FunctionalGroupToBonesMap;

// Primary Variants
extern const TMap<EOHSkeletalBone, EOHFunctionalBoneGroup>
    PrimaryBoneToFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, EOHSkeletalBone>
    FunctionalGroupToPrimaryBoneMap;
extern const TMap<EOHSkeletalBone, TArray<EOHFunctionalBoneGroup>>
    PrimaryBoneToFunctionalGroupsMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHSkeletalBone>>
    FunctionalGroupToPrimaryBonesMap;

// =========================
// EOHBodyZone <-> EOHBodyPart
// =========================
// One-to-One
extern const TMap<EOHBodyZone, EOHBodyPart> ZoneToBodyPartMap;
extern const TMap<EOHBodyPart, EOHBodyZone> BodyPartToZoneMap;

// One-to-Many
extern const TMap<EOHBodyZone, TArray<EOHBodyPart>> ZoneToBodyPartsMap;
extern const TMap<EOHBodyPart, TArray<EOHBodyZone>> BodyPartToZonesMap;

// Primary Variants
extern const TMap<EOHBodyZone, EOHBodyPart> ZoneToPrimaryBodyPartMap;
extern const TMap<EOHBodyPart, EOHBodyZone> BodyPartToPrimaryZoneMap;
extern const TMap<EOHBodyZone, TArray<EOHBodyPart>> ZoneToPrimaryBodyPartsMap;
extern const TMap<EOHBodyPart, TArray<EOHBodyZone>> BodyPartToPrimaryZonesMap;

// =========================
// EOHBodyZone <-> EOHFunctionalBoneGroup
// =========================
// One-to-One
extern const TMap<EOHBodyZone, EOHFunctionalBoneGroup> ZoneToFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, EOHBodyZone> FunctionalGroupToZoneMap;

// One-to-Many
extern const TMap<EOHBodyZone, TArray<EOHFunctionalBoneGroup>>
    ZoneToFunctionalGroupsMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHBodyZone>>
    FunctionalGroupToZonesMap;

// Primary Variants
extern const TMap<EOHBodyZone, EOHFunctionalBoneGroup>
    ZoneToPrimaryFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, EOHBodyZone>
    FunctionalGroupToPrimaryZoneMap;
extern const TMap<EOHBodyZone, TArray<EOHFunctionalBoneGroup>>
    ZoneToPrimaryFunctionalGroupsMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHBodyZone>>
    FunctionalGroupToPrimaryZonesMap;

// =========================
// EOHBodyPart <-> EOHFunctionalBoneGroup
// =========================
// One-to-One
extern const TMap<EOHBodyPart, EOHFunctionalBoneGroup>
    BodyPartToFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, EOHBodyPart>
    FunctionalGroupToBodyPartMap;

// One-to-Many
extern const TMap<EOHBodyPart, TArray<EOHFunctionalBoneGroup>>
    BodyPartToFunctionalGroupsMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHBodyPart>>
    FunctionalGroupToBodyPartsMap;

// Primary Variants
extern const TMap<EOHBodyPart, EOHFunctionalBoneGroup>
    BodyPartToPrimaryFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, EOHBodyPart>
    FunctionalGroupToPrimaryBodyPartMap;
extern const TMap<EOHBodyPart, TArray<EOHFunctionalBoneGroup>>
    BodyPartToPrimaryFunctionalGroupsMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHBodyPart>>
    FunctionalGroupToPrimaryBodyPartsMap;

// ==========================================
// ADDITIONAL COMBINATION MAPS
// ==========================================

// =========================
// EOHBodyZone -> EOHSkeletalBone (Reverse Primary)
// =========================
extern const TMap<EOHBodyZone, EOHSkeletalBone> ZoneToPrimaryBoneMap;
extern const TMap<EOHBodyZone, TArray<EOHSkeletalBone>> ZoneToPrimaryBonesMap;

// =========================
// EOHBodyPart -> EOHSkeletalBone (Reverse Primary)
// =========================
extern const TMap<EOHBodyPart, EOHSkeletalBone> BodyPartToPrimaryBoneMap;
extern const TMap<EOHBodyPart, TArray<EOHSkeletalBone>>
    BodyPartToPrimaryBonesMap;

// =========================
// EOHFunctionalBoneGroup -> EOHSkeletalBone (Reverse Primary)
// =========================
extern const TMap<EOHFunctionalBoneGroup, EOHSkeletalBone>
    FunctionalGroupToPrimarySkeletalBoneMap;
extern const TMap<EOHFunctionalBoneGroup, TArray<EOHSkeletalBone>>
    FunctionalGroupToPrimarySkeletalBonesMap;

// ==========================================
// SPECIAL PURPOSE MAPS
// ==========================================

// Finger-specific mappings
extern const TMap<EOHSkeletalBone, bool> IsFingerBoneMap;
extern const TMap<EOHBodyZone, bool> ZoneContainsFingersMap;
extern const TMap<EOHBodyPart, bool> BodyPartContainsFingersMap;
extern const TMap<EOHFunctionalBoneGroup, bool>
    FunctionalGroupContainsFingersMap;

// Side-specific mappings
extern const TMap<EOHSkeletalBone, bool> IsLeftSideBoneMap;
extern const TMap<EOHSkeletalBone, bool> IsRightSideBoneMap;
extern const TMap<EOHBodyZone, bool> IsLeftSideZoneMap;
extern const TMap<EOHBodyZone, bool> IsRightSideZoneMap;
extern const TMap<EOHBodyPart, bool> IsLeftSideBodyPartMap;
extern const TMap<EOHBodyPart, bool> IsRightSideBodyPartMap;
extern const TMap<EOHFunctionalBoneGroup, bool> IsLeftSideFunctionalGroupMap;
extern const TMap<EOHFunctionalBoneGroup, bool> IsRightSideFunctionalGroupMap;

// ==========================================
// UTILITY ARRAYS & CONSTANTS
// ==========================================
extern const TArray<EOHSkeletalBone> EmptyBoneArray;
extern const TArray<EOHBodyZone> EmptyZoneArray;
extern const TArray<EOHBodyPart> EmptyBodyPartArray;
extern const TArray<EOHFunctionalBoneGroup> EmptyGroupArray;

extern const TArray<EOHSkeletalBone> PrimarySkeletalBones;
extern const TArray<EOHSkeletalBone> AllFingerBones;
extern const TArray<EOHSkeletalBone> AllPrimaryBones;
extern const TArray<EOHSkeletalBone> AllLeftBones;
extern const TArray<EOHSkeletalBone> AllRightBones;

} // namespace OHSkeletalMappings

#if 0
	// ==========================================
	// ACCESSOR FUNCTIONS
	// ==========================================
	const TArray<EOHBodyZone>& GetBodyZonesFromPrimaryBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetPrimaryBonesFromBodyZone(EOHBodyZone Zone);
	const TArray<EOHBodyPart>& GetBodyPartsFromPrimaryBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetPrimaryBonesFromBodyPart(EOHBodyPart Part);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromPrimaryBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetPrimaryBonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);

	const TArray<EOHBodyZone>& GetBodyZonesFromBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetBonesFromBodyZone(EOHBodyZone Zone);
	const TArray<EOHBodyPart>& GetBodyPartsFromBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetBonesFromBodyPart(EOHBodyPart Part);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetBonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);

	EOHBodyZone GetBodyZoneFromBone(EOHSkeletalBone Bone);
	EOHBodyPart GetBodyPartFromBone(EOHSkeletalBone Bone);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromBone(EOHSkeletalBone Bone);
	const TArray<EOHSkeletalBone>& GetBonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	
	EOHBodyPart GetPrimaryBodyPartFromZone(EOHBodyZone Zone);
	const TArray<EOHBodyPart>& GetBodyPartsFromZone(EOHBodyZone Zone);
	EOHFunctionalBoneGroup GetPrimaryFunctionalGroupFromZone(EOHBodyZone Zone);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromZone(EOHBodyZone Zone);
	const TArray<EOHSkeletalBone>& GetBonesFromZone(EOHBodyZone Zone);

	EOHBodyZone GetPrimaryZoneFromBodyPart(EOHBodyPart Part);
	const TArray<EOHBodyZone>& GetZonesFromBodyPart(EOHBodyPart Part);
	EOHFunctionalBoneGroup GetPrimaryFunctionalGroupFromBodyPart(EOHBodyPart Part);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromBodyPart(EOHBodyPart Part);

	EOHBodyZone GetPrimaryZoneFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	const TArray<EOHBodyZone>& GetZonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	EOHBodyPart GetPrimaryBodyPartFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	const TArray<EOHBodyPart>& GetBodyPartsFromFunctionalGroup(EOHFunctionalBoneGroup Group);

	TArray<EOHSkeletalBone> GetBonesFromZones(const TArray<EOHBodyZone>& Zones);
	TArray<EOHSkeletalBone> GetBonesFromBodyParts(const TArray<EOHBodyPart>& Parts);
	TArray<EOHSkeletalBone> GetBonesFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups);
	TArray<EOHBodyZone> GetZonesFromBodyParts(const TArray<EOHBodyPart>& Parts);
	TArray<EOHBodyZone> GetZonesFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups);
	TArray<EOHBodyPart> GetBodyPartsFromZones(const TArray<EOHBodyZone>& Zones);
	TArray<EOHBodyPart> GetBodyPartsFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups);


	EOHSkeletalBone GetSkeletalBoneFromFName(FName BoneName);
	FName GetFNameFromSkeletalBone(EOHSkeletalBone Bone);
	bool IsBoneNameValidInSkeletalMesh(USkeletalMesh* SkeletalMesh, FName BoneName);
	bool IsBoneNameValidInSkeleton(USkeleton* Skeleton, FName BoneName);
	void ValidateSkeletalBoneNamesInMesh(USkeletalMesh* SkeletalMesh);
	void ValidateSkeletalBoneNamesInSkeleton(USkeleton* Skeleton);

	int32 LevenshteinDistance(const FString& A, const FString& B);
	FString SuggestClosestBoneName_Levenshtein(const FName& MissingBoneName, const TArray<FName>& ExistingBoneNames);
	
	
	
	// Bone → Zone / BodyPart / FunctionalGroup
	EOHBodyZone GetBodyZoneFromBone(EOHSkeletalBone Bone);
	EOHBodyPart GetBodyPartFromBone(EOHSkeletalBone Bone);
	EOHFunctionalBoneGroup GetFunctionalGroupFromBone(EOHSkeletalBone Bone);
	const TArray<EOHBodyZone>& GetZonesFromBone(EOHSkeletalBone Bone);
	const TArray<EOHBodyPart>& GetBodyPartsFromBone(EOHSkeletalBone Bone);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromBone(EOHSkeletalBone Bone);

	// Zone → BodyPart / FunctionalGroup / Bones
	EOHBodyPart GetPrimaryBodyPartFromZone(EOHBodyZone Zone);
	const TArray<EOHBodyPart>& GetBodyPartsFromZone(EOHBodyZone Zone);
	EOHFunctionalBoneGroup GetPrimaryFunctionalGroupFromZone(EOHBodyZone Zone);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromZone(EOHBodyZone Zone);
	const TArray<EOHSkeletalBone>& GetBonesFromZone(EOHBodyZone Zone);
	const TArray<EOHSkeletalBone>& GetPrimaryBonesFromZone(EOHBodyZone Zone);

	// BodyPart → Zone / Group / Bones
	EOHBodyZone GetPrimaryZoneFromBodyPart(EOHBodyPart Part);
	const TArray<EOHBodyZone>& GetZonesFromBodyPart(EOHBodyPart Part);
	EOHFunctionalBoneGroup GetPrimaryFunctionalGroupFromBodyPart(EOHBodyPart Part);
	const TArray<EOHFunctionalBoneGroup>& GetFunctionalGroupsFromBodyPart(EOHBodyPart Part);
	const TArray<EOHSkeletalBone>& GetBonesFromBodyPart(EOHBodyPart Part);
	const TArray<EOHSkeletalBone>& GetPrimaryBonesFromBodyPart(EOHBodyPart Part);
	TArray<FName> GetPrimaryBoneNamesFromBodyPart(EOHBodyPart Part);
	TArray<FName> GetBoneNamesFromBodyPartPreferPrimary(EOHBodyPart Part);


	// FunctionalGroup → Zone / BodyPart / Bones
	EOHBodyZone GetPrimaryZoneFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	const TArray<EOHBodyZone>& GetZonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	EOHBodyPart GetPrimaryBodyPartFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	const TArray<EOHBodyPart>& GetBodyPartsFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	const TArray<EOHSkeletalBone>& GetBonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	const TArray<EOHSkeletalBone>& GetPrimaryBonesFromFunctionalGroup(EOHFunctionalBoneGroup Group);

	// Body Zone
	TArray<FName> GetPrimaryBoneNamesFromBodyZone(EOHBodyZone Zone);
	TArray<FName> GetBoneNamesFromBodyZonePreferPrimary(EOHBodyZone Zone);

	// Functional Group
	TArray<FName> GetPrimaryBoneNamesFromFunctionalGroup(EOHFunctionalBoneGroup Group);
	TArray<FName> GetBoneNamesFromFunctionalGroupPreferPrimary(EOHFunctionalBoneGroup Group);
	

	
	// Batch Accessors
	TArray<EOHSkeletalBone> GetBonesFromZones(const TArray<EOHBodyZone>& Zones);
	TArray<EOHSkeletalBone> GetBonesFromBodyParts(const TArray<EOHBodyPart>& Parts);
	TArray<EOHSkeletalBone> GetBonesFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups);
	TArray<EOHBodyZone> GetZonesFromBodyParts(const TArray<EOHBodyPart>& Parts);
	TArray<EOHBodyZone> GetZonesFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups);
	TArray<EOHBodyPart> GetBodyPartsFromZones(const TArray<EOHBodyZone>& Zones);
	TArray<EOHBodyPart> GetBodyPartsFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups);

	// FName Conversions
	EOHSkeletalBone GetSkeletalBoneFromFName(FName BoneName);
	FName GetFNameFromSkeletalBone(EOHSkeletalBone Bone);

	// Validation & Verification
	void VerifyMappings();
	bool IsBoneNameValidInSkeletalMesh(USkeletalMesh* SkeletalMesh, FName BoneName);
	bool IsBoneNameValidInSkeleton(USkeleton* Skeleton, FName BoneName);
	void ValidateSkeletalBoneNamesInMesh(USkeletalMesh* SkeletalMesh);
	void ValidateSkeletalBoneNamesInSkeleton(USkeleton* Skeleton);

	// Enum to String
	FString ToString(EOHSkeletalBone Bone);
	FString ToString(EOHBodyPart Part);
	FString ToString(EOHBodyZone Zone);
	FString ToString(EOHFunctionalBoneGroup Group);

	// String to Enum with Levenshtein fallback
	bool TryParseSkeletalBone(const FString& Name, EOHSkeletalBone& Out);
	bool TryParseBodyPart(const FString& Name, EOHBodyPart& Out, bool bUseLevenshteinFallback = true);
	bool TryParseBodyZone(const FString& Name, EOHBodyZone& Out);
	bool TryParseFunctionalGroup(const FString& Name, EOHFunctionalBoneGroup& Out);

	// SkeletalBone <-> FName mapping
	FName ToFName(EOHSkeletalBone Bone);
	EOHSkeletalBone FromFName(FName BoneName);
	// Enum <-> FName conversion
	FName ToFName(EOHBodyPart Part);
	FName ToFName(EOHBodyZone Zone);
	FName ToFName(EOHFunctionalBoneGroup Group);

	EOHBodyPart FromFNameToBodyPart(FName Name, bool bUseLevenshteinFallback = true);
	EOHBodyZone FromFNameToBodyZone(FName Name, bool bUseLevenshteinFallback = true);
	TArray<FName> GetBoneNamesFromBodyPart(EOHBodyPart BodyPart);
	TArray<FName> GetBoneNamesFromBodyPart(EOHBodyPart BodyPart, bool bUseLevenshteinFallback = true);

	TArray<FName> GetBoneNamesFromBodyZone(EOHBodyZone Zone, bool bUseLevenshteinFallback = true);
	TArray<FName> GetBoneNamesFromFunctionalGroup(EOHFunctionalBoneGroup Group, bool bUseLevenshteinFallback = true);

	FName GetMostLikelyBoneNameFromGroup(EOHFunctionalBoneGroup Group, bool bVerbose = false);
	FName GetMostLikelyBoneNameFromBodyPart(EOHBodyPart BodyPart, bool bVerbose = false);
	FName GetMostLikelyBoneNameFromBodyZone(EOHBodyZone Zone, bool bVerbose = false);
	
	TArray<FName> GetMostLikelyBoneNamesFromBodyParts(const TArray<EOHBodyPart>& Parts, bool bVerbose = false);
	TArray<FName> GetMostLikelyBoneNamesFromBodyZones(const TArray<EOHBodyZone>& Zones, bool bVerbose = false);

	FName GetMostLikelyBoneName(EOHBodyPart Part, EOHBodyZone Zone, EOHFunctionalBoneGroup Group, bool bVerbose = false);

	FName GetPreferredBoneNameFromBodyPart(EOHBodyPart Part, EOHBodyZone Zone = EOHBodyZone::None, EOHFunctionalBoneGroup Group = EOHFunctionalBoneGroup::None, bool bVerbose = false);

	
	EOHFunctionalBoneGroup FromFNameToFunctionalGroup(FName Name, bool bUseLevenshteinFallback = true);
	// Utilities

	TArray<FName> GetBoneNamesFromMapPreferPrimary(
		TFunction<const TArray<EOHSkeletalBone>*()> GetPrimary,
		TFunction<const TArray<EOHSkeletalBone>*()> GetFallback
	);

	void GetBonesMatching(
		TOptional<EOHBodyZone> Zone,
		TOptional<EOHBodyPart> Part,
		TOptional<EOHFunctionalBoneGroup> Group,
		bool bRequirePrimary,
		TArray<EOHSkeletalBone>& OutBones);

	/** Returns array of all primary skeletal bones (enum form) */
	TArray<EOHSkeletalBone> GetPrimarySkeletalBones();

	
	/** Returns array of all primary skeletal bones (as FNames) */
	TArray<FName> GetPrimaryBones();

	static bool IsBoneValidForSimulation(FName Bone, const USkeletalMeshComponent* Mesh, const UOHPhysicsManager* Manager);
	static bool IsBoneValidForTracking(FName Bone, const USkeletalMeshComponent* Mesh);
	
	int32 LevenshteinDistance(const FString& A, const FString& B);
	FString SuggestClosestBoneName_Levenshtein(const FName& MissingBoneName, const TArray<FName>& ExistingBoneNames);

	static EOHSkeletalBone ResolveSkeletalBoneFromNameSmart_Trackable(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold = 0.6f);
	static EOHSkeletalBone ResolveSkeletalBoneFromNameSmart_Simulatable(FName RawName, const USkeletalMeshComponent* Mesh, const UOHPhysicsManager* PhysicsManager, float MinScoreThreshold = 0.6f);
	static EOHSkeletalBone ResolveSkeletalBoneFromSocketSmart_Simulatable(FName SocketName, const USkeletalMeshComponent* Mesh, const UOHPhysicsManager* PhysicsManager, float MinScoreThreshold = 0.6f);
	
	EOHSkeletalBone ResolveSkeletalBoneFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold = 0.75f);
	EOHBodyZone ResolveBodyZoneFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold = 0.75f);
	EOHBodyPart ResolveBodyPartFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold = 0.75f);
	EOHFunctionalBoneGroup ResolveFunctionalGroupFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold = 0.75f);

#endif
