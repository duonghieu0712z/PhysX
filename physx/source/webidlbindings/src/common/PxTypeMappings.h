#ifndef PX_TYPE_MAPPINGS_H
#define PX_TYPE_MAPPINGS_H

#include "PxPhysicsAPI.h"

using namespace physx;

typedef PxActor *PxActorPtr;

typedef PxOverlapBufferN<10> PxOverlapBuffer10;
typedef PxRaycastBufferN<10> PxRaycastBuffer10;
typedef PxSweepBufferN<10> PxSweepBuffer10;

typedef PxCustomGeometry::Callbacks PxCustomGeometryCallbacks;

template <class T>
class PxVector : public PxArray<T>
{
public:
    PX_INLINE PxVector() : PxArray<T>() {}
    PX_INLINE PxVector(uint32_t size, const T &value = T()) : PxArray<T>(size, value) {}

    PX_INLINE T &get(uint32_t index) { return this->operator[](index); }
    PX_INLINE void set(uint32_t index, const T &value) { this->operator[](index) = value; }

    PX_INLINE const T *data() const { return this->begin(); }
};

typedef PxVector<PxContactPairPoint> Vector_PxContactPairPoint;
typedef PxVector<PxHeightFieldSample> Vector_PxHeightFieldSample;
typedef PxVector<PxRaycastHit> Vector_PxRaycastHit;
typedef PxVector<PxSweepHit> Vector_PxSweepHit;
typedef PxVector<PxVec3> Vector_PxVec3;
typedef PxVector<PxU16> Vector_PxU16;

// enums within namespaces are not supported by webidl binder, use typedefs to work around that

typedef PxActorFlag::Enum PxActorFlagEnum;
typedef PxActorType::Enum PxActorTypeEnum;
typedef PxActorTypeFlag::Enum PxActorTypeFlagEnum;
typedef PxArticulationAxis::Enum PxArticulationAxisEnum;
typedef PxArticulationDriveType::Enum PxArticulationDriveTypeEnum;
typedef PxArticulationFlag::Enum PxArticulationFlagEnum;
typedef PxArticulationJointType::Enum PxArticulationJointTypeEnum;
typedef PxArticulationKinematicFlag::Enum PxArticulationKinematicFlagEnum;
typedef PxArticulationMotion::Enum PxArticulationMotionEnum;
// typedef PxBaseFlag::Enum PxBaseFlagEnum;
typedef PxCapsuleClimbingMode::Enum PxCapsuleClimbingModeEnum;
typedef PxCombineMode::Enum PxCombineModeEnum;
typedef PxConstraintFlag::Enum PxConstraintFlagEnum;
// typedef PxConstraintVisualizationFlag::Enum PxConstraintVisualizationFlagEnum;
// typedef PxContactPairExtraDataType::Enum PxContactPairExtraDataTypeEnum;
typedef PxContactPairFlag::Enum PxContactPairFlagEnum;
typedef PxContactPairHeaderFlag::Enum PxContactPairHeaderFlagEnum;
// typedef PxControllerBehaviorFlag::Enum PxControllerBehaviorFlagEnum;
// typedef PxControllerCollisionFlag::Enum PxControllerCollisionFlagEnum;
typedef PxControllerDebugRenderFlag::Enum PxControllerDebugRenderFlagEnum;
typedef PxControllerNonWalkableMode::Enum PxControllerNonWalkableModeEnum;
typedef PxControllerShapeType::Enum PxControllerShapeTypeEnum;
typedef PxConvexFlag::Enum PxConvexFlagEnum;
typedef PxConvexMeshCookingType::Enum PxConvexMeshCookingTypeEnum;
typedef PxConvexMeshGeometryFlag::Enum PxConvexMeshGeometryFlagEnum;
typedef PxD6Axis::Enum PxD6AxisEnum;
typedef PxD6Drive::Enum PxD6DriveEnum;
typedef PxD6JointDriveFlag::Enum PxD6JointDriveFlagEnum;
typedef PxD6Motion::Enum PxD6MotionEnum;
typedef PxDebugColor::Enum PxDebugColorEnum;
typedef PxDistanceJointFlag::Enum PxDistanceJointFlagEnum;
typedef PxFilterFlag::Enum PxFilterFlagEnum;
typedef PxFilterObjectFlag::Enum PxFilterObjectFlagEnum;
typedef PxFilterObjectType::Enum PxFilterObjectTypeEnum;
typedef PxForceMode::Enum PxForceModeEnum;
typedef PxFrictionType::Enum PxFrictionTypeEnum;
typedef PxGeometryType::Enum PxGeometryTypeEnum;
typedef PxHeightFieldFlag::Enum PxHeightFieldFlagEnum;
typedef PxHeightFieldFormat::Enum PxHeightFieldFormatEnum;
typedef PxHitFlag::Enum PxHitFlagEnum;
typedef PxJointActorIndex::Enum PxJointActorIndexEnum;
typedef PxMaterialFlag::Enum PxMaterialFlagEnum;
typedef PxMeshFlag::Enum PxMeshFlagEnum;
typedef PxMeshGeometryFlag::Enum PxMeshGeometryFlagEnum;
typedef PxMeshPreprocessingFlag::Enum PxMeshPreprocessingFlagEnum;
typedef PxPairFilteringMode::Enum PxPairFilteringModeEnum;
typedef PxPairFlag::Enum PxPairFlagEnum;
typedef PxPrismaticJointFlag::Enum PxPrismaticJointFlagEnum;
typedef PxQueryFlag::Enum PxQueryFlagEnum;
typedef PxQueryHitType::Enum PxQueryHitTypeEnum;
typedef PxRevoluteJointFlag::Enum PxRevoluteJointFlagEnum;
typedef PxRigidBodyFlag::Enum PxRigidBodyFlagEnum;
typedef PxRigidDynamicLockFlag::Enum PxRigidDynamicLockFlagEnum;
typedef PxSceneFlag::Enum PxSceneFlagEnum;
typedef PxShapeFlag::Enum PxShapeFlagEnum;
typedef PxSolverType::Enum PxSolverTypeEnum;
typedef PxSphericalJointFlag::Enum PxSphericalJointFlagEnum;
typedef PxTriangleMeshFlag::Enum PxTriangleMeshFlagEnum;
typedef PxTriggerPairFlag::Enum PxTriggerPairFlagEnum;
typedef PxVisualizationParameter::Enum PxVisualizationParameterEnum;

typedef PxEMPTY PxEMPTYEnum;
typedef PxIDENTITY PxIDENTITYEnum;
typedef PxZERO PxZEROEnum;

#endif
