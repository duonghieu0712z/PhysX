#ifndef WEB_IDL_BINDINGS_H
#define WEB_IDL_BINDINGS_H

#include <iostream>

#include "PxPhysicsAPI.h"

#include "PxTypeMappings.h"

using namespace std;
using namespace physx;

PxFilterFlags defaultFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                                  PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                                  PxPairFlags &pairFlags, const void *, PxU32)
{
    if (!(filterData0.word0 & filterData1.word1) && !(filterData1.word0 & filterData0.word1))
    {
        return PxFilterFlag::eSUPPRESS;
    }

    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
    {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
    }
    else
    {
        pairFlags = PxPairFlag::eCONTACT_DEFAULT;
    }
    pairFlags |= PxPairFlags(static_cast<PxU16>(filterData0.word2 | filterData1.word2));

    return PxFilterFlag::eDEFAULT;
};

class PxSimulationFilterShaderCallback
{
public:
    virtual ~PxSimulationFilterShaderCallback() {};

    virtual PxU32 onFilterShader(PxFilterObjectAttributes attributes0, const PxFilterData &filterData0,
                                 PxFilterObjectAttributes attributes1, const PxFilterData &filterData1,
                                 const PxPairFlags &pairFlags) = 0;
};

static PxSimulationFilterShaderCallback *gFilterShaderCallback = nullptr;

static PxFilterFlags customFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                                        PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                                        PxPairFlags &pairFlags, const void *, PxU32)
{
    PxU32 filterFlags = gFilterShaderCallback->onFilterShader(attributes0, filterData0, attributes1, filterData1, pairFlags);
    return PxFilterFlags(static_cast<PxU16>(filterFlags));
}

class PxSimulationEventCallbackSimple : public PxSimulationEventCallback
{
public:
    virtual void onAdvance(const PxRigidBody *const *, const PxTransform *, const PxU32) override {}
};

class PxQueryFilterCallbackSimple : public PxQueryFilterCallback
{
public:
    virtual PxU32 onPreFilter(const PxFilterData &filterData, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &queryFlags) = 0;
    virtual PxU32 onPostFilter(const PxFilterData &filterData, const PxQueryHit &hit, const PxShape *shape, const PxRigidActor *actor) = 0;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &filterData, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &queryFlags) override
    {
        return static_cast<PxQueryHitType::Enum>(onPreFilter(filterData, shape, actor, queryFlags));
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &filterData, const PxQueryHit &hit, const PxShape *shape, const PxRigidActor *actor) override
    {
        return static_cast<PxQueryHitType::Enum>(onPostFilter(filterData, hit, shape, actor));
    }
};

struct TopLevelFunctions
{
    static const PxU32 PHYSICS_VERSION = PX_PHYSICS_VERSION;

    // Create filter shaders

    static PxSimulationFilterShader DefaultFilterShader()
    {
        return &defaultFilterShader;
    }

    static PxSimulationFilterShader CustomFilterShader(PxSimulationFilterShaderCallback *filterShader)
    {
        gFilterShaderCallback = filterShader;
        return &customFilterShader;
    }

    // Filter object attributes

    static bool FilterObjectIsKinematic(PxFilterObjectAttributes attributes)
    {
        return PxFilterObjectIsKinematic(attributes);
    }

    static bool FilterObjectIsTrigger(PxFilterObjectAttributes attributes)
    {
        return PxFilterObjectIsTrigger(attributes);
    }

    // Create physics

    static PxFoundation *CreateFoundation(PxU32 version, PxAllocatorCallback &allocator, PxErrorCallback &errorCallback)
    {
        return PxCreateFoundation(version, allocator, errorCallback);
    }

    static PxPhysics *CreatePhysics(PxU32 version, PxFoundation &foundation, const PxTolerancesScale &scale, bool trackOutstandingAllocations = false, PxPvd *pvd = nullptr)
    {
        return PxCreatePhysics(version, foundation, scale, trackOutstandingAllocations, pvd);
    }

    static PxPvd *CreatePvd(PxFoundation &foundation)
    {
        return PxCreatePvd(foundation);
    }

    static PxControllerManager *CreateControllerManager(PxScene &scene, bool lockingEnabled = false)
    {
        return PxCreateControllerManager(scene, lockingEnabled);
    }

    static PxDefaultCpuDispatcher *CreateDefaultCpuDispatcher(PxU32 numThreads)
    {
        return PxDefaultCpuDispatcherCreate(numThreads);
    }

    static bool InitExtensions(PxPhysics &physics, PxPvd *pvd)
    {
        return PxInitExtensions(physics, pvd);
    }

    static void CloseExtensions()
    {
        PxCloseExtensions();
    }

    // Create joints

    static PxD6Joint *CreateD6Joint(PxPhysics &physics, PxRigidActor *actor0, PxTransform &localFrame0, PxRigidActor *actor1, PxTransform &localFrame1)
    {
        return PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static PxDistanceJoint *CreateDistanceJoint(PxPhysics &physics, PxRigidActor *actor0, PxTransform &localFrame0, PxRigidActor *actor1, PxTransform &localFrame1)
    {
        return PxDistanceJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static PxFixedJoint *CreateFixedJoint(PxPhysics &physics, PxRigidActor *actor0, PxTransform &localFrame0, PxRigidActor *actor1, PxTransform &localFrame1)
    {
        return PxFixedJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static PxPrismaticJoint *CreatePrismaticJoint(PxPhysics &physics, PxRigidActor *actor0, PxTransform &localFrame0, PxRigidActor *actor1, PxTransform &localFrame1)
    {
        return PxPrismaticJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static PxRevoluteJoint *CreateRevoluteJoint(PxPhysics &physics, PxRigidActor *actor0, PxTransform &localFrame0, PxRigidActor *actor1, PxTransform &localFrame1)
    {
        return PxRevoluteJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static PxSphericalJoint *CreateSphericalJoint(PxPhysics &physics, PxRigidActor *actor0, PxTransform &localFrame0, PxRigidActor *actor1, PxTransform &localFrame1)
    {
        return PxSphericalJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    // Create meshes

    static PxConvexMesh *CreateConvexMesh(const PxCookingParams &params, const Vector_PxVec3 &points)
    {
        PxConvexMeshDesc desc;
        desc.points.count = points.size();
        desc.points.stride = sizeof(PxVec3);
        desc.points.data = points.data();
        desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
        return PxCreateConvexMesh(params, desc);
    }

    static PxTriangleMesh *CreateTriangleMesh(const PxCookingParams &params, const Vector_PxVec3 &points, const Vector_PxU16 &triangles)
    {
        PxTriangleMeshDesc desc;
        desc.points.count = points.size();
        desc.points.stride = sizeof(PxVec3);
        desc.points.data = points.data();
        desc.triangles.count = triangles.size() / 3;
        desc.triangles.stride = 3 * sizeof(PxU16);
        desc.triangles.data = triangles.data();
        desc.flags = PxMeshFlag::e16_BIT_INDICES;
        return PxCreateTriangleMesh(params, desc);
    }

    static PxHeightField *CreateHeightField(PxU32 nbColumns, PxU32 nbRows, const Vector_PxHeightFieldSample &samples)
    {
        PxHeightFieldDesc desc;
        desc.nbColumns = nbColumns;
        desc.nbRows = nbRows;
        desc.samples.data = samples.data();
        desc.samples.stride = sizeof(PxHeightFieldSample);
        return PxCreateHeightField(desc);
    }

    static PxRigidStatic *CreatePlane(PxPhysics &physics, const PxPlane &plane, PxMaterial &material, PxShapeFlags shapeFlags, const PxFilterData &filterData)
    {
        PxRigidStatic *actor = PxCreatePlane(physics, plane, material);
        PxShape *shape;
        actor->getShapes(&shape, 1, 0);
        shape->setFlags(shapeFlags);
        shape->setQueryFilterData(filterData);
        shape->setSimulationFilterData(filterData);
        return actor;
    }
};

struct ExtensionFunctions
{
    static PxArticulationTendonJoint *PxArticulationFixedTendon_getTendonJoint(const PxArticulationFixedTendon &tendon, PxU32 index)
    {
        PxArticulationTendonJoint *joint;
        tendon.getTendonJoints(&joint, 1, index);
        return joint;
    }

    static PxArticulationLink *PxArticulationLink_getChild(const PxArticulationLink &link, PxU32 index)
    {
        PxArticulationLink *child;
        link.getChildren(&child, 1, index);
        return child;
    }

    static PxArticulationLink *PxArticulationReducedCoordinate_getLink(const PxArticulationReducedCoordinate &articulation, PxU32 index)
    {
        PxArticulationLink *link;
        articulation.getLinks(&link, 1, index);
        return link;
    }

    static PxArticulationSpatialTendon *PxArticulationReducedCoordinate_getSpatialTendon(const PxArticulationReducedCoordinate &articulation, PxU32 index)
    {
        PxArticulationSpatialTendon *tendon;
        articulation.getSpatialTendons(&tendon, 1, index);
        return tendon;
    }

    static PxArticulationFixedTendon *PxArticulationReducedCoordinate_getFixedTendon(const PxArticulationReducedCoordinate &articulation, PxU32 index)
    {
        PxArticulationFixedTendon *tendon;
        articulation.getFixedTendons(&tendon, 1, index);
        return tendon;
    }

    static PxArticulationAttachment *PxArticulationSpatialTendon_getAttachment(const PxArticulationSpatialTendon &tendon, PxU32 index)
    {
        PxArticulationAttachment *attachment;
        tendon.getAttachments(&attachment, 1, index);
        return attachment;
    }

    static PxRigidActor *PxConstraint_getActor0(const PxConstraint &constraint)
    {
        PxRigidActor *actor0, *actor1;
        constraint.getActors(actor0, actor1);
        return actor0;
    }

    static PxRigidActor *PxConstraint_getActor1(const PxConstraint &constraint)
    {
        PxRigidActor *actor0, *actor1;
        constraint.getActors(actor0, actor1);
        return actor1;
    }

    static PxVec3 PxConstraint_getLinearForce(const PxConstraint &constraint)
    {
        PxVec3 linear, angular;
        constraint.getForce(linear, angular);
        return linear;
    }

    static PxVec3 PxConstraint_getAngularForce(const PxConstraint &constraint)
    {
        PxVec3 linear, angular;
        constraint.getForce(linear, angular);
        return angular;
    }

    static PxHullPolygon PxConvexMesh_getPolygon(const PxConvexMesh &mesh, PxU32 index)
    {
        PxHullPolygon polygon;
        mesh.getPolygonData(index, polygon);
        return polygon;
    }

    static PxRigidActor *PxJoint_getActor0(const PxJoint &joint)
    {
        PxRigidActor *actor0, *actor1;
        joint.getActors(actor0, actor1);
        return actor0;
    }

    static PxRigidActor *PxJoint_getActor1(const PxJoint &joint)
    {
        PxRigidActor *actor0, *actor1;
        joint.getActors(actor0, actor1);
        return actor1;
    }

    static PxReal PxJoint_getBreakForce(const PxJoint &joint)
    {
        PxReal force, torque;
        joint.getBreakForce(force, torque);
        return force;
    }

    static PxReal PxJoint_getBreakTorque(const PxJoint &joint)
    {
        PxReal force, torque;
        joint.getBreakForce(force, torque);
        return torque;
    }

    static PxTriangleMesh *PxPhysics_getTriangleMesh(const PxPhysics &physics, PxU32 index)
    {
        PxTriangleMesh *mesh;
        physics.getTriangleMeshes(&mesh, 1, index);
        return mesh;
    }

    static PxHeightField *PxPhysics_getHeightField(const PxPhysics &physics, PxU32 index)
    {
        PxHeightField *heightField;
        physics.getHeightFields(&heightField, 1, index);
        return heightField;
    }

    static PxConvexMesh *PxPhysics_getConvexMesh(const PxPhysics &physics, PxU32 index)
    {
        PxConvexMesh *mesh;
        physics.getConvexMeshes(&mesh, 1, index);
        return mesh;
    }

    static PxScene *PxPhysics_getScene(const PxPhysics &physics, PxU32 index)
    {
        PxScene *scene;
        physics.getScenes(&scene, 1, index);
        return scene;
    }

    static PxShape *PxPhysics_getShape(const PxPhysics &physics, PxU32 index)
    {
        PxShape *shape;
        physics.getShapes(&shape, 1, index);
        return shape;
    }

    static PxMaterial *PxPhysics_getMaterial(const PxPhysics &physics, PxU32 index)
    {
        PxMaterial *material;
        physics.getMaterials(&material, 1, index);
        return material;
    }

    static PxShape *PxRigidActor_getShape(const PxRigidActor &actor, PxU32 index)
    {
        PxShape *shape;
        actor.getShapes(&shape, 1, index);
        return shape;
    }

    static PxConstraint *PxRigidActor_getConstraint(const PxRigidActor &actor, PxU32 index)
    {
        PxConstraint *constraint;
        actor.getConstraints(&constraint, 1, index);
        return constraint;
    }

    static void PxRigidBody_applyGlobalForce(PxRigidBody &body, const PxVec3 &force, PxForceMode::Enum mode, const PxVec3 &point)
    {
        if (!force.isZero())
        {
            body.addForce(force, mode);
            PxVec3 torque = point.cross(force);
            if (!torque.isZero())
            {
                body.addTorque(torque, mode);
            }
        }
    }

    static void PxRigidBody_applyLocalForce(PxRigidBody &body, const PxVec3 &force, PxForceMode::Enum mode, const PxVec3 &point)
    {
        if (!force.isZero())
        {
            PxTransform pose = body.getGlobalPose();
            PxVec3 worldForce = pose.rotate(force);
            body.addForce(worldForce, mode);

            PxVec3 worldPoint = pose.rotate(point);
            PxVec3 torque = worldPoint.cross(worldForce);
            if (!torque.isZero())
            {
                body.addTorque(torque, mode);
            }
        }
    }

    static PxTransform PxRigidDynamic_getKinematicTarget(const PxRigidDynamic &actor)
    {
        PxTransform target;
        actor.getKinematicTarget(target);
        return target;
    }

    static PxActor *PxScene_getActor(const PxScene &scene, PxActorTypeFlags types, PxU32 index)
    {
        PxActor *actor;
        scene.getActors(types, &actor, 1, index);
        return actor;
    }

    static PxArticulationReducedCoordinate *PxScene_getArticulation(const PxScene &scene, PxU32 index)
    {
        PxArticulationReducedCoordinate *articulation;
        scene.getArticulations(&articulation, 1, index);
        return articulation;
    }

    static PxConstraint *PxScene_getConstraint(const PxScene &scene, PxU32 index)
    {
        PxConstraint *constraint;
        scene.getConstraints(&constraint, 1, index);
        return constraint;
    }

    static void PxShape_setMaterial(PxShape &shape, const PxMaterial &material)
    {
        PxMaterial *materialPtr = const_cast<PxMaterial *>(&material);
        shape.setMaterials(&materialPtr, 1);
    }

    static void PxShape_setMaterials(PxShape &shape, const PxMaterial *materials, PxU16 count)
    {
        PxMaterial *materialPtr = const_cast<PxMaterial *>(materials);
        shape.setMaterials(&materialPtr, count);
    }

    static PxMaterial *PxShape_getMaterial(const PxShape &shape, PxU32 index)
    {
        PxMaterial *material;
        shape.getMaterials(&material, 1, index);
        return material;
    }
};

struct ArrayHelpers
{
    // Event callback helpers

    static PxContactPair *getContactPairAt(PxContactPair *base, PxU32 index)
    {
        return &base[index];
    }

    static PxTriggerPair *getTriggerPairAt(PxTriggerPair *base, PxU32 index)
    {
        return &base[index];
    }

    // Debug helpers

    static PxDebugPoint *getDebugPointAt(PxDebugPoint *base, PxU32 index)
    {
        return &base[index];
    }

    static PxDebugLine *getDebugLineAt(PxDebugLine *base, PxU32 index)
    {
        return &base[index];
    }

    static PxDebugTriangle *getDebugTriangleAt(PxDebugTriangle *base, PxU32 index)
    {
        return &base[index];
    }
};

#endif // WEB_IDL_BINDING_H
