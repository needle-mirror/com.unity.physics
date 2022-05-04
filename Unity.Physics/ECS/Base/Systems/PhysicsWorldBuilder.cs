using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine.Assertions;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// Utilities for building a physics world.
    /// </summary>
    public static class PhysicsWorldBuilder
    {
        /// <summary>
        /// Schedule jobs to fill the PhysicsWorld in specified physicsData with bodies and joints (using entities from physicsData's queries) and build broadphase BoundingVolumeHierarchy.
        /// Needs a system to get read handles to Entity and all physics-related components.
        /// </summary>
        public static JobHandle SchedulePhysicsWorldBuild(SystemBase system, ref PhysicsWorldData physicsData,
            in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion)
            =>
            SchedulePhysicsWorldBuild(system, ref physicsData.PhysicsWorld, ref physicsData.HaveStaticBodiesChanged,
                inputDep, timeStep, isBroadphaseBuildMultiThreaded, gravity, lastSystemVersion,
                physicsData.DynamicEntityGroup, physicsData.StaticEntityGroup, physicsData.JointEntityGroup);

        /// <summary>
        /// Schedule jobs to fill specified PhysicsWorld with bodies and joints (using entities from specified queries) and build broadphase BoundingVolumeHierarchy.
        /// Needs a system to get read handles to Entity and all physics-related components.
        /// </summary>
        public static JobHandle SchedulePhysicsWorldBuild(
            SystemBase system,
            ref PhysicsWorld world, ref NativeArray<int> haveStaticBodiesChanged,
            in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion,
            EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)
        {
            JobHandle finalHandle = inputDep;

            EntityTypeHandle entityType = system.GetEntityTypeHandle();
            ComponentTypeHandle<LocalToWorld> localToWorldType = system.GetComponentTypeHandle<LocalToWorld>(true);
            ComponentTypeHandle<Parent> parentType = system.GetComponentTypeHandle<Parent>(true);
            ComponentTypeHandle<Translation> positionType = system.GetComponentTypeHandle<Translation>(true);
            ComponentTypeHandle<Rotation> rotationType = system.GetComponentTypeHandle<Rotation>(true);
            ComponentTypeHandle<PhysicsCollider> physicsColliderType = system.GetComponentTypeHandle<PhysicsCollider>(true);
            ComponentTypeHandle<PhysicsVelocity> physicsVelocityType = system.GetComponentTypeHandle<PhysicsVelocity>(true);
            ComponentTypeHandle<PhysicsMass> physicsMassType = system.GetComponentTypeHandle<PhysicsMass>(true);
            ComponentTypeHandle<PhysicsMassOverride> physicsMassOverrideType = system.GetComponentTypeHandle<PhysicsMassOverride>(true);
            ComponentTypeHandle<PhysicsDamping> physicsDampingType = system.GetComponentTypeHandle<PhysicsDamping>(true);
            ComponentTypeHandle<PhysicsGravityFactor> physicsGravityFactorType = system.GetComponentTypeHandle<PhysicsGravityFactor>(true);
            ComponentTypeHandle<PhysicsCustomTags> physicsCustomTagsType = system.GetComponentTypeHandle<PhysicsCustomTags>(true);
            ComponentTypeHandle<PhysicsConstrainedBodyPair> physicsConstrainedBodyPairType = system.GetComponentTypeHandle<PhysicsConstrainedBodyPair>(true);
            ComponentTypeHandle<PhysicsJoint> physicsJointType = system.GetComponentTypeHandle<PhysicsJoint>(true);

            int numDynamicBodies = dynamicEntityGroup.CalculateEntityCount();
            int numStaticBodies = staticEntityGroup.CalculateEntityCount();
            int numJoints = jointEntityGroup.CalculateEntityCount();

            int previousStaticBodyCount = world.NumStaticBodies;

            // Early out if world is empty and it's been like that in previous frame as well (it contained only the default static body)
            if (numDynamicBodies + numStaticBodies == 0 && world.NumBodies == 1)
            {
                // No bodies in the scene, no need to do anything else
                haveStaticBodiesChanged[0] = 0;
                return finalHandle;
            }

            // Resize the world's native arrays
            world.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            // Determine if the static bodies have changed in any way that will require the static broadphase tree to be rebuilt
            JobHandle staticBodiesCheckHandle = default;

            haveStaticBodiesChanged[0] = 0;
            {
                if (world.NumStaticBodies != previousStaticBodyCount)
                {
                    haveStaticBodiesChanged[0] = 1;
                }
                else
                {
                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        m_LastSystemVersion = lastSystemVersion,
                        Result = haveStaticBodiesChanged
                    }.ScheduleParallel(staticEntityGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, inputDep);
                }
            }

            using (var jobHandles = new NativeList<JobHandle>(4, Allocator.Temp))
            {
                // Static body changes check jobs
                jobHandles.Add(staticBodiesCheckHandle);

                // Create the default static body at the end of the body list
                // TODO: could skip this if no joints present
                jobHandles.Add(new Jobs.CreateDefaultStaticRigidBody
                {
                    NativeBodies = world.Bodies,
                    BodyIndex = world.Bodies.Length - 1,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                }.Schedule(inputDep));

                // Dynamic bodies.
                // Create these separately from static bodies to maintain a 1:1 mapping
                // between dynamic bodies and their motions.
                if (numDynamicBodies > 0)
                {
                    jobHandles.Add(new Jobs.CreateRigidBodies
                    {
                        EntityType = entityType,
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomTagsType = physicsCustomTagsType,

                        FirstBodyIndex = 0,
                        RigidBodies = world.Bodies,
                        EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                    }.ScheduleParallel(dynamicEntityGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, inputDep));

                    jobHandles.Add(new Jobs.CreateMotions
                    {
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsVelocityType = physicsVelocityType,
                        PhysicsMassType = physicsMassType,
                        PhysicsMassOverrideType = physicsMassOverrideType,
                        PhysicsDampingType = physicsDampingType,
                        PhysicsGravityFactorType = physicsGravityFactorType,

                        MotionDatas = world.MotionDatas,
                        MotionVelocities = world.MotionVelocities
                    }.ScheduleParallel(dynamicEntityGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, inputDep));
                }

                // Now, schedule creation of static bodies, with FirstBodyIndex pointing after
                // the dynamic and kinematic bodies
                if (numStaticBodies > 0)
                {
                    jobHandles.Add(new Jobs.CreateRigidBodies
                    {
                        EntityType = entityType,
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomTagsType = physicsCustomTagsType,

                        FirstBodyIndex = numDynamicBodies,
                        RigidBodies = world.Bodies,
                        EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                    }.ScheduleParallel(staticEntityGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, inputDep));
                }

                var combinedHandle = JobHandle.CombineDependencies(jobHandles);
                jobHandles.Clear();

                // Build joints
                if (numJoints > 0)
                {
                    jobHandles.Add(new Jobs.CreateJoints
                    {
                        ConstrainedBodyPairComponentType = physicsConstrainedBodyPairType,
                        JointComponentType = physicsJointType,
                        EntityType = entityType,
                        RigidBodies = world.Bodies,
                        Joints = world.Joints,
                        DefaultStaticBodyIndex = world.Bodies.Length - 1,
                        NumDynamicBodies = numDynamicBodies,
                        EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap,
                        EntityJointIndexMap = world.DynamicsWorld.EntityJointIndexMap.AsParallelWriter(),
                    }.ScheduleParallel(jointEntityGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, combinedHandle));
                }

                JobHandle buildBroadphaseHandle = world.CollisionWorld.ScheduleBuildBroadphaseJobs(
                    ref world, timeStep, gravity,
                    haveStaticBodiesChanged, combinedHandle, isBroadphaseBuildMultiThreaded);
                jobHandles.Add(buildBroadphaseHandle);

                finalHandle = JobHandle.CombineDependencies(inputDep, JobHandle.CombineDependencies(jobHandles));
            }

            return finalHandle;
        }

        /// <summary>
        /// Schedule jobs to build broadphase BoundingVolumeHierarchy of the specified PhysicsWorld
        /// </summary>
        public static JobHandle ScheduleBroadphaseBVHBuild(ref PhysicsWorld world, ref NativeArray<int> haveStaticBodiesChanged,
            in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity)
        {
            return world.CollisionWorld.ScheduleBuildBroadphaseJobs(
                ref world, timeStep, gravity,
                haveStaticBodiesChanged, inputDep, isBroadphaseBuildMultiThreaded);
        }

        /// <summary>
        /// Fill the PhysicsWorld in specified physicsData with bodies and joints (using entities from physicsData's queries) and build broadphase BoundingVolumeHierarchy (run immediately on the current thread).
        /// Needs a system to get read handles to Entity and all physics-related components.
        /// </summary>
        public static void BuildPhysicsWorldImmediate(SystemBase system, ref PhysicsWorldData physicsData,
            float timeStep, float3 gravity, uint lastSystemVersion)
            =>
            BuildPhysicsWorldImmediate(system, ref physicsData.PhysicsWorld, ref physicsData.HaveStaticBodiesChanged,
                timeStep, gravity, lastSystemVersion, physicsData.DynamicEntityGroup, physicsData.StaticEntityGroup, physicsData.JointEntityGroup);

        /// <summary>
        /// Fill specified PhysicsWorld with bodies and joints (using entities from specified queries) and build broadphase BoundingVolumeHierarchy (run immediately on the current thread).
        /// Needs a system to get read handles to Entity and all physics-related components.
        /// </summary>
        public static void BuildPhysicsWorldImmediate(
            SystemBase system,
            ref PhysicsWorld world, ref NativeArray<int> haveStaticBodiesChanged,
            float timeStep, float3 gravity, uint lastSystemVersion,
            EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)
        {
            EntityTypeHandle entityType = system.GetEntityTypeHandle();
            ComponentTypeHandle<LocalToWorld> localToWorldType = system.GetComponentTypeHandle<LocalToWorld>(true);
            ComponentTypeHandle<Parent> parentType = system.GetComponentTypeHandle<Parent>(true);
            ComponentTypeHandle<Translation> positionType = system.GetComponentTypeHandle<Translation>(true);
            ComponentTypeHandle<Rotation> rotationType = system.GetComponentTypeHandle<Rotation>(true);
            ComponentTypeHandle<PhysicsCollider> physicsColliderType = system.GetComponentTypeHandle<PhysicsCollider>(true);
            ComponentTypeHandle<PhysicsVelocity> physicsVelocityType = system.GetComponentTypeHandle<PhysicsVelocity>(true);
            ComponentTypeHandle<PhysicsMass> physicsMassType = system.GetComponentTypeHandle<PhysicsMass>(true);
            ComponentTypeHandle<PhysicsMassOverride> physicsMassOverrideType = system.GetComponentTypeHandle<PhysicsMassOverride>(true);
            ComponentTypeHandle<PhysicsDamping> physicsDampingType = system.GetComponentTypeHandle<PhysicsDamping>(true);
            ComponentTypeHandle<PhysicsGravityFactor> physicsGravityFactorType = system.GetComponentTypeHandle<PhysicsGravityFactor>(true);
            ComponentTypeHandle<PhysicsCustomTags> physicsCustomTagsType = system.GetComponentTypeHandle<PhysicsCustomTags>(true);
            ComponentTypeHandle<PhysicsConstrainedBodyPair> physicsConstrainedBodyPairType = system.GetComponentTypeHandle<PhysicsConstrainedBodyPair>(true);
            ComponentTypeHandle<PhysicsJoint> physicsJointType = system.GetComponentTypeHandle<PhysicsJoint>(true);

            int numDynamicBodies = dynamicEntityGroup.CalculateEntityCount();
            int numStaticBodies = staticEntityGroup.CalculateEntityCount();
            int numJoints = jointEntityGroup.CalculateEntityCount();

            // Early out if world is empty and it's been like that in previous frame as well (it contained only the default static body)
            if (numDynamicBodies + numStaticBodies == 0 && world.NumBodies == 1)
            {
                // No bodies in the scene, no need to do anything else
                haveStaticBodiesChanged[0] = 0;
                return;
            }

            int previousStaticBodyCount = world.NumStaticBodies;

            // Resize the world's native arrays
            world.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            haveStaticBodiesChanged[0] = 0;
            {
                if (world.NumStaticBodies != previousStaticBodyCount)
                {
                    haveStaticBodiesChanged[0] = 1;
                }
                else
                {
                    new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        m_LastSystemVersion = lastSystemVersion,
                        Result = haveStaticBodiesChanged
                    }.Run(staticEntityGroup);
                }
            }

            // Create the default static body at the end of the body list
            // TODO: could skip this if no joints present
            new Jobs.CreateDefaultStaticRigidBody
            {
                NativeBodies = world.Bodies,
                BodyIndex = world.Bodies.Length - 1,
                EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter()
            }.Run();

            // Dynamic bodies.
            // Create these separately from static bodies to maintain a 1:1 mapping
            // between dynamic bodies and their motions.
            if (numDynamicBodies > 0)
            {
                new Jobs.CreateRigidBodies
                {
                    EntityType = entityType,
                    LocalToWorldType = localToWorldType,
                    ParentType = parentType,
                    PositionType = positionType,
                    RotationType = rotationType,
                    PhysicsColliderType = physicsColliderType,
                    PhysicsCustomTagsType = physicsCustomTagsType,

                    FirstBodyIndex = 0,
                    RigidBodies = world.Bodies,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                }.Run(dynamicEntityGroup);

                new Jobs.CreateMotions
                {
                    PositionType = positionType,
                    RotationType = rotationType,
                    PhysicsVelocityType = physicsVelocityType,
                    PhysicsMassType = physicsMassType,
                    PhysicsMassOverrideType = physicsMassOverrideType,
                    PhysicsDampingType = physicsDampingType,
                    PhysicsGravityFactorType = physicsGravityFactorType,

                    MotionDatas = world.MotionDatas,
                    MotionVelocities = world.MotionVelocities
                }.Run(dynamicEntityGroup);
            }

            // Now, schedule creation of static bodies, with FirstBodyIndex pointing after
            // the dynamic and kinematic bodies
            if (numStaticBodies > 0)
            {
                new Jobs.CreateRigidBodies
                {
                    EntityType = entityType,
                    LocalToWorldType = localToWorldType,
                    ParentType = parentType,
                    PositionType = positionType,
                    RotationType = rotationType,
                    PhysicsColliderType = physicsColliderType,
                    PhysicsCustomTagsType = physicsCustomTagsType,

                    FirstBodyIndex = numDynamicBodies,
                    RigidBodies = world.Bodies,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                }.Run(staticEntityGroup);
            }

            // Build joints
            if (numJoints > 0)
            {
                new Jobs.CreateJoints
                {
                    ConstrainedBodyPairComponentType = physicsConstrainedBodyPairType,
                    JointComponentType = physicsJointType,
                    EntityType = entityType,
                    RigidBodies = world.Bodies,
                    Joints = world.Joints,
                    DefaultStaticBodyIndex = world.Bodies.Length - 1,
                    NumDynamicBodies = numDynamicBodies,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap,
                    EntityJointIndexMap = world.DynamicsWorld.EntityJointIndexMap.AsParallelWriter(),
                }.Run(jointEntityGroup);
            }

            world.CollisionWorld.BuildBroadphase(ref world, timeStep, gravity, haveStaticBodiesChanged[0] != 0);
        }

        /// <summary>
        /// Build broadphase BoundingVolumeHierarchy of the specified PhysicsWorld (run immediately on the current thread)
        /// </summary>
        public static void BuildBroadphaseBVHImmediate(ref PhysicsWorld world, bool haveStaticBodiesChanged, float timeStep, float3 gravity)
        {
            world.CollisionWorld.BuildBroadphase(ref world, timeStep, gravity, haveStaticBodiesChanged);
        }

        #region Jobs

        private static class Jobs
        {
            [BurstCompile]
            internal struct CheckStaticBodyChangesJob : IJobEntityBatch
            {
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;

                [NativeDisableParallelForRestriction]
                public NativeArray<int> Result;

                public uint m_LastSystemVersion;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
                {
                    bool didBatchChange =
                        batchInChunk.DidChange(LocalToWorldType, m_LastSystemVersion) ||
                        batchInChunk.DidChange(PositionType, m_LastSystemVersion) ||
                        batchInChunk.DidChange(RotationType, m_LastSystemVersion) ||
                        batchInChunk.DidChange(PhysicsColliderType, m_LastSystemVersion) ||
                        batchInChunk.DidOrderChange(m_LastSystemVersion);
                    if (didBatchChange)
                    {
                        // Note that multiple worker threads may be running at the same time.
                        // They either write 1 to Result[0] or not write at all.  In case multiple
                        // threads are writing 1 to this variable, in C#, reads or writes of int
                        // data type are atomic, which guarantees that Result[0] is 1.
                        Result[0] = 1;
                    }
                }
            }

            [BurstCompile]
            internal struct CreateDefaultStaticRigidBody : IJob
            {
                [NativeDisableContainerSafetyRestriction]
                public NativeArray<RigidBody> NativeBodies;
                public int BodyIndex;

                [NativeDisableContainerSafetyRestriction]
                public NativeParallelHashMap<Entity, int>.ParallelWriter EntityBodyIndexMap;

                public void Execute()
                {
                    NativeBodies[BodyIndex] = new RigidBody
                    {
                        WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                        Collider = default,
                        Entity = Entity.Null,
                        CustomTags = 0
                    };
                    EntityBodyIndexMap.TryAdd(Entity.Null, BodyIndex);
                }
            }

            [BurstCompile]
            internal struct CreateRigidBodies : IJobEntityBatchWithIndex
            {
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCustomTags> PhysicsCustomTagsType;
                [ReadOnly] public int FirstBodyIndex;

                [NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> RigidBodies;
                [NativeDisableContainerSafetyRestriction] public NativeParallelHashMap<Entity, int>.ParallelWriter EntityBodyIndexMap;

                //public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsCollider> chunkColliders = batchInChunk.GetNativeArray(PhysicsColliderType);
                    NativeArray<LocalToWorld> chunkLocalToWorlds = batchInChunk.GetNativeArray(LocalToWorldType);
                    NativeArray<Translation> chunkPositions = batchInChunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = batchInChunk.GetNativeArray(RotationType);
                    NativeArray<Entity> chunkEntities = batchInChunk.GetNativeArray(EntityType);
                    NativeArray<PhysicsCustomTags> chunkCustomTags = batchInChunk.GetNativeArray(PhysicsCustomTagsType);

                    int instanceCount = batchInChunk.Count;
                    int rbIndex = FirstBodyIndex + firstEntityIndex;

                    bool hasChunkPhysicsColliderType = batchInChunk.Has(PhysicsColliderType);
                    bool hasChunkPhysicsCustomTagsType = batchInChunk.Has(PhysicsCustomTagsType);
                    bool hasChunkParentType = batchInChunk.Has(ParentType);
                    bool hasChunkLocalToWorldType = batchInChunk.Has(LocalToWorldType);
                    bool hasChunkPositionType = batchInChunk.Has(PositionType);
                    bool hasChunkRotationType = batchInChunk.Has(RotationType);

                    RigidTransform worldFromBody = RigidTransform.identity;
                    for (int i = 0; i < instanceCount; i++, rbIndex++)
                    {
                        // if entities are in a transform hierarchy then Translation/Rotation are in the space of their parents
                        // in that case, LocalToWorld is the only common denominator for world space
                        if (hasChunkParentType)
                        {
                            if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody = Math.DecomposeRigidBodyTransform(localToWorld.Value);
                            }
                        }
                        else
                        {
                            if (hasChunkPositionType)
                            {
                                worldFromBody.pos = chunkPositions[i].Value;
                            }
                            else if (hasChunkLocalToWorldType)
                            {
                                worldFromBody.pos = chunkLocalToWorlds[i].Position;
                            }

                            if (hasChunkRotationType)
                            {
                                worldFromBody.rot = chunkRotations[i].Value;
                            }
                            else if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody.rot = Math.DecomposeRigidBodyOrientation(localToWorld.Value);
                            }
                        }

                        RigidBodies[rbIndex] = new RigidBody
                        {
                            WorldFromBody = new RigidTransform(worldFromBody.rot, worldFromBody.pos),
                            Collider = hasChunkPhysicsColliderType ? chunkColliders[i].Value : default,
                            Entity = chunkEntities[i],
                            CustomTags = hasChunkPhysicsCustomTagsType ? chunkCustomTags[i].Value : (byte)0
                        };
                        EntityBodyIndexMap.TryAdd(chunkEntities[i], rbIndex);
                    }
                }
            }

            [BurstCompile]
            internal struct CreateMotions : IJobEntityBatchWithIndex
            {
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
                [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
                [ReadOnly] public ComponentTypeHandle<PhysicsMassOverride> PhysicsMassOverrideType;
                [ReadOnly] public ComponentTypeHandle<PhysicsDamping> PhysicsDampingType;
                [ReadOnly] public ComponentTypeHandle<PhysicsGravityFactor> PhysicsGravityFactorType;

                [NativeDisableParallelForRestriction] public NativeArray<MotionData> MotionDatas;
                [NativeDisableParallelForRestriction] public NativeArray<MotionVelocity> MotionVelocities;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int firstEntityIndex)
                {
                    NativeArray<Translation> chunkPositions = batchInChunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = batchInChunk.GetNativeArray(RotationType);
                    NativeArray<PhysicsVelocity> chunkVelocities = batchInChunk.GetNativeArray(PhysicsVelocityType);
                    NativeArray<PhysicsMass> chunkMasses = batchInChunk.GetNativeArray(PhysicsMassType);
                    NativeArray<PhysicsMassOverride> chunkMassOverrides = batchInChunk.GetNativeArray(PhysicsMassOverrideType);
                    NativeArray<PhysicsDamping> chunkDampings = batchInChunk.GetNativeArray(PhysicsDampingType);
                    NativeArray<PhysicsGravityFactor> chunkGravityFactors = batchInChunk.GetNativeArray(PhysicsGravityFactorType);

                    int motionStart = firstEntityIndex;
                    int instanceCount = batchInChunk.Count;

                    bool hasChunkPhysicsGravityFactorType = batchInChunk.Has(PhysicsGravityFactorType);
                    bool hasChunkPhysicsDampingType = batchInChunk.Has(PhysicsDampingType);
                    bool hasChunkPhysicsMassType = batchInChunk.Has(PhysicsMassType);
                    bool hasChunkPhysicsMassOverrideType = batchInChunk.Has(PhysicsMassOverrideType);

                    // Note: Transform and AngularExpansionFactor could be calculated from PhysicsCollider.MassProperties
                    // However, to avoid the cost of accessing the collider we assume an infinite mass at the origin of a ~1m^3 box.
                    // For better performance with spheres, or better behavior for larger and/or more irregular colliders
                    // you should add a PhysicsMass component to get the true values
                    var defaultPhysicsMass = new PhysicsMass
                    {
                        Transform = RigidTransform.identity,
                        InverseMass = 0.0f,
                        InverseInertia = float3.zero,
                        AngularExpansionFactor = 1.0f,
                    };
                    var zeroPhysicsVelocity = new PhysicsVelocity
                    {
                        Linear = float3.zero,
                        Angular = float3.zero
                    };

                    // Note: if a dynamic body has infinite mass then assume no gravity should be applied
                    float defaultGravityFactor = hasChunkPhysicsMassType ? 1.0f : 0.0f;

                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        // A Body is Kinematic if it has no Mass component, or the Mass component is being overridden.
                        var isKinematic = !hasChunkPhysicsMassType || hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].IsKinematic != 0;
                        PhysicsMass mass = isKinematic ? defaultPhysicsMass : chunkMasses[i];
                        // If the Body is Kinematic its corresponding velocities may be optionally set to zero.
                        var setVelocityToZero = isKinematic && hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].SetVelocityToZero != 0;
                        PhysicsVelocity velocity = setVelocityToZero ? zeroPhysicsVelocity : chunkVelocities[i];
                        // If the Body is Kinematic or has an infinite mass gravity should also have no affect on the body's motion.
                        var hasInfiniteMass = isKinematic || mass.HasInfiniteMass;
                        float gravityFactor = hasInfiniteMass ? 0 : hasChunkPhysicsGravityFactorType ? chunkGravityFactors[i].Value : defaultGravityFactor;

                        MotionVelocities[motionIndex] = new MotionVelocity
                        {
                            LinearVelocity = velocity.Linear,
                            AngularVelocity = velocity.Angular,
                            InverseInertia = mass.InverseInertia,
                            InverseMass = mass.InverseMass,
                            AngularExpansionFactor = mass.AngularExpansionFactor,
                            GravityFactor = gravityFactor
                        };
                    }

                    // Note: these defaults assume a dynamic body with infinite mass, hence no damping
                    var defaultPhysicsDamping = new PhysicsDamping
                    {
                        Linear = 0.0f,
                        Angular = 0.0f,
                    };

                    // Create motion datas
                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        // Note that the assignment of the PhysicsMass component is different from the previous loop
                        // as the motion space transform, and not mass & inertia properties are needed here.
                        PhysicsMass mass = hasChunkPhysicsMassType ? chunkMasses[i] : defaultPhysicsMass;
                        PhysicsDamping damping = hasChunkPhysicsDampingType ? chunkDampings[i] : defaultPhysicsDamping;
                        // A Body is Kinematic if it has no Mass component, or the Mass component is being overridden.
                        var isKinematic = !hasChunkPhysicsMassType || hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].IsKinematic != 0;
                        // If the Body is Kinematic no resistive damping should be applied to it.

                        MotionDatas[motionIndex] = new MotionData
                        {
                            WorldFromMotion = new RigidTransform(
                                math.mul(chunkRotations[i].Value, mass.InertiaOrientation),
                                math.rotate(chunkRotations[i].Value, mass.CenterOfMass) + chunkPositions[i].Value
                                ),
                            BodyFromMotion = new RigidTransform(mass.InertiaOrientation, mass.CenterOfMass),
                            LinearDamping = isKinematic || mass.HasInfiniteMass ? 0.0f : damping.Linear,
                            AngularDamping = isKinematic || mass.HasInfiniteInertia ? 0.0f : damping.Angular
                        };
                    }
                }
            }

            [BurstCompile]
            internal struct CreateJoints : IJobEntityBatchWithIndex
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsConstrainedBodyPair> ConstrainedBodyPairComponentType;
                [ReadOnly] public ComponentTypeHandle<PhysicsJoint> JointComponentType;
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public NativeArray<RigidBody> RigidBodies;
                [ReadOnly] public int NumDynamicBodies;
                [ReadOnly] public NativeParallelHashMap<Entity, int> EntityBodyIndexMap;

                [NativeDisableParallelForRestriction] public NativeArray<Joint> Joints;
                [NativeDisableParallelForRestriction] public NativeParallelHashMap<Entity, int>.ParallelWriter EntityJointIndexMap;

                public int DefaultStaticBodyIndex;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsConstrainedBodyPair> chunkBodyPair = batchInChunk.GetNativeArray(ConstrainedBodyPairComponentType);
                    NativeArray<PhysicsJoint> chunkJoint = batchInChunk.GetNativeArray(JointComponentType);
                    NativeArray<Entity> chunkEntities = batchInChunk.GetNativeArray(EntityType);

                    int instanceCount = batchInChunk.Count;
                    for (int i = 0; i < instanceCount; i++)
                    {
                        var bodyPair = chunkBodyPair[i];
                        var entityA = bodyPair.EntityA;
                        var entityB = bodyPair.EntityB;
                        Assert.IsTrue(entityA != entityB);

                        PhysicsJoint joint = chunkJoint[i];

                        // TODO find a reasonable way to look up the constraint body indices
                        // - stash body index in a component on the entity? But we don't have random access to Entity data in a job
                        // - make a map from entity to rigid body index? Sounds bad and I don't think there is any NativeArray-based map data structure yet

                        // If one of the entities is null, use the default static entity
                        var pair = new BodyIndexPair
                        {
                            BodyIndexA = entityA == Entity.Null ? DefaultStaticBodyIndex : -1,
                            BodyIndexB = entityB == Entity.Null ? DefaultStaticBodyIndex : -1,
                        };

                        // Find the body indices
                        pair.BodyIndexA = EntityBodyIndexMap.TryGetValue(entityA, out var idxA) ? idxA : -1;
                        pair.BodyIndexB = EntityBodyIndexMap.TryGetValue(entityB, out var idxB) ? idxB : -1;

                        bool isInvalid = false;
                        // Invalid if we have not found the body indices...
                        isInvalid |= (pair.BodyIndexA == -1 || pair.BodyIndexB == -1);
                        // ... or if we are constraining two static bodies
                        // Mark static-static invalid since they are not going to affect simulation in any way.
                        isInvalid |= (pair.BodyIndexA >= NumDynamicBodies && pair.BodyIndexB >= NumDynamicBodies);
                        if (isInvalid)
                        {
                            pair = BodyIndexPair.Invalid;
                        }

                        Joints[firstEntityIndex + i] = new Joint
                        {
                            BodyPair = pair,
                            Entity = chunkEntities[i],
                            EnableCollision = (byte)chunkBodyPair[i].EnableCollision,
                            AFromJoint = joint.BodyAFromJoint.AsMTransform(),
                            BFromJoint = joint.BodyBFromJoint.AsMTransform(),
                            Version = joint.Version,
                            Constraints = joint.GetConstraints()
                        };
                        EntityJointIndexMap.TryAdd(chunkEntities[i], firstEntityIndex + i);
                    }
                }
            }
        }

        #endregion
    }
}
