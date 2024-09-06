using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine.Assertions;
using Unity.Physics.Extensions;

namespace Unity.Physics.Systems
{
    /// <summary>   Utilities for building a physics world. </summary>
    [BurstCompile]
    public static class PhysicsWorldBuilder
    {
        /// <summary>
        /// Schedule jobs to fill the PhysicsWorld in specified physicsData with bodies and joints (using
        /// entities from physicsData's queries) and build broadphase BoundingVolumeHierarchy. Needs a
        /// SystemState to update component handles.
        /// </summary>
        ///
        /// <param name="systemState">                      [in,out] State of the system. </param>
        /// <param name="physicsData">                    [in,out] Information describing the physics. </param>
        /// <param name="inputDep">                         The input dependency. </param>
        /// <param name="timeStep">                         The time step. </param>
        /// <param name="isBroadphaseBuildMultiThreaded"> True if the broadphase build is multi threaded, false
        /// if not. </param>
        /// <param name="gravity">                          The gravity. </param>
        /// <param name="lastSystemVersion">                The last system version. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public static JobHandle SchedulePhysicsWorldBuild(ref SystemState systemState, ref PhysicsWorldData physicsData,
            in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion)
        {
            physicsData.Update(ref systemState);
            return SchedulePhysicsWorldBuild(ref systemState, ref physicsData.PhysicsWorld, ref physicsData.HaveStaticBodiesChanged, physicsData.ComponentHandles,
                inputDep, timeStep, isBroadphaseBuildMultiThreaded, gravity, lastSystemVersion,
                physicsData.DynamicEntityGroup, physicsData.StaticEntityGroup, physicsData.JointEntityGroup);
        }

        internal static JobHandle SchedulePhysicsWorldBuild(ref SystemState systemState, ref PhysicsWorldData physicsData,
            in JobHandle inputDep, float timeStep, float collisionTolerance, bool isBroadphaseBuildMultiThreaded,
            bool isDynamicBroadphaseIncremental, bool isStaticBroadphaseIncremental, float3 gravity, uint lastSystemVersion)
        {
            physicsData.Update(ref systemState);
            return SchedulePhysicsWorldBuild(ref systemState, ref physicsData.PhysicsWorld, ref physicsData.HaveStaticBodiesChanged, physicsData.ComponentHandles,
                inputDep, timeStep, collisionTolerance, isBroadphaseBuildMultiThreaded, isDynamicBroadphaseIncremental, isStaticBroadphaseIncremental, gravity, lastSystemVersion,
                physicsData.DynamicEntityGroup, physicsData.StaticEntityGroup, physicsData.JointEntityGroup, physicsData.InvalidatedTemporalCoherenceInfoGroup);
        }

        /// <summary>
        /// Schedule jobs to fill specified PhysicsWorld with bodies and joints (using entities from
        /// specified queries) and build broadphase BoundingVolumeHierarchy.
        /// </summary>
        ///
        /// <param name="systemState">                      [in,out] State of the system. </param>
        /// <param name="world">                            [in,out] The world. </param>
        /// <param name="haveStaticBodiesChanged">        [in,out] The have static bodies changed. </param>
        /// <param name="componentHandles">                 The component handles. </param>
        /// <param name="inputDep">                         The input dependency. </param>
        /// <param name="timeStep">                         The time step. </param>
        /// <param name="isBroadphaseBuildMultiThreaded">   True if the broadphase build is multi threaded, false
        /// if not. </param>
        /// <param name="gravity">                          The gravity. </param>
        /// <param name="lastSystemVersion">                The last system version. </param>
        /// <param name="dynamicEntityQuery">               Group the dynamic entity belongs to. </param>
        /// <param name="staticEntityQuery">                The static entity query. </param>
        /// <param name="jointEntityQuery">                 Group the joint entity belongs to. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public static JobHandle SchedulePhysicsWorldBuild(ref SystemState systemState,
            ref PhysicsWorld world, ref NativeReference<int> haveStaticBodiesChanged, in PhysicsWorldData.PhysicsWorldComponentHandles componentHandles,
            in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion,
            EntityQuery dynamicEntityQuery, EntityQuery staticEntityQuery, EntityQuery jointEntityQuery)
        {
            return SchedulePhysicsWorldBuild(ref systemState, ref world, ref haveStaticBodiesChanged, componentHandles,
                inputDep, timeStep, CollisionWorld.DefaultCollisionTolerance, isBroadphaseBuildMultiThreaded, false, false, gravity, lastSystemVersion,
                dynamicEntityQuery, staticEntityQuery, jointEntityQuery, default);
        }

        static JobHandle SchedulePhysicsWorldBuild(ref SystemState systemState,
            ref PhysicsWorld world, ref NativeReference<int> haveStaticBodiesChanged, in PhysicsWorldData.PhysicsWorldComponentHandles componentHandles,
            in JobHandle inputDep, float timeStep, float collisionTolerance, bool isBroadphaseBuildMultiThreaded, bool isDynamicBroadphaseIncremental, bool isStaticBroadphaseIncremental, float3 gravity, uint lastSystemVersion,
            EntityQuery dynamicEntityQuery, EntityQuery staticEntityQuery, EntityQuery jointEntityQuery, EntityQuery invalidatedTemporalCoherenceInfoQuery)
        {
            JobHandle finalHandle = inputDep;

            int numDynamicBodies = dynamicEntityQuery.CalculateEntityCount();
            int numStaticBodies = staticEntityQuery.CalculateEntityCount();
            int numJoints = jointEntityQuery.CalculateEntityCount();

            int previousStaticBodyCount = world.NumStaticBodies;

            // Early out if world is empty and it's been like that in previous frame as well (it contained only the default static body)
            if (numDynamicBodies + numStaticBodies == 0 && world.NumBodies == 1)
            {
                // No bodies in the scene, no need to do anything else
                haveStaticBodiesChanged.Value = 0;
                return finalHandle;
            }

            // Resize the world's native arrays
            world.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            // Set the desired collision tolerance
            world.CollisionWorld.CollisionTolerance = collisionTolerance;

            // Determine if the static bodies have changed in any way that will require the static broadphase tree to be rebuilt
            JobHandle staticBodiesCheckHandle = default;

            haveStaticBodiesChanged.Value = 0;
            {
                if (world.NumStaticBodies != previousStaticBodyCount ||
                    isStaticBroadphaseIncremental)
                {
                    haveStaticBodiesChanged.Value = 1;
                }
                else
                {
                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = componentHandles.LocalToWorldType,
                        LocalTransformType = componentHandles.LocalTransformType,
                        PhysicsColliderType = componentHandles.PhysicsColliderType,
                        m_LastSystemVersion = lastSystemVersion,
                        Result = haveStaticBodiesChanged
                    }.ScheduleParallel(staticEntityQuery, inputDep);
                }
            }

            using (var jobHandles = new NativeList<JobHandle>(16, Allocator.Temp))
            {
                NativeArray<int> dynamicBodyChunkBaseEntityIndices = default;
                NativeArray<int> staticBodyChunkBaseEntityIndices = default;

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
                    // Since these two jobs are scheduled against the same query, they can share a single
                    // entity index array.
                    dynamicBodyChunkBaseEntityIndices =
                        dynamicEntityQuery.CalculateBaseEntityIndexArrayAsync(systemState.WorldUpdateAllocator, inputDep,
                            out var baseIndexJob);
                    var createBodiesJob = new Jobs.CreateRigidBodies
                    {
                        EntityType = componentHandles.EntityType,
                        LocalToWorldType = componentHandles.LocalToWorldType,
                        ParentType = componentHandles.ParentType,

                        LocalTransformType = componentHandles.LocalTransformType,
                        PhysicsColliderType = componentHandles.PhysicsColliderType,
                        PhysicsCustomTagsType = componentHandles.PhysicsCustomTagsType,

                        FirstBodyIndex = 0,
                        RigidBodies = world.Bodies,
                        EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                        ChunkBaseEntityIndices = dynamicBodyChunkBaseEntityIndices,
                    }.ScheduleParallel(dynamicEntityQuery, baseIndexJob);
                    jobHandles.Add(createBodiesJob);

                    var createMotionsJob = new Jobs.CreateMotions
                    {
                        LocalTransformType = componentHandles.LocalTransformType,
                        PhysicsVelocityType = componentHandles.PhysicsVelocityType,
                        PhysicsMassType = componentHandles.PhysicsMassType,
                        PhysicsMassOverrideType = componentHandles.PhysicsMassOverrideType,
                        PhysicsDampingType = componentHandles.PhysicsDampingType,
                        PhysicsGravityFactorType = componentHandles.PhysicsGravityFactorType,
                        SimulateType = componentHandles.SimulateType,

                        MotionDatas = world.MotionDatas,
                        MotionVelocities = world.MotionVelocities,
                        ChunkBaseEntityIndices = dynamicBodyChunkBaseEntityIndices,
                    }.ScheduleParallel(dynamicEntityQuery, baseIndexJob);
                    jobHandles.Add(createMotionsJob);
                }

                // Now, schedule creation of static bodies, with FirstBodyIndex pointing after
                // the dynamic and kinematic bodies
                if (numStaticBodies > 0)
                {
                    staticBodyChunkBaseEntityIndices =
                        staticEntityQuery.CalculateBaseEntityIndexArrayAsync(systemState.WorldUpdateAllocator, inputDep,
                            out var baseIndexJob);
                    var createBodiesJob = new Jobs.CreateRigidBodies
                    {
                        EntityType = componentHandles.EntityType,
                        LocalToWorldType = componentHandles.LocalToWorldType,
                        ParentType = componentHandles.ParentType,

                        LocalTransformType = componentHandles.LocalTransformType,
                        PhysicsColliderType = componentHandles.PhysicsColliderType,
                        PhysicsCustomTagsType = componentHandles.PhysicsCustomTagsType,

                        FirstBodyIndex = numDynamicBodies,
                        RigidBodies = world.Bodies,
                        EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                        ChunkBaseEntityIndices = staticBodyChunkBaseEntityIndices,
                    }.ScheduleParallel(staticEntityQuery, baseIndexJob);
                    jobHandles.Add(createBodiesJob);
                }

                var combinedHandle = JobHandle.CombineDependencies(jobHandles.AsArray());
                jobHandles.Clear();

                // Build joints
                if (numJoints > 0)
                {
                    var chunkBaseEntityIndices =
                        jointEntityQuery.CalculateBaseEntityIndexArrayAsync(systemState.WorldUpdateAllocator, combinedHandle,
                            out var baseIndexJob);
                    var createJointsJob = new Jobs.CreateJoints
                    {
                        ConstrainedBodyPairComponentType = componentHandles.PhysicsConstrainedBodyPairType,
                        JointComponentType = componentHandles.PhysicsJointType,
                        EntityType = componentHandles.EntityType,
                        Joints = world.Joints,
                        DefaultStaticBodyIndex = world.Bodies.Length - 1,
                        NumDynamicBodies = numDynamicBodies,
                        EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap,
                        EntityJointIndexMap = world.DynamicsWorld.EntityJointIndexMap.AsParallelWriter(),
                        ChunkBaseEntityIndices = chunkBaseEntityIndices,
                    }.ScheduleParallel(jointEntityQuery, baseIndexJob);
                    jobHandles.Add(createJointsJob);
                }

                var buildBroadphaseHandle = world.CollisionWorld.ScheduleBuildBroadphaseJobs(
                    ref world, timeStep, gravity, numDynamicBodies, numStaticBodies, haveStaticBodiesChanged,
                    dynamicEntityQuery, staticEntityQuery, invalidatedTemporalCoherenceInfoQuery, dynamicBodyChunkBaseEntityIndices, staticBodyChunkBaseEntityIndices,
                    combinedHandle, systemState.WorldUpdateAllocator, componentHandles, lastSystemVersion,
                    isBroadphaseBuildMultiThreaded, isDynamicBroadphaseIncremental, isStaticBroadphaseIncremental);

                jobHandles.Add(buildBroadphaseHandle);

                finalHandle = JobHandle.CombineDependencies(inputDep, JobHandle.CombineDependencies(jobHandles.AsArray()));
            }

            return finalHandle;
        }

        /// <summary>
        /// Schedule jobs to build the broadphase of the specified PhysicsWorld.
        /// </summary>
        ///
        /// <param name="world">                            [in,out] The world. </param>
        /// <param name="haveStaticBodiesChanged">          The have static bodies changed. </param>
        /// <param name="inputDep">                         The input dependency. </param>
        /// <param name="timeStep">                         The time step. </param>
        /// <param name="isBroadphaseUpdateMultiThreaded">  True if the broadphase update is multi threaded, false
        /// if not. </param>
        /// <param name="gravity">                          The gravity. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public static JobHandle ScheduleBroadphaseBVHBuild(ref PhysicsWorld world, NativeReference<int>.ReadOnly haveStaticBodiesChanged,
            in JobHandle inputDep, float timeStep, bool isBroadphaseUpdateMultiThreaded, float3 gravity)
        {
            return world.CollisionWorld.ScheduleBuildBroadphaseJobs(
                ref world, timeStep, gravity,
                haveStaticBodiesChanged, inputDep, isBroadphaseUpdateMultiThreaded);
        }

        /// <summary>
        /// Schedule jobs to update the broadphase of the specified PhysicsWorld.
        /// </summary>
        ///
        /// <param name="physicsWorldData">                 [in,out] Information describing the physics world. </param>
        /// <param name="inputDep">                         The input dependency. </param>
        /// <param name="timeStep">                         The time step. </param>
        /// <param name="isBroadphaseUpdatedMultiThreaded"> True if is broadphase update is multi threaded, false if not. </param>s
        /// <param name="gravity">                          The gravity. </param>
        /// <param name="lastSystemVersion">                The last system version. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public static JobHandle ScheduleUpdateBroadphase(ref PhysicsWorldData physicsWorldData,
            float timeStep, float3 gravity, uint lastSystemVersion, in JobHandle inputDep, bool isBroadphaseUpdatedMultiThreaded)
        {
            physicsWorldData.HaveStaticBodiesChanged.Value = 0;
            var jobHandle = physicsWorldData.PhysicsWorld.CollisionWorld.ScheduleUpdateDynamicTree(
                ref physicsWorldData.PhysicsWorld, timeStep, gravity, inputDep, isBroadphaseUpdatedMultiThreaded);
            jobHandle = new Jobs.CheckStaticBodyChangesJob
            {
                LocalToWorldType = physicsWorldData.ComponentHandles.LocalToWorldType,
                LocalTransformType = physicsWorldData.ComponentHandles.LocalTransformType,
                PhysicsColliderType = physicsWorldData.ComponentHandles.PhysicsColliderType,
                m_LastSystemVersion = lastSystemVersion,
                Result = physicsWorldData.HaveStaticBodiesChanged
            }.ScheduleParallel(physicsWorldData.StaticEntityGroup, inputDep);
            jobHandle = physicsWorldData.PhysicsWorld.CollisionWorld.ScheduleUpdateStaticTree(
                ref physicsWorldData.PhysicsWorld, physicsWorldData.HaveStaticBodiesChanged.AsReadOnly(), jobHandle, isBroadphaseUpdatedMultiThreaded);
            return jobHandle;
        }

        /// <summary>
        /// Update the pre-existing <see cref="MotionData"/> and <see cref="MotionVelocity"/> using the current state of the physic object (transform and physics velocities).
        /// This method can be used to update the state of the physic simulation when running physic multiple time in the same frame.
        /// Assumes that the number of physics objects (static and dynamic) and joints remain the same after the physics world as been built.
        /// </summary>
        /// <param name="systemState"> [in,out] State of the system. </param>
        /// <param name="physicsData">[in,out] Information describing the physics. </param>
        /// <param name="inputDeps">the input dependencies</param>
        public static JobHandle ScheduleUpdateMotionData(ref SystemState systemState, ref PhysicsWorldData physicsData, JobHandle inputDeps)
        {
            var numDynamicBodies = physicsData.StaticEntityGroup.CalculateEntityCount();
            if (numDynamicBodies == 0)
            {
                return inputDeps;
            }
            using var chunkBaseEntityIndices = physicsData.DynamicEntityGroup.CalculateBaseEntityIndexArrayAsync(
                systemState.WorldUpdateAllocator, inputDeps, out var jobHandle);
            inputDeps = new Jobs.CreateMotions
            {
                LocalTransformType = physicsData.ComponentHandles.LocalTransformType,
                PhysicsVelocityType = physicsData.ComponentHandles.PhysicsVelocityType,
                PhysicsMassType = physicsData.ComponentHandles.PhysicsMassType,
                PhysicsMassOverrideType = physicsData.ComponentHandles.PhysicsMassOverrideType,
                PhysicsDampingType = physicsData.ComponentHandles.PhysicsDampingType,
                PhysicsGravityFactorType = physicsData.ComponentHandles.PhysicsGravityFactorType,
                SimulateType = physicsData.ComponentHandles.SimulateType,

                MotionDatas = physicsData.PhysicsWorld.MotionDatas,
                MotionVelocities = physicsData.PhysicsWorld.MotionVelocities,
                ChunkBaseEntityIndices = chunkBaseEntityIndices,
            }.ScheduleParallel(physicsData.DynamicEntityGroup, jobHandle);
            return inputDeps;
        }

        /// <summary>
        /// Update the pre-existing <see cref="MotionData"/> and <see cref="MotionVelocity"/> using the current state of the physic object (transform and physics velocities).
        /// This method can be used to update the state of the physic simulation when running physic multiple time in the same frame.
        /// Assumes that the number of physics objects (static and dynamic) and joints remain the same after the physics world as been built.
        /// </summary>
        /// <param name="systemState"> [in,out] State of the system. </param>
        /// <param name="physicsData">[in,out] Information describing the physics. </param>
        public static void UpdateMotionDataImmediate(ref SystemState systemState, ref PhysicsWorldData physicsData)
        {
            var numStaticBodies = physicsData.DynamicEntityGroup.CalculateEntityCount();
            var numDynamicBodies = physicsData.StaticEntityGroup.CalculateEntityCount();
            if (numStaticBodies + numDynamicBodies == 0)
            {
                physicsData.HaveStaticBodiesChanged.Value = 0;
                return;
            }
            using var chunkBaseEntityIndices = physicsData.DynamicEntityGroup.CalculateBaseEntityIndexArray(systemState.WorldUpdateAllocator);
            new Jobs.CreateMotions
            {
                LocalTransformType = physicsData.ComponentHandles.LocalTransformType,
                PhysicsVelocityType = physicsData.ComponentHandles.PhysicsVelocityType,
                PhysicsMassType = physicsData.ComponentHandles.PhysicsMassType,
                PhysicsMassOverrideType = physicsData.ComponentHandles.PhysicsMassOverrideType,
                PhysicsDampingType = physicsData.ComponentHandles.PhysicsDampingType,
                PhysicsGravityFactorType = physicsData.ComponentHandles.PhysicsGravityFactorType,
                SimulateType = physicsData.ComponentHandles.SimulateType,

                MotionDatas = physicsData.PhysicsWorld.MotionDatas,
                MotionVelocities = physicsData.PhysicsWorld.MotionVelocities,
                ChunkBaseEntityIndices = chunkBaseEntityIndices,
            }.Run(physicsData.DynamicEntityGroup);
        }

        /// <summary>
        /// Fill specified PhysicsWorld with bodies and joints (using entities from specified queries)
        /// and build broadphase BoundingVolumeHierarchy (run immediately on the current thread). Needs a
        /// system to to update type handles of physics-related components.
        /// </summary>
        ///
        /// <param name="systemState">          [in,out] State of the system. </param>
        /// <param name="physicsData">          [in,out] Information describing the physics. </param>
        /// <param name="timeStep">             The time step. </param>
        /// <param name="gravity">              The gravity. </param>
        /// <param name="lastSystemVersion">    The last system version. </param>
        public static void BuildPhysicsWorldImmediate(ref SystemState systemState, ref PhysicsWorldData physicsData,
            float timeStep, float3 gravity, uint lastSystemVersion)
        {
            physicsData.Update(ref systemState);
            BuildPhysicsWorldImmediate(ref physicsData.PhysicsWorld, physicsData.HaveStaticBodiesChanged, physicsData.ComponentHandles,
                timeStep, gravity, lastSystemVersion, physicsData.DynamicEntityGroup, physicsData.StaticEntityGroup, physicsData.JointEntityGroup);
        }

        /// <summary>
        /// Fill specified PhysicsWorld with bodies and joints (using entities from specified queries)
        /// and build broadphase BoundingVolumeHierarchy (run immediately on the current thread).
        /// </summary>
        ///
        /// <param name="world">                    [in,out] The world. </param>
        /// <param name="haveStaticBodiesChanged">  [in,out] The have static bodies changed. </param>
        /// <param name="componentHandles">         The component handles. </param>
        /// <param name="timeStep">                 The time step. </param>
        /// <param name="gravity">                  The gravity. </param>
        /// <param name="lastSystemVersion">        The last system version. </param>
        /// <param name="dynamicEntityGroup">       Group the dynamic entity belongs to. </param>
        /// <param name="staticEntityGroup">        Group the static entity belongs to. </param>
        /// <param name="jointEntityGroup">         Group the joint entity belongs to. </param>
        public static void BuildPhysicsWorldImmediate(
            ref PhysicsWorld world, NativeReference<int> haveStaticBodiesChanged, in PhysicsWorldData.PhysicsWorldComponentHandles componentHandles,
            float timeStep, float3 gravity, uint lastSystemVersion,
            EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)
        {
            int numDynamicBodies = dynamicEntityGroup.CalculateEntityCount();
            int numStaticBodies = staticEntityGroup.CalculateEntityCount();
            int numJoints = jointEntityGroup.CalculateEntityCount();

            // Early out if world is empty and it's been like that in previous frame as well (it contained only the default static body)
            if (numDynamicBodies + numStaticBodies == 0 && world.NumBodies == 1)
            {
                // No bodies in the scene, no need to do anything else
                haveStaticBodiesChanged.Value = 0;
                return;
            }

            int previousStaticBodyCount = world.NumStaticBodies;

            // Resize the world's native arrays
            world.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            haveStaticBodiesChanged.Value = 0;
            {
                if (world.NumStaticBodies != previousStaticBodyCount)
                {
                    haveStaticBodiesChanged.Value = 1;
                }
                else
                {
                    new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = componentHandles.LocalToWorldType,
                        LocalTransformType = componentHandles.LocalTransformType,
                        PhysicsColliderType = componentHandles.PhysicsColliderType,
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
                using var chunkBaseEntityIndices = dynamicEntityGroup.CalculateBaseEntityIndexArray(Allocator.TempJob);
                new Jobs.CreateRigidBodies
                {
                    EntityType = componentHandles.EntityType,
                    LocalToWorldType = componentHandles.LocalToWorldType,
                    ParentType = componentHandles.ParentType,
                    LocalTransformType = componentHandles.LocalTransformType,
                    PhysicsColliderType = componentHandles.PhysicsColliderType,
                    PhysicsCustomTagsType = componentHandles.PhysicsCustomTagsType,

                    FirstBodyIndex = 0,
                    RigidBodies = world.Bodies,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                    ChunkBaseEntityIndices = chunkBaseEntityIndices,
                }.Run(dynamicEntityGroup);

                new Jobs.CreateMotions
                {
                    LocalTransformType = componentHandles.LocalTransformType,
                    PhysicsVelocityType = componentHandles.PhysicsVelocityType,
                    PhysicsMassType = componentHandles.PhysicsMassType,
                    PhysicsMassOverrideType = componentHandles.PhysicsMassOverrideType,
                    PhysicsDampingType = componentHandles.PhysicsDampingType,
                    PhysicsGravityFactorType = componentHandles.PhysicsGravityFactorType,
                    SimulateType = componentHandles.SimulateType,

                    MotionDatas = world.MotionDatas,
                    MotionVelocities = world.MotionVelocities,
                    ChunkBaseEntityIndices = chunkBaseEntityIndices,
                }.Run(dynamicEntityGroup);
            }

            // Now, schedule creation of static bodies, with FirstBodyIndex pointing after
            // the dynamic and kinematic bodies
            if (numStaticBodies > 0)
            {
                using var chunkBaseEntityIndices = staticEntityGroup.CalculateBaseEntityIndexArray(Allocator.TempJob);
                new Jobs.CreateRigidBodies
                {
                    EntityType = componentHandles.EntityType,
                    LocalToWorldType = componentHandles.LocalToWorldType,
                    ParentType = componentHandles.ParentType,
                    LocalTransformType = componentHandles.LocalTransformType,
                    PhysicsColliderType = componentHandles.PhysicsColliderType,
                    PhysicsCustomTagsType = componentHandles.PhysicsCustomTagsType,
                    FirstBodyIndex = numDynamicBodies,
                    RigidBodies = world.Bodies,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                    ChunkBaseEntityIndices = chunkBaseEntityIndices,
                }.Run(staticEntityGroup);
            }

            // Build joints
            if (numJoints > 0)
            {
                using var chunkBaseEntityIndices = jointEntityGroup.CalculateBaseEntityIndexArray(Allocator.TempJob);
                new Jobs.CreateJoints
                {
                    ConstrainedBodyPairComponentType = componentHandles.PhysicsConstrainedBodyPairType,
                    JointComponentType = componentHandles.PhysicsJointType,
                    EntityType = componentHandles.EntityType,
                    Joints = world.Joints,
                    DefaultStaticBodyIndex = world.Bodies.Length - 1,
                    NumDynamicBodies = numDynamicBodies,
                    EntityBodyIndexMap = world.CollisionWorld.EntityBodyIndexMap,
                    EntityJointIndexMap = world.DynamicsWorld.EntityJointIndexMap.AsParallelWriter(),
                    ChunkBaseEntityIndices = chunkBaseEntityIndices,
                }.Run(jointEntityGroup);
            }

            world.CollisionWorld.BuildBroadphase(ref world, timeStep, gravity, haveStaticBodiesChanged.Value != 0);
        }

        /// <summary>
        /// Build broadphase BoundingVolumeHierarchy of the specified PhysicsWorld (run immediately on
        /// the current thread)
        /// </summary>
        ///
        /// <param name="world">                    [in,out] The world. </param>
        /// <param name="haveStaticBodiesChanged">  True if have static bodies changed. </param>
        /// <param name="timeStep">                 The time step. </param>
        /// <param name="gravity">                  The gravity. </param>
        public static void BuildBroadphaseBVHImmediate(ref PhysicsWorld world, bool haveStaticBodiesChanged, float timeStep, float3 gravity)
        {
            world.CollisionWorld.BuildBroadphase(ref world, timeStep, gravity, haveStaticBodiesChanged);
        }

        /// <summary>
        /// Update the broadphase BoundingVolumeHierarchy of the of the specified PhysicsWorld (run immediately on
        /// the current thread).
        /// </summary>
        /// <param name="physicsWorldData"></param>
        /// <param name="timeStep"></param>
        /// <param name="gravity"></param>
        /// <param name="lastSystemVersion"></param>
        public static void UpdateBroadphaseImmediate(ref PhysicsWorldData physicsWorldData, float timeStep, float3 gravity, uint lastSystemVersion)
        {
            physicsWorldData.HaveStaticBodiesChanged.Value = 0;
            physicsWorldData.PhysicsWorld.CollisionWorld.UpdateDynamicTree(ref physicsWorldData.PhysicsWorld, timeStep, gravity);
            new Jobs.CheckStaticBodyChangesJob
            {
                LocalToWorldType = physicsWorldData.ComponentHandles.LocalToWorldType,
                LocalTransformType = physicsWorldData.ComponentHandles.LocalTransformType,
                PhysicsColliderType = physicsWorldData.ComponentHandles.PhysicsColliderType,
                m_LastSystemVersion = lastSystemVersion,
                Result = physicsWorldData.HaveStaticBodiesChanged
            }.Run(physicsWorldData.StaticEntityGroup);
            if (physicsWorldData.HaveStaticBodiesChanged.Value != 0)
            {
                physicsWorldData.PhysicsWorld.CollisionWorld.UpdateStaticTree(ref physicsWorldData.PhysicsWorld);
            }
        }

        #region Jobs

        [BurstCompile]
        private static class Jobs
        {
            [BurstCompile]
            internal struct CheckStaticBodyChangesJob : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                [NativeDisableParallelForRestriction]
                public NativeReference<int> Result;

                public uint m_LastSystemVersion;

                public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
                {
                    SafetyChecks.CheckAreEqualAndThrow(false, useEnabledMask);
                    bool didBatchChange =
                        chunk.DidChange(ref LocalToWorldType, m_LastSystemVersion)       ||
                        chunk.DidChange(ref LocalTransformType, m_LastSystemVersion)     ||
                        chunk.DidChange(ref PhysicsColliderType, m_LastSystemVersion)    ||
                        chunk.DidOrderChange(m_LastSystemVersion);
                    if (didBatchChange)
                    {
                        // Note that multiple worker threads may be running at the same time.
                        // They either write 1 to Result[0] or not write at all.  In case multiple
                        // threads are writing 1 to this variable, in C#, reads or writes of int
                        // data type are atomic, which guarantees that Result[0] is 1.
                        Result.Value = 1;
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

                [BurstCompile]
                public void Execute()
                {
                    NativeBodies[BodyIndex] = new RigidBody
                    {
                        WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                        Scale = 1.0f,
                        Collider = default,
                        Entity = Entity.Null,
                        CustomTags = 0
                    };
                    EntityBodyIndexMap.TryAdd(Entity.Null, BodyIndex);
                }
            }

            [BurstCompile]
            internal struct CreateRigidBodies : IJobChunk
            {
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
                [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCustomTags> PhysicsCustomTagsType;
                [ReadOnly] public int FirstBodyIndex;

                [NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> RigidBodies;
                [NativeDisableContainerSafetyRestriction] public NativeParallelHashMap<Entity, int>.ParallelWriter EntityBodyIndexMap;
                [ReadOnly] public NativeArray<int> ChunkBaseEntityIndices;

                public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
                {
                    int firstEntityIndexInQuery = ChunkBaseEntityIndices[unfilteredChunkIndex];
                    NativeArray<PhysicsCollider> chunkColliders = chunk.GetNativeArray(ref PhysicsColliderType);
                    NativeArray<LocalToWorld> chunkLocalToWorlds = chunk.GetNativeArray(ref LocalToWorldType);
                    NativeArray<LocalTransform> chunkLocalTransforms = chunk.GetNativeArray(ref LocalTransformType);
                    NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);
                    NativeArray<PhysicsCustomTags> chunkCustomTags = chunk.GetNativeArray(ref PhysicsCustomTagsType);

                    bool hasChunkPhysicsColliderType = chunkColliders.IsCreated;
                    bool hasChunkPhysicsCustomTagsType = chunk.Has(ref PhysicsCustomTagsType);
                    bool hasChunkParentType = chunk.Has(ref ParentType);
                    bool hasChunkLocalToWorldType = chunkLocalToWorlds.IsCreated;
                    bool hasChunkLocalTransformType = chunkLocalTransforms.IsCreated;

                    RigidTransform worldFromBody = RigidTransform.identity;
                    var entityEnumerator =
                        new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                    while (entityEnumerator.NextEntityIndex(out int i))
                    {
                        int rbIndex = FirstBodyIndex + firstEntityIndexInQuery + i;

                        // We support rigid body entities with various different transformation data components.
                        // Here, we are extracting their world space transformation and feed it into the underlying
                        // physics engine for processing in the pipeline (e.g., collision detection, solver, integration).
                        // If the rigid body has a Parent, we obtain their world space transformation from their up-to date
                        // LocalToWorld matrix. Any shear or scale in this matrix is ignored and only the rigid body transformation,
                        // i.e., the position and orientation, is extracted.
                        // If the rigid body has no Parent, we use the position and orientation of its LocalTransform component as world transform
                        // if present. Otherwise, we again extract the transformation from the LocalToWorld matrix.
                        if (hasChunkParentType || !hasChunkLocalTransformType)
                        {
                            if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody = Math.DecomposeRigidBodyTransform(localToWorld.Value);
                            }
                        }
                        else
                        {
                            worldFromBody.pos = chunkLocalTransforms[i].Position;
                            worldFromBody.rot = chunkLocalTransforms[i].Rotation;
                        }

                        float scale = 1.0f;
                        if (hasChunkLocalTransformType)
                        {
                            scale = chunkLocalTransforms[i].Scale;
                        }

                        RigidBodies[rbIndex] = new RigidBody
                        {
                            WorldFromBody = new RigidTransform(worldFromBody.rot, worldFromBody.pos),
                            Scale = scale,
                            Collider = hasChunkPhysicsColliderType ? chunkColliders[i].Value : default,
                            Entity = chunkEntities[i],
                            CustomTags = hasChunkPhysicsCustomTagsType ? chunkCustomTags[i].Value : (byte)0
                        };

                        EntityBodyIndexMap.TryAdd(chunkEntities[i], rbIndex);
                    }
                }
            }

            [BurstCompile]
            internal struct CreateMotions : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformType;
                [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
                [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
                [ReadOnly] public ComponentTypeHandle<PhysicsMassOverride> PhysicsMassOverrideType;
                [ReadOnly] public ComponentTypeHandle<PhysicsDamping> PhysicsDampingType;
                [ReadOnly] public ComponentTypeHandle<PhysicsGravityFactor> PhysicsGravityFactorType;
                [ReadOnly] public ComponentTypeHandle<Simulate> SimulateType;

                [NativeDisableParallelForRestriction] public NativeArray<MotionData> MotionDatas;
                [NativeDisableParallelForRestriction] public NativeArray<MotionVelocity> MotionVelocities;
                [ReadOnly] public NativeArray<int> ChunkBaseEntityIndices;

                public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
                {
                    int firstEntityIndexInQuery = ChunkBaseEntityIndices[unfilteredChunkIndex];
                    NativeArray<LocalTransform> chunkLocalTransforms = chunk.GetNativeArray(ref LocalTransformType);
                    NativeArray<PhysicsVelocity> chunkVelocities = chunk.GetNativeArray(ref PhysicsVelocityType);
                    NativeArray<PhysicsMass> chunkMasses = chunk.GetNativeArray(ref PhysicsMassType);
                    NativeArray<PhysicsMassOverride> chunkMassOverrides = chunk.GetNativeArray(ref PhysicsMassOverrideType);
                    NativeArray<PhysicsDamping> chunkDampings = chunk.GetNativeArray(ref PhysicsDampingType);
                    NativeArray<PhysicsGravityFactor> chunkGravityFactors = chunk.GetNativeArray(ref PhysicsGravityFactorType);

                    int motionStart = firstEntityIndexInQuery;

                    bool hasChunkPhysicsGravityFactorType = chunkGravityFactors.IsCreated;
                    bool hasChunkPhysicsDampingType = chunkDampings.IsCreated;
                    bool hasChunkPhysicsMassType = chunkMasses.IsCreated;
                    bool hasChunkPhysicsMassOverrideType = chunkMassOverrides.IsCreated;
                    bool hasChunkLocalTransformType = chunkLocalTransforms.IsCreated;
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

                    var entityEnumerator1 =
                        new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                    while (entityEnumerator1.NextEntityIndex(out int i))
                    {
                        int motionIndex = motionStart + i;
                        // A Body is Kinematic if it has no Mass component, or the Mass component is being overridden.
                        var isKinematic = !hasChunkPhysicsMassType || (hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].IsKinematic != 0) || !chunk.IsComponentEnabled(ref SimulateType, i);
                        PhysicsMass mass = isKinematic ? defaultPhysicsMass : chunkMasses[i];
                        // If the Body is Kinematic its corresponding velocities may be optionally set to zero.
                        var setVelocityToZero = isKinematic && ((hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].SetVelocityToZero != 0) || !chunk.IsComponentEnabled(ref SimulateType, i));
                        PhysicsVelocity velocity = setVelocityToZero ? zeroPhysicsVelocity : chunkVelocities[i];
                        // If the Body is Kinematic or has an infinite mass gravity should also have no affect on the body's motion.
                        var hasInfiniteMass = isKinematic || mass.HasInfiniteMass;
                        float gravityFactor = hasInfiniteMass ? 0 : hasChunkPhysicsGravityFactorType ? chunkGravityFactors[i].Value : defaultGravityFactor;

                        if (hasChunkLocalTransformType)
                        {
                            mass = mass.ApplyScale(chunkLocalTransforms[i].Scale);
                        }

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
                    var entityEnumerator2 =
                        new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                    while (entityEnumerator2.NextEntityIndex(out int i))
                    {
                        int motionIndex = motionStart + i;
                        // Note that the assignment of the PhysicsMass component is different from the previous loop
                        // as the motion space transform, and not mass & inertia properties are needed here.
                        PhysicsMass mass = hasChunkPhysicsMassType ? chunkMasses[i] : defaultPhysicsMass;
                        PhysicsDamping damping = hasChunkPhysicsDampingType ? chunkDampings[i] : defaultPhysicsDamping;
                        // A Body is Kinematic if it has no Mass component, or the Mass component is being overridden.
                        var isKinematic = !hasChunkPhysicsMassType || (hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].IsKinematic != 0) || !chunk.IsComponentEnabled(ref SimulateType, i);
                        // If the Body is Kinematic no resistive damping should be applied to it.

                        quaternion bodyRotationInWorld = quaternion.identity;
                        float3 bodyPosInWorld = float3.zero;

                        if (hasChunkLocalTransformType)
                        {
                            bodyRotationInWorld = chunkLocalTransforms[i].Rotation;
                            bodyPosInWorld = chunkLocalTransforms[i].Position;

                            mass = mass.ApplyScale(chunkLocalTransforms[i].Scale);
                        }

                        MotionDatas[motionIndex] = new MotionData
                        {
                            WorldFromMotion = new RigidTransform(
                                math.mul(bodyRotationInWorld, mass.InertiaOrientation),
                                math.rotate(bodyRotationInWorld, mass.CenterOfMass) + bodyPosInWorld),
                            BodyFromMotion = new RigidTransform(mass.InertiaOrientation, mass.CenterOfMass),
                            LinearDamping = isKinematic || mass.HasInfiniteMass ? 0.0f : damping.Linear,
                            AngularDamping = isKinematic || mass.HasInfiniteInertia ? 0.0f : damping.Angular
                        };
                    }
                }
            }

            [BurstCompile]
            internal struct CreateJoints : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsConstrainedBodyPair> ConstrainedBodyPairComponentType;
                [ReadOnly] public ComponentTypeHandle<PhysicsJoint> JointComponentType;
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public int NumDynamicBodies;
                [ReadOnly] public NativeParallelHashMap<Entity, int> EntityBodyIndexMap;

                [NativeDisableParallelForRestriction] public NativeArray<Joint> Joints;
                [NativeDisableParallelForRestriction] public NativeParallelHashMap<Entity, int>.ParallelWriter EntityJointIndexMap;
                [ReadOnly] public NativeArray<int> ChunkBaseEntityIndices;

                public int DefaultStaticBodyIndex;

                public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
                {
                    int firstEntityIndex = ChunkBaseEntityIndices[unfilteredChunkIndex];
                    NativeArray<PhysicsConstrainedBodyPair> chunkBodyPair = chunk.GetNativeArray(ref ConstrainedBodyPairComponentType);
                    NativeArray<PhysicsJoint> chunkJoint = chunk.GetNativeArray(ref JointComponentType);
                    NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);

                    var entityEnumerator =
                        new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                    while (entityEnumerator.NextEntityIndex(out var i))
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
                            Constraints = joint.m_Constraints
                        };
                        EntityJointIndexMap.TryAdd(chunkEntities[i], firstEntityIndex + i);
                    }
                }
            }
        }

        #endregion
    }
}
