using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// Utilities for exporting physics world data to ECS components.
    /// </summary>
    public static class PhysicsWorldExporter
    {
        public unsafe struct SharedData : IDisposable
        {
            [NativeDisableUnsafePtrRestriction]
            internal AtomicSafetyManager* SafetyManager;

            public static SharedData Create()
            {
                var sharedData = new SharedData();
                sharedData.SafetyManager = (AtomicSafetyManager*)UnsafeUtility.Malloc(UnsafeUtility.SizeOf<AtomicSafetyManager>(), 16, Allocator.Persistent);
                *sharedData.SafetyManager = AtomicSafetyManager.Create();

                return sharedData;
            }

            public void Dispose()
            {
                SafetyManager->Dispose();
            }

            public void Sync()
            {
                SafetyManager->BumpTemporaryHandleVersions();
            }
        }

        /// <summary>
        /// Schedules a job that copies positions and velocities of all dynamic bodies from specified PhysicsWorld to ECS components of entities returned by specified query.
        /// Needs a system to get read/write handles to Translation, Rotation and PhysicsVelocity.
        /// </summary>
        public static JobHandle SchedulePhysicsWorldExport(
            SystemBase system,
            in PhysicsWorld world,
            in JobHandle inputDep,
            EntityQuery dynamicEntities)
        {
            JobHandle handle = inputDep;
            if (world.NumDynamicBodies > 0)
            {
                handle = new ExportDynamicBodiesJob
                {
                    MotionVelocities = world.MotionVelocities,
                    MotionDatas = world.MotionDatas,
                    PositionType = system.GetComponentTypeHandle<Translation>(),
                    RotationType = system.GetComponentTypeHandle<Rotation>(),
                    VelocityType = system.GetComponentTypeHandle<PhysicsVelocity>()
                }.ScheduleParallel(dynamicEntities, ScheduleGranularity.Chunk, limitToEntityArray: default, inputDep);
            }

            return handle;
        }

        /// <summary>
        /// Schedules a job that copies CollisionWorld from specified PhysicsWorld to CollisionWorldProxy component of entities returned by specified query.
        /// Needs a system to get read/write handle to CollisionWorldProxy.
        /// </summary>
        public static JobHandle ScheduleCollisionWorldCopy(
            SystemBase system,
            ref SharedData sharedData,
            in PhysicsWorld world,
            in JobHandle inputDep,
            EntityQuery collisionWorldProxyEntities)
        {
            JobHandle handle = inputDep;

            int numCollisionWorldProxies = collisionWorldProxyEntities.CalculateEntityCount();
            if (numCollisionWorldProxies > 0)
            {
                // Sync shared data.
                sharedData.Sync();

                handle = new CopyCollisionWorld
                {
                    World = world,
                    SharedData = sharedData,
                    ProxyType = system.GetComponentTypeHandle<CollisionWorldProxy>()
                }.ScheduleParallel(collisionWorldProxyEntities, ScheduleGranularity.Chunk, limitToEntityArray: default, handle);
            }

            return handle;
        }

        /// <summary>
        /// Copies positions and velocities of all dynamic bodies from specified PhysicsWorld to ECS components of entities returned by specified query (run on current thread)
        /// Needs a system to get read/write handles to Translation, Rotation and PhysicsVelocity.
        /// </summary>
        public static void ExportPhysicsWorldImmediate(
            SystemBase system,
            in PhysicsWorld world,
            EntityQuery dynamicEntities)
        {
            if (world.NumDynamicBodies > 0)
            {
                new ExportDynamicBodiesJob
                {
                    MotionVelocities = world.MotionVelocities,
                    MotionDatas = world.MotionDatas,
                    PositionType = system.GetComponentTypeHandle<Translation>(),
                    RotationType = system.GetComponentTypeHandle<Rotation>(),
                    VelocityType = system.GetComponentTypeHandle<PhysicsVelocity>()
                }.Run(dynamicEntities);
            }
        }

        /// <summary>
        /// Copies CollisionWorld from specified PhysicsWorld to CollisionWorldProxy component of entities returned by specified query (run on current thread).
        /// Needs a system to get read/write handle to CollisionWorldProxy.
        /// </summary>
        public static void CopyCollisionWorldImmediate(
            SystemBase system,
            ref SharedData sharedData,
            in PhysicsWorld world,
            EntityQuery collisionWorldProxyEntities)
        {
            int numCollisionWorldProxies = collisionWorldProxyEntities.CalculateEntityCount();
            if (numCollisionWorldProxies > 0)
            {
                // Sync shared data.
                sharedData.Sync();

                new CopyCollisionWorld
                {
                    World = world,
                    SharedData = sharedData,
                    ProxyType = system.GetComponentTypeHandle<CollisionWorldProxy>()
                }.Run(collisionWorldProxyEntities);
            }
        }

        #region Jobs

        [BurstCompile]
        internal struct ExportDynamicBodiesJob : IJobEntityBatchWithIndex
        {
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeArray<MotionData> MotionDatas;

            public ComponentTypeHandle<Translation> PositionType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int entityStartIndex)
            {
                var chunkPositions = batchInChunk.GetNativeArray(PositionType);
                var chunkRotations = batchInChunk.GetNativeArray(RotationType);
                var chunkVelocities = batchInChunk.GetNativeArray(VelocityType);

                int numItems = batchInChunk.Count;

                for (int i = 0, motionIndex = entityStartIndex; i < numItems; i++, motionIndex++)
                {
                    MotionData md = MotionDatas[motionIndex];
                    RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
                    chunkPositions[i] = new Translation { Value = worldFromBody.pos };
                    chunkRotations[i] = new Rotation { Value = worldFromBody.rot };
                    chunkVelocities[i] = new PhysicsVelocity
                    {
                        Linear = MotionVelocities[motionIndex].LinearVelocity,
                        Angular = MotionVelocities[motionIndex].AngularVelocity
                    };
                }
            }
        }

        [BurstCompile]
        internal unsafe struct CopyCollisionWorld : IJobEntityBatch
        {
            [ReadOnly] public PhysicsWorld World;
            public SharedData SharedData;
            public ComponentTypeHandle<CollisionWorldProxy> ProxyType;

            //public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                NativeArray<CollisionWorldProxy> chunkProxies = batchInChunk.GetNativeArray(ProxyType);

                var proxy = new CollisionWorldProxy(World.CollisionWorld, SharedData.SafetyManager);

                for (var i = 0; i < batchInChunk.Count; ++i)
                    chunkProxies[i] = proxy;
            }
        }

        #endregion
    }
}
