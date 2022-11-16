using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    /// <summary>   Utilities for exporting physics world data to ECS components. </summary>
    public static class PhysicsWorldExporter
    {
        /// <summary>   Stores the ECS component handles needed to export <see cref="PhysicsWorld"/> to ECS components. </summary>
        public struct ExportPhysicsWorldTypeHandles
        {
            /// <summary>   Constructor. </summary>
            ///
            /// <param name="systemState">  [in,out] State of the system. </param>
            public ExportPhysicsWorldTypeHandles(ref SystemState systemState)
            {
#if !ENABLE_TRANSFORM_V1
                LocalTransformType = systemState.GetComponentTypeHandle<LocalTransform>(false);
#else
                PositionType = systemState.GetComponentTypeHandle<Translation>(false);
                RotationType = systemState.GetComponentTypeHandle<Rotation>(false);
#endif
                PhysicsVelocityType = systemState.GetComponentTypeHandle<PhysicsVelocity>(false);
                SimulateType = systemState.GetComponentTypeHandle<Simulate>(true);
            }

            /// <summary>   Updates the component handles. Call this in OnUpdate() methods of the system you want to export physics world. </summary>
            ///
            /// <param name="systemState">  [in,out] State of the system. </param>
            public void Update(ref SystemState systemState)
            {
#if !ENABLE_TRANSFORM_V1
                LocalTransformType.Update(ref systemState);
#else
                PositionType.Update(ref systemState);
                RotationType.Update(ref systemState);
#endif
                PhysicsVelocityType.Update(ref systemState);
                SimulateType.Update(ref systemState);
            }

#if !ENABLE_TRANSFORM_V1
            internal ComponentTypeHandle<LocalTransform> LocalTransformType;
#else
            internal ComponentTypeHandle<Translation> PositionType;
            internal ComponentTypeHandle<Rotation> RotationType;
#endif
            internal ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            internal ComponentTypeHandle<Simulate> SimulateType;
        }

        /// <summary>
        /// Schedules a job that copies positions and velocities of all dynamic bodies from specified
        /// PhysicsWorld to ECS components of entities returned by specified query. No need to call <see cref="ExportPhysicsWorldTypeHandles.Update(ref SystemState)"/>
        /// prior to calling this method.
        /// </summary>
        ///
        /// <param name="systemState">          [in,out] State of the system. </param>
        /// <param name="componentTypeHandles"> [in,out] The component type handles. </param>
        /// <param name="world">                The world. </param>
        /// <param name="inputDep">             The input dependency. </param>
        /// <param name="dynamicEntities">      The dynamic entities. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public static JobHandle SchedulePhysicsWorldExport(
            ref SystemState systemState,
            ref ExportPhysicsWorldTypeHandles componentTypeHandles,
            in PhysicsWorld world,
            in JobHandle inputDep,
            EntityQuery dynamicEntities)
        {
            if (world.NumDynamicBodies > 0)
            {
                var chunkBaseEntityIndices =
                    dynamicEntities.CalculateBaseEntityIndexArrayAsync(systemState.WorldUpdateAllocator, inputDep,
                        out var baseIndexJob);
                componentTypeHandles.Update(ref systemState);
                return new ExportDynamicBodiesJob
                {
                    MotionVelocities = world.MotionVelocities,
                    MotionDatas = world.MotionDatas,
#if !ENABLE_TRANSFORM_V1
                    LocalTransformType = componentTypeHandles.LocalTransformType,
#else
                    PositionType = componentTypeHandles.PositionType,
                    RotationType = componentTypeHandles.RotationType,
#endif
                    VelocityType = componentTypeHandles.PhysicsVelocityType,
                    SimulateType = componentTypeHandles.SimulateType,
                    ChunkBaseEntityIndices = chunkBaseEntityIndices,
                }.ScheduleParallel(dynamicEntities, baseIndexJob);
            }
            return inputDep;
        }

        /// <summary>
        /// Copies positions and velocities of all dynamic bodies from specified PhysicsWorld to ECS
        /// components of entities returned by specified query. No need to call <see cref="ExportPhysicsWorldTypeHandles.Update(ref SystemState)"/>
        /// prior to calling this method.
        /// </summary>
        ///
        /// <param name="systemState">                  [in,out] State of the system. </param>
        /// <param name="exportPhysicsWorldHandles">    [in,out] The export physics world handles. </param>
        /// <param name="world">                        The world. </param>
        /// <param name="dynamicEntities">              The dynamic entities. </param>
        public static void ExportPhysicsWorldImmediate(
            ref SystemState systemState,
            ref ExportPhysicsWorldTypeHandles exportPhysicsWorldHandles,
            in PhysicsWorld world,
            EntityQuery dynamicEntities
        )
        {
            if (world.NumDynamicBodies > 0)
            {
                using var chunkBaseEntityIndices = dynamicEntities.CalculateBaseEntityIndexArray(Allocator.TempJob);
                exportPhysicsWorldHandles.Update(ref systemState);
                new ExportDynamicBodiesJob
                {
                    MotionVelocities = world.MotionVelocities,
                    MotionDatas = world.MotionDatas,
#if !ENABLE_TRANSFORM_V1
                    LocalTransformType = exportPhysicsWorldHandles.LocalTransformType,
#else
                    PositionType = exportPhysicsWorldHandles.PositionType,
                    RotationType = exportPhysicsWorldHandles.RotationType,
#endif
                    SimulateType = exportPhysicsWorldHandles.SimulateType,
                    VelocityType = exportPhysicsWorldHandles.PhysicsVelocityType,
                    ChunkBaseEntityIndices = chunkBaseEntityIndices,
                }.Run(dynamicEntities);
            }
        }

        #region Jobs

        [BurstCompile]
        internal struct ExportDynamicBodiesJob : IJobChunk
        {
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeArray<MotionData> MotionDatas;
            [ReadOnly] public NativeArray<int> ChunkBaseEntityIndices;

#if !ENABLE_TRANSFORM_V1
            public ComponentTypeHandle<LocalTransform> LocalTransformType;
#else
            public ComponentTypeHandle<Translation> PositionType;
            public ComponentTypeHandle<Rotation> RotationType;
#endif
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;
            [ReadOnly] public ComponentTypeHandle<Simulate> SimulateType;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                int entityStartIndex = ChunkBaseEntityIndices[unfilteredChunkIndex];
#if !ENABLE_TRANSFORM_V1
                var chunkLocalTransforms = chunk.GetNativeArray(ref LocalTransformType);
#else
                var chunkPositions = chunk.GetNativeArray(ref PositionType);
                var chunkRotations = chunk.GetNativeArray(ref RotationType);
#endif
                var chunkVelocities = chunk.GetNativeArray(ref VelocityType);

                var entityEnumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                var isEntitySimulated = chunk.GetEnabledMask(ref SimulateType);
                while (entityEnumerator.NextEntityIndex(out var i))
                {
                    //If an entity is not simulated (so it is marked as kinematic and has also zero velocity)
                    //we should not export the velocity, position and orientation.
                    //Technically would be better to add the Simulate component to the query. But this will break other jobs
                    //that can't handle the enable mask. So we are doing a check here on an entity by entity basis.
                    if (!isEntitySimulated[i])
                        continue;
                    int motionIndex = entityStartIndex + i;
                    MotionData md = MotionDatas[motionIndex];
                    RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
#if !ENABLE_TRANSFORM_V1
                    var localToWorldTransform = chunkLocalTransforms[i];
                    localToWorldTransform.Position = worldFromBody.pos;
                    localToWorldTransform.Rotation = worldFromBody.rot;
                    chunkLocalTransforms[i] = localToWorldTransform;
#else
                    chunkPositions[i] = new Translation { Value = worldFromBody.pos };
                    chunkRotations[i] = new Rotation { Value = worldFromBody.rot };
#endif
                    chunkVelocities[i] = new PhysicsVelocity
                    {
                        Linear = MotionVelocities[motionIndex].LinearVelocity,
                        Angular = MotionVelocities[motionIndex].AngularVelocity
                    };
                }
            }
        }
        #endregion
    }
}
