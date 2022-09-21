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
                PositionType = systemState.GetComponentTypeHandle<Translation>(false);
                RotationType = systemState.GetComponentTypeHandle<Rotation>(false);
                PhysicsVelocityType = systemState.GetComponentTypeHandle<PhysicsVelocity>(false);
            }

            /// <summary>   Updates the component handles. Call this in OnUpdate() methods of the system you want to export physics world. </summary>
            ///
            /// <param name="systemState">  [in,out] State of the system. </param>
            public void Update(ref SystemState systemState)
            {
                PositionType.Update(ref systemState);
                RotationType.Update(ref systemState);
                PhysicsVelocityType.Update(ref systemState);
            }

            internal ComponentTypeHandle<Translation> PositionType;
            internal ComponentTypeHandle<Rotation> RotationType;
            internal ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
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
                    PositionType = componentTypeHandles.PositionType,
                    RotationType = componentTypeHandles.RotationType,
                    VelocityType = componentTypeHandles.PhysicsVelocityType,
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
                    PositionType = exportPhysicsWorldHandles.PositionType,
                    RotationType = exportPhysicsWorldHandles.RotationType,
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

            public ComponentTypeHandle<Translation> PositionType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                int entityStartIndex = ChunkBaseEntityIndices[unfilteredChunkIndex];
                var chunkPositions = chunk.GetNativeArray(PositionType);
                var chunkRotations = chunk.GetNativeArray(RotationType);
                var chunkVelocities = chunk.GetNativeArray(VelocityType);

                var entityEnumerator =
                    new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.ChunkEntityCount);
                while (entityEnumerator.NextEntityIndex(out var i))
                {
                    int motionIndex = entityStartIndex + i;
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
        #endregion
    }
}
