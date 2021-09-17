using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics.Systems;

namespace Unity.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that writes to a body's <see cref="PhysicsGraphicalSmoothing"/> component by copying its <see cref="PhysicsVelocity"/> after physics has stepped.
    /// These values are used for bodies whose graphics representations will be smoothed by the <see cref="SmoothRigidBodiesGraphicalMotion"/> system.
    /// Add a <c>WriteGroupAttribute</c> to your own component if you need to use a different value (as with a character controller).
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(ExportPhysicsWorld))]
    public partial class CopyPhysicsVelocityToSmoothing : SystemBase
    {
        /// <summary>
        /// An entity query matching dynamic rigid bodies whose motion should be smoothed.
        /// </summary>
        public EntityQuery SmoothedDynamicBodiesGroup { get; private set; }

        [Obsolete("AddInputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public void AddInputDependency(JobHandle inputDep) {}

        [Obsolete("GetOutputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public JobHandle GetOutputDependency() => default;

        protected override void OnCreate()
        {
            base.OnCreate();

            SmoothedDynamicBodiesGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(PhysicsGraphicalSmoothing),
                    typeof(PhysicsWorldIndex)
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });

            RequireForUpdate(SmoothedDynamicBodiesGroup);
        }

        protected override void OnUpdate()
        {
            Dependency = new CopyPhysicsVelocityJob
            {
                PhysicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true),
                PhysicsGraphicalSmoothingType = GetComponentTypeHandle<PhysicsGraphicalSmoothing>()
            }.ScheduleParallel(SmoothedDynamicBodiesGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, Dependency);
        }

        [BurstCompile]
        unsafe struct CopyPhysicsVelocityJob : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            public ComponentTypeHandle<PhysicsGraphicalSmoothing> PhysicsGraphicalSmoothingType;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                NativeArray<PhysicsVelocity> physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityType);
                NativeArray<PhysicsGraphicalSmoothing> physicsGraphicalSmoothings = batchInChunk.GetNativeArray(PhysicsGraphicalSmoothingType);

                UnsafeUtility.MemCpyStride(
                    physicsGraphicalSmoothings.GetUnsafePtr(), UnsafeUtility.SizeOf<PhysicsGraphicalSmoothing>(),
                    physicsVelocities.GetUnsafeReadOnlyPtr(), UnsafeUtility.SizeOf<PhysicsVelocity>(),
                    UnsafeUtility.SizeOf<PhysicsVelocity>(),
                    physicsVelocities.Length
                );
            }
        }
    }
}
