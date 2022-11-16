using System;
using Unity.Assertions;
using Unity.Burst;
using Unity.Burst.Intrinsics;
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
    [UpdateInGroup(typeof(PhysicsSystemGroup), OrderLast = true)]
    public partial class CopyPhysicsVelocityToSmoothing : SystemBase
    {
        /// <summary>
        /// An entity query matching dynamic rigid bodies whose motion should be smoothed.
        /// </summary>
        public EntityQuery SmoothedDynamicBodiesQuery { get; private set; }

        protected override void OnCreate()
        {
            base.OnCreate();

            SmoothedDynamicBodiesQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(PhysicsGraphicalSmoothing),
                    typeof(PhysicsWorldIndex)
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
            RequireForUpdate(SmoothedDynamicBodiesQuery);
        }

        protected override void OnUpdate()
        {
            var bpwd = EntityManager.GetComponentData<BuildPhysicsWorldData>(World.GetExistingSystem<BuildPhysicsWorld>());
            SmoothedDynamicBodiesQuery.SetSharedComponentFilter(bpwd.WorldFilter);
            Dependency = new CopyPhysicsVelocityJob
            {
                PhysicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true),
                PhysicsGraphicalSmoothingType = GetComponentTypeHandle<PhysicsGraphicalSmoothing>()
            }.ScheduleParallel(SmoothedDynamicBodiesQuery, Dependency);
        }

        /// <summary>   An <see cref="IJobChunk"/> which writes to a body's <see cref="PhysicsGraphicalSmoothing"/> by copying it's <see cref="PhysicsVelocity"/>. </summary>
        [BurstCompile]
        public struct CopyPhysicsVelocityJob : IJobChunk
        {
            /// <summary>   <see cref="PhysicsVelocity"/> component type handle. </summary>
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            /// <summary>   <see cref="PhysicsGraphicalSmoothing"/> component type handle. </summary>
            public ComponentTypeHandle<PhysicsGraphicalSmoothing> PhysicsGraphicalSmoothingType;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                Assert.IsFalse(useEnabledMask);
                NativeArray<PhysicsVelocity> physicsVelocities = chunk.GetNativeArray(ref PhysicsVelocityType);
                NativeArray<PhysicsGraphicalSmoothing> physicsGraphicalSmoothings = chunk.GetNativeArray(ref PhysicsGraphicalSmoothingType);
                unsafe
                {
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
}
