using System;
using Unity.Assertions;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Unity.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that writes to a rigid body's <see cref="PhysicsGraphicalInterpolationBuffer"/>
    /// component by copying its <c>Translation</c>, <c>Rotation</c>, and <see cref="PhysicsVelocity"/>
    /// before physics steps. These values are used for bodies whose graphics representations will be
    /// interpolated by the <see cref="SmoothRigidBodiesGraphicalMotion"/> system. Add a <c>
    /// WriteGroupAttribute</c> to your own component if you need to use different values (as with a
    /// character controller).
    ///
    /// NOTE: Consider the case when an interpolated rigid body needs to be teleported (i.e. have its
    /// <c>Translation</c>,
    /// <c>Rotation</c> or <see cref="PhysicsVelocity"/> components changed directly), specifically
    /// after this system is updated and before <see cref="SmoothRigidBodiesGraphicalMotion"/> is
    /// updated. In that case, you should set associated <see cref="PhysicsGraphicalSmoothing.ApplySmoothing"/>
    /// to 0. or assign the appropriate new <see cref="PhysicsGraphicalInterpolationBuffer"/>
    /// component values as well.
    /// </summary>
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsInitializeGroup)), UpdateBefore(typeof(ExportPhysicsWorld))]
    public partial class BufferInterpolatedRigidBodiesMotion : SystemBase
    {
        /// <summary>
        /// An entity query matching dynamic rigid bodies whose graphical motion should be interpolated.
        /// </summary>
        public EntityQuery InterpolatedDynamicBodiesQuery { get; private set; }

        protected override void OnCreate()
        {
            InterpolatedDynamicBodiesQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(Translation),
                    typeof(Rotation),
                    typeof(PhysicsGraphicalInterpolationBuffer),
                    typeof(PhysicsWorldIndex)
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
            RequireForUpdate(InterpolatedDynamicBodiesQuery);
        }

        protected override void OnUpdate()
        {
            var bpwd = EntityManager.GetComponentData<BuildPhysicsWorldData>(World.GetExistingSystem<BuildPhysicsWorld>());
            InterpolatedDynamicBodiesQuery.SetSharedComponentFilter(bpwd.WorldFilter);
            Dependency = new UpdateInterpolationBuffersJob
            {
                TranslationType = GetComponentTypeHandle<Translation>(true),
                RotationType = GetComponentTypeHandle<Rotation>(true),
                PhysicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true),
                InterpolationBufferType = GetComponentTypeHandle<PhysicsGraphicalInterpolationBuffer>()
            }.ScheduleParallel(InterpolatedDynamicBodiesQuery, Dependency);
        }

        /// <summary>
        /// An <see cref="IJobChunk"/> which updates <see cref="PhysicsGraphicalInterpolationBuffer"/> of the entities in
        /// chunk specified by <see cref="PhysicsVelocity"/>, <see cref="Translation"/>, <see cref="Rotation"/>
        /// and <see cref="PhysicsGraphicalInterpolationBuffer"/>.
        /// </summary>
        [BurstCompile]
        public struct UpdateInterpolationBuffersJob : IJobChunk
        {
            /// <summary>   Physics velocity component type handle. (Readonly) </summary>
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            /// <summary>   Translation component type handle. (Readonly) </summary>
            [ReadOnly] public ComponentTypeHandle<Translation> TranslationType;
            /// <summary>   Rotation component type handle. (Readonly) </summary>
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            /// <summary>   PhysicsGraphicalInterpolationBuffer component type handle.. </summary>
            public ComponentTypeHandle<PhysicsGraphicalInterpolationBuffer> InterpolationBufferType;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                Assert.IsFalse(useEnabledMask);
                NativeArray<PhysicsVelocity> physicsVelocities = chunk.GetNativeArray(PhysicsVelocityType);
                NativeArray<Translation> positions = chunk.GetNativeArray(TranslationType);
                NativeArray<Rotation> orientations = chunk.GetNativeArray(RotationType);
                NativeArray<PhysicsGraphicalInterpolationBuffer> interpolationBuffers = chunk.GetNativeArray(InterpolationBufferType);

                unsafe
                {
                    var dst = interpolationBuffers.GetUnsafePtr();
                    var count = chunk.Count;

                    var sizeBuffer = UnsafeUtility.SizeOf<PhysicsGraphicalInterpolationBuffer>();
                    var sizeOrientation = UnsafeUtility.SizeOf<quaternion>();
                    var sizePosition = UnsafeUtility.SizeOf<float3>();
                    var sizeVelocity = UnsafeUtility.SizeOf<PhysicsVelocity>();

                    UnsafeUtility.MemCpyStride(
                        dst, sizeBuffer,
                        orientations.GetUnsafeReadOnlyPtr(), sizeOrientation,
                        sizeOrientation,
                        count
                    );
                    UnsafeUtility.MemCpyStride(
                        (void*)((long)dst + sizeOrientation), sizeBuffer,
                        positions.GetUnsafeReadOnlyPtr(), sizePosition,
                        sizePosition,
                        count
                    );
                    UnsafeUtility.MemCpyStride(
                        (void*)((long)dst + sizeOrientation + sizePosition), sizeBuffer,
                        physicsVelocities.GetUnsafeReadOnlyPtr(), sizeVelocity,
                        sizeVelocity,
                        count
                    );
                }
            }
        }
    }
}
