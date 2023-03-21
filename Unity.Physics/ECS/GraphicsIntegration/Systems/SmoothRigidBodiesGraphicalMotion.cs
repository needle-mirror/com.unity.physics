using System;
using System.Collections.Generic;
using Unity.Assertions;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that can smooth out the motion of rigid bodies if the fixed physics tick rate is slower than the variable graphics framerate.
    /// Each affected body's <see cref="Unity.Transforms.LocalToWorld"/> matrix is adjusted before rendering, but its underlying
    /// <see cref="Unity.Transforms.LocalTransform"/> component is left alone.
    /// </summary>
    [UpdateInGroup(typeof(TransformSystemGroup))]

    [UpdateBefore(typeof(LocalToWorldSystem))]

    public partial class SmoothRigidBodiesGraphicalMotion : SystemBase
    {
        /// <summary>
        /// An entity query matching dynamic rigid bodies whose motion should be smoothed.
        /// </summary>
        public EntityQuery SmoothedDynamicBodiesQuery { get; private set; }

        private Entity m_MostRecentTimeEntity;

        //Declaring big capacity since buffers with RigidBodySmoothingWorldIndex and MostRecentFixedTime will be stored together in a singleton Entity
        //and that Entity will get a whole Chunk allocated anyway. This capacity is just a limit for keeping the buffer inside the Chunk,
        //reducing it does not affect memory consumption
        [InternalBufferCapacity(256)]
        struct RigidBodySmoothingWorldIndex : IBufferElementData,
                                              IEquatable<RigidBodySmoothingWorldIndex>, IComparable<RigidBodySmoothingWorldIndex>
        {
            public int Value;
            public RigidBodySmoothingWorldIndex(PhysicsWorldIndex index)
            {
                Value = (int)index.Value;
            }

            public bool Equals(RigidBodySmoothingWorldIndex other)
            {
                return other.Value == Value;
            }

            public int CompareTo(RigidBodySmoothingWorldIndex other)
            {
                return Value.CompareTo(other.Value);
            }
        }

        /// <summary>
        /// Registers the physics world for smooth rigid body motion described by physicsWorldIndex.
        /// </summary>
        ///
        /// <param name="physicsWorldIndex">    Zero-based index of the physics world. </param>
        public void RegisterPhysicsWorldForSmoothRigidBodyMotion(PhysicsWorldIndex physicsWorldIndex)
        {
            var mostRecentFixedTimes = SystemAPI.GetBuffer<MostRecentFixedTime>(m_MostRecentTimeEntity);
            var worldIndexToUpdate = SystemAPI.GetBuffer<RigidBodySmoothingWorldIndex>(m_MostRecentTimeEntity);
            var rbSmoothIndex = new RigidBodySmoothingWorldIndex(physicsWorldIndex);
            if (mostRecentFixedTimes.Length <= rbSmoothIndex.Value)
                mostRecentFixedTimes.ResizeUninitialized(rbSmoothIndex.Value + 1);
            if (worldIndexToUpdate.AsNativeArray().IndexOf(rbSmoothIndex) == -1)
            {
                worldIndexToUpdate.Add(rbSmoothIndex);
                worldIndexToUpdate.AsNativeArray().Sort();
            }
        }

        /// <summary>
        /// Unregisters the physics world for smooth rigid body motion described by physicsWorldIndex.
        /// </summary>
        ///
        /// <param name="physicsWorldIndex">    Zero-based index of the physics world. </param>
        public void UnregisterPhysicsWorldForSmoothRigidBodyMotion(PhysicsWorldIndex physicsWorldIndex)
        {
            var worldIndexToUpdate = SystemAPI.GetBuffer<RigidBodySmoothingWorldIndex>(m_MostRecentTimeEntity);
            for (int i = 0; i < worldIndexToUpdate.Length; ++i)
            {
                //Don't use swap back to keep sorting
                if (worldIndexToUpdate[i].Value == physicsWorldIndex.Value)
                {
                    worldIndexToUpdate.RemoveAt(i);
                    break;
                }
            }
        }

        protected override void OnCreate()
        {
            base.OnCreate();
            SmoothedDynamicBodiesQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {

                    typeof(LocalTransform),

                    typeof(PhysicsGraphicalSmoothing),
                    typeof(LocalToWorld),
                    typeof(PhysicsWorldIndex)
                }
            });
            // Store a buffer of MostRecentFixedTime element, one for each physics world.
            m_MostRecentTimeEntity = EntityManager.CreateEntity(typeof(MostRecentFixedTime), typeof(RigidBodySmoothingWorldIndex));
            EntityManager.SetName(m_MostRecentTimeEntity, "MostRecentFixedTime");

            RequireForUpdate(SmoothedDynamicBodiesQuery);
            RequireForUpdate<MostRecentFixedTime>();
        }

        protected override void OnUpdate()
        {
            var mostRecentTimes = SystemAPI.GetBuffer<MostRecentFixedTime>(m_MostRecentTimeEntity);
            var mostRecentTimeToUpdate = SystemAPI.GetBuffer<RigidBodySmoothingWorldIndex>(m_MostRecentTimeEntity);
            for (int i = 0; i < mostRecentTimeToUpdate.Length; ++i)
            {
                var worldIndex = mostRecentTimeToUpdate[i];
                var timeAhead = (float)(SystemAPI.Time.ElapsedTime - mostRecentTimes[worldIndex.Value].ElapsedTime);
                var timeStep = (float)mostRecentTimes[worldIndex.Value].DeltaTime;
                if (timeAhead < 0f || timeStep == 0f)
                    continue;

                var normalizedTimeAhead = math.clamp(timeAhead / timeStep, 0f, 1f);
                SmoothedDynamicBodiesQuery.SetSharedComponentFilter(new PhysicsWorldIndex((uint)worldIndex.Value));
                Dependency = new SmoothMotionJob
                {
                    LocalTransformType = GetComponentTypeHandle<LocalTransform>(true),
                    PostTransformMatrixType = GetComponentTypeHandle<PostTransformMatrix>(true),
                    PhysicsMassType = GetComponentTypeHandle<PhysicsMass>(true),
                    InterpolationBufferType = GetComponentTypeHandle<PhysicsGraphicalInterpolationBuffer>(true),
                    PhysicsGraphicalSmoothingType = GetComponentTypeHandle<PhysicsGraphicalSmoothing>(),
                    LocalToWorldType = GetComponentTypeHandle<LocalToWorld>(),
                    TimeAhead = timeAhead,
                    NormalizedTimeAhead = normalizedTimeAhead
                }.ScheduleParallel(SmoothedDynamicBodiesQuery, Dependency);
            }
        }

        [BurstCompile]
        struct SmoothMotionJob : IJobChunk
        {

            [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformType;
            [ReadOnly] public ComponentTypeHandle<PostTransformMatrix> PostTransformMatrixType;
            [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
            [ReadOnly] public ComponentTypeHandle<PhysicsGraphicalInterpolationBuffer> InterpolationBufferType;
            public ComponentTypeHandle<PhysicsGraphicalSmoothing> PhysicsGraphicalSmoothingType;
            public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
            public float TimeAhead;
            public float NormalizedTimeAhead;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                Assert.IsFalse(useEnabledMask);

                NativeArray<LocalTransform> localTransforms = chunk.GetNativeArray(ref LocalTransformType);
                NativeArray<PostTransformMatrix> postTransformMatrices = chunk.GetNativeArray(ref PostTransformMatrixType);
                NativeArray<PhysicsMass> physicsMasses = chunk.GetNativeArray(ref PhysicsMassType);
                NativeArray<PhysicsGraphicalSmoothing> physicsGraphicalSmoothings = chunk.GetNativeArray(ref PhysicsGraphicalSmoothingType);
                NativeArray<PhysicsGraphicalInterpolationBuffer> interpolationBuffers = chunk.GetNativeArray(ref InterpolationBufferType);
                NativeArray<LocalToWorld> localToWorlds = chunk.GetNativeArray(ref LocalToWorldType);


                // GameObjects with non-identity scale have their scale baked into their collision shape and mass, so
                // the entity's transform scale (if any) should not be applied again here. Entities that did not go
                // through baking should apply their uniform scale value to the rigid body.
                // Baking also adds a PostTransformMatrix component to apply the GameObject's authored scale in the
                // rendering code, so we test for that component to determine whether the entity's current scale
                // should be applied or ignored.
                // TODO(DOTS-7098): More robust check here?
                var hasPostTransformMatrix = postTransformMatrices.IsCreated;
                var hasLocalTransform = localTransforms.IsCreated;

                var hasPhysicsMass = physicsMasses.IsCreated;
                var hasInterpolationBuffer = interpolationBuffers.IsCreated;

                var defaultPhysicsMass = PhysicsMass.CreateKinematic(MassProperties.UnitSphere);
                for (int i = 0, count = chunk.Count; i < count; ++i)
                {
                    var physicsMass = hasPhysicsMass ? physicsMasses[i] : defaultPhysicsMass;
                    var smoothing = physicsGraphicalSmoothings[i];
                    var currentVelocity = smoothing.CurrentVelocity;


                    var currentTransform = hasLocalTransform ? new RigidTransform(localTransforms[i].Rotation, localTransforms[i].Position) : RigidTransform.identity;

                    RigidTransform smoothedTransform;

                    // apply no smoothing (i.e., teleported bodies)
                    if (smoothing.ApplySmoothing == 0 || TimeAhead == 0)
                    {
                        if (hasInterpolationBuffer && smoothing.ApplySmoothing != 0)
                            // When using interpolation the smoothed transform is one physics tick behind, if physics updated this frame we need to apply the state from the history buffer in order to stay one frame behind
                            smoothedTransform = interpolationBuffers[i].PreviousTransform;
                        else
                            smoothedTransform = currentTransform;
                    }
                    else
                    {
                        if (hasInterpolationBuffer)
                        {
                            smoothedTransform = GraphicalSmoothingUtility.Interpolate(
                                interpolationBuffers[i].PreviousTransform, currentTransform, NormalizedTimeAhead);
                        }
                        else
                        {
                            smoothedTransform = GraphicalSmoothingUtility.Extrapolate(
                                currentTransform, currentVelocity, physicsMass, TimeAhead);
                        }
                    }

                    localToWorlds[i] = GraphicalSmoothingUtility.BuildLocalToWorld(

                        i, smoothedTransform,
                        hasLocalTransform ? localTransforms[i].Scale : 1.0f,
                        hasPostTransformMatrix, postTransformMatrices
                    );

                    // reset smoothing to apply again next frame (i.e., finish teleportation)
                    smoothing.ApplySmoothing = 1;
                    physicsGraphicalSmoothings[i] = smoothing;
                }
            }
        }
    }
}
