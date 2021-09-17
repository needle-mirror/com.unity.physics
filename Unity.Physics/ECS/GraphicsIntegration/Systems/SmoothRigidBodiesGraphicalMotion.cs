using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Unity.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that can smooth out the motion of rigid bodies if the fixed physics tick rate is slower than the variable graphics framerate.
    /// Each affected body's LocalToWorld is adjusted before rendering, but its underlying Translation and Rotation values are left alone.
    /// </summary>
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateBefore(typeof(TRSToLocalToWorldSystem))]
    public partial class SmoothRigidBodiesGraphicalMotion : SystemBase
    {
        RecordMostRecentFixedTime m_RecordMostRecentFixedTime;

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
                    typeof(Translation),
                    typeof(Rotation),
                    typeof(PhysicsGraphicalSmoothing),
                    typeof(LocalToWorld),
                    typeof(PhysicsWorldIndex)
                }
            });
            RequireForUpdate(SmoothedDynamicBodiesGroup);
            m_RecordMostRecentFixedTime = World.GetOrCreateSystem<RecordMostRecentFixedTime>();
        }

        protected override void OnUpdate()
        {
            var timeAhead = (float)(Time.ElapsedTime - m_RecordMostRecentFixedTime.MostRecentElapsedTime);
            var timeStep = (float)m_RecordMostRecentFixedTime.MostRecentDeltaTime;
            if (timeAhead < 0f || timeStep == 0f)
                return;
            var normalizedTimeAhead = math.clamp(timeAhead / timeStep, 0f, 1f);

            Dependency = new SmoothMotionJob
            {
                TranslationType = GetComponentTypeHandle<Translation>(true),
                RotationType = GetComponentTypeHandle<Rotation>(true),
                NonUniformScaleType = GetComponentTypeHandle<NonUniformScale>(true),
                ScaleType = GetComponentTypeHandle<Scale>(true),
                CompositeScaleType = GetComponentTypeHandle<CompositeScale>(true),
                PhysicsMassType = GetComponentTypeHandle<PhysicsMass>(true),
                InterpolationBufferType = GetComponentTypeHandle<PhysicsGraphicalInterpolationBuffer>(true),
                PhysicsGraphicalSmoothingType = GetComponentTypeHandle<PhysicsGraphicalSmoothing>(),
                LocalToWorldType = GetComponentTypeHandle<LocalToWorld>(),
                TimeAhead = timeAhead,
                NormalizedTimeAhead = normalizedTimeAhead
            }.ScheduleParallel(SmoothedDynamicBodiesGroup, ScheduleGranularity.Chunk, limitToEntityArray: default, Dependency);
        }

        [BurstCompile]
        struct SmoothMotionJob : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<Translation> TranslationType;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<NonUniformScale> NonUniformScaleType;
            [ReadOnly] public ComponentTypeHandle<Scale> ScaleType;
            [ReadOnly] public ComponentTypeHandle<CompositeScale> CompositeScaleType;
            [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
            [ReadOnly] public ComponentTypeHandle<PhysicsGraphicalInterpolationBuffer> InterpolationBufferType;
            public ComponentTypeHandle<PhysicsGraphicalSmoothing> PhysicsGraphicalSmoothingType;
            public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
            public float TimeAhead;
            public float NormalizedTimeAhead;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                var hasNonUniformScale = batchInChunk.Has(NonUniformScaleType);
                var hasScale = batchInChunk.Has(ScaleType);
                var hasAnyScale = hasNonUniformScale || hasScale || batchInChunk.Has(CompositeScaleType);
                var hasPhysicsMass = batchInChunk.Has(PhysicsMassType);
                var hasInterpolationBuffer = batchInChunk.Has(InterpolationBufferType);

                NativeArray<Translation> positions = batchInChunk.GetNativeArray(TranslationType);
                NativeArray<Rotation> orientations = batchInChunk.GetNativeArray(RotationType);
                NativeArray<NonUniformScale> nonUniformScales = batchInChunk.GetNativeArray(NonUniformScaleType);
                NativeArray<Scale> scales = batchInChunk.GetNativeArray(ScaleType);
                NativeArray<CompositeScale> compositeScales = batchInChunk.GetNativeArray(CompositeScaleType);
                NativeArray<PhysicsMass> physicsMasses = batchInChunk.GetNativeArray(PhysicsMassType);
                NativeArray<PhysicsGraphicalSmoothing> physicsGraphicalSmoothings = batchInChunk.GetNativeArray(PhysicsGraphicalSmoothingType);
                NativeArray<PhysicsGraphicalInterpolationBuffer> interpolationBuffers = batchInChunk.GetNativeArray(InterpolationBufferType);
                NativeArray<LocalToWorld> localToWorlds = batchInChunk.GetNativeArray(LocalToWorldType);

                var defaultPhysicsMass = PhysicsMass.CreateKinematic(MassProperties.UnitSphere);
                for (int i = 0, count = batchInChunk.Count; i < count; ++i)
                {
                    var physicsMass = hasPhysicsMass ? physicsMasses[i] : defaultPhysicsMass;
                    var smoothing = physicsGraphicalSmoothings[i];
                    var currentVelocity = smoothing.CurrentVelocity;

                    var currentTransform = new RigidTransform(orientations[i].Value, positions[i].Value);
                    RigidTransform smoothedTransform;

                    // apply no smoothing (i.e., teleported bodies)
                    if (smoothing.ApplySmoothing == 0 || TimeAhead == 0)
                    {
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
                        hasAnyScale,
                        hasNonUniformScale, nonUniformScales,
                        hasScale, scales,
                        compositeScales
                    );

                    // reset smoothing to apply again next frame (i.e., finish teleportation)
                    smoothing.ApplySmoothing = 1;
                    physicsGraphicalSmoothings[i] = smoothing;
                }
            }
        }
    }
}
