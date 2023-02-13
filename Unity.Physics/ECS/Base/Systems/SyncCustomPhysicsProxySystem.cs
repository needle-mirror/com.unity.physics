using Unity.Burst;
using Unity.Entities;
using Unity.Jobs;
using Unity.Collections;
using Unity.Transforms;
using Unity.Mathematics;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// Synchronize the movement of the custom physics proxy using kinematic velocities.
    /// The kinematic entity is moved from its current position/rotation to the position/rotation of the driving entity in one frame, by computing the
    /// necessary angular and linear velocities.
    /// </summary>
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateBefore(typeof(PhysicsInitializeGroup))]
    [BurstCompile]
    public partial struct SyncCustomPhysicsProxySystem : ISystem
    {
#if !ENABLE_TRANSFORM_V1
        private ComponentLookup<LocalTransform> m_LocalTransformLookup;
#else
        private ComponentLookup<Translation> m_TranslationFromEntity;
        private ComponentLookup<Rotation> m_RotationFromEntity;
#endif
        private EntityQuery m_Query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
#if !ENABLE_TRANSFORM_V1
            m_LocalTransformLookup = state.GetComponentLookup<LocalTransform>(false);
#else
            m_TranslationFromEntity = state.GetComponentLookup<Translation>(false);
            m_RotationFromEntity = state.GetComponentLookup<Rotation>(false);
#endif

            var builder = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<PhysicsMass, CustomPhysicsProxyDriver, PhysicsWorldIndex>()
#if !ENABLE_TRANSFORM_V1
                .WithAllRW<LocalTransform>()
#else
                .WithAllRW<Translation, Rotation>()
#endif
                .WithAllRW<PhysicsVelocity>();

            m_Query = state.GetEntityQuery(builder);

            state.RequireForUpdate(m_Query);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
#if !ENABLE_TRANSFORM_V1
            m_LocalTransformLookup.Update(ref state);
#else
            m_TranslationFromEntity.Update(ref state);
            m_RotationFromEntity.Update(ref state);
#endif
            m_Query.SetSharedComponentFilter(SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorldIndex);

            var job = new SyncCustomPhysicsProxy
            {
#if !ENABLE_TRANSFORM_V1
                localTransformLookup = m_LocalTransformLookup,
#else
                translationFromEntity = m_TranslationFromEntity,
                rotationFromEntity = m_RotationFromEntity,
#endif
                updateFrequency = math.rcp(SystemAPI.Time.DeltaTime)
            };
            state.Dependency = job.ScheduleParallel(m_Query, state.Dependency);
        }

        partial struct SyncCustomPhysicsProxy : IJobEntity
        {
            [NativeDisableParallelForRestriction]
#if !ENABLE_TRANSFORM_V1
            public ComponentLookup<LocalTransform> localTransformLookup;
#else
            public ComponentLookup<Translation> translationFromEntity;
            [NativeDisableParallelForRestriction]
            public ComponentLookup<Rotation> rotationFromEntity;
#endif

            public float updateFrequency;

            public void Execute(Entity entity, ref PhysicsVelocity physicsVelocity,
                in PhysicsMass physicsMass, in CustomPhysicsProxyDriver proxyDriver)
            {
#if !ENABLE_TRANSFORM_V1
                var localTransform = localTransformLookup[entity];
                var targetTransform = new RigidTransform(localTransformLookup[proxyDriver.rootEntity].Rotation, localTransformLookup[proxyDriver.rootEntity].Position);
#else
                var position = translationFromEntity[entity];
                var rotation = rotationFromEntity[entity];
                var targetTransform = new RigidTransform(rotationFromEntity[proxyDriver.rootEntity].Value, translationFromEntity[proxyDriver.rootEntity].Value);
#endif

                // First order changes - position/rotation
                if (proxyDriver.FirstOrderGain != 0.0f)
                {
#if !ENABLE_TRANSFORM_V1
                    localTransform.Position = math.lerp(localTransform.Position, targetTransform.pos, proxyDriver.FirstOrderGain);
                    localTransform.Rotation = math.slerp(localTransform.Rotation, targetTransform.rot, proxyDriver.FirstOrderGain);

                    localTransformLookup[entity] = localTransform;
#else
                    position.Value = math.lerp(position.Value, targetTransform.pos, proxyDriver.FirstOrderGain);
                    rotation.Value = math.slerp(rotation.Value, targetTransform.rot, proxyDriver.FirstOrderGain);

                    translationFromEntity[entity] = position;
                    rotationFromEntity[entity] = rotation;
#endif
                }

                // Second order changes - velocity
                if (proxyDriver.FirstOrderGain != 1.0f)
                {
#if !ENABLE_TRANSFORM_V1
                    physicsVelocity = PhysicsVelocity.CalculateVelocityToTarget(physicsMass, localTransform.Position, localTransform.Rotation, targetTransform, updateFrequency);
#else
                    physicsVelocity = PhysicsVelocity.CalculateVelocityToTarget(physicsMass, position.Value, rotation.Value, targetTransform, updateFrequency);
#endif
                }
            }
        }
    }
}
