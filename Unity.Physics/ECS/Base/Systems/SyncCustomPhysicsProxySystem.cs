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
        private ComponentLookup<Translation> m_TranslationFromEntity;
        private ComponentLookup<Rotation> m_RotationFromEntity;
        private EntityQuery m_Query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_TranslationFromEntity = state.GetComponentLookup<Translation>(false);
            m_RotationFromEntity = state.GetComponentLookup<Rotation>(false);

            var builder = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<PhysicsMass, CustomPhysicsProxyDriver, PhysicsWorldIndex>()
                .WithAllRW<Translation, Rotation>()
                .WithAllRW<PhysicsVelocity>();

            m_Query = state.GetEntityQuery(builder);

            state.RequireForUpdate(m_Query);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {}

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            m_TranslationFromEntity.Update(ref state);
            m_RotationFromEntity.Update(ref state);
            m_Query.SetSharedComponentFilter(SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorldIndex);

            var job = new SyncCustomPhysicsProxy
            {
                translationFromEntity = m_TranslationFromEntity,
                rotationFromEntity = m_RotationFromEntity,
                updateFrequency = math.rcp(SystemAPI.Time.DeltaTime)
            };
            state.Dependency = job.ScheduleParallel(m_Query, state.Dependency);
        }

        partial struct SyncCustomPhysicsProxy : IJobEntity
        {
            [NativeDisableParallelForRestriction]
            public ComponentLookup<Translation> translationFromEntity;
            [NativeDisableParallelForRestriction]
            public ComponentLookup<Rotation> rotationFromEntity;

            public float updateFrequency;

            public void Execute(Entity entity, ref PhysicsVelocity physicsVelocity,
                in PhysicsMass physicsMass, in CustomPhysicsProxyDriver proxyDriver)
            {
                var position = translationFromEntity[entity];
                var rotation = rotationFromEntity[entity];
                var targetTransform = new RigidTransform(rotationFromEntity[proxyDriver.rootEntity].Value, translationFromEntity[proxyDriver.rootEntity].Value);

                // First order changes - position/rotation
                if (proxyDriver.FirstOrderGain != 0.0f)
                {
                    position.Value = math.lerp(position.Value, targetTransform.pos, proxyDriver.FirstOrderGain);
                    rotation.Value = math.slerp(rotation.Value, targetTransform.rot, proxyDriver.FirstOrderGain);

                    translationFromEntity[entity] = position;
                    rotationFromEntity[entity] = rotation;
                }

                // Second order changes - velocity
                if (proxyDriver.FirstOrderGain != 1.0f)
                {
                    physicsVelocity = PhysicsVelocity.CalculateVelocityToTarget(physicsMass, position, rotation, targetTransform, updateFrequency);
                }
            }
        }
    }
}
