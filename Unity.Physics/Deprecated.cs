using System;
using System.ComponentModel;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;


// all deprecated types in this assembly should go in this file, if possible
// deprecated members in core types should appear in-line rather than making structs partial

namespace Unity.Physics
{
    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("CustomDataPair has been deprecated. Use CustomTagsPair instead. (RemovedAfter 2019-10-03)", true)]
    public struct CustomDataPair { }

    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("PhysicsCustomData has been deprecated. Use PhysicsCustomTags instead. (RemovedAfter 2019-10-03) (UnityUpgradable) -> PhysicsCustomTags", true)]
    public struct PhysicsCustomData : IComponentData { }
}


namespace Unity.Physics.Extensions
{
    public static partial class ComponentExtensions
    {
        private static EntityManager GetDefaultEntityManager()
        {
#if UNITY_ENTITIES_0_2_0_OR_NEWER
            return World.DefaultGameObjectInjectionWorld.EntityManager;
#else
            return World.Active.EntityManager;
#endif
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsCollider component instead. (RemovedAfter 2019-11-14")]
        public static CollisionFilter GetCollisionFilter(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsCollider>(entity))
            {
                var collider = entityManager.GetComponentData<PhysicsCollider>(entity);
                if (collider.IsValid)
                {
                    return collider.Value.Value.Filter;
                }
            }
            return CollisionFilter.Zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsMass component instead. (RemovedAfter 2019-11-14")]
        public static float GetMass(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsMass>(entity))
            {
                var mass = entityManager.GetComponentData<PhysicsMass>(entity);
                if (mass.InverseMass != 0)
                {
                    return math.rcp(mass.InverseMass);
                }
            }
            return 0;   // a.k.a infinite
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use PhysicsWorldExtensions.GetEffectiveMass() instead. (RemovedAfter 2019-11-14")]
        public static float GetEffectiveMass(this Entity entity, float3 impulse, float3 point)
        {
            var entityManager = GetDefaultEntityManager();
            float effMass = 0;

            // TODO: convert over from WorldExtensions
            //MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            //float3 pointDir = math.normalizesafe(point - GetCenterOfMass(world, rigidBodyIndex));
            //float3 impulseDir = math.normalizesafe(impulse);

            //float3 jacobian = math.cross(pointDir, impulseDir);
            //float invEffMass = math.csum(math.dot(jacobian, jacobian) * mv.InverseInertiaAndMass.xyz);
            //effMass = math.select(1.0f / invEffMass, 0.0f, math.abs(invEffMass) < 1e-5);

            return effMass;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Get the center of mass in world space
        public static float3 GetCenterOfMass(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsMass>(entity) &&
                entityManager.HasComponent<Translation>(entity) &&
                entityManager.HasComponent<Rotation>(entity))
            {
                var massCData = entityManager.GetComponentData<PhysicsMass>(entity);
                var posCData = entityManager.GetComponentData<Translation>(entity);
                var rotCData = entityManager.GetComponentData<Rotation>(entity);

                return GetCenterOfMass(massCData, posCData, rotCData);
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's Translation component instead. (RemovedAfter 2019-11-14")]
        public static float3 GetPosition(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<Translation>(entity))
            {
                return entityManager.GetComponentData<Translation>(entity).Value;
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's Rotation component instead. (RemovedAfter 2019-11-14")]
        public static quaternion GetRotation(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<Rotation>(entity))
            {
                return entityManager.GetComponentData<Rotation>(entity).Value;
            }
            return quaternion.identity;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component instead. (RemovedAfter 2019-11-14")]
        public static void SetVelocities(this Entity entity, float3 linearVelocity, float3 angularVelocity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                velocityData.Linear = linearVelocity;
                velocityData.Angular = angularVelocity;

                entityManager.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component instead. (RemovedAfter 2019-11-14")]
        public static void GetVelocities(this Entity entity, out float3 linearVelocity, out float3 angularVelocity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                linearVelocity = velocityData.Linear;
                angularVelocity = velocityData.Angular;
            }
            else
            {
                linearVelocity = float3.zero;
                angularVelocity = float3.zero;
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component data instead. (RemovedAfter 2019-11-14")]
        // Get the linear velocity of a rigid body (in world space)
        public static float3 GetLinearVelocity(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                return velocityData.Linear;
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Get the linear velocity of a rigid body at a given point (in world space)
        public static float3 GetLinearVelocity(this Entity entity, float3 point)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity) &&
                entityManager.HasComponent<PhysicsMass>(entity) &&
                entityManager.HasComponent<Translation>(entity) &&
                entityManager.HasComponent<Rotation>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                var massData = entityManager.GetComponentData<PhysicsMass>(entity);
                var posCData = entityManager.GetComponentData<Translation>(entity);
                var rotCData = entityManager.GetComponentData<Rotation>(entity);

                return velocityData.GetLinearVelocity(massData, posCData, rotCData, point);
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component instead. (RemovedAfter 2019-11-14")]
        // Set the linear velocity of a rigid body (in world space)
        public static void SetLinearVelocity(this Entity entity, float3 linearVelocity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                velocityData.Linear = linearVelocity;
                entityManager.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static float3 GetAngularVelocity(this Entity entity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity) &&
                entityManager.HasComponent<PhysicsMass>(entity) &&
                entityManager.HasComponent<Rotation>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                var massData = entityManager.GetComponentData<PhysicsMass>(entity);
                var rotCData = entityManager.GetComponentData<Rotation>(entity);

                return velocityData.GetAngularVelocity(massData, rotCData);
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(this Entity entity, float3 angularVelocity)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity) &&
                entityManager.HasComponent<PhysicsMass>(entity) &&
                entityManager.HasComponent<Rotation>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                var massData = entityManager.GetComponentData<PhysicsMass>(entity);
                var rotCData = entityManager.GetComponentData<Rotation>(entity);

                velocityData.SetAngularVelocity(massData, rotCData, angularVelocity);

                entityManager.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        public static void ApplyImpulse(this Entity entity, float3 impulse, float3 point)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity) &&
                entityManager.HasComponent<PhysicsMass>(entity) &&
                entityManager.HasComponent<Translation>(entity) &&
                entityManager.HasComponent<Rotation>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                var massData = entityManager.GetComponentData<PhysicsMass>(entity);
                var posCData = entityManager.GetComponentData<Translation>(entity);
                var rotCData = entityManager.GetComponentData<Rotation>(entity);

                velocityData.ApplyImpulse(massData, posCData, rotCData, impulse, point);

                entityManager.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        public static void ApplyLinearImpulse(this Entity entity, float3 impulse)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity) &&
                entityManager.HasComponent<PhysicsMass>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                var massData = entityManager.GetComponentData<PhysicsMass>(entity);

                velocityData.ApplyLinearImpulse(massData, impulse);

                entityManager.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        public static void ApplyAngularImpulse(this Entity entity, float3 impulse)
        {
            var entityManager = GetDefaultEntityManager();
            if (entityManager.HasComponent<PhysicsVelocity>(entity) &&
                entityManager.HasComponent<PhysicsMass>(entity))
            {
                var velocityData = entityManager.GetComponentData<PhysicsVelocity>(entity);
                var massData = entityManager.GetComponentData<PhysicsMass>(entity);

                velocityData.ApplyAngularImpulse(massData, impulse);

                entityManager.SetComponentData(entity, velocityData);
            }
        }
    }
}
