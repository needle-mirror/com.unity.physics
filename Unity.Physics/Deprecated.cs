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

    internal static class EntitiesBackwardCompatibility
    {
    #if !UNITY_ENTITIES_0_1_0_OR_NEWER
        public static int CalculateEntityCount(this EntityQuery entityQuery) => entityQuery.CalculateLength();
    #endif
    }
}

namespace Unity.Physics.Extensions
{
    public static partial class ComponentExtensions
    {
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsCollider component instead. (RemovedAfter 2019-11-14")]
        public static CollisionFilter GetCollisionFilter(this Entity entity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsCollider>(entity))
            {
                var collider = em.GetComponentData<PhysicsCollider>(entity);
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
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsMass>(entity))
            {
                var mass = em.GetComponentData<PhysicsMass>(entity);
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
            var em = World.Active.EntityManager;

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
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsMass>(entity) &&
                em.HasComponent<Translation>(entity) &&
                em.HasComponent<Rotation>(entity))
            {
                var massCData = em.GetComponentData<PhysicsMass>(entity);
                var posCData = em.GetComponentData<Translation>(entity);
                var rotCData = em.GetComponentData<Rotation>(entity);

                return GetCenterOfMass(massCData, posCData, rotCData);
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's Translation component instead. (RemovedAfter 2019-11-14")]
        public static float3 GetPosition(this Entity entity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<Translation>(entity))
            {
                return em.GetComponentData<Translation>(entity).Value;
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's Rotation component instead. (RemovedAfter 2019-11-14")]
        public static quaternion GetRotation(this Entity entity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<Rotation>(entity))
            {
                return em.GetComponentData<Rotation>(entity).Value;
            }
            return quaternion.identity;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component instead. (RemovedAfter 2019-11-14")]
        public static void SetVelocities(this Entity entity, float3 linearVelocity, float3 angularVelocity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                velocityData.Linear = linearVelocity;
                velocityData.Angular = angularVelocity;

                em.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component instead. (RemovedAfter 2019-11-14")]
        public static void GetVelocities(this Entity entity, out float3 linearVelocity, out float3 angularVelocity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
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
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                return velocityData.Linear;
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Get the linear velocity of a rigid body at a given point (in world space)
        public static float3 GetLinearVelocity(this Entity entity, float3 point)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity) &&
                em.HasComponent<PhysicsMass>(entity) &&
                em.HasComponent<Translation>(entity) &&
                em.HasComponent<Rotation>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                var massData = em.GetComponentData<PhysicsMass>(entity);
                var posCData = em.GetComponentData<Translation>(entity);
                var rotCData = em.GetComponentData<Rotation>(entity);

                return velocityData.GetLinearVelocity(massData, posCData, rotCData, point);
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the entity's PhysicsVelocity component instead. (RemovedAfter 2019-11-14")]
        // Set the linear velocity of a rigid body (in world space)
        public static void SetLinearVelocity(this Entity entity, float3 linearVelocity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                velocityData.Linear = linearVelocity;
                em.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static float3 GetAngularVelocity(this Entity entity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity) &&
                em.HasComponent<PhysicsMass>(entity) &&
                em.HasComponent<Rotation>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                var massData = em.GetComponentData<PhysicsMass>(entity);
                var rotCData = em.GetComponentData<Rotation>(entity);

                return velocityData.GetAngularVelocity(massData, rotCData);
            }
            return float3.zero;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(this Entity entity, float3 angularVelocity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity) &&
                em.HasComponent<PhysicsMass>(entity) &&
                em.HasComponent<Rotation>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                var massData = em.GetComponentData<PhysicsMass>(entity);
                var rotCData = em.GetComponentData<Rotation>(entity);

                velocityData.SetAngularVelocity(massData, rotCData, angularVelocity);

                em.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        public static void ApplyImpulse(this Entity entity, float3 impulse, float3 point)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity) &&
                em.HasComponent<PhysicsMass>(entity) &&
                em.HasComponent<Translation>(entity) &&
                em.HasComponent<Rotation>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                var massData = em.GetComponentData<PhysicsMass>(entity);
                var posCData = em.GetComponentData<Translation>(entity);
                var rotCData = em.GetComponentData<Rotation>(entity);

                velocityData.ApplyImpulse(massData, posCData, rotCData, impulse, point);

                em.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        public static void ApplyLinearImpulse(this Entity entity, float3 impulse)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity) &&
                em.HasComponent<PhysicsMass>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                var massData = em.GetComponentData<PhysicsMass>(entity);

                velocityData.ApplyLinearImpulse(massData, impulse);

                em.SetComponentData(entity, velocityData);
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This method has been deprecated. Use the variant passing component data instead. (RemovedAfter 2019-11-14")]
        public static void ApplyAngularImpulse(this Entity entity, float3 impulse)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<PhysicsVelocity>(entity) &&
                em.HasComponent<PhysicsMass>(entity))
            {
                var velocityData = em.GetComponentData<PhysicsVelocity>(entity);
                var massData = em.GetComponentData<PhysicsMass>(entity);

                velocityData.ApplyAngularImpulse(massData, impulse);

                em.SetComponentData(entity, velocityData);
            }
        }
    }
}
