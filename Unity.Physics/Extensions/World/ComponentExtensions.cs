using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Extensions
{
    // Utility functions acting on physics components
    public static class ComponentExtensions
    {
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

        // Get the center of mass in world space
        public static float3 GetCenterOfMass(PhysicsMass massData, Translation posData, Rotation rotData)
        {
            return math.rotate(rotData.Value, massData.CenterOfMass) + posData.Value;
        }

        public static float3 GetPosition(this Entity entity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<Translation>(entity))
            {
                return em.GetComponentData<Translation>(entity).Value;
            }
            return float3.zero;
        }

        public static quaternion GetRotation(this Entity entity)
        {
            var em = World.Active.EntityManager;

            if (em.HasComponent<Rotation>(entity))
            {
                return em.GetComponentData<Rotation>(entity).Value;
            }
            return quaternion.identity;
        }

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

        // Get the linear velocity of a rigid body at a given point (in world space)
        public static float3 GetLinearVelocity(this PhysicsVelocity velocityData, PhysicsMass massData, Translation posData, Rotation rotData, float3 point)
        {
            var worldFromEntity = new RigidTransform(rotData.Value, posData.Value);
            var worldFromMotion = math.mul(worldFromEntity, massData.Transform);

            float3 angularVelocity = math.rotate(worldFromMotion, velocityData.Angular);
            float3 linearVelocity = math.cross(angularVelocity, (point - worldFromMotion.pos));
            return velocityData.Linear + linearVelocity;
        }

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

        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static float3 GetAngularVelocity(this PhysicsVelocity velocityData, PhysicsMass massData, Rotation rotData)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(rotData.Value, massData.InertiaOrientation);
            return math.rotate(inertiaOrientationInWorldSpace, velocityData.Angular);
        }

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

        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(ref this PhysicsVelocity velocityData, PhysicsMass massData, Rotation rotData, float3 angularVelocity)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(rotData.Value, massData.InertiaOrientation);
            float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), angularVelocity);

            velocityData.Angular = angularVelocityInertiaSpace;
        }

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

        public static void ApplyImpulse(ref this PhysicsVelocity pv, PhysicsMass pm, Translation t, Rotation r, float3 impulse, float3 point)
        {
            // Linear
            pv.Linear += impulse;

            // Angular
            {
                // Calculate point impulse
                var worldFromEntity = new RigidTransform(r.Value, t.Value);
                var worldFromMotion = math.mul(worldFromEntity, pm.Transform);
                float3 angularImpulseWorldSpace = math.cross(point - worldFromMotion.pos, impulse);
                float3 angularImpulseInertiaSpace = math.rotate(math.inverse(worldFromMotion.rot), angularImpulseWorldSpace);

                pv.Angular += angularImpulseInertiaSpace * pm.InverseInertia;
            }
        }

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

        public static void ApplyLinearImpulse(ref this PhysicsVelocity velocityData, PhysicsMass massData, float3 impulse)
        {
            velocityData.Linear += impulse * massData.InverseMass;
        }

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

        public static void ApplyAngularImpulse(ref this PhysicsVelocity velocityData, PhysicsMass massData, float3 impulse)
        {
            velocityData.Angular += impulse * massData.InverseInertia;
        }
    }
}
