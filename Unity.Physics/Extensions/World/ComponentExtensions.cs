using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Extensions
{
    // Utility functions acting on physics components
    public static partial class ComponentExtensions
    {
        // Get the center of mass in world space
        public static float3 GetCenterOfMass(PhysicsMass massData, Translation posData, Rotation rotData)
        {
            return math.rotate(rotData.Value, massData.CenterOfMass) + posData.Value;
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

        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static float3 GetAngularVelocity(this PhysicsVelocity velocityData, PhysicsMass massData, Rotation rotData)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(rotData.Value, massData.InertiaOrientation);
            return math.rotate(inertiaOrientationInWorldSpace, velocityData.Angular);
        }

        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(ref this PhysicsVelocity velocityData, PhysicsMass massData, Rotation rotData, float3 angularVelocity)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(rotData.Value, massData.InertiaOrientation);
            float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), angularVelocity);

            velocityData.Angular = angularVelocityInertiaSpace;
        }

        public static void ApplyImpulse(ref this PhysicsVelocity pv, PhysicsMass pm, Translation t, Rotation r, float3 impulse, float3 point)
        {
            // Linear
            pv.ApplyLinearImpulse(pm, impulse);

            // Angular
            {
                // Calculate point impulse
                var worldFromEntity = new RigidTransform(r.Value, t.Value);
                var worldFromMotion = math.mul(worldFromEntity, pm.Transform);
                float3 angularImpulseWorldSpace = math.cross(point - worldFromMotion.pos, impulse);
                float3 angularImpulseInertiaSpace = math.rotate(math.inverse(worldFromMotion.rot), angularImpulseWorldSpace);

                pv.ApplyAngularImpulse(pm, angularImpulseInertiaSpace);
            }
        }

        public static void ApplyLinearImpulse(ref this PhysicsVelocity velocityData, PhysicsMass massData, float3 impulse)
        {
            velocityData.Linear += impulse * massData.InverseMass;
        }

        public static void ApplyAngularImpulse(ref this PhysicsVelocity velocityData, PhysicsMass massData, float3 impulse)
        {
            velocityData.Angular += impulse * massData.InverseInertia;
        }
    }
}
