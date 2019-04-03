using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics.Extensions
{
    // Utility functions acting on a physics world
    public static class PhysicsWorldExtensions
    {
        // Find the index in the Bodies array of a particular Rigid Body from its Entity
        public static int GetRigidBodyIndex(this in PhysicsWorld world, Entity entity)
        {
            int idx = 0;
            for( int i = 0; i < world.Bodies.Length; i++)
            {
                if (world.Bodies[i].Entity == entity) break;
                idx++;
            }
            return (idx < world.NumBodies) ? idx : -1;
        }

        public static CollisionFilter GetCollisionFilter(this in PhysicsWorld world, int rigidBodyIndex)
        {
            CollisionFilter filter = CollisionFilter.Default;
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumBodies)) return filter;

            unsafe { filter = world.Bodies[rigidBodyIndex].Collider->Filter; }

            return filter;
        }

        public static float GetMass(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return 0;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            return 0 == mv.InverseInertiaAndMass.w ? 0.0f : 1.0f / mv.InverseInertiaAndMass.w;
        }

        // Get the effective mass of a Rigid Body in a given direction and from a particular point (in World Space)
        public static float GetEffectiveMass(this in PhysicsWorld world, int rigidBodyIndex, float3 impulse, float3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return 0;

            float effMass = 0;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            float3 pointDir = math.normalizesafe(point - GetCenterOfMass(world, rigidBodyIndex));
            float3 impulseDir = math.normalizesafe(impulse);

            float3 jacobian = math.cross(pointDir, impulseDir);
            float invEffMass = math.csum(math.dot(jacobian, jacobian) * mv.InverseInertiaAndMass.xyz);
            effMass = math.select(1.0f / invEffMass, 0.0f, math.abs(invEffMass) < 1e-5);

            return effMass;
        }

        // Get the Rigid Bodies Center of Mass (in World Space)
        public static float3 GetCenterOfMass(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;
            
            return world.MotionDatas[rigidBodyIndex].WorldFromMotion.pos;
        }

        public static float3 GetPosition(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            // Motion to body transform
            MotionData md = world.MotionDatas[rigidBodyIndex];

            RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
            return worldFromBody.pos;
        }

        public static quaternion GetRotation(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return quaternion.identity;

            // Motion to body transform
            MotionData md = world.MotionDatas[rigidBodyIndex];

            RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
            return worldFromBody.rot;
        }

        // Get the linear velocity of a rigid body (in world space)
        public static float3 GetLinearVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            return world.MotionVelocities[rigidBodyIndex].LinearVelocity;
        }

        // Set the linear velocity of a rigid body (in world space)
        public static void SetLinearVelocity(this PhysicsWorld world, int rigidBodyIndex, float3 linearVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            Unity.Collections.NativeSlice<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.LinearVelocity = linearVelocity;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Get the linear velocity of a rigid body at a given point (in world space)
        public static float3 GetLinearVelocity(this in PhysicsWorld world, int rigidBodyIndex, float3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            float3 av = math.rotate(md.WorldFromMotion, mv.AngularVelocity);
            float3 lv = float3.zero;
            lv += math.cross(av, (point - md.WorldFromMotion.pos));
            lv += mv.LinearVelocity;

            return lv;
        }

        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static float3 GetAngularVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            return math.rotate(md.WorldFromMotion, mv.AngularVelocity);
        }

        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(this PhysicsWorld world, int rigidBodyIndex, float3 angularVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularVelocityMotionSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularVelocity);

            Unity.Collections.NativeSlice<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.AngularVelocity = angularVelocityMotionSpace;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply an impulse to a rigid body at a point (in world space)
        public static void ApplyImpulse(this PhysicsWorld world, int rigidBodyIndex, float3 linearImpulse, float3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularImpulseWorldSpace = math.cross(point - md.WorldFromMotion.pos, linearImpulse);
            float3 angularImpulseMotionSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularImpulseWorldSpace);

            Unity.Collections.NativeSlice<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyLinearImpulse(linearImpulse); 
            mv.ApplyAngularImpulse(angularImpulseMotionSpace);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply a linear impulse to a rigid body (in world space)
        public static void ApplyLinearImpulse(this PhysicsWorld world, int rigidBodyIndex, float3 linearImpulse)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            Unity.Collections.NativeSlice<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyLinearImpulse(linearImpulse);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply an angular impulse to a rigidBodyIndex (in world space)
        public static void ApplyAngularImpulse(this PhysicsWorld world, int rigidBodyIndex, float3 angularImpulse)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularImpulseInertiaSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularImpulse);

            Unity.Collections.NativeSlice<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyAngularImpulse(angularImpulseInertiaSpace);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Calculate a linear and angular velocity required to move the given rigid body to the given target transform
        // in the given time step.
        public static void CalculateVelocityToTarget(
            this PhysicsWorld world, int rigidBodyIndex, float3 targetPosition, quaternion targetOrientation, float timeStep,
            out float3 requiredLinearVelocity, out float3 requiredAngularVelocity)
        {
            // TODO
            requiredLinearVelocity = float3.zero;
            requiredAngularVelocity = float3.zero;
        }
    }
}
