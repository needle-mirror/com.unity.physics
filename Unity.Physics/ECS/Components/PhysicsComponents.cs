using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    // The collision geometry of a rigid body.
    // This causes the entity to appear in the physics world.
    public struct PhysicsCollider : IComponentData
    {
        public BlobAssetReference<Collider> Value;  // null is allowed

        public unsafe bool IsValid => Value.GetUnsafePtr() != null;
        public unsafe Collider* ColliderPtr => (Collider*)Value.GetUnsafePtr();
        public unsafe MassProperties MassProperties => Value.GetUnsafePtr() != null ? Value.Value.MassProperties : MassProperties.UnitSphere;
    }
    
    // The mass properties of a rigid body.
    // If not present, the rigid body has infinite mass and inertia.
    public struct PhysicsMass : IComponentData
    {
        public RigidTransform Transform;        // center of mass and orientation of principal axes
        public float InverseMass;               // zero is allowed, for infinite mass
        public float3 InverseInertia;           // zero is allowed, for infinite inertia
        public float AngularExpansionFactor;    // see MassProperties.AngularExpansionFactor

        public float3 CenterOfMass => Transform.pos;
        public quaternion InertiaOrientation => Transform.rot;

        public static PhysicsMass CreateDynamic(MassProperties massProperties, float mass)
        {
            if (mass <= 0)
                throw new System.ArgumentOutOfRangeException();

            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = math.rcp(mass),
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

        public static PhysicsMass CreateKinematic(MassProperties massProperties)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = 0f,
                InverseInertia = float3.zero,
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }
    }

    // The velocity of a rigid body.
    // If not present, the rigid body is static.
    public struct PhysicsVelocity : IComponentData
    {
        public float3 Linear;   // in world space
        public float3 Angular;  // in inertia space, around the rigid body's center of mass     // TODO: make this world space
    }

    // Optional damping applied to the rigid body velocities during each simulation step.
    // This scales the velocities using: math.clamp(1 - damping * Timestep, 0, 1)
    public struct PhysicsDamping : IComponentData
    {
        public float Linear;     // damping applied to the linear velocity
        public float Angular;    // damping applied to the angular velocity
    }

    // Optional gravity factor applied to a rigid body during each simulation step.
    // This scales the gravity vector supplied to the simulation step.
    public struct PhysicsGravityFactor : IComponentData
    {
        public float Value;
    }

    // Optional custom data attached to a rigid body.
    // This will be copied to any contacts and Jacobians involving this rigid body,
    // providing additional context to any user logic operating on those structures.
    public struct PhysicsCustomData : IComponentData
    {
        public byte Value;
    }

    // A set of constraints on the relative motion of a pair of rigid bodies.
    public struct PhysicsJoint : IComponentData
    {
        public BlobAssetReference<JointData> JointData;
        public Entity EntityA;
        public Entity EntityB;
        public int EnableCollision; // If non-zero, the constrained entities can collide with each other
    }

    // Parameters describing how to step the physics world.
    // If none is present in the scene, default values will be used.
    public struct PhysicsStep : IComponentData
    {
        public SimulationType SimulationType;
        public float3 Gravity;
        public int SolverIterationCount;
        public int ThreadCountHint;

        public static readonly PhysicsStep Default = new PhysicsStep
        {
            SimulationType = SimulationType.UnityPhysics,
            Gravity = -9.81f * math.up(),
            SolverIterationCount = 4,
            // Unity DOTS framework doesn't expose number worker threads in current version,
            // For this reason we have to make a guess.
            // For optimal physics performance set ThreadCountHint to number of physical CPU cores on your target device.
            ThreadCountHint = 4
        };
    }
}
