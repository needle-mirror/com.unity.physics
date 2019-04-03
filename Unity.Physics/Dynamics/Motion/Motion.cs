using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Unity.Physics
{
    // Describes how mass is distributed within an object.
    // Represented by a transformed box inertia of unit mass.
    public struct MassDistribution
    {
        // The center of mass and the orientation to principal axis space
        public RigidTransform Transform;

        // Diagonalized inertia tensor for a unit mass
        public float3 InertiaTensor;

        // Get the inertia as a 3x3 matrix
        public float3x3 InertiaMatrix
        {
            get
            {
                var r = new float3x3(Transform.rot);
                var r2 = new float3x3(InertiaTensor.x * r.c0, InertiaTensor.y * r.c1, InertiaTensor.z * r.c2);
                return math.mul(r2, math.inverse(r));
            }
        }
    }

    // The mass properties of an object.
    public struct MassProperties
    {
        // The distribution of a unit mass throughout the object.
        public MassDistribution MassDistribution;

        // The volume of the object.
        public float Volume;

        // Upper bound on the rate of change of the object's extent in any direction,
        // with respect to angular speed around its center of mass.
        // Used to determine how much to expand a rigid body's AABB to enclose its swept volume.
        public float AngularExpansionFactor;

        // The mass properties of a unit sphere
        public static readonly MassProperties UnitSphere = new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = RigidTransform.identity,
                InertiaTensor = new float3(2.0f / 5.0f)
            },
            Volume = (4.0f / 3.0f) * (float)math.PI,
            AngularExpansionFactor = 0.0f
        };
    }

    // A dynamic rigid body's "cold" motion data, used during Jacobian building and integration.
    public struct MotionData
    {
        // Center of mass and inertia orientation in world space
        public RigidTransform WorldFromMotion;

        // Center of mass and inertia orientation in rigid body space
        public RigidTransform BodyFromMotion;

        // Damping applied to the motion during each simulation step
        public float LinearDamping;
        public float AngularDamping;

        // A multiplier applied to the simulation step's gravity vector
        public float GravityFactor;

        public static readonly MotionData Zero = new MotionData
        {
            WorldFromMotion = RigidTransform.identity,
            BodyFromMotion = RigidTransform.identity,
            LinearDamping = 0.0f,
            AngularDamping = 0.0f,
            GravityFactor = 0.0f
        };
    }

    // A dynamic rigid body's "hot" motion data, used during solving.
    public struct MotionVelocity
    {
        public float3 LinearVelocity;   // world space
        public float3 AngularVelocity;  // motion space
        public float4 InverseInertiaAndMass;
        public float AngularExpansionFactor;

        public static readonly MotionVelocity Zero = new MotionVelocity
        {
            LinearVelocity = new float3(0),
            AngularVelocity = new float3(0),
            InverseInertiaAndMass = new float4(0),
            AngularExpansionFactor = 0.0f
        };

        // Apply a linear impulse (in world space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulse(float3 impulse)
        {
            LinearVelocity += impulse * InverseInertiaAndMass.w;
        }

        // Apply an angular impulse (in motion space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyAngularImpulse(float3 impulse)
        {
            AngularVelocity += impulse * InverseInertiaAndMass.xyz;
        }

        // Calculate the distances by which to expand collision tolerances based on the speed of the object.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public MotionExpansion CalculateExpansion(float timeStep) => new MotionExpansion
        {
            Linear = LinearVelocity * timeStep,
            Uniform = math.min(math.length(AngularVelocity) * timeStep, (float)math.PI / 2.0f) * AngularExpansionFactor
        };
    }

    // Provides an upper bound on change in a body's extents in any direction during a step.
    // Used to determine how far away from the body to look for collisions.
    public struct MotionExpansion
    {
        public float3 Linear;   // how far to look ahead of the object
        public float Uniform;   // how far to look around the object

        public float MaxDistance => math.length(Linear) + Uniform;

        public static readonly MotionExpansion Zero = new MotionExpansion
        {
            Linear = new float3(0),
            Uniform = 0.0f
        };

        // Expand an AABB
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Aabb ExpandAabb(Aabb aabb) => new Aabb
        {
            Max = math.max(aabb.Max, aabb.Max + Linear) + Uniform,
            Min = math.min(aabb.Min, aabb.Min + Linear) - Uniform
        };
    }
}
