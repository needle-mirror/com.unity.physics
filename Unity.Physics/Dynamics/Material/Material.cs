using System;
using Unity.Mathematics;

namespace Unity.Physics
{
    // Describes how an object should respond to collisions with other objects.
    public struct Material
    {
        public MaterialFlags Flags;
        public CombinePolicy FrictionCombinePolicy;
        public CombinePolicy RestitutionCombinePolicy;
        public float Friction;
        public float Restitution;

        // If true, the object does not collide but raises trigger events instead
        public bool IsTrigger => (Flags & MaterialFlags.IsTrigger) != 0;

        // If true, the object raises collision events if an impulse is applied during solving
        public bool EnableCollisionEvents => (Flags & MaterialFlags.EnableCollisionEvents) != 0;

        // If true, the object can have its inertia and mass overridden during solving
        public bool EnableMassFactors => (Flags & MaterialFlags.EnableMassFactors) != 0;

        // If true, the object can apply a surface velocity to its contact points
        public bool EnableSurfaceVelocity => (Flags & MaterialFlags.EnableSurfaceVelocity) != 0;

        // If true, the object can limit the impulses applied to its contact points
        public bool EnableMaxImpulse => (Flags & MaterialFlags.EnableMaxImpulse) != 0;

        [Flags]
        public enum MaterialFlags : byte
        {
            IsTrigger = 1 << 0,
            EnableCollisionEvents = 1 << 1,
            EnableMassFactors = 1 << 2,
            EnableSurfaceVelocity = 1 << 3,
            EnableMaxImpulse = 1 << 4
        }

        // Defines how a value from a pair of materials should be combined.
        public enum CombinePolicy : byte
        {
            GeometricMean,  // sqrt(a * b)
            Minimum,        // min(a, b)
            Maximum,        // max(a, b)
            ArithmeticMean  // (a + b) / 2
        }

        // A default material.
        public static readonly Material Default = new Material
        {
            FrictionCombinePolicy = CombinePolicy.GeometricMean,
            RestitutionCombinePolicy = CombinePolicy.GeometricMean,
            Friction = 0.5f,
            Restitution = 0.0f
        };

        // Get a combined friction value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static float GetCombinedFriction(Material materialA, Material materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.FrictionCombinePolicy, (int)materialB.FrictionCombinePolicy);
            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Friction * materialB.Friction);
                case CombinePolicy.Minimum:
                    return math.min(materialA.Friction, materialB.Friction);
                case CombinePolicy.Maximum:
                    return math.max(materialA.Friction, materialB.Friction);
                case CombinePolicy.ArithmeticMean:
                    return (materialA.Friction + materialB.Friction) * 0.5f;
                default:
                    return 0;
            }
        }

        // Get a combined restitution value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static float GetCombinedRestitution(Material materialA, Material materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.RestitutionCombinePolicy, (int)materialB.RestitutionCombinePolicy);
            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Restitution * materialB.Restitution);
                case CombinePolicy.Minimum:
                    return math.min(materialA.Restitution, materialB.Restitution);
                case CombinePolicy.Maximum:
                    return math.max(materialA.Restitution, materialB.Restitution);
                case CombinePolicy.ArithmeticMean:
                    return (materialA.Restitution + materialB.Restitution) * 0.5f;
                default:
                    return 0;
            }
        }
    }
}
