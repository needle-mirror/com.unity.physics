using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    public enum ConstraintType : byte
    {
        Linear,
        Angular
    }

    // A linear or angular constraint in 1, 2, or 3 dimensions.
    public struct Constraint
    {
        // TODO think more about these
        // Current values give tau = 0.6 damping = 0.99 at 50hz
        // The values are huge and we can't get damping = 1 -- a stiff constraint is the limit of a damped spring as spring params go to infinity.
        public const float DefaultSpringFrequency = 61950.977267809007887192914302327f;
        public const float DefaultSpringDamping = 2530.12155587434178122630287018f;

        public bool3 ConstrainedAxes;
        public ConstraintType Type;

        public float Min;
        public float Max;
        public float SpringFrequency;
        public float SpringDamping;

        // Number of affected degrees of freedom.  1, 2, or 3.
        public int Dimension => math.select(math.select(2, 1, ConstrainedAxes.x ^ ConstrainedAxes.y ^ ConstrainedAxes.z), 3, math.all(ConstrainedAxes));

        // Selects the free axis from a constraint with Dimension == 1
        public int FreeAxis2D
        {
            get
            {
                Assert.IsTrue(Dimension == 2);
                return math.select(2, math.select(0, 1, ConstrainedAxes[0]), ConstrainedAxes[2]);
            }
        }

        // Selects the constrained axis from a constraint with Dimension == 2
        public int ConstrainedAxis1D
        {
            get
            {
                Assert.IsTrue(Dimension == 1);
                return math.select(math.select(1, 0, ConstrainedAxes[0]), 2, ConstrainedAxes[2]);
            }
        }

        #region Common linear constraints

        // 3 DOF linear constraint with limits at zero
        public static Constraint BallAndSocket(float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        // 3 DOF linear constraint
        public static Constraint StiffSpring(float minDistance, float maxDistance, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = minDistance,
                Max = maxDistance,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        // 2 DOF linear constraint
        public static Constraint Cylindrical(int freeAxis, float minDistance, float maxDistance, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Linear,
                Min = minDistance,
                Max = maxDistance,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        // 1 DOF linear constraint
        public static Constraint Planar(int limitedAxis, float minDistance, float maxDistance, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(limitedAxis >= 0 && limitedAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(limitedAxis == 0, limitedAxis == 1, limitedAxis == 2),
                Type = ConstraintType.Linear,
                Min = minDistance,
                Max = maxDistance,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        #endregion

        #region Common angular constraints

        // 3DOF angular constraint with limits at zero
        public static Constraint FixedAngle(float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Angular,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        // 2 DOF angular constraint with limits at zero
        public static Constraint Hinge(int freeAxis, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Angular,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        // 2 DOF angular constraint
        public static Constraint Cone(int freeAxis, float minAngle, float maxAngle, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Angular,
                Min = minAngle,
                Max = maxAngle,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        // 1 DOF angular constraint
        public static Constraint Twist(int limitedAxis, float minAngle, float maxAngle, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(limitedAxis >= 0 && limitedAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(limitedAxis == 0, limitedAxis == 1, limitedAxis == 2),
                Type = ConstraintType.Angular,
                Min = minAngle,
                Max = maxAngle,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        #endregion
    }

    // A set of constraints on the relative motion between a pair of rigid bodies.
    // Warning: This is just the header, the joint's variable sized data follows it in memory.
    // Therefore this struct must always be passed by reference, never by value.
    public struct JointData
    {
        // Transform from joint definition space to body space
        public MTransform AFromJoint { get; private set; }
        public MTransform BFromJoint { get; private set; }

        // Array of constraints
        private BlobArray m_ConstraintsBlob;
        public byte Version { get; private set; }

        // Accessor for the constraints
        public BlobArray.Accessor<Constraint> Constraints => new BlobArray.Accessor<Constraint>(ref m_ConstraintsBlob);
        public int NumConstraints => m_ConstraintsBlob.Length;

        // Create a joint asset with the given constraints
        public static unsafe BlobAssetReference<JointData> Create(MTransform aFromJoint, MTransform bFromJoint, Constraint[] constraints)
        {
            // Allocate
            int totalSize = sizeof(JointData) + sizeof(Constraint) * constraints.Length;
            JointData* jointData = (JointData*)UnsafeUtility.Malloc(totalSize, 16, Allocator.Temp);
            UnsafeUtility.MemClear(jointData, totalSize);

            // Initialize
            {
                jointData->AFromJoint = aFromJoint;
                jointData->BFromJoint = bFromJoint;
                jointData->Version = 1;

                byte* end = (byte*)jointData + sizeof(JointData);
                jointData->m_ConstraintsBlob.Offset = UnsafeEx.CalculateOffset(end, ref jointData->m_ConstraintsBlob);
                jointData->m_ConstraintsBlob.Length = constraints.Length;

                for (int i = 0; i < constraints.Length; i++)
                {
                    jointData->Constraints[i] = constraints[i];
                }
            }

            var blob = BlobAssetReference<JointData>.Create(jointData, totalSize);
            
            UnsafeUtility.Free(jointData, Allocator.Temp);

            return blob;
        }

        #region Common joint descriptions

        public static BlobAssetReference<JointData> CreateBallAndSocket(float3 positionAinA, float3 positionBinB)
        {
            return Create(
                new MTransform(float3x3.identity, positionAinA),
                new MTransform(float3x3.identity, positionBinB),
                new[]
                {
                    Constraint.BallAndSocket()
                }
            );
        }

        public static BlobAssetReference<JointData> CreateStiffSpring(float3 positionAinA, float3 positionBinB, float minDistance, float maxDistance)
        {
            return Create(
                new MTransform(float3x3.identity, positionAinA),
                new MTransform(float3x3.identity, positionBinB),
                new[]
                {
                    Constraint.StiffSpring(minDistance, maxDistance)
                }
            );
        }

        public static BlobAssetReference<JointData> CreatePrismatic(float3 positionAinA, float3 positionBinB, float3 axisInA, float3 axisInB,
            float3 perpendicularAxisInA, float3 perpendicularAxisInB, float minDistanceOnAxis, float maxDistanceOnAxis, float minDistanceFromAxis, float maxDistanceFromAxis)
        {
            // Check that the perpendicular axes are perpendicular
            Assert.IsTrue(math.abs(math.dot(axisInA, perpendicularAxisInA)) < 1e-5f);
            Assert.IsTrue(math.abs(math.dot(axisInB, perpendicularAxisInB)) < 1e-5f);

            return Create(
                new MTransform(new float3x3(axisInA, perpendicularAxisInA, math.cross(axisInA, perpendicularAxisInA)), positionAinA),
                new MTransform(new float3x3(axisInB, perpendicularAxisInB, math.cross(axisInB, perpendicularAxisInB)), positionBinB),
                new[]
                {
                    Constraint.FixedAngle(),
                    Constraint.Planar(0, minDistanceOnAxis, maxDistanceOnAxis),
                    Constraint.Cylindrical(0, minDistanceFromAxis, maxDistanceFromAxis)
                }
            );
        }

        public static BlobAssetReference<JointData> CreateHinge(float3 positionAinA, float3 positionBinB, float3 axisInA, float3 axisInB)
        {
            CalculatePerpendicularNormalized(axisInA, out float3 perpendicularA1, out float3 perpendicularA2);
            CalculatePerpendicularNormalized(axisInB, out float3 perpendicularB1, out float3 perpendicularB2);
            return Create(
                new MTransform(new float3x3(axisInA, perpendicularA1, perpendicularA2), positionAinA),
                new MTransform(new float3x3(axisInB, perpendicularB1, perpendicularB2), positionBinB),
                new[]
                {
                    Constraint.Hinge(0),
                    Constraint.BallAndSocket()
                }
            );
        }

        public static BlobAssetReference<JointData> CreateLimitedHinge(float3 positionAinA, float3 positionBinB,
            float3 axisInA, float3 axisInB, float3 perpendicularInA, float3 perpendicularInB, float minAngle, float maxAngle)
        {
            return Create(
                new MTransform(new float3x3(axisInA, perpendicularInA, math.cross(axisInA, perpendicularInA)), positionAinA),
                new MTransform(new float3x3(axisInB, perpendicularInB, math.cross(axisInB, perpendicularInB)), positionBinB),
                new[]
                {
                    Constraint.Twist(0, minAngle, maxAngle),
                    Constraint.Hinge(0),
                    Constraint.BallAndSocket()
                });
        }

        public static void CreateRagdoll(float3 positionAinA, float3 positionBinB,
            float3 twistAxisInA, float3 twistAxisInB, float3 perpendicularAxisInA, float3 perpendicularAxisInB,
            float maxConeAngle, float minPerpendicularAngle, float maxPerpendicularAngle, float minTwistAngle, float maxTwistAngle,
            out BlobAssetReference<JointData> jointData0, out BlobAssetReference<JointData> jointData1)
        {
            // TODO - the hkpRagdollConstraint can't be represented with a single joint.  The reason is that the hkpRagdollConstraint
            // cone limit and plane limit apply to different axes in A and B.  For example, if the cone limit constrains the angle between
            // axis 0 in each body, then the plane limit must constraint axis 0 in one body to a non-0 axis in the other.
            // Currently I use two joints to solve this problem.  Another solution would be to extend Constraint so that it can swizzle the axes.

            // Check that the perpendicular axes are perpendicular
            Assert.IsTrue(math.abs(math.dot(twistAxisInA, perpendicularAxisInA)) < 1e-5f);
            Assert.IsTrue(math.abs(math.dot(twistAxisInB, perpendicularAxisInB)) < 1e-5f);

            float3 crossA = math.cross(twistAxisInA, perpendicularAxisInA);
            float3 crossB = math.cross(twistAxisInB, perpendicularAxisInB);
            var transformA = new MTransform(new float3x3(twistAxisInA, perpendicularAxisInA, crossA), positionAinA);

            // First joint data: twist and primary cone
            jointData0 = Create(
                transformA,
                new MTransform(new float3x3(twistAxisInB, perpendicularAxisInB, crossB), positionBinB),
                new[] {
                    Constraint.Twist(0, minTwistAngle, maxTwistAngle),  // Twist limit
                    Constraint.Cone(0, 0.0f, maxConeAngle)              // Cone about the twist axis
                }
            );

            // Second joint data: perpendicular cone and ball socket
            jointData1 = Create(
                transformA,
                new MTransform(new float3x3(perpendicularAxisInB, twistAxisInB, -crossB), positionBinB),
                new[]
                {
                    Constraint.Cone(0, minPerpendicularAngle, maxPerpendicularAngle), // Cone about the reference axis
                    Constraint.BallAndSocket()
                }
            );
        }

        public static BlobAssetReference<JointData> CreateFixed(float3 positionAinA, float3 positionBinB, quaternion orientationAinA, quaternion orientationBinB)
        {
            return Create(
                new MTransform(orientationAinA, positionAinA),
                new MTransform(orientationBinB, positionBinB),
                new[]
                {
                    Constraint.BallAndSocket(),
                    Constraint.FixedAngle()
                }
            );
        }

        #endregion
    }

    // A runtime joint instance, attached to specific rigid bodies
    public unsafe struct Joint
    {
        public JointData* JointData;
        public BodyIndexPair BodyPair;
        public int EnableCollision; // If non-zero, allows these bodies to collide

        // The entity that contained the component which created this joint
        // Note, this isn't necessarily an entity with a rigid body, as a pair
        // of bodies can have an arbitrary number of constraints, but only one
        // instance of a particular component type per entity
        public Entity Entity;
    }
}
