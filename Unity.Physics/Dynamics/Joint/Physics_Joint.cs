using System;
using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    /// <summary>Values that represent constraint types. </summary>
    public enum ConstraintType : byte
    {
        /// <summary>An enum constant representing the linear type. </summary>
        Linear,
        /// <summary>An enum constant representing the angular type. </summary>
        Angular,
        /// <summary>An enum constant representing the rotation motor type. </summary>
        RotationMotor,
        /// <summary>An enum constant representing the angular velocity motor type. </summary>
        AngularVelocityMotor,
        /// <summary>An enum constant representing the position motor type. </summary>
        PositionMotor,
        /// <summary>An enum constant representing the linear velocity motor type. </summary>
        LinearVelocityMotor
    }

    /// <summary>A linear or angular constraint in 1, 2, or 3 dimensions. </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Constraint : IEquatable<Constraint>
    {
        /// <summary>
        /// (Immutable) Current values give tau = 0.6 damping = 0.99 at 60hz The values are huge and we
        /// can't get damping = 1 -- a stiff constraint is the limit of a damped spring as spring params
        /// go to infinity. Rather then baking them these values could be calculated using
        /// JacobianUtilities.CalculateSpringFrequencyAndDamping(0.6f, 0.99f, math.rcp(60.0f), 4, out DefaultSpringFrequency, out DefaultSpringDamping);
        /// </summary>
        public const float DefaultSpringFrequency = 74341.31f;
        /// <summary>The default spring damping. </summary>
        public const float DefaultSpringDamping = 2530.126f;
        /// <summary>The default maximum impulse. </summary>
        public const float DefaultMaxImpulse = float.PositiveInfinity;

        /// <summary>The constrained axes. </summary>
        public bool3 ConstrainedAxes;
        /// <summary>The constraint type. </summary>
        public ConstraintType Type;

        /// <summary>The minimum. </summary>
        public float Min;
        /// <summary>The maximum. </summary>
        public float Max;
        /// <summary>The spring frequency. </summary>
        public float SpringFrequency;
        /// <summary>The spring damping. </summary>
        public float SpringDamping;

        /// <summary>The maximum impulse. </summary>
        public float3 MaxImpulse;
        /// <summary>The target a motor will drive towards. Can be set to zero for non-motor type constraints </summary>
        public float3 Target;
        /// <summary>True to enable, false to disable the impulse events. </summary>
        public bool EnableImpulseEvents;

        // Number of affected degrees of freedom.  1, 2, or 3.
        internal int Dimension
        {
            get
            {
                return math.select(math.select(math.select(2, 0, !math.any(ConstrainedAxes)),
                    1, ConstrainedAxes.x ^ ConstrainedAxes.y ^ ConstrainedAxes.z), 3,
                    math.all(ConstrainedAxes));
            }
        }

        // Selects the free axis from a constraint with Dimension == 1
        internal int FreeAxis2D
        {
            get
            {
                Assert.IsTrue(Dimension == 2);
                return math.select(2, math.select(0, 1, ConstrainedAxes[0]), ConstrainedAxes[2]);
            }
        }

        // Selects the constrained axis from a constraint with Dimension == 2
        internal int ConstrainedAxis1D
        {
            get
            {
                Assert.IsTrue(Dimension == 1);
                return math.select(math.select(1, 0, ConstrainedAxes[0]), 2, ConstrainedAxes[2]);
            }
        }

        #region Common linear constraints

        /// <summary>Constrains linear motion about all three axes to zero. </summary>
        ///
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint BallAndSocket(float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return BallAndSocket(new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>Constrains linear motion about all three axes to zero. </summary>
        ///
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint BallAndSocket(float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>Constrains linear motion about all three axes within the specified range. </summary>
        ///
        /// <param name="distanceRange">The minimum required distance and maximum possible distance
        /// between the constrained bodies. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint LimitedDistance(FloatRange distanceRange, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return LimitedDistance(distanceRange, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>Constrains linear motion about all three axes within the specified range. </summary>
        ///
        /// <param name="distanceRange">The minimum required distance and maximum possible distance
        /// between the constrained bodies. </param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint LimitedDistance(FloatRange distanceRange, float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            distanceRange = distanceRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>
        /// Constrains linear motion about two axes within the specified range. Movement about the third
        /// is unrestricted.
        /// </summary>
        ///
        /// <param name="freeAxis">The axis along which the bodies may freely translate. </param>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance
        /// between the constrained bodies about the two constrained axes. A minimum value of zero
        /// produces a cylindrical range of motion, while a minimum value greater than zero results in a
        /// tube-shaped range of motion. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Cylindrical(int freeAxis, FloatRange distanceRange, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return Cylindrical(freeAxis, distanceRange, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Constrains linear motion about two axes within the specified range. Movement about the third
        /// is unrestricted.
        /// </summary>
        ///
        /// <param name="freeAxis">The axis along which the bodies may freely translate. </param>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance
        /// between the constrained bodies about the two constrained axes. A minimum value of zero
        /// produces a cylindrical range of motion, while a minimum value greater than zero results in a
        /// tube-shaped range of motion. </param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Cylindrical(int freeAxis, FloatRange distanceRange, float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            distanceRange = distanceRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>
        /// Constrains linear motion about one axis within the specified range. Movement about the other
        /// two is unrestricted.
        /// </summary>
        ///
        /// <param name="limitedAxis">The axis along which the bodies' translation is restricted. </param>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance
        /// between the constrained bodies about the constrained axis. Identical minimum and maximum
        /// values result in a plane, while different values constrain the bodies between two parallel
        /// planes. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Planar(int limitedAxis, FloatRange distanceRange, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return Planar(limitedAxis, distanceRange, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Constrains linear motion about one axis within the specified range. Movement about the other
        /// two is unrestricted.
        /// </summary>
        ///
        /// <param name="limitedAxis">The axis along which the bodies' translation is restricted. </param>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance
        /// between the constrained bodies about the constrained axis. Identical minimum and maximum
        /// values result in a plane, while different values constrain the bodies between two parallel
        /// planes. </param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Planar(int limitedAxis, FloatRange distanceRange, float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(limitedAxis >= 0 && limitedAxis <= 2);
            distanceRange = distanceRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(limitedAxis == 0, limitedAxis == 1, limitedAxis == 2),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>
        /// Moves body along one axis to specified target. Movement about the other two perpendicular axes is unrestricted.
        /// </summary>
        /// <param name="target">The target position the motor drives towards.</param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        /// <returns>A Constraint. </returns>
        public static Constraint MotorPlanar(float3 target, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return MotorPlanar(target, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Moves body along one axis to specified target. Movement about the other two perpendicular axes is unrestricted.
        /// </summary>
        /// <param name="target">The target position the motor drives towards.</param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this constraint.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        /// <returns>A constraint</returns>
        public static Constraint MotorPlanar(float3 target, float3 maxImpulse, float springFrequency = DefaultSpringFrequency,
            float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true, false, false),
                Type = ConstraintType.PositionMotor,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = new float3(maxImpulse),
                EnableImpulseEvents = false,
                Target = target
            };
        }

        /// <summary>
        /// Drives a body's velocity to a specified target velocity.
        /// </summary>
        /// <param name="target">The target velocity for the motor.</param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        /// <returns>A Constraint. </returns>
        public static Constraint LinearVelocityMotor(float3 target, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return LinearVelocityMotor(target, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Drives a body's velocity to a specified target velocity.
        /// </summary>
        /// <param name="target">The target velocity for the motor.</param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this constraint.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        /// <returns>A constraint.</returns>
        public static Constraint LinearVelocityMotor(float3 target, float3 maxImpulse, float springFrequency = DefaultSpringFrequency,
            float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true, false, false),
                Type = ConstraintType.LinearVelocityMotor,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = new float3(maxImpulse),
                EnableImpulseEvents = false,
                Target = target
            };
        }

        #endregion

        #region Common angular constraints

        /// <summary>Constrains angular motion about all three axes to zero. </summary>
        ///
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint FixedAngle(float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return FixedAngle(new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>Constrains angular motion about all three axes to zero. </summary>
        ///
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint FixedAngle(float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Angular,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>
        /// Constrains angular motion about two axes to zero. Rotation around the third is unrestricted.
        /// </summary>
        ///
        /// <param name="freeAxis">The axis around which the bodies may freely rotate. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Hinge(int freeAxis, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return Hinge(freeAxis, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Constrains angular motion about two axes to zero. Rotation around the third is unrestricted.
        /// </summary>
        ///
        /// <param name="freeAxis">The axis around which the bodies may freely rotate. </param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Hinge(int freeAxis, float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Angular,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>
        /// Constrains angular motion about two axes within the specified range. Rotation around the
        /// third is unrestricted.
        /// </summary>
        ///
        /// <param name="freeAxis">The axis specifying the height of the cone within which the
        /// bodies may rotate. </param>
        /// <param name="angularRange">The minimum required angle and maximum possible angle between
        /// the free axis and its bind pose orientation. A minimum value of zero produces a conical range
        /// of motion, while a minimum value greater than zero results in motion restricted to the
        /// intersection of the inner and outer cones. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Cone(int freeAxis, FloatRange angularRange, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return Cone(freeAxis, angularRange, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Constrains angular motion about two axes within the specified range. Rotation around the
        /// third is unrestricted.
        /// </summary>
        ///
        /// <param name="freeAxis">The axis specifying the height of the cone within which the
        /// bodies may rotate. </param>
        /// <param name="angularRange">The minimum required angle and maximum possible angle between
        /// the free axis and its bind pose orientation. A minimum value of zero produces a conical range
        /// of motion, while a minimum value greater than zero results in motion restricted to the
        /// intersection of the inner and outer cones. </param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Cone(int freeAxis, FloatRange angularRange, float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            angularRange = angularRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Angular,
                Min = angularRange.Min,
                Max = angularRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>Constrains angular motion about one axis within the specified range. </summary>
        ///
        /// <param name="limitedAxis">The axis around which the bodies' rotation is restricted. </param>
        /// <param name="angularRange">The minimum required angle and maximum possible angle of
        /// rotation between the constrained bodies around the constrained axis. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Twist(int limitedAxis, FloatRange angularRange, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return Twist(limitedAxis, angularRange, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>Constrains angular motion about one axis within the specified range. </summary>
        ///
        /// <param name="limitedAxis">The axis around which the bodies' rotation is restricted. </param>
        /// <param name="angularRange">The minimum required angle and maximum possible angle of
        /// rotation between the constrained bodies around the constrained axis. </param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this
        /// constraint. </param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        ///
        /// <returns>A Constraint. </returns>
        public static Constraint Twist(int limitedAxis, FloatRange angularRange, float3 maxImpulse, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            Assert.IsTrue(limitedAxis >= 0 && limitedAxis <= 2);
            angularRange = angularRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(limitedAxis == 0, limitedAxis == 1, limitedAxis == 2),
                Type = ConstraintType.Angular,
                Min = angularRange.Min,
                Max = angularRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = float3.zero
            };
        }

        /// <summary>
        /// Constrains angular motion about one axis and drives towards a specified rotation.
        /// </summary>
        /// <param name="target">The target rotation the motor is driving towards, in radians.</param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        /// <returns>A Constraint. </returns>
        public static Constraint MotorTwist(float3 target, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return MotorTwist(target, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Constrains angular motion about one axis and drives towards a specified rotation.
        /// </summary>
        /// <param name="target">The target rotation the motor is driving towards, in radians.</param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this constraint.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        /// <returns>A constraint.</returns>
        public static Constraint MotorTwist(float3 target, float3 maxImpulse, float springFrequency = DefaultSpringFrequency,
            float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true, false, false),
                Type = ConstraintType.RotationMotor,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = target
            };
        }

        /// <summary>
        /// Constrains angular rotation about one axis and drives to a specific target.
        /// </summary>
        /// <param name="target">The target angular velocity of the motor, in rad/s.</param>
        /// <param name="springFrequency">(Optional) </param>
        /// <param name="springDamping">(Optional) </param>
        /// <returns>A Constraint. </returns>
        public static Constraint AngularVelocityMotor(float3 target, float springFrequency = DefaultSpringFrequency, float springDamping = DefaultSpringDamping)
        {
            return AngularVelocityMotor(target, new float3(DefaultMaxImpulse), springFrequency, springDamping);
        }

        /// <summary>
        /// Constrains angular rotation about one axis and drives to a specific target.
        /// </summary>
        /// <param name="target">The target angular velocity of the motor, in rad/s.</param>
        /// <param name="maxImpulse">The max impulse needed to receive an impulse event for this constraint.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        /// <returns>A Constraint. </returns>
        public static Constraint AngularVelocityMotor(float3 target, float3 maxImpulse, float springFrequency = DefaultSpringFrequency,
            float springDamping = DefaultSpringDamping)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true, false, false),
                Type = ConstraintType.AngularVelocityMotor,
                Min = 0.0f,
                Max = 0.0f,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping,
                MaxImpulse = maxImpulse,
                EnableImpulseEvents = false,
                Target = target
            };
        }

        #endregion

        /// <summary>Tests if this Constraint is considered equal to another. </summary>
        ///
        /// <param name="other">The constraint to compare to this object. </param>
        ///
        /// <returns>True if the objects are considered equal, false if they are not. </returns>
        public bool Equals(Constraint other)
        {
            return ConstrainedAxes.Equals(other.ConstrainedAxes)
                && Type == other.Type
                && Min.Equals(other.Min)
                && Max.Equals(other.Max)
                && SpringFrequency.Equals(other.SpringFrequency)
                && SpringDamping.Equals(other.SpringDamping)
                && MaxImpulse.Equals(other.MaxImpulse)
                && EnableImpulseEvents.Equals(other.EnableImpulseEvents)
                && Target.Equals(other.Target);
        }

        /// <summary>Tests if this object is considered equal to another. </summary>
        ///
        /// <param name="obj">The object to compare to this object. </param>
        ///
        /// <returns>True if the objects are considered equal, false if they are not. </returns>
        public override bool Equals(object obj) =>obj is Constraint other && Equals(other);

        /// <summary>Calculates a hash code for this object. </summary>
        ///
        /// <returns>A hash code for this object. </returns>
        public override int GetHashCode()
        {
            unchecked
            {
                return (int)math.hash(new uint3x3(
                    new uint3((uint)Type, (uint)Min.GetHashCode(), (uint)Max.GetHashCode()),
                    new uint3(math.hash(ConstrainedAxes), (uint)SpringFrequency.GetHashCode(), (uint)SpringDamping.GetHashCode()),
                    new uint3(math.hash(MaxImpulse), (uint)EnableImpulseEvents.GetHashCode(), (uint)Target.GetHashCode())
                ));
            }
        }
    }

    /// <summary>A runtime joint instance, attached to specific rigid bodies. </summary>
    public struct Joint
    {
        /// <summary>The body pair. </summary>
        public BodyIndexPair BodyPair;
        /// <summary>Joint in the space of the body A. </summary>
        public MTransform AFromJoint;
        /// <summary>Joint in the space of the body B. </summary>
        public MTransform BFromJoint;
        /// <summary>Constraint block. Note that Constraints needs to be 4-byte aligned for Android 32. </summary>
        public ConstraintBlock3 Constraints;
        /// <summary>If non-zero, allows these bodies to collide. </summary>
        public byte EnableCollision;
        /// <summary>The version. </summary>
        public byte Version;

        /// <summary>
        /// The entity that contained the component which created this joint Note, this isn't necessarily
        /// an entity with a rigid body, as a pair of bodies can have an arbitrary number of constraints,
        /// but only one instance of a particular component type per entity.
        /// </summary>
        public Entity Entity;
    }
}
