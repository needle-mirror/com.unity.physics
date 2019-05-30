using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace Unity.Physics
{
    public enum JacobianType : byte
    {
        // Contact Jacobians
        Contact,
        Trigger,

        // Joint Jacobians
        LinearLimit,
        AngularLimit1D,
        AngularLimit2D,
        AngularLimit3D
    }

    // Flags which enable optional Jacobian behaviors
    [Flags]
    public enum JacobianFlags : byte
    {
        // These flags apply to all Jacobians
        Disabled = 1 << 0,
        EnableMassFactors = 1 << 1,
        UserFlag0 = 1 << 2,
        UserFlag1 = 1 << 3,

        // These flags apply only to contact Jacobians
        IsTrigger = 1 << 4,
        EnableCollisionEvents = 1 << 5,
        EnableSurfaceVelocity = 1 << 6,
        EnableMaxImpulse = 1 << 7
    }

    // Jacobian header, first part of each Jacobian in the stream
    public struct JacobianHeader
    {
        public BodyIndexPair BodyPair { get; internal set; }
        public JacobianType Type { get; internal set; }
        public JacobianFlags Flags { get; internal set; }

        // Whether the Jacobian should be solved or not
        public bool Enabled
        {
            get => ((Flags & JacobianFlags.Disabled) == 0);
            set => Flags = value ? (Flags & ~JacobianFlags.Disabled) : (Flags | JacobianFlags.Disabled);
        }

        // Collider keys for the collision events
        public bool HasColliderKeys => (Flags & JacobianFlags.EnableCollisionEvents) != 0;
        public ColliderKeyPair ColliderKeys
        {
            get => HasColliderKeys ? AccessColliderKeys() : ColliderKeyPair.Empty;
            set
            {
                if (HasColliderKeys)
                    AccessColliderKeys() = value;
                else
                    throw new NotSupportedException("Jacobian does not have collision events enabled");
            }
        }

        // Overrides for the mass properties of the pair of bodies
        public bool HasMassFactors => (Flags & JacobianFlags.EnableMassFactors) != 0;
        public MassFactors MassFactors
        {
            get => HasMassFactors ? AccessMassFactors() : MassFactors.Default;
            set
            {
                if (HasMassFactors)
                    AccessMassFactors() = value;
                else
                    throw new NotSupportedException("Jacobian does not have mass factors enabled");
            }
        }

        // The surface velocity to apply to contact points
        public bool HasSurfaceVelocity => (Flags & JacobianFlags.EnableSurfaceVelocity) != 0;
        public SurfaceVelocity SurfaceVelocity
        {
            get => HasSurfaceVelocity ? AccessSurfaceVelocity() : new SurfaceVelocity();
            set
            {
                if (HasSurfaceVelocity)
                    AccessSurfaceVelocity() = value;
                else
                    throw new NotSupportedException("Jacobian does not have surface velocity enabled");
            }
        }

        // The maximum impulse that can be applied by contact points
        public bool HasMaxImpulse => (Flags & JacobianFlags.EnableMaxImpulse) != 0;
        public float MaxImpulse
        {
            get => HasMaxImpulse ? AccessMaxImpulse() : float.MaxValue;
            set
            {
                if (HasMaxImpulse)
                    AccessMaxImpulse() = value;
                else
                    throw new NotSupportedException("Jacobian does not have max impulse enabled");
            }
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput,
            ref BlockStream.Writer collisionEventsWriter, ref BlockStream.Writer triggerEventsWriter)
        {
            if (Enabled)
            {
                switch (Type)
                {
                    case JacobianType.Contact:
                        AccessBaseJacobian<ContactJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter);
                        break;
                    case JacobianType.Trigger:
                        AccessBaseJacobian<TriggerJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref triggerEventsWriter);
                        break;
                    case JacobianType.LinearLimit:
                        AccessBaseJacobian<LinearLimitJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep);
                        break;
                    case JacobianType.AngularLimit1D:
                        AccessBaseJacobian<AngularLimit1DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep);
                        break;
                    case JacobianType.AngularLimit2D:
                        AccessBaseJacobian<AngularLimit2DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep);
                        break;
                    case JacobianType.AngularLimit3D:
                        AccessBaseJacobian<AngularLimit3DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep);
                        break;
                    default:
                        throw new NotImplementedException();
                }
            }
        }

        #region Helpers

        public static int CalculateSize(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return UnsafeUtility.SizeOf<JacobianHeader>() +
                SizeOfBaseJacobian(type) + SizeOfModifierData(type, flags) +
                numContactPoints * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>();
        }

        private static int SizeOfColliderKeys(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<ColliderKeyPair>() : 0;
        }

        private static int SizeOfSurfaceVelocity(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableSurfaceVelocity) != 0) ?
                UnsafeUtility.SizeOf<SurfaceVelocity>() : 0;
        }

        private static int SizeOfMaxImpulse(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableMaxImpulse) != 0) ?
                UnsafeUtility.SizeOf<float>() : 0;
        }

        private static int SizeOfMassFactors(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableMassFactors) != 0) ?
                UnsafeUtility.SizeOf<MassFactors>() : 0;
        }

        private static int SizeOfModifierData(JacobianType type, JacobianFlags flags)
        {
            return SizeOfColliderKeys(type, flags) + SizeOfSurfaceVelocity(type, flags) +
                SizeOfMaxImpulse(type, flags) + SizeOfMassFactors(type, flags);
        }

        private static int SizeOfBaseJacobian(JacobianType type)
        {
            switch (type)
            {
                case JacobianType.Contact:
                    return UnsafeUtility.SizeOf<ContactJacobian>();
                case JacobianType.Trigger:
                    return UnsafeUtility.SizeOf<TriggerJacobian>();
                case JacobianType.LinearLimit:
                    return UnsafeUtility.SizeOf<LinearLimitJacobian>();
                case JacobianType.AngularLimit1D:
                    return UnsafeUtility.SizeOf<AngularLimit1DJacobian>();
                case JacobianType.AngularLimit2D:
                    return UnsafeUtility.SizeOf<AngularLimit2DJacobian>();
                case JacobianType.AngularLimit3D:
                    return UnsafeUtility.SizeOf<AngularLimit3DJacobian>();
                default:
                    throw new NotImplementedException();
            }
        }

        // Access to "base" jacobian - a jacobian that comes after the header
        public unsafe ref T AccessBaseJacobian<T>() where T : struct
        {
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>();
            return ref UnsafeUtilityEx.AsRef<T>(ptr);
        }

        public unsafe ref ColliderKeyPair AccessColliderKeys()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type);
            return ref UnsafeUtilityEx.AsRef<ColliderKeyPair>(ptr);
        }

        public unsafe ref SurfaceVelocity AccessSurfaceVelocity()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableSurfaceVelocity) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags);
            return ref UnsafeUtilityEx.AsRef<SurfaceVelocity>(ptr);
        }

        public unsafe ref float AccessMaxImpulse()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableMaxImpulse) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfSurfaceVelocity(Type, Flags);
            return ref UnsafeUtilityEx.AsRef<float>(ptr);
        }

        public unsafe ref MassFactors AccessMassFactors()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableMassFactors) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfSurfaceVelocity(Type, Flags) + SizeOfMaxImpulse(Type, Flags);
            return ref UnsafeUtilityEx.AsRef<MassFactors>(ptr);
        }

        public unsafe ref ContactJacAngAndVelToReachCp AccessAngularJacobian(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact || Type == JacobianType.Trigger);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                pointIndex * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>();
            return ref UnsafeUtilityEx.AsRef<ContactJacAngAndVelToReachCp>(ptr);
        }

        #endregion
    }

    // Helper functions for working with Jacobians
    public static class JacobianUtilities
    {
        public static void CalculateTauAndDamping(float springFrequency, float springDampingRatio, float timestep, int iterations, out float tau, out float damping)
        {
            // TODO
            // - it's a significant amount of work to calculate tau and damping.  They depend on step length, so they have to be calculated each step.
            //   probably worth caching tau and damping for the default spring constants on the world and branching.
            // - you always get a higher effective damping ratio than you ask for because of the error from to discrete integration. The error varies
            //   with step length.  Can we estimate or bound that error and compensate for it?

            /*
            
            How to derive these formulas for tau and damping:

            1) implicit euler integration of a damped spring

               damped spring equation: x'' = -kx - cx'
               h = step length

               x2 = x1 + hv2
               v2 = v1 + h(-kx2 - cv2)/m
                  = v1 + h(-kx1 - hkv2 - cv2)/m
                  = v1 / (1 + h^2k/m + hc/m) - hkx1 / (m + h^2k + hc)

            2) gauss-seidel iterations of a stiff constraint.  Example for four iterations:

               t = tau, d = damping, a = 1 - d
               v2 = av1 - (t / h)x1
               v3 = av2 - (t / h)x1
               v4 = av3 - (t / h)x1
               v5 = av4 - (t / h)x1
                  = a^4v1 - (a^3 + a^2 + a + 1)(t / h)x1

            3) by matching coefficients of v1 and x1 in the formulas for v2 in step (1) and v5 in step (2), we see that if:

               (1 - damping)^4 = 1 / (1 + h^2k / m + hc / m)
               ((1 - damping)^3 + (1 - damping)^2 + (1 - damping) + 1)(tau / h) = hk / (m + h^2k + hc)

               then our constraint is equivalent to the implicit euler integration of a spring.
               solve the first equation for damping, then solve the second equation for tau.
               then substitute in k = mw^2, c = 2mzw.

            */

            float h = timestep;
            float w = springFrequency * 2.0f * (float)math.PI; // convert oscillations/sec to radians/sec
            float z = springDampingRatio;
            float hw = h * w;
            float hhww = hw * hw;

            // a = 1-d, aExp = a^iterations, aSum = aExp / sum(i in [0, iterations), a^i)
            float aExp = 1.0f / (1.0f + hhww + 2.0f * hw * z);
            float a, aSum;
            if (iterations == 4)
            {
                // special case expected iterations = 4
                float invA2 = math.rsqrt(aExp);
                float a2 = invA2 * aExp;
                a = math.rsqrt(invA2);
                aSum = (1.0f + a2 + a * (1.0f + a2));
            }
            else
            {
                a = math.pow(aExp, 1.0f / iterations);
                aSum = 1.0f;
                for (int i = 1; i < iterations; i++)
                {
                    aSum = a * aSum + 1.0f;
                }
            }

            damping = 1 - a;
            tau = hhww * aExp / aSum;
        }

        public static void CalculateTauAndDamping(Constraint constraint, float timestep, int iterations, out float tau, out float damping)
        {
            CalculateTauAndDamping(constraint.SpringFrequency, constraint.SpringDamping, timestep, iterations, out tau, out damping);
        }

        // Returns x - clamp(x, min, max)
        public static float CalculateError(float x, float min, float max)
        {
            float error = math.max(x - max, 0.0f);
            error = math.min(x - min, error);
            return error;
        }

        // Returns the amount of error for the solver to correct, where initialError is the pre-integration error and predictedError is the expected post-integration error
        public static float CalculateCorrection(float predictedError, float initialError, float tau, float damping)
        {
            return math.max(predictedError - initialError, 0.0f) * damping + math.min(predictedError, initialError) * tau;
        }

        // Integrate the relative orientation of a pair of bodies, faster and less memory than storing both bodies' orientations and integrating them separately
        public static quaternion IntegrateOrientationBFromA(quaternion bFromA, float3 angularVelocityA, float3 angularVelocityB, float timestep)
        {
            quaternion dqA = Integrator.IntegrateAngularVelocity(angularVelocityA, timestep);
            quaternion dqB = Integrator.IntegrateAngularVelocity(angularVelocityB, timestep);
            return math.normalize(math.mul(math.mul(math.inverse(dqB), bFromA), dqA));
        }

        // Calculate the inverse effective mass of a linear jacobian
        public static float CalculateInvEffectiveMassDiag(
            float3 angA, float4 invInertiaAndMassA,
            float3 angB, float4 invInertiaAndMassB)
        {
            float3 angularPart = angA * angA * invInertiaAndMassA.xyz + angB * angB * invInertiaAndMassB.xyz;
            float linearPart = invInertiaAndMassA.w + invInertiaAndMassB.w;
            return (angularPart.x + angularPart.y) + (angularPart.z + linearPart);
        }

        // Calculate the inverse effective mass for a pair of jacobians with perpendicular linear parts
        public static float CalculateInvEffectiveMassOffDiag(
            float3 angA0, float3 angA1, float3 invInertiaA,
            float3 angB0, float3 angB1, float3 invInertiaB)
        {
            return math.csum(angA0 * angA1 * invInertiaA + angB0 * angB1 * invInertiaB);
        }

        // Inverts a symmetrix 3x3 matrix with diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static bool InvertSymmetricMatrix(float3 diag, float3 offDiag, out float3 invDiag, out float3 invOffDiag)
        {
            float3 offDiagSq = offDiag.zyx * offDiag.zyx;
            float determinant = (Math.HorizontalMul(diag) + 2.0f * Math.HorizontalMul(offDiag) - math.csum(offDiagSq * diag));
            bool determinantOk = (determinant != 0);
            float invDeterminant = math.select(0.0f, 1.0f / determinant, determinantOk);
            invDiag = (diag.yxx * diag.zzy - offDiagSq) * invDeterminant;
            invOffDiag = (offDiag.yxx * offDiag.zzy - diag.zyx * offDiag) * invDeterminant;
            return determinantOk;
        }

        // Builds a symmetric 3x3 matrix from diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static float3x3 BuildSymmetricMatrix(float3 diag, float3 offDiag)
        {
            return new float3x3(
                new float3(diag.x, offDiag.x, offDiag.y),
                new float3(offDiag.x, diag.y, offDiag.z),
                new float3(offDiag.y, offDiag.z, diag.z)
            );
        }
    }

    // Iterator (and modifier) for jacobians
    public unsafe struct JacobianIterator
    {
        BlockStream.Reader m_Reader;
        int m_CurrentWorkItem;
        readonly bool m_IterateAll;
        readonly int m_MaxWorkItemIndex;

        public JacobianIterator(BlockStream.Reader jacobianStreamReader, int workItemIndex, bool iterateAll = false)
        {
            m_Reader = jacobianStreamReader;
            m_IterateAll = iterateAll;
            m_MaxWorkItemIndex = workItemIndex;

            if (iterateAll)
            {
                m_CurrentWorkItem = 0;
                MoveReaderToNextForEachIndex();
            }
            else
            {
                m_CurrentWorkItem = workItemIndex;
                m_Reader.BeginForEachIndex(workItemIndex);
            }
        }

        public bool HasJacobiansLeft()
        {
            return m_Reader.RemainingItemCount > 0;
        }

        public ref JacobianHeader ReadJacobianHeader(out short readSize)
        {
            readSize = Read<short>();
            return ref UnsafeUtilityEx.AsRef<JacobianHeader>(Read(readSize));
        }

        public ref JacobianHeader ReadJacobianHeader()
        {
            short readSize = Read<short>();
            return ref UnsafeUtilityEx.AsRef<JacobianHeader>(Read(readSize));
        }

        private void MoveReaderToNextForEachIndex()
        {
            while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_MaxWorkItemIndex)
            {
                m_Reader.BeginForEachIndex(m_CurrentWorkItem);
                m_CurrentWorkItem++;
            }
        }

        private byte* Read(int size)
        {
            byte* dataPtr = m_Reader.Read(size);

            if (m_IterateAll)
            {
                MoveReaderToNextForEachIndex();
            }

            return dataPtr;
        }

        private ref T Read<T>() where T : struct
        {
            int size = UnsafeUtility.SizeOf<T>();
            return ref UnsafeUtilityEx.AsRef<T>(Read(size));
        }
    }
}
