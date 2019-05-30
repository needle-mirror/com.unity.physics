using System;
using System.Linq;
using System.Text;
using NUnit.Framework.Constraints;
using Unity.Mathematics;
using Assert = UnityEngine.Assertions.Assert;
using Unity.Entities;
using Unity.Collections;
using Unity.Jobs;
using Random = Unity.Mathematics.Random;

namespace Unity.Physics.Tests
{
    static class TestExtensions
    {
        public static string ToReadableString(this EntityQueryDesc query)
        {
            var sb = new StringBuilder();

            if (query.All.Any())
                sb.Append($"with [{string.Join(", ", query.All)}]");

            if (query.Any.Any())
            {
                if (sb.Length > 0) sb.Append(", ");
                sb.Append($"with any of [{string.Join(", ", query.Any)}]");
            }
            if (query.None.Any())
            {
                if (sb.Length > 0) sb.Append(", ");
                sb.Append($"without [{string.Join(", ", query.None)}]");
            }

            if (query.Options != EntityQueryOptions.Default)
                sb.Append($" ({string.Join(", ", Enum.GetValues(typeof(EntityQueryOptions)).Cast<EntityQueryOptions>().Where(v => (query.Options & v) == v))})");

            return sb.ToString();
        }

        static MatrixPrettyCloseConstraint PrettyClose(
            this ConstraintExpression expression,
            float4x4 expected, float tolerance = MatrixPrettyCloseConstraint.DefaultTolerance
        )
        {
            var constraint = new MatrixPrettyCloseConstraint(expected, tolerance);
            expression.Append(constraint);
            return constraint;
        }
    }

    class Is : NUnit.Framework.Is
    {
        public static MatrixPrettyCloseConstraint PrettyCloseTo(
            float4x4 actual, float tolerance = MatrixPrettyCloseConstraint.DefaultTolerance
        )
        {
            return new MatrixPrettyCloseConstraint(actual, tolerance);
        }
    }

    class MatrixPrettyCloseConstraint : NUnit.Framework.Constraints.Constraint
    {
        public const float DefaultTolerance = 0.0001f;

        readonly float4x4 m_Expected;
        readonly float m_ToleranceSq;

        public MatrixPrettyCloseConstraint(float4x4 expected, float tolerance = DefaultTolerance)
            : base((object)expected, (object)tolerance)
        {
            m_Expected = expected;
            m_ToleranceSq = tolerance * tolerance;
        }

        public override string Description => $"each column to be within {math.sqrt(m_ToleranceSq)} of {m_Expected}";

        public override ConstraintResult ApplyTo(object actual)
        {
            var m = (float4x4)actual;
            var cmp =
                math.lengthsq(m.c0 - m_Expected.c0) < m_ToleranceSq
                && math.lengthsq(m.c1 - m_Expected.c1) < m_ToleranceSq
                && math.lengthsq(m.c2 - m_Expected.c2) < m_ToleranceSq
                && math.lengthsq(m.c3 - m_Expected.c3) < m_ToleranceSq;
            return new ConstraintResult(this, actual, cmp);
        }
    }
}

namespace Unity.Physics.Tests.Utils
{
    class TestUtils
    {
        public static void AreEqual(bool a, bool b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(int a, int b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(uint a, uint b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(long a, long b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(ulong a, ulong b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(float a, float b, float delta = 0.0f)
        {
            Assert.AreApproximatelyEqual(a, b, delta);
        }

        public static void AreEqual(float a, float b, int maxUlp, bool signedZeroEqual)
        {
            if (signedZeroEqual && a == b)
                return;

            if (math.isfinite(a) && math.isfinite(b))
            {
                int ia = math.asint(a);
                int ib = math.asint(b);
                if ((ia ^ ib) < 0)
                    Assert.AreEqual(true, false);
                int ulps = math.abs(ia - ib);
                Assert.AreEqual(true, ulps <= maxUlp);
            }
            else
            {
                if (a != b && (!math.isnan(a) || !math.isnan(b)))
                    Assert.AreEqual(true, false);
            }
        }

        public static void AreEqual(double a, double b, double delta = 0.0)
        {
            Assert.IsTrue(math.abs(a - b) < delta);
        }

        public static void AreEqual(double a, double b, int maxUlp, bool signedZeroEqual)
        {
            if (signedZeroEqual && a == b)
                return;

            if (math.isfinite(a) && math.isfinite(b))
            {
                long la = math.aslong(a);
                long lb = math.aslong(b);
                if ((la ^ lb) < 0)
                    Assert.AreEqual(true, false);
                long ulps = la > lb ? la - lb : lb - la;
                Assert.AreEqual(true, ulps <= maxUlp);
            }
            else
            {
                if (a != b && (!math.isnan(a) || !math.isnan(b)))
                    Assert.AreEqual(true, false);
            }
        }


        // bool
        public static void AreEqual(bool2 a, bool2 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
        }

        public static void AreEqual(bool3 a, bool3 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
        }

        public static void AreEqual(bool4 a, bool4 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
            AreEqual(a.w, b.w);
        }


        public static void AreEqual(bool2x2 a, bool2x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(bool3x2 a, bool3x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(bool4x2 a, bool4x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }


        public static void AreEqual(bool2x3 a, bool2x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(bool3x3 a, bool3x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(bool4x3 a, bool4x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }


        public static void AreEqual(bool2x4 a, bool2x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(bool3x4 a, bool3x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(bool4x4 a, bool4x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        // int
        public static void AreEqual(int2 a, int2 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
        }

        public static void AreEqual(int3 a, int3 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
        }

        public static void AreEqual(int4 a, int4 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
            AreEqual(a.w, b.w);
        }


        public static void AreEqual(int2x2 a, int2x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(int3x2 a, int3x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(int4x2 a, int4x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }


        public static void AreEqual(int2x3 a, int2x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(int3x3 a, int3x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(int4x3 a, int4x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }



        public static void AreEqual(int2x4 a, int2x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(int3x4 a, int3x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(int4x4 a, int4x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }


        // uint
        public static void AreEqual(uint2 a, uint2 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
        }

        public static void AreEqual(uint3 a, uint3 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
        }

        public static void AreEqual(uint4 a, uint4 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
            AreEqual(a.w, b.w);
        }


        public static void AreEqual(uint2x2 a, uint2x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(uint3x2 a, uint3x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(uint4x2 a, uint4x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }


        public static void AreEqual(uint2x3 a, uint2x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(uint3x3 a, uint3x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(uint4x3 a, uint4x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }


        public static void AreEqual(uint2x4 a, uint2x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(uint3x4 a, uint3x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(uint4x4 a, uint4x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        // float
        public static void AreEqual(float2 a, float2 b, float delta = 0.0f)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
        }

        public static void AreEqual(float2 a, float2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float3 a, float3 b, float delta = 0.0f)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
        }

        public static void AreEqual(float3 a, float3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float4 a, float4 b, float delta = 0.0f)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
            AreEqual(a.w, b.w, delta);
        }

        public static void AreEqual(float4 a, float4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
            AreEqual(a.w, b.w, maxUlp, signedZeroEqual);
        }


        public static void AreEqual(float2x2 a, float2x2 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(float2x2 a, float2x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float3x2 a, float3x2 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(float3x2 a, float3x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float4x2 a, float4x2 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(float4x2 a, float4x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }


        public static void AreEqual(float2x3 a, float2x3 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(float2x3 a, float2x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float3x3 a, float3x3 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(float3x3 a, float3x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float4x3 a, float4x3 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(float4x3 a, float4x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }


        public static void AreEqual(float2x4 a, float2x4 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(float2x4 a, float2x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float3x4 a, float3x4 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(float3x4 a, float3x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(float4x4 a, float4x4 b, float delta = 0.0f)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(float4x4 a, float4x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        // double
        public static void AreEqual(double2 a, double2 b, double delta = 0.0)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
        }

        public static void AreEqual(double2 a, double2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3 a, double3 b, double delta = 0.0)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
        }

        public static void AreEqual(double3 a, double3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4 a, double4 b, double delta = 0.0)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
            AreEqual(a.w, b.w, delta);
        }

        public static void AreEqual(double4 a, double4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
            AreEqual(a.w, b.w, maxUlp, signedZeroEqual);
        }


        public static void AreEqual(double2x2 a, double2x2 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(double2x2 a, double2x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3x2 a, double3x2 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(double3x2 a, double3x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4x2 a, double4x2 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(double4x2 a, double4x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double2x3 a, double2x3 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(double2x3 a, double2x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3x3 a, double3x3 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(double3x3 a, double3x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4x3 a, double4x3 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(double4x3 a, double4x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }


        public static void AreEqual(double2x4 a, double2x4 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(double2x4 a, double2x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3x4 a, double3x4 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(double3x4 a, double3x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4x4 a, double4x4 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(double4x4 a, double4x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(quaternion a, quaternion b, float delta = 0.0f)
        {
            AreEqual(a.value, b.value, delta);
        }

        public static void AreEqual(RigidTransform a, RigidTransform b, float delta = 0.0f)
        {
            AreEqual(a.rot, b.rot, delta);
            AreEqual(a.pos, b.pos, delta);
        }

        public delegate void FuncDelegate();
        public static void ThrowsException<T>(FuncDelegate func) where T : System.Exception
        {
            try
            {
                func();
            }
            catch (T)
            {
                return;
            }
            Assert.IsTrue(false, "Expected exception to be thrown");
        }

        //
        // Random generation
        //

        public static BlobAssetReference<Collider> GenerateRandomMesh(ref Random rnd)
        {
            int numTriangles = rnd.NextInt(1, 250);
            float3[] vertices = new float3[numTriangles * 3];
            int[] indices = new int[numTriangles * 3];
            int nextIndex = 0;

            while (numTriangles > 0)
            {
                int featureTriangles = math.min(rnd.NextInt(1, 100), numTriangles);

                int featureType = rnd.NextInt(0, 3);
                switch (featureType)
                {
                    case 0:
                    {
                        // Soup
                        for (int i = 0; i < featureTriangles; i++)
                        {
                            float size = rnd.NextFloat(0.1f, 2.5f);
                            size *= size;

                            float3 center = rnd.NextFloat3(-10.0f, 10.0f);
                            for (int j = 0; j < 3; j++)
                            {
                                vertices[nextIndex++] = center + rnd.NextFloat3(-size, size);
                            }
                        }
                        break;
                    }

                    case 1:
                    {
                        // Fan
                        float3 center = rnd.NextFloat3(-10.0f, 10.0f);
                        float3 arm = rnd.NextFloat3Direction() * rnd.NextFloat(0.1f, 1.0f);
                        float3 axis;
                        {
                            float3 unused;
                            Math.CalculatePerpendicularNormalized(math.normalize(arm), out axis, out unused);
                        }
                        float arc = rnd.NextFloat(0.1f, 2.0f * (float)math.PI);
                        arc = math.min(arc, featureTriangles * (float)math.PI / 2.0f); // avoid degenerate triangles
                        featureTriangles = math.min(featureTriangles, (int)(arc / 0.025f)); // avoid degenerate triangles
                        quaternion q = Unity.Mathematics.quaternion.AxisAngle(axis, arc / numTriangles);
                        for (int i = 0; i < featureTriangles; i++)
                        {
                            vertices[nextIndex++] = center;
                            vertices[nextIndex++] = center + arm;
                            arm = math.mul(q, arm);
                            vertices[nextIndex++] = center + arm;
                        }
                        break;
                    }

                    case 2:
                    {
                        // Strip
                        float3 v0 = rnd.NextFloat3(-10.0f, 10.0f);
                        float3 v1 = v0 + rnd.NextFloat3(-0.5f, 0.5f);
                        float3 dir;
                        {
                            float3 unused;
                            Math.CalculatePerpendicularNormalized(math.normalize(v1 - v0), out dir, out unused);
                        }
                        for (int i = 0; i < featureTriangles; i++)
                        {
                            float3 v2 = v0 + rnd.NextFloat(0.25f, 0.5f) * dir;
                            dir = math.mul(Unity.Mathematics.quaternion.AxisAngle(rnd.NextFloat3Direction(), rnd.NextFloat(0.0f, 0.3f)), dir);

                            vertices[nextIndex++] = v0;
                            vertices[nextIndex++] = v1;
                            vertices[nextIndex++] = v2;

                            v0 = v1;
                            v1 = v2;
                        }
                        break;
                    }

                    case 3:
                    {
                        // Grid
                        int quads = featureTriangles / 2;
                        if (quads == 0)
                        {
                            featureTriangles = 0; // Too small, try again for a different feature
                            break;
                        }

                        int rows = rnd.NextInt(1, (int)math.sqrt(quads));
                        int cols = quads / rows;
                        quads = rows * cols;
                        featureTriangles = quads * 2;

                        float3 origin = rnd.NextFloat3(-10.0f, 10.0f);
                        float3 x = rnd.NextFloat3(-0.5f, 0.5f);
                        float3 y = rnd.NextFloat3(-0.5f, 0.5f);
                        for (int i = 0; i < rows; i++)
                        {
                            for (int j = 0; j < cols; j++)
                            {
                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 0);
                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 1);
                                vertices[nextIndex++] = origin + x * (i + 1) + y * (j + 1);

                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 0);
                                vertices[nextIndex++] = origin + x * (i + 1) + y * (j + 1);
                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 1);
                            }
                        }

                        break;
                    }
                }

                numTriangles -= featureTriangles;
            }

            for (int i = 0; i < indices.Length; i++)
            {
                indices[i] = i;
            }

            return MeshCollider.Create(vertices, indices);
        }

        public static unsafe BlobAssetReference<Collider> GenerateRandomCompound(ref Random rnd)
        {
            int numChildren = rnd.NextInt(1, 10);
            var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(numChildren, Allocator.Temp);
            for (int i = 0; i < numChildren; i++)
            {
                children[i] = new CompoundCollider.ColliderBlobInstance
                {
                    CompoundFromChild = new RigidTransform
                    {
                        pos = (rnd.NextInt(10) > 0) ? rnd.NextFloat3(-5.0f, 5.0f) : float3.zero,
                        rot = (rnd.NextInt(10) > 0) ? rnd.NextQuaternionRotation() : quaternion.identity
                    },
                    Collider = GenerateRandomCollider(ref rnd)
                };
            }

            return CompoundCollider.Create(children);
        }

        public static unsafe BlobAssetReference<Collider> GenerateRandomConvex(ref Random rnd)
        {
            ColliderType colliderType = (ColliderType)rnd.NextInt((int)ColliderType.Cylinder + 1);
            float convexRadius = (rnd.NextInt(4) > 0) ? rnd.NextFloat(0.5f) : 0.0f;
            switch (colliderType)
            {
                case ColliderType.Convex:
                {
                    int numPoints = rnd.NextInt(1, 16);
                    if (numPoints == 3) // TODO - hull builder doesn't build faces for flat shapes, work around it for now to run the test
                    {
                        numPoints++;
                    }
                    var points = new NativeArray<float3>(numPoints, Allocator.Temp);
                    for (int i = 0; i < numPoints; i++)
                    {
                        points[i] = rnd.NextFloat3(-1.0f, 1.0f);
                    }
                    var collider = ConvexCollider.Create(points, convexRadius);
                    points.Dispose();
                    return collider;
                }

                case ColliderType.Sphere:
                {
                    float3 center = (rnd.NextInt(4) > 0) ? float3.zero : rnd.NextFloat3(-0.5f, 0.5f);
                    return SphereCollider.Create(center, rnd.NextFloat(0.01f, 0.5f));
                }

                case ColliderType.Capsule:
                {
                    float3 point0 = rnd.NextFloat3(0.0f, 1.0f);
                    float3 point1 = (rnd.NextInt(4) > 0) ? -point0 : rnd.NextFloat3(-1.0f, 1.0f);
                    return CapsuleCollider.Create(point0, point1, rnd.NextFloat(0.01f, 0.5f));
                }

                case ColliderType.Triangle:
                {
                    return PolygonCollider.CreateTriangle(rnd.NextFloat3(-1.0f, 1.0f), rnd.NextFloat3(-1.0f, 1.0f), rnd.NextFloat3(-1.0f, 1.0f));
                }

                case ColliderType.Quad:
                {
                    // Pick 3 totally random points, then choose a fourth that makes a flat and convex quad
                    float3 point0 = rnd.NextFloat3(-1.0f, 1.0f);
                    float3 point1 = rnd.NextFloat3(-1.0f, 1.0f);
                    float3 point3 = rnd.NextFloat3(-1.0f, 1.0f);
                    float t0 = rnd.NextFloat(0.0f, 1.0f);
                    float t1 = rnd.NextFloat(0.0f, 1.0f);
                    float3 e = point1 + point1 - point0;
                    float3 a = math.lerp(point1, e, t0);
                    float3 b = math.lerp(point3, point3 + point3 - point0, t0);
                    float3 point2 = math.lerp(a, b, t1);

                    return PolygonCollider.CreateQuad(point0, point1, point2, point3);
                }

                case ColliderType.Box:
                {
                    float3 center = (rnd.NextInt(4) > 0) ? Unity.Mathematics.float3.zero : rnd.NextFloat3(-0.5f, 0.5f);
                    quaternion orientation = (rnd.NextInt(4) > 0) ? Unity.Mathematics.quaternion.identity : rnd.NextQuaternionRotation();
                    float minSize = 0.05f; // TODO - work around hull builder problems with small faces, sometimes doesn't extend 1D->2D based on face area
                    float3 size = rnd.NextFloat3(minSize, 1.0f);
                    float maxConvexRadius = math.max(math.cmin((size - minSize) / (2.0f * (1.0f + float.Epsilon))), 0.0f);
                    return BoxCollider.Create(center, orientation, size, math.min(maxConvexRadius, convexRadius));
                }

                case ColliderType.Cylinder:
                {
                    float3 center = (rnd.NextInt(4) > 0) ? Unity.Mathematics.float3.zero : rnd.NextFloat3(-0.5f, 0.5f);
                    quaternion orientation = (rnd.NextInt(4) > 0) ? Unity.Mathematics.quaternion.identity : rnd.NextQuaternionRotation();
                    float minSize = 0.01f; // TODO - cylinder gets degenerate faces if radius-convexRadius=0 or height/2-convexRadius=0, decide how to handle this in CylinderCollider
                    float height = rnd.NextFloat(2.0f * minSize, 1f);
                    float cylinderRadius = rnd.NextFloat(minSize, 1.0f);
                    float maxConvexRadius = math.max(math.min(height / 2, cylinderRadius) - minSize, 0.0f);
                    return CylinderCollider.Create(center, height, cylinderRadius, orientation, math.min(maxConvexRadius, convexRadius));
                }

                default: throw new System.NotImplementedException();
            }
        }

        public static BlobAssetReference<Collider> GenerateRandomCollider(ref Random rnd)
        {
            if (rnd.NextInt(10) > 0)
            {
                return GenerateRandomConvex(ref rnd);
            }
            else if (rnd.NextInt(4) > 0)
            {
                return GenerateRandomMesh(ref rnd);
            }
            return GenerateRandomCompound(ref rnd);
        }

        public static unsafe PhysicsWorld GenerateRandomWorld(ref Random rnd, int numBodies, float size)
        {
            // Create the world
            PhysicsWorld world = new PhysicsWorld(numBodies, 0, 0);

            // Create bodies
            NativeSlice<RigidBody> bodies = world.StaticBodies;
            for (int i = 0; i < numBodies; i++)
            {
                bodies[i] = new RigidBody
                {
                    WorldFromBody = new RigidTransform
                    {
                        pos = rnd.NextFloat3(-size, size),
                        rot = (rnd.NextInt(10) > 0) ? rnd.NextQuaternionRotation() : quaternion.identity
                    },
                    Collider = (Collider*)GenerateRandomCollider(ref rnd).GetUnsafePtr(),   // Not safe, could be garbage collected
                    Entity = Entity.Null,
                    CustomData = 0
                };
            }

            StaticLayerChangeInfo staticLayerChangeInfo = new StaticLayerChangeInfo();
            staticLayerChangeInfo.Init(Allocator.TempJob);
            staticLayerChangeInfo.NumStaticBodies = numBodies;
            staticLayerChangeInfo.HaveStaticBodiesChanged = 1;

            // Build the broadphase
            world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, timeStep: 1.0f, numThreadsHint: 1, ref staticLayerChangeInfo, inputDeps: new JobHandle()).Complete();
            staticLayerChangeInfo.Deallocate();

            return world;
        }
    }
}
