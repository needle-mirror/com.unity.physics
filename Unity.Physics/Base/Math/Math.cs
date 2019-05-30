using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace Unity.Physics
{
    // Common math helper functions
    [DebuggerStepThrough]
    public static partial class Math
    {
        // Constants
        [DebuggerStepThrough]
        public static class Constants
        {
            public static float4 One4F => new float4(1);
            public static float4 Min4F => new float4(float.MinValue);
            public static float4 Max4F => new float4(float.MaxValue);
            public static float3 Min3F => new float3(float.MinValue);
            public static float3 Max3F => new float3(float.MaxValue);

            // Smallest float such that 1.0 + eps != 1.0
            // Different from float.Epsilon which is the smallest value greater than zero.
            public const float Eps = 1.192092896e-07F;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf16(int input) => ((input + 15) >> 4) << 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf16(ulong input) => ((input + 15) >> 4) << 4;
		
        /// Note that alignment must be a power of two for this to work.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf(int input, int alignment) => (input + (alignment - 1)) & (~(alignment - 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf(ulong input, ulong alignment) => (input + (alignment - 1)) & (~(alignment - 1));
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float2 v) => v.x < v.y ? 0 : 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float3 v) => v.x < v.y ? (v.x < v.z ? 0 : 2) : (v.y < v.z ? 1 : 2);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float4 v) { int xyz = IndexOfMinComponent(v.xyz); return v[xyz] < v.w ? xyz : 3; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float2 v) => IndexOfMinComponent(-v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float3 v) => IndexOfMinComponent(-v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float4 v) => IndexOfMinComponent(-v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float HorizontalMul(float3 v) => v.x * v.y * v.z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float HorizontalMul(float4 v) => (v.x * v.y) * (v.z * v.w);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dotxyz1(float4 lhs, float3 rhs) => math.dot(lhs, new float4(rhs, 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Det(float3 a, float3 b, float3 c) => math.dot(math.cross(a, b), c); // TODO: use math.determinant()?

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float RSqrtSafe(float v) => math.select(math.rsqrt(v), 0.0f, math.abs(v) < 1e-10);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float NormalizeWithLength(float3 v, out float3 n)
        {
            float lengthSq = math.lengthsq(v);
            float invLength = math.rsqrt(lengthSq);
            n = v * invLength;
            return lengthSq * invLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsNormalized(float3 v)
        {
            float lenZero = math.lengthsq(v) - 1.0f;
            float absLenZero = math.abs(lenZero);
            return absLenZero < Constants.Eps;
        }

        // Return two normals perpendicular to the input vector
        public static void CalculatePerpendicularNormalized(float3 v, out float3 p, out float3 q)
        {
            float3 vSquared = v * v;
            float3 lengthsSquared = vSquared + vSquared.xxx; // y = ||j x v||^2, z = ||k x v||^2
            float3 invLengths = math.rsqrt(lengthsSquared);

            // select first direction, j x v or k x v, whichever has greater magnitude
            float3 dir0 = new float3(-v.y, v.x, 0.0f);
            float3 dir1 = new float3(-v.z, 0.0f, v.x);
            bool cmp = (lengthsSquared.y > lengthsSquared.z);
            float3 dir = math.select(dir1, dir0, cmp);

            // normalize and get the other direction
            float invLength = math.select(invLengths.z, invLengths.y, cmp);
            p = dir * invLength;
            float3 cross = math.cross(v, dir);
            q = cross * invLength;
        }

        // Calculate the eigenvectors and eigenvalues of a symmetric 3x3 matrix
        public static void DiagonalizeSymmetricApproximation(float3x3 a, out float3x3 eigenVectors, out float3 eigenValues)
        {
            float GetMatrixElement(float3x3 m, int row, int col)
            {
                switch (col)
                {
                    case 0: return m.c0[row];
                    case 1: return m.c1[row];
                    case 2: return m.c2[row];
                    default: UnityEngine.Assertions.Assert.IsTrue(false); return 0.0f;
                }
            }

            void SetMatrixElement(ref float3x3 m, int row, int col, float x)
            {
                switch (col)
                {
                    case 0: m.c0[row] = x; break;
                    case 1: m.c1[row] = x; break;
                    case 2: m.c2[row] = x; break;
                    default: UnityEngine.Assertions.Assert.IsTrue(false); break;
                }
            }

            eigenVectors = float3x3.identity;
            float epsSq = 1e-14f * (math.lengthsq(a.c0) + math.lengthsq(a.c1) + math.lengthsq(a.c2));
            const int maxIterations = 10;
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                // Find the row (p) and column (q) of the off-diagonal entry with greater magnitude
                int p = 0, q = 1;
                {
                    float maxEntry = math.abs(a.c1[0]);
                    float mag02 = math.abs(a.c2[0]);
                    float mag12 = math.abs(a.c2[1]);
                    if (mag02 > maxEntry)
                    {
                        maxEntry = mag02;
                        p = 0;
                        q = 2;
                    }
                    if (mag12 > maxEntry)
                    {
                        maxEntry = mag12;
                        p = 1;
                        q = 2;
                    }

                    // Terminate if it's small enough
                    if (maxEntry * maxEntry < epsSq)
                    {
                        break;
                    }
                }

                // Calculate jacobia rotation
                float3x3 j = float3x3.identity;
                {
                    float apq = GetMatrixElement(a, p, q);
                    float tau = (GetMatrixElement(a, q, q) - GetMatrixElement(a, p, p)) / (2.0f * apq);
                    float t = math.sqrt(1.0f + tau * tau);
                    if (tau > 0.0f)
                    {
                        t = 1.0f / (tau + t);
                    }
                    else
                    {
                        t = 1.0f / (tau - t);
                    }
                    float c = math.rsqrt(1.0f + t * t);
                    float s = t * c;

                    SetMatrixElement(ref j, p, p, c);
                    SetMatrixElement(ref j, q, q, c);
                    SetMatrixElement(ref j, p, q, s);
                    SetMatrixElement(ref j, q, p, -s);
                }

                // Rotate a
                a = math.mul(math.transpose(j), math.mul(a, j));
                eigenVectors = math.mul(eigenVectors, j);
            }
            eigenValues = new float3(a.c0.x, a.c1.y, a.c2.z);
        }

        // Returns the twist angle of the swing-twist decomposition of q about i, j, or k corresponding to index = 0, 1, or 2 respectively.
        public static float CalculateTwistAngle(quaternion q, int twistAxisIndex)
        {
            // q = swing * twist, twist = normalize(twistAxis * twistAxis dot q.xyz, q.w)
            float dot = q.value[twistAxisIndex];
            float w = q.value.w;
            float lengthSq = dot * dot + w * w;
            float invLength = RSqrtSafe(lengthSq);
            float sinHalfAngle = dot * invLength;
            float cosHalfAngle = w * invLength;
            float halfAngle = math.atan2(sinHalfAngle, cosHalfAngle);
            return halfAngle + halfAngle;
        }

        // Returns a quaternion q with q * from = to
        public static quaternion FromToRotation(float3 from, float3 to)
        {
            Assert.IsTrue(math.abs(math.lengthsq(from) - 1.0f) < 1e-4f);
            Assert.IsTrue(math.abs(math.lengthsq(to) - 1.0f) < 1e-4f);
            float3 cross = math.cross(from, to);
            CalculatePerpendicularNormalized(from, out float3 safeAxis, out float3 unused); // for when angle ~= 180
            float dot = math.dot(from, to);
            float3 squares = new float3(0.5f - new float2(dot, -dot) * 0.5f, math.lengthsq(cross));
            float3 inverses = math.select(math.rsqrt(squares), 0.0f, squares < 1e-10f);
            float2 sinCosHalfAngle = squares.xy * inverses.xy;
            float3 axis = math.select(cross * inverses.z, safeAxis, squares.z < 1e-10f);
            return new quaternion(new float4(axis * sinCosHalfAngle.x, sinCosHalfAngle.y));
        }
    }
}
