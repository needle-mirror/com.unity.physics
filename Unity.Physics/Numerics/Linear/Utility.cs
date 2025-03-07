using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Numerics.Linear
{
    internal enum Side
    {
        Left, Right
    }

    internal enum Op
    {
        None, Transpose
    }

    internal enum TriangularType
    {
        Upper, Lower
    }

    internal enum DiagonalType
    {
        Explicit,
        Unit
    }

    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal struct Utility
    {
        // Returns sqrt(x^2+y^2), taking care to avoid unnecessary under- or overflow
        public static float Magnitude(float2 p)
        {
            if (math.isnan(p.x))
                return p.x;
            if (math.isnan(p.y))
                return p.y;

            p = math.abs(p);
            float w = math.max(p.x, p.y);
            float z = math.min(p.x, p.y);
            if (z == 0)
                return w;

            var zw = z / w;
            return w * math.sqrt(1.0f + zw * zw);
        }

        // Returns sqrt(x^2+y^2+z^2), taking care to avoid unnecessary under- or overflow
        public static float Magnitude(float3 p)
        {
            p = math.abs(p);

            float w = math.max(p.x, math.max(p.y, p.z));
            if (w == 0)
            {
                // w can be zero if one of the components is NaN, so we add the
                // components together to make sure NaNs don't disappear
                return p.x + p.y + p.z;
            }

            p /= w;
            p *= p;
            return w * math.sqrt(p.x + p.y + p.z);
        }

        // returns x with the sign of y
        public static float CopySign(float x, float y)
        {
            return (y >= 0) ? math.abs(x) : -math.abs(x);
        }

        public static float SafeMin { get; } = CalculateSafeMin();

        private static float CalculateSafeMin()
        {
            float one = 1.0f;
            float eps = math.EPSILON;
            float tiny = math.FLT_MIN_NORMAL;
            float huge = float.MaxValue;
            float small = one / huge;

            var sfmin = tiny;
            if (small >= tiny)
            {
                sfmin = small * (one + eps);
            }
            return sfmin;
        }

        /// <summary>
        /// Compute a Givens rotation.
        ///
        /// The computation uses the formulas
        ///     sigma = sgn(a)    if |a| >  |b|
        ///           = sgn(b)    if |b| >= |a|,
        ///
        ///     r = sigma * sqrt(a^2 + b^2),
        ///
        ///     c = 1; s = 0      if r = 0
        ///     c = a/r; s = b/r  if r != 0.
        ///
        ///  The function also computes
        ///     z = s    if |a| > |b|,
        ///       = 1/c  if |b| >= |a| and c != 0
        ///       = 1    if c = 0.
        ///
        ///  This allows c and s to be reconstructed from z as follows:
        ///
        ///     If z = 1, set c = 0, s = 1.
        ///     If |z| < 1, set c = sqrt(1 - z**2) and s = z.
        ///     If |z| > 1, set c = 1/z and s = sqrt( 1 - c**2).
        /// </summary>
        /// <param name="a">The x coordinate of the vector that will be rotated into (r,0) by the rotation.</param>
        /// <param name="b">The y coordinate of the vector that will be rotated into (r,0) by the rotation.</param>
        /// <param name="c">[out]The cosine of the rotation</param>
        /// <param name="s">[out]The sine of the rotation</param>
        /// <param name="r">[out]If viewed as a 2d vector, (a,b) will be rotated into (r,0).</param>
        /// <param name="z">[out]Used for reconstructing the rotation.</param>
        /// <seealso href="https://en.wikipedia.org/wiki/Givens_rotation"/>
        /// <remarks>BLAS equivalent: SROTG</remarks>
        public static void Givens(float a, float b, out float c, out float s, out float r, out float z)
        {
            Givens(new float2(a, b), out var cs, out r, out z);
            c = cs.x;
            s = cs.y;
        }

        /// <summary>
        /// Compute a Givens rotation, using the shorthand p = (a,b) and cs = (c,s).
        /// <seealso cref="Givens(float, float, out float, out float, out float, out float)"/>
        /// </summary>
        /// <param name="p">The (a,b) vector.</param>
        /// <param name="cs">[out]The (c,s) vector.</param>
        /// <param name="r">[out]If viewed as a 2d vector, (a,b) will be rotated into (r,0).</param>
        /// <param name="z">[out]Used for reconstructing the rotation.</param>
        /// <seealso href="https://en.wikipedia.org/wiki/Givens_rotation"/>
        /// <remarks>BLAS equivalent: SROTG</remarks>
        public static void Givens(float2 p, out float2 cs, out float r, out float z)
        {
            var absp = math.abs(p);

            var dir = (absp.y < absp.x) ? p.x : p.y;
            var scale = absp.x + absp.y;

            if (scale == 0.0)
            {
                cs = new float2(1, 0);
                r = 0;
            }
            else
            {
                r = scale * math.length(p / scale);
                r = math.sign(dir) * r;
                cs = p / r;
            }

            if (0 < math.abs(cs.x) && math.abs(cs.x) <= cs.y)
            {
                z = 1 / cs.x;
            }
            else
            {
                z = cs.y;
            }
        }
    }
}
