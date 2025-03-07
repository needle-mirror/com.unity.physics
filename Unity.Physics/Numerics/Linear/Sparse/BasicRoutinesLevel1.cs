using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Sparse.Primitives
{
    //
    // Level 1 BLAS equivalents
    //

    [BurstCompile]
    internal unsafe static partial class DenseVectorExtensions
    {
        public static void AddScaled(this Dense.Primitives.Vector u, ref Vector v, float alpha)
        {
            for (int i = 0; i < v.NonZeroElements; i++)
            {
                var idx = v.Indices[i];
                u[idx] += v.Values[i];
            }
        }

        public static float Dot(this Dense.Primitives.Vector u, ref Vector v)
        {
            float sum = 0;
            for (int i = 0; i < v.NonZeroElements; i++)
            {
                var idx = v.Indices[i];
                sum += u[idx] * v.Values[i];
            }
            return sum;
        }
    }

    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Vector
    {
        /// <summary>
        /// Computes the 2-norm, taking care to avoid underflow/overflow
        /// </summary>
        /// <returns>The Euclidean norm of the vector</returns>
        /// <remarks>BLAS equivalent: snrm2</remarks>
        public float Norm()
        {
            float norm;
            if (NonZeroElements < 1)
            {
                norm = 0.0f;
            }
            else
            {
                // TODO:  specialize for stride=1
                float scale = 0;
                float ssq = 1;

                var ptr = Values;
                var idx = Indices;

                for (int i = 0; i < NonZeroElements; i++, ptr++)
                {
                    var x = *ptr;
                    if (x != 0.0f)
                    {
                        x = math.abs(x);
                        if (scale < x)
                        {
                            var rx = scale / x;
                            ssq = 1.0f + ssq * rx * rx;
                            scale = x;
                        }
                        else
                        {
                            x = x / scale;
                            ssq = ssq + x * x;
                        }
                    }
                }

                norm = scale * math.sqrt(ssq);
            }

            return norm;
        }

        /// <summary>
        /// Finds the element with the greatest absolute value.
        /// </summary>
        /// <returns>The element index.</returns>
        /// <remarks>BLAS equivalent: ISAMAX</remarks>
        public int AbsMaxIndex()
        {
            int index = 0;

            if (NonZeroElements < 1)
            {
                return index;
            }

            // TODO: vectorized version
            float maxValue = math.abs(Values[0]);

            var ptr = Values + 1;
            for (int i = 1; i < NonZeroElements; i++, ptr++)
            {
                if (maxValue < math.abs(*ptr))
                {
                    index = i;
                    maxValue = math.abs(*ptr);
                }
            }

            return index;
        }

        /// <summary>
        /// Computes the 1-norm.
        /// </summary>
        /// <returns>The sum of absolute values of the vector.</returns>
        /// <remarks>BLAS equivalent: SASUM</remarks>
        public float AbsSum()
        {
            float sum = 0;
            var ptr = Values;
            for (int i = 0; i < NonZeroElements; i++, ptr++)
            {
                sum += math.abs(*ptr);
            }
            return sum;
        }

        /// <summary>
        /// Multiplies this vector by a scalar factor
        /// </summary>
        /// <param name="s">The scalar to multiply by.</param>
        /// <remarks>BLAS equivalent: SSCAL</remarks>
        public void Scale(float s)
        {
            if (s == 1.0f)
            {
                return;
            }

            var ptr = values;
            for (int i = 0; i < NonZeroElements; i++, ptr++)
            {
                *ptr *= s;
            }
        }

        /// <summary>
        /// Sparse gather into compressed form.  Replaces the nonzero elements of this vector
        /// with the corresponding elements of a dense vector.
        /// </summary>
        /// <param name="v">The dense vector supplying the elements.</param>
        public void Gather(ref Dense.Primitives.Vector v)
        {
            for (int i = 0; i < NonZeroElements; i++)
            {
                Values[i] = v[Indices[i]];
            }
        }

        /// <summary>
        /// Sparse gather and zero.  Replaces the nonzero elements of this vector
        /// with the corresponding elements of a dense vector, and zeroes out those elements.
        /// </summary>
        /// <param name="v">The dense vector supplying the elements.</param>
        public void GatherAndZero(ref Dense.Primitives.Vector v)
        {
            for (int i = 0; i < NonZeroElements; i++)
            {
                int idx = Indices[i];
                Values[i] = v[idx];
                v[idx] = 0;
            }
        }

        /// <summary>
        /// Sparse scatter.  Copies the nonzero elements of this vector into
        /// the corresponding locations in a dense vector.
        /// </summary>
        /// <param name="v"></param>
        public void SparseScatter(ref Dense.Primitives.Vector v)
        {
            for (int i = 0; i < NonZeroElements; i++)
            {
                v[Indices[i]] = Values[i];
            }
        }

        public float Dot(ref Dense.Primitives.Vector u)
        {
            float sum = 0;
            for (int i = 0; i < NonZeroElements; i++)
            {
                var idx = Indices[i];
                sum += u[idx] * Values[i];
            }
            return sum;
        }
    }
}
