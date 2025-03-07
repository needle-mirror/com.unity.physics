using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    //
    // Level 1 BLAS equivalents
    //

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
            if (Dimension < 1 || Stride < 1)
            {
                norm = 0.0f;
            }
            else if (Dimension == 1)
            {
                norm = math.abs(data[0]);
            }
            else
            {
                // TODO:  specialize for stride=1
                float scale = 0;
                float ssq = 1;

                var ptr = data;
                for (int i = 0; i < Dimension; i++, ptr += Stride)
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

            if (Dimension < 1 || Stride <= 0)
            {
                return index;
            }

            if (Dimension == 1)
            {
                return index;
            }

            float maxValue;
            if (Stride == 1 && Dimension > 4)
            {
                maxValue = math.abs(data[0]);
                maxValue = 0;

                int start = 0;
                if (Dimension >= 4)
                {
                    float4* ptr = (float4*)data;

                    int dim4 = Dimension & ~0x3;

                    for (int i = 0; i < dim4; i += 4, ptr++)
                    {
                        var x = math.abs(*ptr);
                        var maxElem = math.cmax(x);

                        if (maxValue < maxElem)
                        {
                            maxValue = maxElem;

                            index = i + math.cmin(math.select(4, new int4(0, 1, 2, 3), maxValue == x));
                        }
                    }

                    start = dim4;
                }

                for (int i = start; i < Dimension; i++)
                {
                    float x = math.abs(data[i]);
                    if (maxValue < x)
                    {
                        maxValue = x;
                        index = i;
                    }
                }
            }
            else
            {
                maxValue = math.abs(data[0]);

                var ptr = data + Stride;
                for (int i = 1; i < Dimension; i++, ptr += Stride)
                {
                    if (maxValue < math.abs(*ptr))
                    {
                        index = i;
                        maxValue = math.abs(*ptr);
                    }
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
            var ptr = data;
            for (int i = 0; i < Dimension; i++, ptr += Stride)
            {
                sum += math.abs(*ptr);
            }
            return sum;
        }

        /// <summary>
        /// Adds s * v to this vector
        /// </summary>
        /// <remarks>BLAS equivalent: SAXPY</remarks>
        public void AddScaled(in Vector v, float s)
        {
            Assertions.AssertCompatibleDimensions(this, v);

            if (Dimension <= 0)
            {
                return;
            }

            if (s == 0.0f)
            {
                return;
            }

            // fast path whenever both strides are 1
            if (Stride == 1 && v.Stride == 1)
            {
                int start = 0;
                if (Dimension >= 4)
                {
                    int dim4 = Dimension & ~0x3;

                    var src = (float4*)v.data;
                    var dst = (float4*)data;
                    for (int i = 0; i < dim4; i += 4, src++, dst++)
                    {
                        *dst += *src * s;
                    }
                    start = dim4;
                }

                for (int i = start; i < Dimension; i++)
                {
                    data[i] = data[i] + s * v.data[i];
                }
            }
            else
            {
                // general path.
                //   TODO: add a fast path when only one of the strides is 1
                //   TODO: add a fast path the strides are equal and multiples of 4
                var src = v.data;
                if (v.Stride < 0)
                {
                    src += (-Dimension + 1) * v.Stride;
                }

                var dst = data;
                if (Stride < 0)
                {
                    dst += (-Dimension + 1) * Stride;
                }

                for (int i = 0; i < Dimension; i++, src += v.Stride, dst += Stride)
                {
                    *dst += *src * s;
                }
            }
        }

        /// <summary>
        /// Copies this vector into another
        /// </summary>
        /// <param name="v">The destination vector</param>
        /// <remarks>BLAS equivalent: SCOPY</remarks>
        public void CopyTo(in Vector v)
        {
            Assertions.AssertCompatibleDimensions(this, v);

            int step = UnsafeUtility.SizeOf<float>();
            UnsafeUtility.MemCpyStride(v.data, v.Stride * step, data, Stride * step, UnsafeUtility.SizeOf<float>(), (int)Dimension);
        }

        /// <summary>
        /// Computes the dot product with the vector v.
        /// </summary>
        /// <param name="v">The other vector.</param>
        /// <seealso cref="AddScaled(Vector, float)"/>
        /// <remarks>BLAS equivalent: SDOT</remarks>
        public float Dot(in Vector v)
        {
            Assertions.AssertCompatibleDimensions(this, v);

            if (Dimension <= 0)
            {
                return 0;
            }

            float sum = 0;

            // fast path whenever both strides are 1
            if (Stride == 1 && v.Stride == 1)
            {
                int start = 0;
                if (Dimension >= 4)
                {
                    int dim4 = Dimension & ~0x3;

                    var ptr1 = (float4*)data;
                    var ptr2 = (float4*)v.data;
                    for (int i = 0; i < dim4; i += 4, ptr1++, ptr2++)
                    {
                        sum += math.dot(*ptr1, *ptr2);
                    }
                    start = dim4;
                }

                for (int i = start; i < Dimension; i++)
                {
                    sum += data[i] * v.data[i];
                }
            }
            else
            {
                // general path.
                //   TODO: add a fast path when only one of the strides is 1
                //   TODO: add a fast path the strides are equal and multiples of 4
                var ptr1 = data;
                if (Stride < 0)
                {
                    ptr1 += (-Dimension + 1) * Stride;
                }

                var ptr2 = v.data;
                if (v.Stride < 0)
                {
                    ptr2 += (-Dimension + 1) * v.Stride;
                }

                for (int i = 0; i < Dimension; i++, ptr1 += Stride, ptr2 += v.Stride)
                {
                    sum += *ptr1 * *ptr2;
                }
            }

            return sum;
        }

        /// <summary>
        /// Applies a plane rotation (c,s) to sets of 2d vectors given by (x[i],y[i])
        /// </summary>
        /// <param name="x">A vector containing the x coordinates</param>
        /// <param name="y">A vector containing the y coordinates</param>
        /// <param name="c">The cosine of the rotation</param>
        /// <param name="s">The sine of the rotation</param>
        /// <remarks>BLAS equivalent: SROT</remarks>
        public static void PlaneRotation(in Vector x, in Vector y, float c, float s)
        {
            float stemp;

            Assertions.AssertCompatibleDimensions(x, y);

            var size = x.Dimension;
            if (size <= 0)
            {
            }
            else if (x.Stride == 1 && y.Stride == 1)
            {
                for (int i = 0; i < size; i++)
                {
                    stemp = c * x[i] + s * y[i];
                    y[i] = c * y[i] - s * x[i];
                    x[i] = stemp;
                }
            }
            else
            {
                var px = x.data;
                var py = y.data;

                if (x.Stride < 0)
                {
                    px += (-size + 1) * x.Stride;
                }

                if (y.Stride < 0)
                {
                    py += (-size + 1) * y.Stride;
                }

                for (int i = 0; i < size; i++, px += x.Stride, py += y.Stride)
                {
                    stemp = c * *px + s * *py;
                    *py = c * *py - s * *px;
                    *px = stemp;
                }
            }
        }

        /// <summary>
        /// Multiplies this vector by a scalar factor
        /// </summary>
        /// <param name="s">The scalar to multiply by.</param>
        /// <remarks>BLAS equivalent: SSCAL</remarks>
        public void Scale(float s, int start = 0, int end = -1)
        {
            if (start < 0 || start >= Dimension)
            {
                return;
            }

            if (end < 0 || end >= Dimension)
            {
                end = Dimension - 1;
            }

            if (start > end || s == 1.0f)
            {
                return;
            }

            int size = end - start + 1;

            var dataStart = data + start;

            // fast path for stride=1
            if (Stride == 1)
            {
                int idx = 0;
                var ptr = (float4*)dataStart;

                if (size >= 4)
                {
                    int dim4 = size & ~0x3;
                    for (int i = 0; i < dim4; i += 4, ptr++)
                    {
                        *ptr *= s;
                    }
                    idx = dim4;
                }

                var sptr = (float*)ptr;
                for (int i = idx; i < size; i++, sptr++)
                {
                    *sptr *= s;
                }
            }
            else
            {
                // general path
                var ptr = dataStart;
                if (Stride < 0)
                {
                    ptr += (-size + 1) * Stride;
                }

                for (int i = 0; i < size; i++, ptr += Stride)
                {
                    *ptr *= s;
                }
            }
        }

        /// <summary>
        /// Swaps the elements of this vector with another
        /// </summary>
        /// <param name="v">The vector to swap with. Must be the same dimension.</param>
        /// <remarks>BLAS equivalent: SSWAP</remarks>
        public void Swap(in Vector v)
        {
            Assertions.AssertCompatibleDimensions(this, v);
            if (Dimension <= 0)
            {
                return;
            }

            // fast path whenever both strides are 1
            if (Stride == 1 && v.Stride == 1)
            {
                int start = 0;

                if (Dimension >= 4)
                {
                    int dim4 = Dimension & ~0x3;

                    var ptr1 = (float4*)data;
                    var ptr2 = (float4*)v.data;
                    for (int i = 0; i < dim4; i += 4, ptr2++, ptr1++)
                    {
                        var tmp = *ptr1;
                        *ptr1 = *ptr2;
                        *ptr2 = tmp;
                    }
                    start = dim4;
                }

                for (int i = start; i < Dimension; i++)
                {
                    var tmp = data[i];
                    data[i] = v.data[i];
                    v.data[i] = tmp;
                }
            }
            else
            {
                // general path.
                //   TODO: add a fast path when only one of the strides is 1
                //   TODO: add a fast path the strides are equal and multiples of 4
                var ptr1 = data;
                if (Stride < 0)
                {
                    ptr1 += (-Dimension + 1) * Stride;
                }

                var ptr2 = v.data;
                if (v.Stride < 0)
                {
                    ptr2 += (-Dimension + 1) * v.Stride;
                }

                for (int i = 0; i < Dimension; i++, ptr1 += Stride, ptr2 += v.Stride)
                {
                    var tmp = *ptr1;
                    *ptr1 = *ptr2;
                    *ptr2 = tmp;
                }
            }
        }

        /// <summary>
        /// Clears the vector.
        /// </summary>
        /// <param name="start">Optional start index</param>
        /// <param name="end">Optional end index. Set to -1 to clear the vector to the end.</param>
        /// <param name="value">Optional clear value.</param>
        public void Clear(int start = 0, int end = -1, float value = 0.0f)
        {
            if (start < 0 || start >= Dimension)
            {
                return;
            }

            if (end < 0 || end >= Dimension)
            {
                end = Dimension - 1;
            }

            if (Stride == 1 && value == 0)
            {
                UnsafeUtility.MemClear(data + start, UnsafeUtility.SizeOf<float>() * (end - start + 1));
            }
            else
            {
                for (int i = start; i <= end; i++)
                {
                    this[i] = value;
                }
            }
        }
    }
}
