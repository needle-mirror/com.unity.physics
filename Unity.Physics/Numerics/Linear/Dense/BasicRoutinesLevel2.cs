using System;
using Unity.Collections;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    //
    // Level 2 BLAS equivalents
    //

    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Vector
    {
        /// <summary>
        /// Compute v := alpha op(A) x + beta v,<br/>
        /// where v denotes this vector.<br/>
        /// Note: elements of A are accessed sequentially with a single pass through A.
        /// </summary>
        /// <param name="A">A matrix of appropriate dimensions</param>
        /// <param name="op">If set to <see cref="Op.Transpose"/>, A^T will be used instead of A</param>
        /// <param name="x">The vector to add</param>
        /// <param name="alpha">The multiplier of the matrix product</param>
        /// <param name="beta">The muliplier of the added vector</param>
        /// <remarks>BLAS equivalent: SGEMV</remarks>
        public void ScaleAndAddProduct(in Matrix A, Op op, in Vector x, float alpha, float beta)
        {
            // Validate input
            if (op == Op.None)
            {
                Assertions.AssertCompatibleDimensionsTranspose(A, this);
                Assertions.AssertCompatibleDimensions(A, x);
            }
            else if (op == Op.Transpose)
            {
                Assertions.AssertCompatibleDimensions(A, this);
                Assertions.AssertCompatibleDimensionsTranspose(A, x);
            }
            else
            {
                UnityEngine.Debug.LogError("Invalid operation in Vector.ScaleAndAddProduct()");
            }

            if (A.NumRows == 0 || A.NumCols == 0 || (alpha == 0.0f && (beta == 1.0)))
            {
                return;
            }

            Scale(beta);

            if (alpha == 0.0f)
                return;

            if (op == Op.None)
            {
                if (alpha == 1.0f)
                {
                    for (int i = 0; i < A.NumCols; i++)
                    {
                        AddScaled(A.Cols[i], x[i]);
                    }
                }
                else
                {
                    for (int i = 0; i < A.NumCols; i++)
                    {
                        AddScaled(A.Cols[i], x[i] * alpha);
                    }
                }
            }
            else
            {
                if (alpha == 1.0f)
                {
                    for (int i = 0; i < A.NumCols; i++)
                    {
                        data[i] = A.Cols[i].Dot(x);
                    }
                }
                else
                {
                    for (int i = 0; i < A.NumCols; i++)
                    {
                        data[i] = alpha * A.Cols[i].Dot(x);
                    }
                }
            }
        }

        // Multiplies this vector by a triangular matrix
        // BLAS equivalent: STRMV
        public void MultiplyTriangular(in Matrix A, Op op, TriangularType type, DiagonalType diag)
        {
            // Validate input
            if (op == Op.None)
            {
                Assertions.AssertCompatibleDimensions(A, this);
            }
            else if (op == Op.Transpose)
            {
                Assertions.AssertCompatibleDimensionsTranspose(A, this);
            }
            else
            {
                UnityEngine.Debug.LogError("Invalid operation in Vector.MultiplyTriangular()");
            }

            if (A.NumRows != A.NumCols)
            {
                UnityEngine.Debug.LogError("Non-square matrix in Vector.MultiplyTriangular()");
            }

            if (A.NumRows == 0 || A.NumCols == 0 || Dimension == 0)
            {
                return;
            }

            // set up the vector starting point
            var start = data;
            if (Stride < 0)
            {
                start += (-Dimension + 1) * Stride;
            }

            // If not transposing, we can access the elements of A sequentially in a single pass
            if (op == Op.None)
            {
                // x <- A x
                if (type == TriangularType.Upper)
                {
                    if (Stride == 1)
                    {
                        for (int j = 0; j < Dimension; j++)
                        {
                            if (data[j] != 0)
                            {
                                float temp = data[j];
                                for (int i = 0; i < j; i++)
                                {
                                    data[i] += temp * A[i, j];
                                }

                                if (diag == DiagonalType.Explicit)
                                {
                                    data[j] *= A[j, j];
                                }
                            }
                        }
                    }
                    else
                    {
                        var subVec = start;
                        for (int j = 0; j < Dimension; j++, subVec += Stride)
                        {
                            if (*subVec != 0.0)
                            {
                                var temp = *subVec;
                                var dst = start;
                                for (int i = 0; i < j; i++, dst += Stride)
                                {
                                    *dst += temp * A[i, j];
                                }

                                if (diag == DiagonalType.Explicit)
                                {
                                    *subVec *= A[j, j];
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (Stride == 1)
                    {
                        for (int j = Dimension - 1; 0 <= j; j--)
                        {
                            if (data[j] != 0.0)
                            {
                                var temp = data[j];
                                for (int i = A.NumRows - 1; j < i; i--)
                                {
                                    data[i] = data[i] + temp * A[i, j];
                                }

                                if (diag == DiagonalType.Explicit)
                                {
                                    data[j] = data[j] * A[j, j];
                                }
                            }
                        }
                    }
                    else
                    {
                        start += (Dimension - 1) * Stride;

                        var subVec = start;
                        for (int j = Dimension - 1; 0 <= j; j--, subVec -= Stride)
                        {
                            if (*subVec != 0)
                            {
                                var temp = *subVec;
                                var dst = start;
                                for (int i = Dimension - 1; j < i; i--, dst -= Stride)
                                {
                                    *dst += temp * A[i, j];
                                }

                                if (diag == DiagonalType.Explicit)
                                {
                                    *start *= A[j, j];
                                }
                            }
                        }
                    }
                }
            }
            // x <- A^T x
            else
            {
                if (type == TriangularType.Upper)
                {
                    if (Stride == 1)
                    {
                        for (int j = A.NumRows - 1; 0 <= j; j--)
                        {
                            var temp = data[j];
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[j, j];
                            }
                            for (int i = j - 1; 0 <= i; i--)
                            {
                                temp += A[i, j] * data[i];
                            }
                            data[j] = temp;
                        }
                    }
                    else
                    {
                        var subVec = start + (Dimension - 1) * Stride;
                        for (int j = A.NumRows - 1; 0 <= j; j--, subVec -= Stride)
                        {
                            var temp = *subVec;
                            var dst = subVec;

                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[j, j];
                            }
                            for (int i = j - 1; 0 <= i; i--, dst -= Stride)
                            {
                                temp += A[i, j] * *dst;
                            }
                            *subVec = temp;
                        }
                    }
                }
                else
                {
                    if (Stride == 1)
                    {
                        for (int j = 0; j < A.NumRows; j++)
                        {
                            var temp = data[j];
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[j, j];
                            }
                            for (int i = j + 1; i < A.NumRows; i++)
                            {
                                temp += A[i, j] * data[i];
                            }
                            data[j] = temp;
                        }
                    }
                    else
                    {
                        var subVec = start;
                        for (int j = 0; j < Dimension; j++, subVec += Stride)
                        {
                            var temp = *subVec;
                            var dst = subVec;
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[j, j];
                            }
                            for (int i = j + 1; i < A.NumRows; i++, dst += Stride)
                            {
                                temp += A[i, j] * *dst;
                            }
                            *subVec = temp;
                        }
                    }
                }
            }
        }
    }

    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Matrix
    {
        /// <summary>
        /// Compute M := M + alpha x y^T
        /// BLAS equivalent: SGER
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="alpha"></param>
        public void AddOuterProduct(in Vector x, in Vector y, float alpha)
        {
            if (NumRows == 0 || NumCols == 0 || alpha == 0)
            {
                return;
            }

            if (alpha == 1.0f)
            {
                for (int i = 0; i < NumCols; i++)
                    Cols[i].AddScaled(x, y[i]);
            }
            else
            {
                for (int i = 0; i < NumCols; i++)
                    Cols[i].AddScaled(x, alpha * y[i]);
            }
        }
    }
}
