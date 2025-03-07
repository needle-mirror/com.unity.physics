using Unity.Collections;

namespace Unity.Numerics.Linear.Sparse.Primitives
{
    //
    // Level 2 BLAS equivalents
    //

    [GenerateTestsForBurstCompatibility]
    internal unsafe static partial class DenseVectorExtensions
    {
        /// <summary>
        /// Compute v <- alpha op(A) x + beta v
        /// Note: elements of A are accessed sequentially with a single pass through A.
        /// </summary>
        /// <param name="A">A sparse matrix of appropriate dimensions</param>
        /// <param name="op">If set to <see cref="Op.Transpose"/>, A^T will be used instead of A</param>
        /// <param name="x">The vector to add</param>
        /// <param name="alpha">The multiplier of the matrix product</param>
        /// <param name="beta">Te muliplier of the added vector</param>
        /// <remarks>BLAS equivalent: SGEMV</remarks>
        public static void ScaleAndAddProduct(this Dense.Primitives.Vector v, ref Matrix A, Op op, ref Dense.Primitives.Vector x, float alpha, float beta)
        {
            // Validate input
            if (op == Op.None)
            {
                Assertions.AssertCompatibleDimensionsTranspose(A, v);
                Assertions.AssertCompatibleDimensions(A, x);
            }
            else if (op == Op.Transpose)
            {
                Assertions.AssertCompatibleDimensions(A, v);
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

            v.Scale(beta);

            if (op == Op.None)
            {
                if (alpha == 1.0f)
                {
                    for (int i = 0; i < v.Dimension; i++)
                    {
                        v.AddScaled(ref A.Cols[i], x[i]);
                    }
                }
                else
                {
                    for (int i = 0; i < v.Dimension; i++)
                    {
                        v.AddScaled(ref A.Cols[i], x[i] * alpha);
                    }
                }
            }
            else
            {
                if (alpha == 1.0f)
                {
                    for (int i = 0; i < v.Dimension; i++)
                    {
                        v[i] = x.Dot(ref A.Cols[i]);
                    }
                }
                else
                {
                    for (int i = 0; i < v.Dimension; i++)
                    {
                        v[i] = alpha * x.Dot(ref A.Cols[i]);
                    }
                }
            }
        }
    }

    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Vector
    {
    }
}
