using System;
using Unity.Collections;

namespace Unity.Numerics.Linear.Sparse.Primitives
{
    //
    // Level 3 BLAS equivalents
    //
    [GenerateTestsForBurstCompatibility]
    internal unsafe static partial class DenseMatrixExtensions
    {
        /// <summary>
        /// Given a matrix C, computes
        ///
        ///     C <- alpha * op(A) * op(B) + beta * C.
        ///
        /// </summary>
        /// <param name="opA">The operation to perform on A.</param>
        /// <param name="opB">The operation to perform on B.</param>
        /// <param name="alpha">The value to scale the product AB by.</param>
        /// <param name="A">The first matrix in the product.</param>
        /// <param name="B">The second matrix in the product.</param>
        /// <param name="beta">The value used to scale this matrix.</param>
        /// <remarks>BLAS equivalent: SGEMM</remarks>
        public static void ScaleAndAddProduct(this Dense.Primitives.Matrix C, Op opA, Op opB, float alpha, ref Matrix A, ref Dense.Primitives.Matrix B, float beta)
        {
            // Validate input
            if (opA != Op.None && opA != Op.Transpose)
            {
                UnityEngine.Debug.LogError("Invalid operation in Matrix.ScaleAndAddProduct()");
            }
            if (opB != Op.None && opB != Op.Transpose)
            {
                UnityEngine.Debug.LogError("Invalid operation in Matrix.ScaleAndAddProduct()");
            }

            // Precompute the dimensions so that we don't have to query every time:
            //   M = C.numRows = op(A).numRows
            //   N = C.numCols = op(B).numCols
            //   K = op(A).numCols = op(B).numRows
            //
            // In other words, C is MxN, op(A) is MxK, and op(B) is KxN.
            var M = (opA == Op.None ? A.NumRows : A.NumCols);
            var N = (opB == Op.None ? B.NumCols : B.NumRows);
            var K = (opA == Op.None ? A.NumCols : A.NumRows);

            if (C.NumRows != M)
            {
                UnityEngine.Debug.LogError("Invalid matrix dimensions for A in Matrix.ScaleAndAddProduct()");
            }
            if (C.NumCols != N)
            {
                UnityEngine.Debug.LogError("Invalid matrix dimensions for B in Matrix.ScaleAndAddProduct()");
            }
            if (K != (opB == Op.None ? B.NumRows : B.NumCols))
            {
                UnityEngine.Debug.LogError("Mismatched matrix dimensions for A and B in Matrix.ScaleAndAddProduct()");
            }


            if (M == 0 || N == 0)
            {
                return;
            }

            // Adding zero and mulltiplying by 1?
            if ((alpha == 0 || K == 0) && beta == 1)
            {
                return;
            }

            // Fast path if adding zero
            if (alpha == 0)
            {
                if (beta == 0)
                {
                    for (int i = 0; i < M; i++)
                    {
                        C.Cols[i].Clear();
                    }
                }
                else
                {
                    // C *= beta
                    for (int i = 0; i < M; i++)
                    {
                        C.Cols[i].Scale(beta);
                    }
                }
                return;
            }

            // start
            if (opB == Op.None)
            {
                if (opA == Op.None)
                {
                    // C <- alpha*A*B + beta*C
                    for (int i = 0; i < N; i++)
                    {
                        if (beta == 0)
                        {
                            C.Cols[i].Clear();
                        }
                        else if (beta != 1)
                        {
                            C.Cols[i].Scale(beta);
                        }

                        for (int j = 0; j < K; j++)
                        {
                            if (B.Cols[i][j] != 0)
                            {
                                C.Cols[i].AddScaled(ref A.Cols[j], alpha * B.Cols[i][j]);
                            }
                        }
                    }
                }
                else
                {
                    // C <- alpha*A^T*B + beta*C
                    for (int i = 0; i < N; i++)
                    {
                        for (int j = 0; j < M; j++)
                        {
                            float temp = A.Cols[j].Dot(ref B.Cols[i]);

                            if (beta == 0)
                            {
                                C[j, i] = alpha * temp;
                            }
                            else
                            {
                                C[j, i] = alpha * temp + beta * C[j, i];
                            }
                        }
                    }
                }
            }
            else
            {
                // C <- alpha*A*B^T + beta*C
                if (opA == Op.None)
                {
                    for (int i = 0; i < N; i++)
                    {
                        if (beta == 0)
                        {
                            C.Cols[i].Clear();
                        }
                        else if (beta != 1.0)
                        {
                            C.Cols[i].Scale(beta);
                        }

                        for (int j = 0; j < K; j++)
                        {
                            if (B.Cols[j][i] != 0.0)
                            {
                                var temp = alpha * B.Cols[j][i];
                                C.Cols[i].AddScaled(ref A.Cols[j], temp);
                            }
                        }
                    }
                }
                else
                {
                    // C <- alpha*A^T*B^T + beta*C
                    for (int i = 0; i < N; i++)
                    {
                        for (int j = 0; j < M; j++)
                        {
                            float temp = A.Cols[j].Dot(ref B.Rows[i]);

                            if (beta == 0.0)
                            {
                                C[j, i] = alpha * temp;
                            }
                            else
                            {
                                C[j, i] = alpha * temp + beta * C[j, i];
                            }
                        }
                    }
                }
            }
        }
    }
}
