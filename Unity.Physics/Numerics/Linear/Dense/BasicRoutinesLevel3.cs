using System;
using Unity.Collections;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    //
    // Level 3 BLAS equivalents
    //

    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Matrix
    {
        /// <summary>
        /// Given a matrix C, computes
        ///
        ///     C := alpha * op(A) * op(B) + beta * C.
        ///
        /// </summary>
        /// <param name="opA">The operation to perform on A.</param>
        /// <param name="opB">The operation to perform on B.</param>
        /// <param name="alpha">The value to scale the product AB by.</param>
        /// <param name="A">The first matrix in the product.</param>
        /// <param name="B">The second matrix in the product.</param>
        /// <param name="beta">The value used to scale this matrix.</param>
        /// <remarks>BLAS equivalent: SGEMM</remarks>
        public void ScaleAndAddProduct(Op opA, Op opB, float alpha, in Matrix A, in Matrix B, float beta)
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

            if (alpha != 0)
            {
                if (NumRows != M)
                {
                    UnityEngine.Debug.LogError("Invalid matrix dimensions for A in Matrix.ScaleAndAddProduct()");
                }

                if (NumCols != N)
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
            }

            // Adding zero and multiplying by 1?
            if ((alpha == 0 || K == 0) && beta == 1)
            {
                return;
            }

            // Fast path if adding zero
            if (alpha == 0)
            {
                if (beta == 0)
                {
                    for (int i = 0; i < N; i++)
                    {
                        Cols[i].Clear();
                    }
                }
                else
                {
                    // C *= beta
                    for (int i = 0; i < N; i++)
                    {
                        Cols[i].Scale(beta);
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
                            Cols[i].Clear();
                        }
                        else if (beta != 1)
                        {
                            Cols[i].Scale(beta);
                        }

                        for (int j = 0; j < K; j++)
                        {
                            if (B.Cols[i][j] != 0)
                            {
                                Cols[i].AddScaled(A.Cols[j], alpha * B.Cols[i][j]);
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
                            float temp = A.Cols[j].Dot(B.Cols[i]);

                            if (beta == 0)
                            {
                                this[j, i] = alpha * temp;
                            }
                            else
                            {
                                this[j, i] = alpha * temp + beta * this[j, i];
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
                            Cols[i].Clear();
                        }
                        else if (beta != 1)
                        {
                            Cols[i].Scale(beta);
                        }

                        for (int j = 0; j < K; j++)
                        {
                            if (B.Cols[j][i] != 0)
                            {
                                var temp = alpha * B.Cols[j][i];
                                Cols[i].AddScaled(A.Cols[j], temp);
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
                            float temp = A.Cols[j].Dot(B.Rows[i]);

                            if (beta == 0)
                            {
                                this[j, i] = alpha * temp;
                            }
                            else
                            {
                                this[j, i] = alpha * temp + beta * this[j, i];
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Adds the scaled matrix op(A) to this matrix:
        ///     C := alpha * op(A) + C
        /// </summary>
        /// <remarks>Make sure not to hand in this matrix as input</remarks>
        /// <param name="A">Matrix to add.</param>
        /// <param name="alpha">Scaling factor.</param>
        public void AddScaled(Op op, in Matrix A, float alpha)
        {
            if (Rows == A.Rows || Cols == A.Cols)
            {
                UnityEngine.Debug.LogError("Can't add a matrix to itself in Matrix.AddScaled(). This would lead to undefined behavior. Use a copy as input matrix.");
            }

            if (NumRows != A.NumRows || NumCols != A.NumCols)
            {
                UnityEngine.Debug.LogError("Matrix dimensions do not match in Matrix.AddScaled()");
            }

            if (alpha == 0)
            {
                return;
            }

            if (op == Op.None)
            {
                for (int i = 0; i < NumCols; i++)
                {
                    Cols[i].AddScaled(A.Cols[i], alpha);
                }
            }
            else
            {
                for (int i = 0; i < NumCols; i++)
                {
                    Cols[i].AddScaled(A.Rows[i], alpha);
                }
            }
        }

        /// <summary>
        /// Compute the matrix product
        ///   B := alpha op(A) B,
        /// for Side option Left, or
        ///   B := alpha B op(A),
        /// for Side option Right,
        /// where alpha is a scalar, B is this matrix, and A is triangular.
        /// </summary>
        /// <param name="side"></param>
        /// <param name="type"></param>
        /// <param name="op"></param>
        /// <param name="diag"></param>
        /// <param name="alpha"></param>
        /// <param name="A"></param>
        /// <remarks>BLAS equivalent: STRMM</remarks>
        public void MultiplyByTriangular(Side side, TriangularType type, Op op, DiagonalType diag, float alpha, in Matrix A)
        {
            var rowsA = (side == Side.Left) ? A.NumRows : A.NumCols;

            if (NumCols == 0)
            {
                return;
            }

            if (alpha == 0)
            {
                Clear();
                return;
            }

            if (side == Side.Left)
            {
                if (op == Op.None)
                {
                    // B <- alpha A B
                    if (type == TriangularType.Upper)
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            for (int k = 0; k < NumRows; k++)
                            {
                                if (this[k, j] != 0.0f)
                                {
                                    var temp = alpha * this[k, j];
                                    for (int i = 0; i < k; i++)
                                    {
                                        this[i, j] += temp * A[i, k];
                                    }

                                    if (diag == DiagonalType.Explicit)
                                    {
                                        temp *= A[k, k];
                                    }
                                    this[k, j] = temp;
                                }
                            }
                        }
                    }
                    else
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            for (int k = NumRows - 1; 0 <= k; k--)
                            {
                                var elem = this[k, j];
                                if (elem != 0.0)
                                {
                                    var temp = alpha * elem;
                                    this[k, j] = temp;

                                    if (diag == DiagonalType.Explicit)
                                    {
                                        this[k, j] *= A[k, k];
                                    }

                                    for (int i = k + 1; i < NumRows; i++)
                                    {
                                        this[i, j] += temp * A[i, k];
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    // B <- alpha A^T B
                    if (type == TriangularType.Upper)
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            for (int i = NumRows - 1; 0 <= i; i--)
                            {
                                var temp = this[i, j];
                                if (diag == DiagonalType.Explicit)
                                {
                                    temp *= A[i, i];
                                }

                                for (int k = 0; k < i; k++)
                                {
                                    temp += this[k, j] * A[k, i];
                                }
                                this[i, j] = alpha * temp;
                            }
                        }
                    }
                    else
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            for (int i = 0; i < NumRows; i++)
                            {
                                var temp = this[i, j];
                                if (diag == DiagonalType.Explicit)
                                {
                                    temp *= A[i, i];
                                }
                                for (int k = i + 1; k < NumRows; k++)
                                {
                                    temp += temp + A[k, i] * this[k, j];
                                }
                                this[i, j] = alpha * temp;
                            }
                        }
                    }
                }
            }
            else
            {
                if (op == Op.None)
                {
                    // B <- alpha B A
                    if (type == TriangularType.Upper)
                    {
                        for (int j = NumCols - 1; j >= 0; j--)
                        {
                            var temp = alpha;
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[j, j];
                            }

                            for (int i = 0; i < NumRows; i++)
                            {
                                this[i, j] *= temp;
                            }
                            for (int k = 0; k < j; k++)
                            {
                                if (A[k, j] != 0.0f)
                                {
                                    temp = alpha * A[k, j];
                                    for (int i = 0; i < NumRows; i++)
                                    {
                                        this[i, j] += temp * this[i, k];
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            var temp = alpha;
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[j, j];
                            }
                            for (int i = 0; i < NumRows; i++)
                            {
                                this[i, j] *= temp;
                            }

                            for (int k = j + 1; k < NumCols; k++)
                            {
                                if (A[k, j] != 0)
                                {
                                    temp = alpha * A[k, j];
                                    for (int i = 0; i < NumRows; i++)
                                    {
                                        this[i, j] += temp * this[i, k];
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    // B <- alpha B A^T
                    if (type == TriangularType.Upper)
                    {
                        for (int k = 0; k < NumCols; k++)
                        {
                            float temp;
                            for (int j = 0; j < k; j++)
                            {
                                if (A[j, k] != 0)
                                {
                                    temp = alpha * A[j, k];
                                    for (int i = 0; i < NumRows; i++)
                                    {
                                        this[i, j] += temp * this[i, k];
                                    }
                                }
                            }
                            temp = alpha;
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[k, k];
                            }

                            if (temp != 1)
                            {
                                for (int i = 0; i < NumRows; i++)
                                {
                                    this[i, k] *= temp;
                                }
                            }
                        }
                    }
                    else
                    {
                        for (int k = NumCols - 1; 0 <= k; k--)
                        {
                            float temp;
                            for (int j = k + 1; j < NumCols; j++)
                            {
                                if (A[j, k] != 0)
                                {
                                    temp = alpha * A[j, k];
                                    for (int i = 0; i < NumRows; i++)
                                    {
                                        this[i, j] += temp * this[i, k];
                                    }
                                }
                            }

                            temp = alpha;
                            if (diag == DiagonalType.Explicit)
                            {
                                temp *= A[k, k];
                            }
                            if (temp != 1)
                            {
                                for (int i = 0; i < NumRows; i++)
                                {
                                    this[i, k] *= temp;
                                }
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Solves one of the matrix equations
        ///   op(A)*X = alpha*B
        /// or
        ///   X*op(A) = alpha*B,
        /// where alpha is a scalar, X and B (this matrix) are m by n matrices, A is a unit, or
        /// non-unit, upper or lower triangular matrix and op(A) specifies whether A should be transposed.
        /// Upon completion, this matrix is replaced with X.
        /// </summary>
        /// <param name="side"></param>
        /// <param name="type"></param>
        /// <param name="op"></param>
        /// <param name="diag"></param>
        /// <param name="alpha"></param>
        /// <param name="A"></param>
        /// <remarks>BLAS equivalent: STRSM</remarks>
        public void SolveGeneralizedTriangular(Side side, TriangularType type, Op op, DiagonalType diag, float alpha, in Matrix A)
        {
            if (alpha == 0)
            {
                Clear();
                return;
            }

            if (side == Side.Left)
            {
                // Form B = alpha * inv(A) * B.
                if (op == Op.None)
                {
                    if (type == TriangularType.Upper)
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            if (alpha != 1)
                            {
                                Cols[j].Scale(alpha);
                            }

                            for (int k = NumRows - 1; k >= 0; k--)
                            {
                                if (this[k, j] != 0.0)
                                {
                                    if (diag == DiagonalType.Explicit)
                                    {
                                        this[k, j] /= A[k, k];
                                    }

                                    for (int i = 0; i < k; i++)
                                    {
                                        this[i, j] -= this[k, j] * A[i, k];
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            if (alpha != 1)
                            {
                                Cols[j].Scale(alpha);
                            }

                            for (int k = 0; k < NumRows; k++)
                            {
                                if (this[k, j] != 0)
                                {
                                    if (diag == DiagonalType.Explicit)
                                    {
                                        this[k, j] /= A[k, k];
                                    }
                                    for (int i = k + 1; i < NumRows; i++)
                                    {
                                        this[i, j] -= this[k, j] * A[i, k];
                                    }
                                }
                            }
                        }
                    }
                }
                // Form  B = alpha * inv(A') * B.
                else
                {
                    if (type == TriangularType.Upper)
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            for (int i = 0; i < NumRows; i++)
                            {
                                var temp = alpha * this[i, j];
                                for (int k = 0; k < i; k++)
                                {
                                    temp -= A[k, i] * this[k, j];
                                }

                                if (diag == DiagonalType.Explicit)
                                {
                                    temp /= A[i, i];
                                }
                                this[i, j] = temp;
                            }
                        }
                    }
                    else
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            for (int i = NumRows - 1; 0 <= i; i--)
                            {
                                var temp = alpha * this[i, j];
                                for (int k = i + 1; k < NumRows; k++)
                                {
                                    temp -= A[k, i] * this[k, j];
                                }

                                if (diag == DiagonalType.Explicit)
                                {
                                    temp /= A[i, i];
                                }
                                this[i, j] = temp;
                            }
                        }
                    }
                }
            }
            // Form B = alpha * B * inv(A).
            else
            {
                if (op == Op.None)
                {
                    if (type == TriangularType.Upper)
                    {
                        for (int j = 0; j < NumCols; j++)
                        {
                            if (alpha != 1)
                            {
                                Cols[j].Scale(alpha);
                            }

                            for (int k = 0; k < j; k++)
                            {
                                if (A[k, j] != 0)
                                {
                                    for (int i = 0; i < NumRows; i++)
                                    {
                                        this[i, j] -= A[k, j] * this[i, k];
                                    }
                                }
                            }

                            if (diag == DiagonalType.Explicit)
                            {
                                Cols[j].Scale(1 / A[j, j]);
                            }
                        }
                    }
                    else
                    {
                        for (int j = NumCols - 1; j >= 0; j--)
                        {
                            if (alpha != 1)
                            {
                                Cols[j].Scale(alpha);
                            }

                            for (int k = j + 1; k < NumCols; k++)
                            {
                                if (A[k, j] != 0)
                                {
                                    for (int i = 0; i < NumRows; i++)
                                    {
                                        this[i, j] -= A[k, j] * this[i, k];
                                    }
                                }
                            }

                            if (diag == DiagonalType.Explicit)
                            {
                                Cols[j].Scale(1.0f / A[j, j]);
                            }
                        }
                    }
                }
                // Form B = alpha*B*inv(A').
                else
                {
                    if (type == TriangularType.Upper)
                    {
                        for (int k = NumCols - 1; 0 <= k; k--)
                        {
                            if (diag == DiagonalType.Explicit)
                            {
                                Cols[k].Scale(1 / A[k, k]);
                            }

                            for (int j = 0; j < k; j++)
                            {
                                if (A[j, k] != 0)
                                {
                                    Cols[j].AddScaled(Cols[k], A[j, k]);
                                }
                            }

                            if (alpha != 1)
                            {
                                Cols[k].Scale(alpha);
                            }
                        }
                    }
                    else
                    {
                        for (int k = 0; k < NumCols; k++)
                        {
                            if (diag == DiagonalType.Explicit)
                            {
                                Cols[k].Scale(1 / A[k, k]);
                            }

                            for (int j = k + 1; j < NumCols; j++)
                            {
                                if (A[j, k] != 0)
                                {
                                    Cols[j].AddScaled(Cols[k], -A[j, k]);
                                }
                            }

                            if (alpha != 1)
                            {
                                Cols[k].Scale(alpha);
                            }
                        }
                    }
                }
            }
        }
    }
}
