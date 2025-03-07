using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal unsafe struct LQ
    {
        /// <summary>
        /// Computes n LQ factorization of a real m-by-n matrix <paramref name="A"/>:
        ///
        ///    A = (L 0) * Q
        ///
        /// where
        ///    Q is a n-by-n orthogonal matrix
        ///    L is an lower-triangular m-by-m matrix;
        ///    0 is a m-by-(n-m) zero matrix, if m < n.
        /// </summary>
        /// <param name="A">On entry, the m by n matrix <paramref name="A"/>. On exit, the elements on and below the
        /// diagonal of the array contain the m by min(m,n) lower trapezoidal matrix L (L is lower triangular if m <= n).
        /// The elements above the diagonal, with the array <paramref name="tau"/>, represent the orthogonal matrix Q
        /// as a product of elementary reflectors (see Remarks).</param>
        /// <param name="tau">The scalar factors of the elementary reflectors (see Remarks).</param>
        /// <remarks>
        /// The matrix Q is represented as a product of elementary reflectors
        ///
        ///  Q = H(1) H(2) . . . H(k), where k = min(m,n).
        ///
        /// Each H(i) has the form
        ///
        ///  H(i) = I - tau * v v^T
        ///
        /// where tau is a real scalar, and v is a real vector with v(1:i-1) = 0 and v(i) = 1;
        /// v(i+1:m) is stored on exit in A(i,i+1:n), and tau in <paramref name="tau"/>[i].
        /// BLAS equivalent: SGELQ2
        /// </remarks>
        public static void Factor(in Matrix A, in Vector tau)
        {
            var dim = A.MinDimension;

            for (int i = 0; i < dim; i++)
            {
                // Generate elementary reflector H(i) to annihilate A(i,i+1:n)
                var v = A.Rows[i].Subvector(math.min(i + 1, A.NumCols));
                Householder.ComputeReflector(v, A[i, i], out var beta, out var t);
                tau[i] = t;

                if (i < A.NumRows - 1)
                {
                    A[i, i] = 1.0f;

                    using var subM = A.Submatrix(i + 1, i);
                    var subR = A.Rows[i].Subvector(i);
                    Householder.ApplyReflector(subM, Side.Right, subR, tau[i]);
                }
                A[i, i] = beta;
            }
        }

        /// <summary>
        /// Overwrites matrix C with
        ///
        ///    Q C    if <paramref name="side"/> == Left  and <paramref name="op"/> == None, or
        ///
        ///    Q^T C  if <paramref name="side"/> == Left  and <paramref name="op"/> == Transpose, or
        ///
        ///    C Q    if <paramref name="side"/> == Right and <paramref name="op"/> == None, or
        ///
        ///    C Q^T  if <paramref name="side"/> == Right and <paramref name="op"/> == Transpose,
        ///
        /// where Q is a real orthogonal matrix defined as the product of k
        /// elementary reflectors
        ///
        ///    Q = H(1) H(2) . . . H(k)
        ///
        /// as returned by <see cref="Factor(Matrix, out Vector)"/>.
        /// Q is of order m if <paramref name="side"/> ==  Left and of order n if <paramref name="side"/> == Right.
        /// </summary>
        /// <param name="C"></param>
        /// <param name="side"></param>
        /// <param name="op"></param>
        /// <param name="A"></param>
        /// <param name="tau"></param>
        /// <remarks>BLAS equivalent: SORM2R</remarks>
        public static void QROrthogonalProduct(in Matrix C, Side side, Op op, Matrix A, Vector tau)
        {
            // TODO:  FIXME
            int start, end, step;
            if (side == Side.Left && op == Op.None)
            {
                start = 0;
                end = tau.Dimension - 1;
                step = 1;
            }
            else
            {
                start = tau.Dimension - 1;
                end = 0;
                step = -1;
            }

            int rowIdx = 0, colIdx = 0;
            int rows, cols;
            if (side == Side.Left)
            {
                cols = C.NumCols;
                colIdx = 0;
            }
            else
            {
                rows = C.NumRows;
                rowIdx = 0;
            }

            for (int i = start; i != end; i += step)
            {
                if (side == Side.Left)
                {
                    // H(i) is applied to C(i:m,1:n)
                    rows = C.NumRows - i;
                    rowIdx = i;
                }
                else
                {
                    // H(i) is applied to C(1:m,i:n)
                    cols = C.NumCols - i;
                    colIdx = i;
                }

                var diag = A[i, i];
                A[i, i] = 1.0f;
                using var subM = C.Submatrix(rowIdx, colIdx);
                var subC = A.Cols[i].Subvector(i);
                Householder.ApplyReflector(subM, side, subC, tau[i]);
                A[i, i] = diag;
            }
        }

        /// <summary>
        /// generates an m by n real matrix Q with orthonormal columns,
        /// which is defined as the first n columns of a product of k elementary
        /// reflectors of order m
        ///
        ///       Q  =  H(1) H(2) . . . H(k)
        ///
        /// as returned by <see cref="Factor(in Matrix, out Vector)"/>.
        /// </summary>
        /// <param name="A"></param>
        /// <param name="tau"></param>
        /// <remarks>BLAS equivalent: SORGL2</remarks>
        public static void GenerateQ(Matrix A, Vector tau)
        {
            if (A.NumRows > A.NumCols)
            {
                UnityEngine.Debug.LogError("Invalid matrix in GenerateQ");
                return;
            }

            int k = tau.Dimension;

            // Initialize rows k+1:m to identity
            if (k < A.NumRows)
            {
                for (int j = 0; j < A.NumCols; j++)
                {
                    for (int i = k; i < A.NumRows; i++)
                    {
                        A[i, j] = 0;
                    }

                    if (j >= k && j < A.NumRows)
                    {
                        A[j, j] = 1.0f;
                    }
                }
            }

            for (int i = k - 1; i >= 0; i--)
            {
                // Apply H(i) to A(i:m,i:n) from the right
                if (i < A.NumCols - 1)
                {
                    if (i < A.NumRows - 1)
                    {
                        A[i, i] = 1;
                        var subR = A.Rows[i].Subvector(i);
                        using (var subM = A.Submatrix(i + 1, i))
                            Householder.ApplyReflector(subM, Side.Right, subR, tau[i]);
                    }

                    A.Rows[i].Subvector(i + 1).Scale(-tau[i]);
                }

                A[i, i] = 1.0f - tau[i];

                // Set A(i,1:i-1) to zero
                for (int j = 0; j <= i - 1; j++)
                {
                    A[i, j] = 0;
                }
            }
        }

        [BurstCompile]
        [GenerateTestsForBurstCompatibility]
        public struct Job : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;
            [WriteOnly] public Vector tau;
            public void Execute()
            {
                var dim = math.min(M.NumRows, M.NumCols);
                tau = heap.Vector(dim);
                Factor(M, tau);
            }
        }
    }
}
