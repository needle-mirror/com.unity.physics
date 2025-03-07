using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal unsafe struct GolubKahanLanczos
    {
        /// <summary>
        /// Computes a bidiagonal reduction of a real m-by-n matrix <paramref name="A"/> by
        /// orthogonal transformation:
        ///
        ///    Q^T A P = B.
        ///
        /// If m >= n then B is upper bidiagonal, otherwise lower bidiagonal.
        ///
        /// When the solver exits, the m by n matrix <paramref name="A"/> to be reduced is overwritten.  If m >= n,
        /// the diagonal and the first superdiagonal are overwritten with the upper bidiagonal matrix B; the elements
        /// below the diagonal, with the array tauQ, represent the orthogonal matrix Q as a product of elementary
        /// reflectors, and the  elements above the first superdiagonal, with the array tauP, represent the orthogonal
        /// matrix P as a product of elementary reflectors.
        ///
        /// If m<n, the diagonal and the first subdiagonal are overwritten with the lower bidiagonal matrix B; the
        /// elements below the first subdiagonal, with the array tauQ, represent the orthogonal matrix Q as a product
        /// of elementary reflectors, and the elements above the diagonal, with the array tauP, represent the orthogonal
        /// matrix P as a product of elementary reflectors.
        /// (see Remarks).
        ///
        /// </summary>
        /// <param name="A">The matrix to be reduced.</param>
        /// <param name="diag">The diagonal elements of the bidiagonal matrix B.</param>
        /// <param name="offDiag">The off-diagonal elements of B.</param>
        /// <param name="tauQ">The scalar factors of the elementary reflectors of Q (see Remarks).</param>
        /// <param name="tauP">The scalar factors of the elementary reflectors of P (see Remarks).</param>
        /// <remarks>
        /// The matrix Q and P are represented as products of elementary reflectors
        ///
        ///  Q = H(1) H(2) ... and P = G(1) G(2) ...
        ///
        /// Each H(i) and G(i) has the form
        ///
        ///  H(i) = I - tauQ * v v^T
        ///  G(i) = I - tauP * v v^T
        ///
        /// where tau is a real scalar, and v is a real vector with v(1:i-1) = 0 and v(i) = 1.
        /// BLAS equivalent: SGEBD2
        /// </remarks>
        public static void Factor(in Matrix A, in Vector diag, in Vector offDiag, in Vector tauQ, in Vector tauP)
        {
            var dim = A.MinDimension;

            if (A.NumRows >= A.NumCols)
            {
                // Reduce to upper bidiagonal form
                for (int i = 0; i < A.NumCols; i++)
                {
                    // Generate elementary reflector H(i) to annihilate A(i+1:m,i)
                    var subCol = A.Cols[i].Subvector(math.min(i + 1, A.NumRows));
                    Householder.ComputeReflector(subCol, A[i, i], out var betaH, out var tQ);
                    tauQ[i] = tQ;
                    diag[i] = betaH;

                    if (i < A.NumCols - 1)
                    {
                        // Apply H(i) to A(i:m,i+1:n) from the left
                        A[i, i] = 1.0f;
                        var subC = A.Cols[i].Subvector(i);
                        using (var subM = A.Submatrix(i, i + 1))
                            Householder.ApplyReflector(subM, Side.Left, subC, tQ);
                    }
                    A[i, i] = betaH;

                    if (i < A.NumCols - 1)
                    {
                        // Generate elementary reflector G(i) to annihilate A(i,i+2:n)
                        var subRow = A.Rows[i].Subvector(math.min(i + 2, A.NumCols));
                        Householder.ComputeReflector(subRow, A[i, i + 1], out var betaG, out var tP);
                        tauP[i] = tP;

                        offDiag[i] = betaG;
                        A[i, i + 1] = 1;

                        // Apply G(i) to A(i+1:m,i+1:n) from the right
                        var subR = A.Rows[i].Subvector(i + 1);
                        using (var subM = A.Submatrix(i + 1, i + 1))
                            Householder.ApplyReflector(subM, Side.Right, subR, tP);

                        A[i, i + 1] = betaG;
                    }
                    else
                    {
                        tauP[i] = 0;
                    }
                }
            }
            else
            {
                // Reduce to lower bidiagonal form
                for (int i = 0; i < A.NumRows; i++)
                {
                    // Generate elementary reflector G(i) to annihilate A(i,i+1:n)
                    var subRow = A.Rows[i].Subvector(math.min(i + 1, A.NumCols));
                    Householder.ComputeReflector(subRow, A[i, i], out var betaG, out var tP);
                    tauP[i] = tP;
                    diag[i] = betaG;

                    // Apply G(i) to A(i+1:m,i:n) from the right
                    if (i < A.NumRows - 1)
                    {
                        A[i, i] = 1;
                        var subR = A.Rows[i].Subvector(i);
                        using (var subM = A.Submatrix(i + 1, i))
                            Householder.ApplyReflector(subM, Side.Right, subR, tP);
                    }
                    A[i, i] = betaG;

                    if (i < A.NumRows - 1)
                    {
                        // Generate elementary reflector H(i) to annihilate A(i+2:m, i)
                        var subCol = A.Cols[i].Subvector(math.min(i + 2, A.NumRows));
                        Householder.ComputeReflector(subCol, A[i + 1, i], out var betaH, out var tQ);
                        tauQ[i] = tQ;

                        offDiag[i] = betaH;
                        A[i + 1, i] = 1;

                        // Apply H(i) to A(i+1:m,i+1:n) from the left
                        var subC = A.Cols[i].Subvector(i + 1);
                        using (var subM = A.Submatrix(i + 1, i + 1))
                            Householder.ApplyReflector(subM, Side.Left, subC, tQ);

                        A[i + 1, i] = betaH;
                    }
                    else
                    {
                        tauQ[i] = 0;
                    }
                }
            }
        }

        /// <summary>
        /// Generates the real orthogonal matrix Q determined by <see cref="Factor(in Matrix, in Vector, in Vector, in Vector, in Vector)"/>
        /// when reducing a real matrix A to bidiagonal form:
        ///
        ///   A = Q B P^T.
        ///
        /// Q is defined as the product of elementary reflectors H(i).
        /// A is assumed to have been an m by k matrix, and Q is of order m:
        /// if m >= k, then Q = H(1) H(2) . . . H(k) and this function returns the first n columns of Q, where m >= n >= k;
        /// if m < k, then Q = H(1) H(2) . . . H(m-1) and this function returns Q as an m by m matrix.
        /// </summary>
        /// <param name="A">The matrix as returned by the factorization. On exit, contains Q.</param>
        /// <param name="tau">Contains the elementary reflectors H as returned by the factorization.</param>
        public static void GenerateQ(in Matrix A, in Vector tau, int k)
        {
            //var k = tau.Dimension;

            if (A.NumRows >= k)
            {
                // if m >= k, assume m >= n >= k
                QR.GenerateQ(A, tau);
            }
            else
            {
                // If m < k, assume m = n.  Shift the vectors which define the elementary reflectors one
                // column to the right, and set the first row and column of Q to those of the identity
                // matrix.
                int m = A.NumRows;
                for (int j = m - 1; j >= 1; j--)
                {
                    A[0, j] = 0;
                    for (int i = j + 1; i < m; i++)
                    {
                        A[i, j] = A[i, j - 1];
                    }
                }
                A[0, 0] = 1;
                A.Cols[0].Clear(1);

                if (m > 1)
                {
                    using (var subA = A.Submatrix(1, 1))
                        QR.GenerateQ(subA, tau.Subvector(0, m - 1));
                }
            }
        }

        /// <summary>
        /// Generates the real orthogonal matrix P^T determined by <see cref="Factor(in Matrix, in Vector, in Vector, in Vector, in Vector)"/>
        /// when reducing a real matrix A to bidiagonal form:
        ///
        ///   A = Q B P^T.
        ///
        /// P^T is defined as the product of elementary reflectors G(i).
        /// A is assumed to have been a k by n matrix, and Q is of order n:
        /// if k < n, then P^T = G(k) . . . G(2) G(1) and and this function returns the first m rows of
        /// P^T, where n >= m >= k;
        /// if k >= n, then P^T  = G(n-1) . . . G(2) G(1) and this function returns P^T as an n by n matrix.
        /// </summary>
        /// <param name="A">The matrix as returned by the factorization. On exit, contains P^T.</param>
        /// <param name="tau">Contains the elementary reflectors H as returned by the factorization.</param>
        public static void GenerateP(in Matrix A, in Vector tau, int k)
        {
            //var k = tau.Dimension;

            if (k < A.NumCols)
            {
                // if k < n, assume k <= m <= n
                LQ.GenerateQ(A, tau);
            }
            else
            {
                // If k >= n, assume m = n.  Shift the vectors which define the elementary reflectors one
                // row downward, and set the first row and column of P^T to those of the identity
                // matrix.
                int n = A.NumCols;

                A[0, 0] = 1;
                A.Cols[0].Clear(1);

                for (int j = 1; j < n; j++)
                {
                    for (int i = j - 1; i >= 1; i--)
                    {
                        A[i, j] = A[i - 1, j];
                    }
                    A[0, j] = 0;
                }

                if (n > 1)
                {
                    using (var subA = A.Submatrix(1, 1))
                        LQ.GenerateQ(subA, tau.Subvector(0, n - 1));
                }
            }
        }

        [BurstCompile]
        [GenerateTestsForBurstCompatibility]
        public struct Job : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;
            [WriteOnly] public Vector diag, offDiag;
            [WriteOnly] public Vector tauQ, tauP;
            public void Execute()
            {
                tauQ = heap.Vector(M.MinDimension);
                tauP = heap.Vector(M.MinDimension);
                diag = heap.Vector(M.MinDimension);
                offDiag = heap.Vector(M.MinDimension - 1);

                Factor(M, diag, offDiag, tauQ, tauP);
            }
        }
    }
}
