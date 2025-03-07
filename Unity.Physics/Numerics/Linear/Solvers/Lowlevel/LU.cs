using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal unsafe struct LU
    {
        /// <summary>
        /// Compute a factorization of the form
        ///   A = P*L*U
        /// where P is a permutation matrix, L is lower triangular with unit diagonal elements,
        /// and U is upper triangular.
        /// </summary>
        /// <param name="A">The matrix to factor.</param>
        /// <param name="pivots">An array containing at least min(NumRows,NumCols) elements. On output, contains the row permutation.</param>
        /// <param name="singularRow">-1 if algorithm succeeded. Otherwise, contains row index at which matrix was found to be singular. </param>
        public static void Factor(in Matrix A, [WriteOnly] ref NativeArray<int> pivots, out int singularRow)
        {
            singularRow = -1;

            var minDim = A.MinDimension;
            if (pivots.Length < minDim)
            {
                UnityEngine.Debug.LogError("Insufficient space in the pivots array passed to FactorPLU().");
                return;
            }

            if (minDim == 0)
            {
                return;
            }

            float epsilon = float.Epsilon;
            for (int i = 0; i < minDim; i++)
            {
                // find pivot
                var pivotIndex = i + A.Cols[i].Subvector(i).AbsMaxIndex();
                pivots[i] = pivotIndex;

                float pivot = A[pivotIndex, i];
                if (pivot == 0)
                {
                    // The matrix is singular (to the extent of this algorithm's precision)
                    singularRow = i;
                }
                else
                {
                    Vector.Swap(A.Rows[i], A.Rows[pivotIndex]);

                    // compute the remainder of the column
                    if (i < A.NumRows - 1)
                    {
                        if (math.abs(pivot) >= epsilon)
                            A.Cols[i].Subvector(i + 1).Scale(1 / pivot);
                        else
                            A.Cols[i].Subvector(i).Scale(1 / pivot);
                    }
                }

                if (i < A.MinDimension - 1)
                {
                    // update the trailing submatrix
                    var subCol = A.Cols[i].Subvector(i + 1);
                    var subRow = A.Rows[i].Subvector(i + 1);

                    using (var subMatrix = A.Submatrix(i + 1, i + 1))
                        subMatrix.AddOuterProduct(subCol, subRow, -1.0f);
                }
            }
        }

        public static void BlockFactorRecursive(in Matrix A, int blockSize, [WriteOnly] ref NativeArray<int> pivots, out int singularRow)
        {
            // A00 |   A01   |     | L00 |    0    |   | U00 |   U01   |
            // ----+---------+     +-----+---------+   +-----+---------+
            //     |         |  =  |     |         | x |     |         |
            // A10 |   A11   |     | L10 |   L11   |   |  0  |   U11   |
            //     |         |     |     |         |   |     |         |
            //
            // Treating this as a 2x2 matrix, we have the following block factorization:
            //
            //    A00 = (L00,0)^T (U00,0)     = L00 U00
            //    A01 = (L00,0)^T (U01,U11)   = L00 U01
            //    A10 = (L10,L11)^T (U00,0)   = L10 U00
            //    A11 = (L10,L11)^T (U01,U11) = L10 U01 + L11 U11.
            //
            // A00 is bxb and is solved by non-blocked LU, which gives us L00 and U00.
            //
            // Moving on, A01 and U01 are bx(m-b), where U01 = L00^-1 A01.  We solve this using a triangular solver,
            // which can also be done blockwise by dividing A01 into bxb blocks.  Similarly, L10 = A10 U00^-1.
            //
            // Finally, the second term of A11 is the big remaining block L11 U11, which we factor recursively.
            var minDim = A.MinDimension;
            if (blockSize <= 1 || blockSize > minDim)
            {
                Factor(A, ref pivots, out singularRow);
                return;
            }

            singularRow = -1;

            var A00 = A.Submatrix(0, 0, blockSize, blockSize);
            var A01 = A.Submatrix(0, blockSize, blockSize, A.NumCols - blockSize);
            var A10 = A.Submatrix(blockSize, 0, A.NumRows - blockSize, blockSize);
            var A11 = A.Submatrix(blockSize, blockSize, A.NumRows - blockSize, A.NumCols - blockSize);
            var A00A10 = A.Submatrix(0, 0, A.NumRows, blockSize);

            var p00 = pivots.GetSubArray(0, blockSize);
            Factor(A00A10, ref p00, out singularRow);

            // apply row interchanges
            A.InterchangeRows(pivots, 0, blockSize - 1);

            // Solve A01 = L00 U01 for U01
            A01.SolveGeneralizedTriangular(Side.Left, TriangularType.Lower, Op.None, DiagonalType.Unit, 1.0f, A00);
            A01.InterchangeRows(pivots);

            // Solve A10 = L10 U00 for L10
            //A10.SolveGeneralizedTriangular(Side.Right, TriangularType.Upper, Op.None, DiagonalType.Explicit, 1.0f, A00);

            A11.ScaleAndAddProduct(Op.None, Op.None, -1.0f, A10, A01, 1.0f);

            var subPivots = pivots.GetSubArray(blockSize, pivots.Length - blockSize);
            BlockFactorRecursive(A11, blockSize, ref subPivots, out singularRow);

            for (int i = 0; i < subPivots.Length; i++)
            {
                subPivots[i] += blockSize;
            }
        }

        public static void BlockFactor(in Matrix A, int blockSize, [WriteOnly] ref NativeArray<int> pivots, out int singularRow)
        {
            var minDim = A.MinDimension;
            if (blockSize <= 1 || blockSize > minDim)
            {
                Factor(A, ref pivots, out singularRow);
                return;
            }

            singularRow = -1;

            for (int i = 0; i < minDim; i += blockSize)
            {
                int currentBlockSize = math.min(minDim - i, blockSize);

                // factor the diagonal and subdiagonal blocks and test for singularity
                var subPivots = pivots.GetSubArray(i, pivots.Length - i);

                int subSingular;

                using var blockColumn = A.Submatrix(i, i, A.NumRows - i, currentBlockSize);
                Factor(blockColumn, ref subPivots, out subSingular);

                if (subSingular >= 0)
                {
                    singularRow = subSingular + i;
                }

                for (int k = i; k < math.min(A.NumRows, i + currentBlockSize); k++)
                {
                    pivots[k] += i;
                }

                // apply row interchanges to columns 0:i-1
                using (var subA = A.Submatrix(0, 0, A.NumRows, i))
                    subA.InterchangeRows(pivots, i, i + currentBlockSize - 1);

                if (i + currentBlockSize <= A.NumCols)
                {
                    int colBlockSize = A.NumCols - (i + currentBlockSize);

                    // apply row interchanges to columns i+currentBlockSize through NumCols
                    using (var subA = A.Submatrix(0, i + currentBlockSize, currentBlockSize, colBlockSize))
                        subA.InterchangeRows(pivots, i, i + currentBlockSize - 1);

                    // compute block row
                    var blockRow = A.Submatrix(i, i + currentBlockSize, currentBlockSize, colBlockSize);
                    var eqs = A.Submatrix(i, i, currentBlockSize, currentBlockSize);
                    blockRow.SolveGeneralizedTriangular(Side.Left, TriangularType.Lower, Op.None, DiagonalType.Unit, 1.0f, eqs);
                    eqs.Dispose();
                    blockRow.Dispose();

                    if (i + currentBlockSize <= A.NumRows)
                    {
                        // update trailing submatrix
                        using var sub = A.Submatrix(i + blockSize, i + blockSize);
                        using var subA = A.Submatrix(i + blockSize, i, sub.NumRows, blockSize);
                        using var subB = A.Submatrix(i, i + blockSize, blockSize, sub.NumCols);
                        sub.ScaleAndAddProduct(Op.None, Op.None, -1.0f, subA, subB, 1.0f);
                    }
                }
            }
        }

        /// <summary>
        /// Computes the determinant from the factorization returned by <see cref="Factor(in Matrix, in NativeArray{int}, out int)"/>.
        /// </summary>
        /// <param name="A">The matrix to factor.</param>
        /// <param name="pivots">An array containing at least min(NumRows,NumCols) elements. On output, contains the row permutation.</param>
        /// <returns>The computed determinant of A.</returns>
        public static float ComputeDeterminantFromLU(in Matrix A, [WriteOnly] in NativeArray<int> pivots)
        {
            // compute the permutation parity
            var n = A.MinDimension;

            var determinant = 1.0f;
            for (int i = 0; i < n; i++)
            {
                for (int k = i + 1; k < n; k++)
                {
                    if (pivots[i] > pivots[k])
                    {
                        determinant = -determinant;
                    }
                }
            }

            for (int i = 0; i < n; i++)
            {
                determinant *= A[i, i];
            }

            return determinant;
        }

        [BurstCompile]
        [GenerateTestsForBurstCompatibility]
        public struct Job : IJob
        {
            [ReadOnly] public Matrix M;
            [WriteOnly] public NativeArray<int> P;
            public void Execute()
            {
                Factor(M, ref P, out var singularRow);
            }
        }
    }
}
