using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    /// <summary>
    /// Dense matrix class.
    /// </summary>
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal unsafe readonly partial struct Matrix
    {
        /// <summary>
        /// Scans for the last nonzero row
        /// </summary>
        /// <returns>The index of the last nonzero row</returns>
        public int LastNZRowIndex(int startRow = 0, int startColumn = 0, int endRow = -1, int endColumn = -1)
        {
            if (endRow < 0)
                endRow = NumRows;
            if (endColumn < 0)
                endColumn = NumCols;

            endRow--;
            endColumn--;

            // check the common case where a corner is nonzero
            if (startRow >= endRow || startColumn >= endColumn)
                return 0;
            else if (this[endRow, 0] != 0 || this[endRow, endColumn] != 0)
                return endRow;

            // TODO:  check ilaslr for efficiency improvements
            for (int r = endRow; r >= startRow; r--)
            {
                for (int c = startColumn; c <= endColumn; c++)
                {
                    if (this[r, c] != 0)
                        return r;
                }
            }

            // unreachable
            return 0;
        }

        /// <summary>
        /// Scans for the last nonzero column
        /// </summary>
        /// <returns>The index of the last nonzero column</returns>
        public int LastNZColumnIndex(int startRow = 0, int startColumn = 0, int endRow = -1, int endColumn = -1)
        {
            if (endRow < 0)
                endRow = NumRows;
            if (endColumn < 0)
                endColumn = NumCols;

            endRow--;
            endColumn--;

            // check the common case where a corner is nonzero
            if (startRow >= endRow || startColumn >= endColumn)
                return 0;
            else if (this[startRow, endColumn] != 0 || this[endRow, endColumn] != 0)
                return endColumn;

            for (int c = endColumn; c >= startColumn; c--)
            {
                for (int r = startRow; r <= endRow; r++)
                {
                    if (this[r, c] != 0)
                        return c;
                }
            }

            // unreachable
            return 0;
        }

        /// <summary>
        /// Sets the diagonal elements to <paramref name="beta"/>, and the off-diagonal elements to <paramref name="alpha"/>
        /// as specified by the <paramref name="lowerTri"/> and <paramref name="upperTri"/> parameters.
        /// </summary>
        /// <param name="alpha">The off-diagonal value.</param>
        /// <param name="beta">The diagonal value.</param>
        /// <param name="upperTri">If true, the upper triangular (trapezoidal) part of the matrix will be set to alpha.</param>
        /// <param name="lowerTri">If true, the lower triangular (trapezoidal) part of the matrix will be set to alpha.</param>
        /// <param name="diagonal">If true, the diagonal part of the matrix will be set to beta.</param>
        public void SetDiagonal(float alpha, float beta, bool upperTri, bool lowerTri, bool diagonal)
        {
            if (upperTri)
            {
                for (int i = 1; i < NumCols; i++)
                {
                    Cols[i].Clear(0, i - 1, alpha);
                }
            }

            if (lowerTri)
            {
                for (int i = 0; i < NumCols - 1; i++)
                {
                    Cols[i].Clear(i + 1, -1, alpha);
                }
            }

            if (diagonal)
            {
                var dim = MinDimension;
                for (int i = 0; i < dim; i++)
                {
                    this[i, i] = beta;
                }
            }
        }

        /// <summary>
        /// Adds <paramref name="alpha"/> to the diagonal elements.
        /// </summary>
        /// <param name="alpha">Value to add on diagonal.</param>
        public void AddDiagonal(float alpha)
        {
            var dim = MinDimension;
            for (int i = 0; i < dim; i++)
            {
                this[i, i] += alpha;
            }
        }

        /// <summary>
        /// Scales the matrix by <paramref name="alpha"/>.
        /// </summary>
        /// <param name="alpha">The scaling factor.</param>
        public void Scale(float alpha)
        {
            for (int i = 0; i < NumCols; i++)
            {
                Cols[i].Scale(alpha);
            }
        }
    }
}
