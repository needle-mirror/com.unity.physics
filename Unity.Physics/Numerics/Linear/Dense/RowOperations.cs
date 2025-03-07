using Unity.Collections;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Matrix
    {
        public void InterchangeRows(in NativeArray<int> pivots, int startPivot = 0, int endPivot = -1, int pivotStride = 1)
        {
            if (endPivot < 0)
                endPivot = pivots.Length - 1;

            int k1 = startPivot;
            int k2 = endPivot;
            int firstPivot = k1;

            if (pivotStride < 0)
            {
                firstPivot = k1 + (k1 - k2) * pivotStride;
                k1 = endPivot;
                k2 = firstPivot;
            }
            int rowStep = pivotStride < 0 ? -1 : 1;

            var n32 = NumCols & ~0x1f;

            int pivotIndex;
            for (int i = 0; i < n32; i += 32)
            {
                pivotIndex = firstPivot;
                for (int currentRow = k1; currentRow <= k2; currentRow += rowStep, pivotIndex += pivotStride)
                {
                    var pivotRow = pivots[pivotIndex];
                    if (pivotRow != currentRow)
                    {
                        // TODO:  not currently vectorizable -- needs to be specialized for stride=1
                        for (int j = 0; j < 32; j++)
                        {
                            int col = i + j;
                            var temp = this[currentRow, col];
                            this[currentRow, col] = this[pivotRow, col];
                            this[pivotRow, col] = temp;
                        }
                    }
                }
            }

            pivotIndex = firstPivot;
            for (int currentRow = k1; currentRow <= k2; currentRow += rowStep, pivotIndex += pivotStride)
            {
                var pivotRow = pivots[pivotIndex];
                if (pivotRow != currentRow)
                {
                    for (int j = n32; j < NumCols; j++)
                    {
                        var temp = this[currentRow, j];
                        this[currentRow, j] = this[pivotRow, j];
                        this[pivotRow, j] = temp;
                    }
                }
            }
        }
    }
}
