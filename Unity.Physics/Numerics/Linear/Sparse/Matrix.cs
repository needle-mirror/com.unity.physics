using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Numerics.Linear.Sparse.Primitives
{
    /// <summary>
    /// Sparse matrix class.
    /// </summary>
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct Matrix : IDisposable
    {
        public static Matrix Create(int numRows, int numCols, NativeArray<int> columnPtrs, NativeArray<int> rowIndices, NativeArray<float> values, Allocator alloc)
        {
            float* data = (float*)UnsafeUtility.Malloc(values.Length * UnsafeUtility.SizeOf<float>(), 16, alloc);
            UnsafeUtility.MemCpy(data, values.GetUnsafeReadOnlyPtr(), values.Length * UnsafeUtility.SizeOf<float>());

            int* idx = (int*)UnsafeUtility.Malloc(rowIndices.Length * UnsafeUtility.SizeOf<float>(), 16, alloc);
            UnsafeUtility.MemCpy(idx, rowIndices.GetUnsafeReadOnlyPtr(), rowIndices.Length * UnsafeUtility.SizeOf<int>());

            Vector* cols = (Vector*)UnsafeUtility.Malloc(numCols * UnsafeUtility.SizeOf<Vector>(), 16, alloc);

            for (int i = 0; i < numCols; i++)
            {
                int start = columnPtrs[i];
                cols[i] = Vector.Create(data + start, idx + start, numRows, columnPtrs[i + 1] - start);
            }

            return new Matrix(data, idx, cols, numRows, numCols, alloc);
        }

        public void Dispose()
        {
            UnsafeUtility.Free(Values, allocator);
            UnsafeUtility.Free(Indices, allocator);
            for (int i = 0; i < NumCols; i++)
            {
                Cols[i].Dispose();
            }
            UnsafeUtility.Free(Cols, allocator);
        }

        private Matrix(float* values, int* indices, Vector* cols, int numRows, int numCols, Allocator alloc)
        {
            this.values = values;
            this.indices = indices;
            this.cols = cols;
            this.numRows = numRows;
            this.numCols = numCols;
            allocator = alloc;
        }

        [NativeDisableUnsafePtrRestriction]
        private readonly float* values;

        [NativeDisableUnsafePtrRestriction]
        private readonly int* indices;

        [NativeDisableUnsafePtrRestriction]
        private readonly Vector* cols;

        private readonly int numCols;
        private readonly int numRows;

        private readonly Allocator allocator;

        public readonly unsafe float* Values => values;

        public readonly unsafe int* Indices => indices;

        public readonly unsafe Vector* Cols => cols;

        public readonly int NumCols => numCols;

        public int NumRows => numRows;
    }
}
