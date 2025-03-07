using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    unsafe readonly ref struct MatWrapper
    {
        readonly Matrix matrix;
    }

    [GenerateTestsForBurstCompatibility]
    [StructLayout(LayoutKind.Sequential)]
    unsafe partial struct Matrix : IDisposable
    {
        public float this[int i, int j]
        {
            get { return cols[j][i]; }
            set { var c = cols[j]; c[i] = value; }
        }

        public readonly Vector* Rows { get => rows; }
        public readonly Vector* Cols { get => cols; }

        public readonly int NumRows { get; }
        public readonly int NumCols { get; }
        public int MinDimension
        {
            get
            {
                return math.min(NumRows, NumCols);
            }
        }
        public int MaxDimension
        {
            get
            {
                return math.max(NumRows, NumCols);
            }
        }

        public static Matrix Create(in MemoryManager heap, int numRows, int numCols)
        {
            int size = numRows * numCols;

            // allocate a single block at once
            var vectorSize = UnsafeUtility.SizeOf<Vector>();
            var dataSize = size * UnsafeUtility.SizeOf<float>() + (numRows + numCols) * vectorSize;
            var data = heap.Allocate<byte>(dataSize);

            var rows = (Vector*)data;
            data += numRows * vectorSize;

            var cols = (Vector*)data;
            data += numCols * vectorSize;

            Matrix m = new Matrix(heap, (float*)data, rows, cols, numRows, numCols);

            for (int i = 0; i < numRows; i++)
            {
                m.rows[i] = Vector.Create(heap,
                    data:   m.data + i,
                    stride: numRows,
                    size:   numCols);
            }

            for (int i = 0; i < numCols; i++)
            {
                m.cols[i] = Vector.Create(heap,
                    data:   m.data + i * numRows,
                    stride: 1,
                    size:   numRows);
            }

            return m;
        }

        public void Dispose()
        {
            // rows is the beginning of the allocated memory for this matrix
            Heap.Free(rows);
        }

        public Matrix Copy()
        {
            return Copy(Heap);
        }

        public Matrix Copy(in MemoryManager heap)
        {
            var m = heap.Matrix(NumRows, NumCols);
            CopyTo(m);
            return m;
        }

        public void CopyTo(in Matrix m)
        {
            Assertions.AssertCompatibleDimensions(this.NumRows, m.NumRows);
            Assertions.AssertCompatibleDimensions(this.NumCols, m.NumCols);

            for (int i = 0; i < NumCols; i++)
            {
                Cols[i].CopyTo(m.Cols[i]);
            }
        }

        public void CopyTo(in Matrix m, Op op)
        {
            if (op == Op.None)
            {
                Assertions.AssertCompatibleDimensions(this.NumRows, m.NumRows);
                Assertions.AssertCompatibleDimensions(this.NumCols, m.NumCols);

                for (int i = 0; i < NumCols; i++)
                {
                    Cols[i].CopyTo(m.Cols[i]);
                }
            }
            else
            {
                Assertions.AssertCompatibleDimensions(this.NumRows, m.NumCols);
                Assertions.AssertCompatibleDimensions(this.NumCols, m.NumRows);

                for (int i = 0; i < NumCols; i++)
                {
                    Cols[i].CopyTo(m.Rows[i]);
                }
            }
        }

        public void Clear()
        {
            for (int i = 0; i < NumCols; i++)
                Cols[i].Clear();
        }

        public Matrix Submatrix(int startRow, int startColumn, int numRows = -1, int numColumns = -1)
        {
            if (numRows < 0 || numRows > NumRows)
            {
                numRows = NumRows - startRow;
            }

            if (numColumns < 0 || numColumns > NumCols)
            {
                numColumns = NumCols - startColumn;
            }

            var vectorSize = UnsafeUtility.SizeOf<Vector>();
            var vecData = Heap.Allocate<byte>((numRows + numColumns) * vectorSize);

            Matrix m = new Matrix(Heap,
                data:       data + startColumn + startRow * NumCols,
                rows:       (Vector*)vecData,
                cols:       (Vector*)(vecData + numRows * vectorSize),
                numRows:    numRows,
                numCols:    numColumns
            );

            var rows = m.Rows;
            for (int i = 0; i < numRows; i++)
            {
                rows[i] = Rows[startRow + i].Subvector(startColumn, numColumns);
            }

            var cols = m.Cols;
            for (int i = 0; i < numColumns; i++)
            {
                cols[i] = Cols[startColumn + i].Subvector(startRow, numRows);
            }

            return m;
        }

        public void CreateSubmatrix(in Matrix submatrix, in NativeArray<int> rowIndices, in int numRows, in NativeArray<int> columnIndices, in int numColumns)
        {
            if (numRows == 0 || numColumns == 0)
            {
                UnityEngine.Debug.LogError("Submatrix with zero dimensions was requested.");
                return;
            }

            if (numRows > rowIndices.Length || numColumns > columnIndices.Length)
            {
                UnityEngine.Debug.LogError("Insufficient array sizes.");
                return;
            }

            if (submatrix.NumRows != numRows || submatrix.NumCols != numColumns)
            {
                UnityEngine.Debug.LogError("Invalid matrix size.");
                return;
            }

            var subCols = submatrix.Cols;
            for (int c = 0; c < numColumns; ++c)
            {
                ref var subCol = ref subCols[c];
                ref var col = ref Cols[columnIndices[c]];
                for (int r = 0; r < numRows; ++r)
                {
                    subCol[r] = col[rowIndices[r]];
                }
            }
        }

        // The NoAlias attributes are a complete lie, but we ensure non-overlapping operations manually.
        [NativeDisableUnsafePtrRestriction]
        [NoAlias]
        private readonly Vector* rows;

        [NativeDisableUnsafePtrRestriction]
        [NoAlias]
        private readonly Vector* cols;

        [NativeDisableUnsafePtrRestriction]
        [NoAlias]
        private readonly float* data;

        public readonly MemoryManager Heap;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Matrix(in MemoryManager heap, float* data, Vector* rows, Vector* cols, int numRows, int numCols)
        {
            this.data = data;
            this.rows = rows;
            this.cols = cols;
            NumRows = numRows;
            NumCols = numCols;
            this.Heap = heap;
        }

        public T *ReinterpretCast<T>() where T : unmanaged
        {
            return (T*)data;
        }
    }

    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal static class MatrixStorageExtensions
    {
        public static Matrix Matrix(this in MemoryManager heap, int numRows, int numCols)
        {
            return Primitives.Matrix.Create(heap, numRows, numCols);
        }
    }
}
