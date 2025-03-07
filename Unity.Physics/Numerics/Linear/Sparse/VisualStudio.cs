using System;
using System.Diagnostics;
using Unity.Burst;

namespace Unity.Numerics.Linear.Sparse.Primitives
{
    [DebuggerTypeProxy(typeof(VectorDebugView))]
    partial struct Vector
    {
        [BurstDiscard]
        internal unsafe (int, float)[] ToArray()
        {
            var arr = new(int, float)[NonZeroElements];
            for (int i = 0; i < NonZeroElements; i++)
            {
                arr[i] = (indices[i], values[i]);
            }
            return arr;
        }

        internal unsafe float[] ToUncompressedArray()
        {
            var arr = new float[Dimension];
            for (int i = 0; i < NonZeroElements; i++)
            {
                arr[Indices[i]] = Values[i];
            }
            return arr;
        }
    }

    [DebuggerTypeProxy(typeof(MatrixDebugView))]
    partial struct Matrix {}

    internal class VectorDebugView
    {
        private Vector vector;
        public VectorDebugView(Vector v)
        {
            vector = v;
        }

        [DebuggerBrowsable(DebuggerBrowsableState.RootHidden)]
        public (int, float)[] Elements
        {
            get
            {
                return vector.ToArray();
            }
        }

        //[DebuggerBrowsable(DebuggerBrowsableState.RootHidden)]
        public float[] ElementsUncompressed
        {
            get
            {
                return vector.ToUncompressedArray();
            }
        }
    }

    internal class MatrixDebugView
    {
        private Matrix matrix;
        public MatrixDebugView(Matrix m)
        {
            matrix = m;
        }

        public (int, int) Dimensions
        {
            get { return (matrix.NumRows, matrix.NumCols); }
        }

        public Vector[] Columns
        {
            get
            {
                var rows = new Vector[matrix.NumCols];
                unsafe
                {
                    for (int i = 0; i < matrix.NumCols; i++)
                        rows[i] = matrix.Cols[i];
                }
                return rows;
            }
        }
    }
}
