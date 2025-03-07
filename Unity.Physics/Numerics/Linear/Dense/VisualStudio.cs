using System.Linq;
using System.Diagnostics;
using System.Globalization;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [DebuggerTypeProxy(typeof(VectorDebugView))]
    partial struct Vector {}

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
        public float[] Elements
        {
            get
            {
                return vector.ToArray();
            }
        }

        public string MaximaString
        {
            get
            {
                string s;
                unsafe
                {
                    s = "covect([" + string.Join(",", vector.ToArray()
                        .Select(f => f.ToString(CultureInfo.InvariantCulture))) + "])";
                }
                return s;
            }
        }

        public string MatlabString
        {
            get
            {
                var s = "[";
                unsafe
                {
                    s += vector
                        .ToArray()
                        .Select(f => f.ToString(CultureInfo.InvariantCulture))
                        .Aggregate("", (acc, s) => acc + s + "; ");
                    s = s.Substring(0, s.Length - 2) + "]";
                } return s;
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

        public Vector[] Rows
        {
            get
            {
                var rows = new Vector[matrix.NumRows];

                unsafe
                {
                    for (int i = 0; i < matrix.NumRows; i++)
                        rows[i] = matrix.Rows[i];
                }
                return rows;
            }
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

        public string MaximaString
        {
            get
            {
                var s = "matrix(";
                unsafe
                {
                    for (int i = 0; i < matrix.NumRows; i++)
                    {
                        s += matrix.Rows[i]
                            .ToArray()
                            .Select(f => f.ToString(CultureInfo.InvariantCulture))
                            .Aggregate("[", (acc, s) => acc + s + ",");
                        s = s.Substring(0, s.Length - 1) + "], ";
                    }
                }
                s = s.Substring(0, s.Length - 2) + ")";
                return s;
            }
        }

        public string MatlabString
        {
            get
            {
                var s = "[";
                unsafe
                {
                    for (int i = 0; i < matrix.NumRows; i++)
                    {
                        s += matrix.Rows[i]
                            .ToArray()
                            .Select(f => f.ToString(CultureInfo.InvariantCulture))
                            .Aggregate("", (acc, s) => acc + s + ",");
                        s = s.Substring(0, s.Length - 1) + "; ";
                    }
                }
                s = s.Substring(0, s.Length - 2) + "]";
                return s;
            }
        }
    }
}
