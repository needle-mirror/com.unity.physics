namespace Unity.Numerics.Linear.Sparse.Primitives
{
    internal struct Assertions
    {
        static public void AssertCompatibleDimensions(int n, int m)
        {
            UnityEngine.Debug.Assert(n == m, "Vectors must have the same size");
        }

        static public void AssertCompatibleDimensions(Vector a, Vector b)
        {
            AssertCompatibleDimensions(a.Dimension, b.Dimension);
        }

        static public void AssertCompatibleDimensions(Matrix m, Vector a)
        {
            AssertCompatibleDimensions(m.NumCols, a.Dimension);
        }

        static public void AssertCompatibleDimensionsTranspose(Matrix m, Vector a)
        {
            AssertCompatibleDimensions(m.NumRows, a.Dimension);
        }

        static public void AssertCompatibleDimensions(Matrix m, Dense.Primitives.Vector a)
        {
            AssertCompatibleDimensions(m.NumCols, a.Dimension);
        }

        static public void AssertCompatibleDimensionsTranspose(Matrix m, Dense.Primitives.Vector a)
        {
            AssertCompatibleDimensions(m.NumRows, a.Dimension);
        }
    }
}
