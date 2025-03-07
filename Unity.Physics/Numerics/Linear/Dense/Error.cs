namespace Unity.Numerics.Linear.Dense.Primitives
{
    internal struct Assertions
    {
        static public void AssertCompatibleDimensions(int n, int m)
        {
            UnityEngine.Debug.Assert(n == m, "Vectors must have the same size");
        }

        static public void AssertCompatibleDimensions(in Vector a, in Vector b)
        {
            AssertCompatibleDimensions(a.Dimension, b.Dimension);
        }

        static public void AssertCompatibleDimensions(in Matrix m, in Vector a)
        {
            AssertCompatibleDimensions(m.NumCols, a.Dimension);
        }

        static public void AssertCompatibleDimensionsTranspose(in Matrix m, in Vector a)
        {
            AssertCompatibleDimensions(m.NumRows, a.Dimension);
        }
    }
}
