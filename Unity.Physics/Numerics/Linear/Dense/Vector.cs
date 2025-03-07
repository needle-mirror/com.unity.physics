using Unity.Burst;
using Unity.Collections;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    /// <summary>
    /// Dense vector class.
    /// </summary>
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal unsafe readonly partial struct Vector
    {
        // Index of the element with the greatest value
        public int MaxIndex()
        {
            int idx = 0;
            float s = this[0];
            for (int i = 0; i < Dimension; i++)
            {
                float x = this[i];
                if (x > s)
                {
                    idx = i;
                    s = x;
                }
            }
            return idx;
        }

        public float Max()
        {
            return this[MaxIndex()];
        }

        public Vector Copy()
        {
            var v = Heap.Vector(Dimension);
            CopyTo(v);
            return v;
        }

        public static void Swap(in Vector u, in Vector v)
        {
            if (u.data == v.data)
                return;

            u.Swap(v);
        }

        public void Add(in Vector v)
        {
            AddScaled(v, 1.0f);
        }

        public void Normalize()
        {
            Scale(1.0f / Norm());
        }
    }
}
