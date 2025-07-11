using NUnit.Framework;
using Unity.Collections;
using Unity.Numerics.Memory;
using Unity.Numerics.Linear.Dense.Primitives;

namespace Unity.Numerics.Linear.Tests
{
    public unsafe class TestVector
    {
        static void validateSubvector(in Vector subvector, in Vector fullvector, in NativeArray<int> indices)
        {
            for (int i = 0; i < indices.Length; ++i)
            {
                Assert.AreEqual(subvector[i], fullvector[indices[i]]);
            }
        }

        [Test]
        public void TestCreateSubvector()
        {
            using (var heap = MemoryManager.Create(16384, Allocator.Temp))
            {
                var idx1 = new NativeArray<int>(4, Allocator.Temp);
                idx1.CopyFrom(new int[] {0, 1, 3, 9});
                var idx2 = new NativeArray<int>(6, Allocator.Temp);
                idx2.CopyFrom(new int[] {2, 3, 4, 6, 7, 8});

                Vector v = Vector.Create(heap, 10);
                Vector vSub1 = Vector.Create(heap, idx1.Length);
                Vector vSub2 = Vector.Create(heap, idx2.Length);
                try
                {
                    v.FillFromArray(new float[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

                    v.CreateSubvector(vSub1, idx1, idx1.Length);
                    v.CreateSubvector(vSub2, idx2, idx2.Length);

                    Assert.AreEqual(idx1.Length, vSub1.Dimension);
                    Assert.AreEqual(idx2.Length, vSub2.Dimension);

                    validateSubvector(vSub1, v, idx1);
                    validateSubvector(vSub2, v, idx2);
                }
                finally
                {
                    v.Dispose();
                    vSub1.Dispose();
                    vSub2.Dispose();
                }
            }
        }
    }
}
