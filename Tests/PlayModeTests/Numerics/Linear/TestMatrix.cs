using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Numerics.Memory;
using Unity.Numerics.Linear.Dense.Primitives;

namespace Unity.Numerics.Linear.Tests
{
    public unsafe class TestMatrix
    {

        [Test]
        public void TestCreateSubmatrix()
        {
            using (var heap = MemoryManager.Create(16384, Allocator.Temp))
            {
                using (Matrix M = Matrix.Create(heap, 4, 6))
                {
                    M.Rows[0].FillFromArray(new float[] {1, 2, 3, 4, 5, 6});
                    M.Rows[1].FillFromArray(new float[] {7, 8, 9, 10, 11, 12});
                    M.Rows[2].FillFromArray(new float[] {13, 14, 15, 16, 17, 18});
                    M.Rows[3].FillFromArray(new float[] {19, 20, 21, 22, 23, 24});

                    var R = new NativeArray<int>(3, Allocator.Temp);
                    R.CopyFrom(new int[] {0, 1, 3});
                    var C = new NativeArray<int>(4, Allocator.Temp);
                    C.CopyFrom(new int[] {1, 2, 4, 5});

                    Matrix M_RC = Matrix.Create(heap, R.Length, C.Length);
                    try
                    {
                        M.CreateSubmatrix(M_RC, R, R.Length, C, C.Length);

                        Assert.AreEqual(R.Length, M_RC.NumRows);
                        Assert.AreEqual(C.Length, M_RC.NumCols);

                        for (int c = 0; c < C.Length; ++c)
                        {
                            var subCol = M_RC.Cols[c].ToArray();
                            var fullCol = M.Cols[C[c]].ToArray();
                            var expectedSubCol = new float[] {fullCol[R[0]], fullCol[R[1]], fullCol[R[2]]};
                            Assert.IsTrue(subCol.SequenceEqual(expectedSubCol));
                        }
                    }
                    finally
                    {
                        M_RC.Dispose();
                    }
                }
            }
        }
    }
}
