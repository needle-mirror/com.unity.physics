using NUnit.Framework;
using Unity.Numerics.Linear.Dense.Primitives;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Numerics.Memory;
using Unity.Physics.Tests;

namespace Unity.Numerics.Linear.Tests
{
    public unsafe partial class TestDecompositions
    {
        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct LUDecompAndReconstructJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;
            public NativeReference<float> determinant;

            public void Execute()
            {
                var p = new NativeArray<int>(M.NumCols, Allocator.Temp);

                LU.Factor(M, ref p, out var singularRow);
                determinant.Value = LU.ComputeDeterminantFromLU(M, p);

                var U = heap.Matrix(M.NumRows, M.NumCols);
                M.CopyTo(U);

                // extract the triangular parts
                for (int i = 0; i < U.NumCols; i++)
                {
                    U.Cols[i].Clear(i + 1);
                    M.Rows[i].Clear(i + 1);
                    M[i, i] = 1.0f;
                }

                // Compute L U
                M.MultiplyByTriangular(Side.Right, TriangularType.Upper, Op.None, DiagonalType.Explicit, 1.0f, U);

                var pivot = new NativeArray<int>(p.Length, Allocator.Temp);
                for (int i = 0; i < M.NumCols; i++)
                {
                    pivot[i] = p[p[i]];
                }
                M.InterchangeRows(pivot);
                pivot.Dispose();

                /*for (int i = 0; i < M.NumCols; i++)
                {
                    M.Rows[i].Swap(M.Rows[p[p[i]]]);
                }*/

                U.Dispose();
                p.Dispose();
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct BlockLUDecompAndReconstructJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;
            public NativeReference<float> determinant;
            public int blockSize;

            public void Execute()
            {
                var p = new NativeArray<int>(M.NumCols, Allocator.Temp);

                LU.BlockFactor(M, blockSize, ref p, out var singularRow);
                determinant.Value = LU.ComputeDeterminantFromLU(M, p);

                var U = heap.Matrix(M.NumRows, M.NumCols);
                M.CopyTo(U);

                // extract the triangular parts
                for (int i = 0; i < U.NumCols; i++)
                {
                    U.Cols[i].Clear(i + 1);
                    M.Rows[i].Clear(i + 1);
                    M[i, i] = 1.0f;
                }

                // Compute L U
                M.MultiplyByTriangular(Side.Right, TriangularType.Upper, Op.None, DiagonalType.Explicit, 1.0f, U);

                var pivot = new NativeArray<int>(p.Length, Allocator.Temp);
                for (int i = 0; i < M.NumCols; i++)
                {
                    pivot[i] = p[p[i]];
                }
                M.InterchangeRows(pivot);
                pivot.Dispose();

                /*for (int i = 0; i < M.NumCols; i++)
                {
                    M.Rows[i].Swap(M.Rows[p[p[i]]]);
                }*/

                U.Dispose();
                p.Dispose();
            }
        }

        void TestLU(in MemoryManager heap, int m, int n, out Matrix A, out Matrix LU, bool blocked = false)
        {
            LU = heap.Matrix(m, n);
            var generateRandomMatrixJob = new GenerateRandomMatrixJob
            {
                Matrix = LU,
                singularMin = 0.1f,
                singularMax = 1.0f,
            };


            generateRandomMatrixJob.Run();

            var determinant = new NativeReference<float>(Allocator.TempJob);
            A = LU.Copy();

            if (!blocked)
            {
                var luJob = new LUDecompAndReconstructJob
                {
                    M = A,
                    determinant = determinant,
                    heap = heap
                };
                luJob.Run();
            }
            else
            {
                var luJob = new BlockLUDecompAndReconstructJob
                {
                    M = A,
                    determinant = determinant,
                    blockSize = A.NumRows / 4,
                    heap = heap
                };
                luJob.Run();
            }
            determinant.Dispose();
        }

        [Test]
        public void LU_Job_Decompose10x10_CompareProductToOriginal_CheckDeterminant()
        {
            using (var heap = MemoryManager.Create(16384, Allocator.Temp))
            {
                var M = Matrix.Create(heap, 10, 10);

                M.Rows[0].FillFromArray(new float[] { 4, 0, 2, -2, 4, 4, 4, -2, 0, -2 });
                M.Rows[1].FillFromArray(new float[] { 3, 2, -1, -1, -2, -1, 2, -1, 3, 1 });
                M.Rows[2].FillFromArray(new float[] { -2, 1, -1, 2, 1, 2, 2, 0, -1, 3 });
                M.Rows[3].FillFromArray(new float[] { -1, 2, 0, -2, 0, 1, 1, -1, 4, -2 });
                M.Rows[4].FillFromArray(new float[] { 3, -2, -1, 3, 1, -1, 1, 3, 1, 2 });
                M.Rows[5].FillFromArray(new float[] { 4, -2, 2, 3, 4, 0, 1, 3, 2, 0 });
                M.Rows[6].FillFromArray(new float[] { 4, 0, -2, 1, -2, 1, -1, 4, 4, -2 });
                M.Rows[7].FillFromArray(new float[] { -2, 1, 1, 3, 3, 4, 3, 0, 2, 3 });
                M.Rows[8].FillFromArray(new float[] { 2, 1, 3, 1, 2, 4, 3, 2, -1, -1 });
                M.Rows[9].FillFromArray(new float[] { 3, 2, -2, 3, -1, -2, -2, -2, 0, 1 });

                var original = M.Copy();

                var determinant = new NativeReference<float>(Allocator.TempJob);
                var luJob = new LUDecompAndReconstructJob
                {
                    heap = heap,
                    M = M,
                    determinant = determinant
                };
                luJob.Run();

                float maxDiff = MaxAbsoluteDiff(original, M);

                Assert.IsTrue(maxDiff < 1.0e-5f);
                Assert.AreEqual(determinant.Value, -634458);

                original.Dispose();
                M.Dispose();
                determinant.Dispose();
            }
        }

        [Test]
        public void LU_Job_DecomposeRandom1000x1000_CompareProductToOriginal()
        {
            if (!BurstHelper.IsBurstEnabled())
            {
                Assert.Ignore("This test variant is time consuming and is therefore only run with Burst enabled.");
            }

            using (var heap = new MemoryManager(1024 * 1024 * 128, Allocator.Temp))
            {
                TestLU(heap, 1000, 1000, out var A, out var LU);

                float maxDiff = MaxAbsoluteDiff(A, LU);
                Assert.IsTrue(maxDiff < 5.0e-5f);
                LU.Dispose();
                A.Dispose();
            }
        }

        [Test]
        public void BlockLU_Job_DecomposeRandom_CompareProductToOriginal([Values(10, 1000)] int dimension)
        {
            if (dimension == 1000 && !BurstHelper.IsBurstEnabled())
            {
                Assert.Ignore("This test variant is time consuming and is therefore only run with Burst enabled.");
            }

            using (var heap = new MemoryManager(1024 * 1024 * 128, Allocator.Temp))
            {
                TestLU(heap, dimension, dimension, out var A, out var LU, true);

                float maxDiff = MaxAbsoluteDiff(A, LU);
                Assert.IsTrue(maxDiff < 5.0e-5f);
                LU.Dispose();
                A.Dispose();
            }
        }
    }
}
