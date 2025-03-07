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
        struct LQDecompAndReconstructJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;

            public void Execute()
            {
                var tau = heap.Vector(M.MinDimension);
                LQ.Factor(M, tau);

                var L = M.Copy();

                // compute the orthogonal part
                LQ.GenerateQ(M, tau);

                // Compute L Q
                M.MultiplyByTriangular(Side.Left, TriangularType.Lower, Op.None, DiagonalType.Explicit, 1.0f, L);

                L.Dispose();
                tau.Dispose();
            }
        }

        void TestLQ(in MemoryManager heap, int m, int n, out Matrix A, out Matrix LQ)
        {
            LQ = heap.Matrix(m, n);
            var generateRandomMatrixJob = new GenerateRandomMatrixJob()
            {
                Matrix = LQ,
                singularMin = 0.1f,
                singularMax = 1.0f,

                blockDim = 4
            };

            /*  TODO: memory manager is not thread safe.
                Since the GenerateRandomMatrixJob uses the MemoryManager, we can not schedule in using a parallel for-loop and
                are using the serial for-loop version instead.
            */
            //generateRandomMatrixJob.ScheduleParallel(16, 1, default).Complete();
            generateRandomMatrixJob.Schedule(16, default).Complete();

            A = LQ.Copy();
            var lqJob = new LQDecompAndReconstructJob
            {
                M = A,
                heap = heap
            };
            lqJob.Run();
        }

        [Test]
        public void LQ_Job_Decompose10x10_CompareProductToOriginal()
        {
            Assert.DoesNotThrow(() =>
            {
                using (var heap = MemoryManager.Create(65536, Allocator.Temp))
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

                    var lqJob = new LQDecompAndReconstructJob
                    {
                        heap = heap,
                        M = M
                    };
                    lqJob.Run();

                    float maxDiff = MaxAbsoluteDiff(original, M);
                    original.Dispose();
                    M.Dispose();

                    Assert.IsTrue(maxDiff < 1.0e-5f);
                }
            });
        }

        [Test]
        public void LQ_Job_DecomposeRandom1000x1000_CompareProductToOriginal()
        {
            if (!BurstHelper.IsBurstEnabled())
            {
                Assert.Ignore("This test variant is time consuming and is therefore only run with Burst enabled.");
            }

            using (var heap = new MemoryManager(1024 * 1024 * 128, Allocator.Temp))
            {
                TestLQ(heap, 1000, 1000, out var A, out var LQ);

                float maxDiff = MaxAbsoluteDiff(A, LQ);
                Assert.IsTrue(maxDiff < 5.0e-5f);
                LQ.Dispose();
                A.Dispose();
            }
        }
    }
}
