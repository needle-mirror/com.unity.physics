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
        struct QRDecompAndReconstructJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;

            public void Execute()
            {
                var tau = heap.Vector(M.MinDimension);
                QR.Factor(M, tau);

                var R = M.Copy();

                // compute the orthogonal part
                QR.GenerateQ(M, tau);

                // Compute Q R
                M.MultiplyByTriangular(Side.Right, TriangularType.Upper, Op.None, DiagonalType.Explicit, 1.0f, R);

                R.Dispose();
                tau.Dispose();
            }
        }

        [Test]
        public void QR_Job_Decompose10x10_CompareProductToOriginal()
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
                var qrJob = new QRDecompAndReconstructJob
                {
                    heap = heap,
                    M = M
                };
                qrJob.Run();

                float maxDiff = MaxAbsoluteDiff(original, M);

                Assert.IsTrue(maxDiff < 1.0e-5f);

                original.Dispose();
                M.Dispose();
            }
        }

        void TestQR(in MemoryManager heap, int m, int n, out Matrix A, out Matrix QR)
        {
            QR = heap.Matrix(m, n);
            var generateRandomMatrixJob = new GenerateRandomMatrixJob()
            {
                Matrix = QR,
                singularMin = 0.1f,
                singularMax = 1.0f
            };

            generateRandomMatrixJob.Run();

            A = QR.Copy();
            var qrJob = new QRDecompAndReconstructJob
            {
                M = A,
                heap = heap
            };
            qrJob.Run();
        }

        [Test]
        public void QR_Job_DecomposeRandom1000x1000_CompareProductToOriginal()
        {
            if (!BurstHelper.IsBurstEnabled())
            {
                Assert.Ignore("This test variant is time consuming and is therefore only run with Burst enabled.");
            }

            using (var heap = MemoryManager.Create(1024 * 1024 * 128, Allocator.Temp))
            {
                TestQR(heap, 1000, 1000, out var A, out var QR);

                float maxDiff = MaxAbsoluteDiff(A, QR);
                Assert.IsTrue(maxDiff < 5.0e-5f);
                QR.Dispose();
                A.Dispose();
            }
        }

        [Test]
        public void QR_Job_DecomposeRandom1000x100_CompareProductToOriginal()
        {
            using (var heap = MemoryManager.Create(1024 * 1024 * 128, Allocator.Temp))
            {
                TestQR(heap, 1000, 100, out var A, out var QR);

                float maxDiff = MaxAbsoluteDiff(A, QR);
                Assert.IsTrue(maxDiff < 5.0e-5f);
                QR.Dispose();
                A.Dispose();
            }
        }
    }
}
