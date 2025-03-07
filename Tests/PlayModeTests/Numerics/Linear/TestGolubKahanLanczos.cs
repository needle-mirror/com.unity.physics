using NUnit.Framework;
using Unity.Numerics.Linear.Dense.Primitives;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Numerics.Memory;
using Unity.Physics.Tests;

namespace Unity.Numerics.Linear.Tests
{
    public unsafe partial class TestDecompositions
    {
        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct GolubKahanLanczosDecompAndReconstructJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;

            public void Execute()
            {
                var tauQ = heap.Vector(M.MinDimension);
                var tauP = heap.Vector(M.MinDimension);
                var diag = heap.Vector(M.MinDimension);
                var offDiag = heap.Vector(M.MinDimension - 1);

                GolubKahanLanczos.Factor(M, diag, offDiag, tauQ, tauP);

                var Q = heap.Matrix(M.NumRows, M.NumRows);
                var P = heap.Matrix(M.NumCols, M.NumCols);

                if (M.NumRows >= M.NumCols)
                {
                    Q.Clear();
                    using (var subQ = Q.Submatrix(0, 0, M.NumRows, M.MinDimension))
                        M.CopyTo(subQ);
                    using (var subM = M.Submatrix(0, 0, M.NumCols, M.NumCols))
                        subM.CopyTo(P);
                }
                else
                {
                    P.Clear();
                    using (var subP = P.Submatrix(0, 0, M.MinDimension, M.NumCols))
                        M.CopyTo(subP);
                    using (var subM = M.Submatrix(0, 0, M.NumRows, M.NumRows))
                        subM.CopyTo(Q);
                }

                // compute the transformation
                GolubKahanLanczos.GenerateQ(Q, tauQ, M.NumCols);
                GolubKahanLanczos.GenerateP(P, tauP, M.NumRows);

                // Get the bidiagonal matrix
                var B = M.Copy();
                B.Clear();
                if (M.NumRows >= M.NumCols)
                {
                    // upper bidiagonal
                    for (int i = 0; i < diag.Dimension; i++)
                        B[i, i] = diag[i];
                    for (int i = 0; i < offDiag.Dimension; i++)
                        B[i, i + 1] = offDiag[i];
                }
                else
                {
                    // lower bidiagonal
                    for (int i = 0; i < diag.Dimension; i++)
                        B[i, i] = diag[i];
                    for (int i = 0; i < offDiag.Dimension; i++)
                        B[i + 1, i] = offDiag[i];
                }

                // Compute Q B P^T
                {
                    // Q^T A P^T = B
                    var temp = B.Copy();
                    temp.ScaleAndAddProduct(Op.None, Op.None, 1, Q, B, 0);
                    M.ScaleAndAddProduct(Op.None, Op.None, 1, temp, P, 0);
                    temp.Dispose();
                }

                B.Dispose();
                Q.Dispose();
                P.Dispose();
                diag.Dispose();
                offDiag.Dispose();
                tauQ.Dispose();
                tauP.Dispose();
            }
        }

        void TestGolubKahanLanczos(in MemoryManager heap, int m, int n, out Matrix A, out Matrix QBP)
        {
            QBP = heap.Matrix(m, n);
            var generateRandomMatrixJob = new GenerateRandomMatrixJob()
            {
                Matrix = QBP,
                singularMin = 0.1f,
                singularMax = 1.0f,

                blockDim = 4
            };

            generateRandomMatrixJob.Run(16);

            A = QBP.Copy();
            var biredJob = new GolubKahanLanczosDecompAndReconstructJob
            {
                M = A,
                heap = heap
            };
            biredJob.Run();
        }

        [Test]
        public void GolubKahanLanczos_Job_Decompose4x3_CompareProductToOriginal()
        {
            using (var heap = MemoryManager.Create(16384, Allocator.Temp))
            {
                var M = heap.Matrix(4, 3);
                M.Rows[0].FillFromArray(new float[] { 1, 2, 3 });
                M.Rows[1].FillFromArray(new float[] { 4, 5, 6 });
                M.Rows[2].FillFromArray(new float[] { 7, 8, 9 });
                M.Rows[3].FillFromArray(new float[] { 10, 11, 12 });

                var check = M.Copy();

                var biredJob = new GolubKahanLanczosDecompAndReconstructJob
                {
                    M = M,
                    heap = heap
                };
                biredJob.Run();

                float maxDiff = MaxAbsoluteDiff(check, M);

                Assert.IsTrue(maxDiff < 1.0e-5f);
                M.Dispose();
                check.Dispose();
            }
        }

        [Test]
        public void GolubKahanLanczos_Job_Decompose3x4_CompareProductToOriginal()
        {
            using (var heap = MemoryManager.Create(1024 * 1024 * 128, Allocator.Temp))
            {
                var M = heap.Matrix(3, 4);
                M.Cols[0].FillFromArray(new float[] { 1, 2, 3 });
                M.Cols[1].FillFromArray(new float[] { 4, 5, 6 });
                M.Cols[2].FillFromArray(new float[] { 7, 8, 9 });
                M.Cols[3].FillFromArray(new float[] { 10, 11, 12 });

                var check = M.Copy();

                var biredJob = new GolubKahanLanczosDecompAndReconstructJob
                {
                    M = M,
                    heap = heap
                };
                biredJob.Run();

                float maxDiff = MaxAbsoluteDiff(check, M);

                Assert.IsTrue(maxDiff < 1.0e-5f);
                M.Dispose();
                check.Dispose();
            }
        }

        [Test]
        public void GolubKahanLanczos_Job_DecomposeRandom1000x1000_CompareProductToOriginal()
        {
            if (!BurstHelper.IsBurstEnabled())
            {
                Assert.Ignore("This test variant is time consuming and is therefore only run with Burst enabled.");
            }

            using (var heap = MemoryManager.Create(1024 * 1024 * 128, Allocator.Temp))
            {
                TestGolubKahanLanczos(heap, 1000, 1000, out var A, out var GKL);

                float maxDiff = MaxAbsoluteDiff(A, GKL);
                Assert.IsTrue(maxDiff < 5.0e-5f);
                GKL.Dispose();
                A.Dispose();
            }
        }
    }
}
