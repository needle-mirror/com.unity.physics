using Unity.Numerics.Linear.Dense.Primitives;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using System;
using NUnit.Framework;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Linear.Tests
{
    public partial class TestDecompositions
    {
        static float MaxAbsoluteDiff(Matrix A, Matrix B)
        {
            Sparse.Primitives.Assertions.AssertCompatibleDimensions(A.NumRows, B.NumRows);
            Sparse.Primitives.Assertions.AssertCompatibleDimensions(A.NumCols, B.NumCols);

            float max = float.MinValue;
            for (int i = 0; i < A.NumRows; i++)
            {
                for (int j = 0; j < A.NumCols; j++)
                {
                    float diff = math.abs(A[i, j] - B[i, j]);
                    if (diff > max)
                        max = diff;
                }
            }
            return max;
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct GenerateRandomMatrixJob : IJob, IJobFor
        {
            public Matrix Matrix;
            public uint randomSeed;
            public float singularMin, singularMax;

            public int blockDim;

            public void Execute()
            {
                new Random.Random(randomSeed + 1).GenerateRandomGeneralMatrix(Matrix.Heap, Matrix, singularMin, singularMax);
            }

            public void Execute(int index)
            {
                var rand = new Random.Random(randomSeed + (uint)index + 1);
                int sx = index % blockDim;
                int sy = index / blockDim;

                int startRow = sx * Matrix.NumRows / blockDim;
                int startCol = sy * Matrix.NumCols / blockDim;

                sx++; sy++;
                int numRows = -startRow + ((sx == blockDim) ? Matrix.NumRows : sx * Matrix.NumRows / blockDim);
                int numCols = -startCol + ((sy == blockDim) ? Matrix.NumCols : sy * Matrix.NumCols / blockDim);

                using (var block = Matrix.Submatrix(startRow, startCol, numRows, numCols))
                {
                    rand.GenerateRandomGeneralMatrix(Matrix.Heap, block, singularMin, singularMax);
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct GenerateRandomSymmetricPositiveDefiniteMatrixJob : IJob
        {
            public Matrix Matrix;
            public uint randomSeed;

            public void Execute()
            {
                new Random.Random(randomSeed + 1).GenerateRandomSymmetricPositiveDefiniteMatrix(Matrix.Heap, Matrix);
            }
        }
    }
}
