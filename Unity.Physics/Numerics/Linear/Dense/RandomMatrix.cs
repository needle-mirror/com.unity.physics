using Unity.Burst;
using Unity.Collections;
using Unity.Numerics.Memory;
using UnityEngine.Assertions;
using R = Unity.Numerics.Random;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal static class RandomExtensions
    {
        /// <summary>
        /// Generates a vector with uniformly distributed elements
        /// </summary>
        /// <param name="v">The vector to be filled out</param>
        /// <param name="min">Lower distribution bound</param>
        /// <param name="max">Upper distribution bound</param>
        public static void GenerateRandomUniformVector(this R.Random r, in Vector v, float min, float max)
        {
            for (int i = 0; i < v.Dimension; i++)
            {
                v[i] = r.NextUniform(min, max);
            }
        }

        /// <summary>
        /// Generates a random vector uniformly distributed on the sphere.
        /// </summary>
        /// <param name="v">The vector to be filled out</param>
        public static void GenerateRandomDirectionVector(this R.Random r, in Vector v, bool normalize = true)
        {
            for (int i = 0; i < v.Dimension; i++)
            {
                v[i] = r.NextGaussian();
            }

            if (normalize)
            {
                v.Normalize();
            }
        }

        /// <summary>
        /// Generates a random orthogonal matrix
        /// </summary>
        /// <param name="M">The matrix to be filled out</param>
        public static Matrix GenerateRandomOrthogonalMatrix(this R.Random r, in MemoryManager heap, in Matrix M)
        {
            UnityEngine.Debug.Assert(M.NumRows == M.NumCols);

            // TODO:  figure out which version is faster
#if false
            for (int i = 0; i < M.NumRows; i++)
            {
                r.GenerateRandomDirectionVector(M.Cols[i], false);
                for (int k = 0; k < i; k++)
                {
                    // Gram-Schmidt orthogonalization
                    // u.(v - u(u.v)) = u.v - u.u (u.v) = 0
                    float d = M.Cols[i].Dot(M.Cols[k]);
                    M.Cols[i].AddScaled(M.Cols[k], -d);
                }
                M.Cols[i].Normalize();
            }
#else
            M.SetDiagonal(0, 1, true, true, true);

            var v = heap.Vector(M.NumRows - 1);
            for (int i = 0; i < M.NumRows; i++)
            {
                r.GenerateRandomDirectionVector(v, true);
                Householder.ApplyReflector(M, Side.Left, v, 1.0f);
            }
            v.Dispose();
#endif
            return M;
        }

        /// <summary>
        /// Generates a random diagonal matrix with the diagonal elements uniformly distributed in the prescribed range.
        /// </summary>
        /// <param name="M">The matrix to fill out</param>
        /// <param name="min">Lower bound of the elements</param>
        /// <param name="max">Upper bound of the elements</param>
        public static void GenerateRandomDiagonalMatrix(this R.Random r, in Matrix M, float min = 0, float max = 1)
        {
            var dim = M.MinDimension;

            M.Clear();
            for (int i = 0; i < dim; i++)
            {
                M[i, i] = r.NextUniform(min, max);
            }
        }

        /// <summary>
        /// Generates a random general matrix
        /// </summary>
        /// <param name="M">The matrix to be filled out</param>
        public static void GenerateRandomGeneralMatrix(this R.Random r, in MemoryManager heap, in Matrix M, float singularMin = 0.1f, float singularMax = 1.0f)
        {
            // compute U D V^T for a random diagonal D with the specified singular radius,
            // and random orthogonal U, V computed as series of random Householder rotations.
            r.GenerateRandomDiagonalMatrix(M, singularMin, singularMax);

            int maxDim = M.MaxDimension;

            using (var v = heap.Vector(maxDim))
            {
                for (int i = 0; i < M.NumRows; i++)
                {
                    r.GenerateRandomDirectionVector(v, true);
                    Householder.ApplyReflector(M, Side.Right, v, 1.0f);
                }
            }

            using (var v = heap.Vector(maxDim))
            {
                for (int i = 0; i < M.NumCols; i++)
                {
                    r.GenerateRandomDirectionVector(v, true);
                    Householder.ApplyReflector(M, Side.Left, v, 1.0f);
                }
            }
        }

        public static void GenerateRandomSymmetricPositiveDefiniteMatrix(this R.Random r, in MemoryManager heap, in Matrix M)
        {
            // matrix must be square
            Assert.IsTrue(M.NumRows == M.NumCols);

            // generate a random general matrix
            GenerateRandomGeneralMatrix(r, heap, M, singularMin: 0.1f, singularMax: 1.0f);
            // ensure matrix is symmetric
            using var M2 = M.Copy();
            M.AddScaled(Op.Transpose, M2, 1);
            M.Scale(0.5f);
            // ensure matrix is positive definite
            M.AddDiagonal(2f);
        }
    }
}
