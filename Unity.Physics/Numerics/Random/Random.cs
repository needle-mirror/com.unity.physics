using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Numerics.Random
{
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal struct Random
    {
        /// <summary>Returns a uniformly random int value in the interval [min, max).</summary>
        /// <param name="min">The minimum value to generate, inclusive.</param>
        /// <param name="max">The maximum value to generate, exclusive.</param>
        /// <returns>A uniformly random integer between [min, max).</returns>
        public int NextUniformInt(int min = -int.MaxValue, int max = int.MaxValue)
        {
            return Generator.NextInt(min, max);
        }

        /// <summary>Returns a uniformly random float value in the interval [min, max).</summary>
        /// <param name="min">The minimum value to generate, inclusive.</param>
        /// <param name="max">The maximum value to generate, exclusive.</param>
        /// <returns>A uniformly random float value in the range [min, max).</returns>
        public float NextUniform(float min = 0, float max = 1)
        {
            return Generator.NextFloat(min, max);
        }

        /// <summary>
        /// Returns a Gaussian deviate.
        /// </summary>
        /// <param name="mean">The distribution mean.</param>
        /// <param name="stdDev">The standard deviation.</param>
        /// <returns>A Gaussian distributed random number.</returns>
        public float NextGaussian(float mean = 0, float stdDev = 1)
        {
            if (hasSpareGaussian)
            {
                hasSpareGaussian = false;
                return spareGaussian * stdDev + mean;
            }

            float u, v, s;
            do
            {
                u = NextUniform(-1, 1);
                v = NextUniform(-1, 1);
                s = u * u + v * v;
            }
            while (s >= 1 || s == 0);

            s = math.sqrt(-2.0f * math.log(s) / s);
            spareGaussian = v * s;
            hasSpareGaussian = true;

            return mean + stdDev * u * s;
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="seed">Random generator seed</param>
        public Random(uint seed = 0x6E624EB7u)
        {
            spareGaussian = 0;
            hasSpareGaussian = false;
            Generator = new Mathematics.Random(seed);
        }

        Mathematics.Random Generator;
        float spareGaussian;
        bool hasSpareGaussian;
    }
}
