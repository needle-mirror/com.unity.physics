using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Numerics.Linear.Sparse.Primitives
{
    [GenerateTestsForBurstCompatibility]
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe partial struct Vector : IDisposable
    {
        private Vector(float *values, int *indices, int dimension, int nonZeroElements, Allocator allocator)
        {
            this.values = values;
            this.indices = indices;
            Dimension = dimension;
            NonZeroElements = nonZeroElements;
            this.allocator = allocator;
        }

        static public Vector Create(int dimension, int nonZeroElements, Allocator allocator = Allocator.Temp)
        {
            var alloc = allocator;
            if (Jobs.LowLevel.Unsafe.JobsUtility.IsExecutingJob)
            {
                alloc = Allocator.Temp;
            }

            var values = (float*)UnsafeUtility.Malloc(nonZeroElements * UnsafeUtility.SizeOf<float>(), 16, alloc);
            var indices = (int*)UnsafeUtility.Malloc(nonZeroElements * UnsafeUtility.SizeOf<int>(), 16, alloc);
            return new Vector(
                values, indices, dimension, nonZeroElements, alloc);
        }

        static public Vector Create(float* values, int *indices, int dimension, int nonZeroElements)
        {
            return new Vector(values, indices, dimension, nonZeroElements, Allocator.None);
        }

        public void Dispose()
        {
            UnsafeUtility.Free(Values, Allocator);
            UnsafeUtility.Free(Indices, Allocator);
        }

        public readonly int Dimension
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        public readonly int NonZeroElements
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        public readonly float* Values => values;

        public readonly int* Indices => indices;

        public readonly Allocator Allocator => allocator;

        [NativeDisableUnsafePtrRestriction]
        private unsafe readonly float* values;

        [NativeDisableUnsafePtrRestriction]
        private unsafe readonly int* indices;

        private readonly Allocator allocator;
    }
}
