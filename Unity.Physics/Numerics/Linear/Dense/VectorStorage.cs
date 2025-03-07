using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [GenerateTestsForBurstCompatibility]
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe partial struct Vector : IDisposable
    {
        private Vector(in MemoryManager heap, float* data, int stride, int size, bool ownsData = true)
        {
            this.data = data;
            Stride = stride;
            Dimension = size;
            this.Heap = heap;
            this.ownsData = ownsData;
        }

        static public Vector Create(in MemoryManager heap, int size)
        {
            return new Vector(heap, heap.Allocate<float>(size), 1, size, true);
        }

        static public Vector Create(in MemoryManager heap, float* data, int stride, int size)
        {
            return new Vector(heap, data, stride, size, false);
        }

        public void Dispose()
        {
            if (ownsData)
                Heap.Free(data);
        }

        /*public float this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return data[i * Stride]; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { data[i * Stride] = value; }
        }*/

        public ref float this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return ref data[i * Stride]; }
        }

        public Vector this[int from, int to]
        {
            get
            {
                return Subvector(from, to - from);
            }
        }

        public float[] ToArray()
        {
            var elems = new float[Dimension];
            for (int i = 0; i < Dimension; i++)
                elems[i] = this[i];
            return elems;
        }

        public void FillFromArray(float[] array)
        {
            Assertions.AssertCompatibleDimensions(Dimension, array.Length);
            for (int i = 0; i < Dimension; i++)
            {
                this[i] = array[i];
            }
        }

        public Vector Subvector(int startIndex, int size = -1)
        {
            if (size < 0 || size > Dimension)
            {
                size = Dimension - startIndex;
            }

            if (startIndex == 0 && size == Dimension)
            {
                return this;
            }

            return new Vector(Heap, data + startIndex * Stride, Stride, size, false);
        }

        public void CreateSubvector(in Vector subvector, in NativeArray<int> componentIndices, in int numComponents)
        {
            var dim = numComponents;
            if (dim == 0)
            {
                UnityEngine.Debug.LogError("Subvector with zero dimension was requested.");
                return;
            }

            if (dim > componentIndices.Length)
            {
                UnityEngine.Debug.LogError("Insufficient component index array size.");
                return;
            }

            if (subvector.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid vector dimension.");
                return;
            }

            for (int i = 0; i < dim; ++i)
            {
                subvector[i] = this[componentIndices[i]];
            }
        }

        public T* ReinterpretCast<T>() where T : unmanaged
        {
            return (T*)data;
        }

        public readonly int Dimension
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        public readonly int Stride
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        [NativeDisableUnsafePtrRestriction]
        [NoAlias]
        private unsafe readonly float* data;
        public readonly MemoryManager Heap;
        private readonly bool ownsData;
    }

    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    internal static class VectorStorageExtensions
    {
        public static Vector Vector(this in MemoryManager heap, int dimension)
        {
            return Primitives.Vector.Create(heap, dimension);
        }
    }
}
