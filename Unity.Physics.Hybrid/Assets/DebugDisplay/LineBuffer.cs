using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    unsafe internal struct UnsafeArray<T> : IDisposable where T : unmanaged
    {
        T* m_Pointer;
        int m_Length;
        internal T* GetUnsafePtr() { return m_Pointer;  }
        internal UnsafeArray(int length)
        {
            var size = UnsafeUtility.SizeOf<T>() * length;
            var alignment = UnsafeUtility.AlignOf<T>();
            m_Pointer = (T*)UnsafeUtility.Malloc(size, alignment, Allocator.Persistent);
            m_Length = length;
        }

        public void Dispose()
        {
            UnsafeUtility.Free(m_Pointer, Allocator.Persistent);
        }

        internal int Length { get => m_Length; }
        internal ref T this[int index]
        {
            get { return ref UnsafeUtility.AsRef<T>(m_Pointer + index); }
        }

        internal NativeArray<T> ToNativeArray()
        {
            var array = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>(m_Pointer, m_Length, Allocator.Invalid);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref array, AtomicSafetyHandle.GetTempUnsafePtrSliceHandle());
#endif
            return array;
        }
    }

    unsafe internal struct LineBuffer : IDisposable
    {
        const int kMaxLines = 100000;

        internal struct Instance
        {
            internal float4 m_Begin;
            internal float4 m_End;
        }

        internal UnsafeArray<Instance> m_Instance;

        internal void Initialize()
        {
            m_Instance = new UnsafeArray<Instance>(kMaxLines);
        }

        internal void SetLine(float3 begin, float3 end, ColorIndex colorIndex, int index)
        {
            m_Instance[index] = new Instance
            {
                m_Begin = new float4(begin.x, begin.y, begin.z, colorIndex.value),
                m_End = new float4(end.x, end.y, end.z, colorIndex.value)
            };
        }

        internal void ClearLine(int index)
        {
            m_Instance[index] = new Instance {};
        }

        public  void Dispose()
        {
            m_Instance.Dispose();
        }

        internal Unit AllocateAll()
        {
            return new Unit(m_Instance.Length);
        }
    }
}
