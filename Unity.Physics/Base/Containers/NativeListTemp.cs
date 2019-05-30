using System;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

#pragma warning disable 649

namespace Unity.Collections
{
    static unsafe class NativeListUtilityTemp
    {
        struct NativeListDataTemp
        {
            public void*                            buffer;
            public int                              length;
            public int                              capacity;
        }

        [NativeContainer]
        struct NativeListTemp
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            public AtomicSafetyHandle m_Safety;

            [NativeSetClassTypeToNullOnSchedule]
            public DisposeSentinel m_DisposeSentinel;
#endif
            [NativeDisableUnsafePtrRestriction]
            public NativeListDataTemp* m_ListData;

            public Allocator m_Allocator;
        }

        public static JobHandle DisposeHotFix<T>(ref  NativeList<T> list, JobHandle dep)
            where T : struct
        {
            var ptr = UnsafeUtility.AddressOf(ref list);
            ref var listRef = ref UnsafeUtilityEx.AsRef<NativeListTemp>(ptr);

            var jobData = new DisposeListJob { ListTemp = listRef };

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            jobData.Safety = listRef.m_Safety;
            DisposeSentinel.Clear(ref listRef.m_DisposeSentinel);
            listRef.m_ListData = null;
            listRef.m_Safety = new AtomicSafetyHandle();
#endif
            dep = jobData.Schedule(dep);

            return dep;
        }

        static void DeallocateList(NativeListDataTemp* data, Allocator allocator)
        {
            if (data != null)
            {
                UnsafeUtility.Free(data->buffer, allocator);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                data->buffer = (void*)0xDEADF00D;
#endif
                UnsafeUtility.Free(data, allocator);
            }
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            else
                throw new Exception("NativeList has yet to be allocated or has been dealocated!");
#endif
        }

        [BurstCompile]
        struct DisposeListJob : IJob
        {
            public NativeListTemp ListTemp;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            public AtomicSafetyHandle Safety;
#endif
            public void Execute()
            {
                DeallocateList(ListTemp.m_ListData, ListTemp.m_Allocator);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.Release(Safety);
#endif
            }
        }
    }
}
