using System;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace Unity.Physics
{
    struct AtomicSafetyManager : IDisposable
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        private AtomicSafetyHandle m_TemporaryHandle;
#endif
        int m_IsCreated;
       
        public static AtomicSafetyManager Create()
        {
            var ret = new AtomicSafetyManager();
            ret.CreateTemporaryHandle();
            ret.m_IsCreated = 1;
            return ret;
        }
        
        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        void CreateTemporaryHandle()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            m_TemporaryHandle = AtomicSafetyHandle.Create();
            AtomicSafetyHandle.UseSecondaryVersion(ref m_TemporaryHandle);
            AtomicSafetyHandle.SetAllowSecondaryVersionWriting(m_TemporaryHandle, false);
#endif
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        void ReleaseTemporaryHandle()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckDeallocateAndThrow(m_TemporaryHandle);
            AtomicSafetyHandle.Release(m_TemporaryHandle);
#endif
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        public void BumpTemporaryHandleVersions()
        {
            // TODO: There should be a better way to invalidate older versions...
            ReleaseTemporaryHandle();
            CreateTemporaryHandle();
        }
        
        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        public void MarkNativeArrayAsReadOnly<T>(ref NativeArray<T> array)
            where T : struct
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref array, m_TemporaryHandle);
#endif
        }
        
        public void Dispose()
        {
            if (m_IsCreated == 0)
                throw new InvalidOperationException("Atomic Safety Manager already disposed");

            ReleaseTemporaryHandle();

            m_IsCreated = 0;
        }
    }
}