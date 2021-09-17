using System;
using System.Diagnostics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Physics
{
    struct AtomicSafetyManager : IDisposable
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        AtomicSafetyHandle m_TemporaryHandle;
#endif
        int m_IsCreated;

        public static AtomicSafetyManager Create()
        {
            var ret = new AtomicSafetyManager();
            ret.CreateTemporaryHandle();
            ret.m_IsCreated = 1;
            return ret;
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        void CreateTemporaryHandle()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            m_TemporaryHandle = AtomicSafetyHandle.Create();
            AtomicSafetyHandle.UseSecondaryVersion(ref m_TemporaryHandle);
            AtomicSafetyHandle.SetAllowSecondaryVersionWriting(m_TemporaryHandle, false);
#endif
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        void ReleaseTemporaryHandle()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckDeallocateAndThrow(m_TemporaryHandle);
            AtomicSafetyHandle.Release(m_TemporaryHandle);
#endif
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public void BumpTemporaryHandleVersions()
        {
            // TODO: There should be a better way to invalidate older versions...
            ReleaseTemporaryHandle();
            CreateTemporaryHandle();
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public void MarkNativeArrayAsReadOnly<T>(ref NativeArray<T> array)
            where T : struct
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref array, m_TemporaryHandle);
#endif
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        static void CheckCreatedAndThrow(int isCreated)
        {
            if (isCreated == 0)
                throw new InvalidOperationException("Atomic Safety Manager already disposed");
        }

        public void Dispose()
        {
            CheckCreatedAndThrow(m_IsCreated);

            ReleaseTemporaryHandle();

            m_IsCreated = 0;
        }
    }
}
