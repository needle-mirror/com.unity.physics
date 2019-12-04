using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    struct SpookyHashBuilder : IDisposable
    {
        static bool s_Initialized;

        internal static unsafe void Initialize()
        {
            if (s_Initialized)
                return;
            // ensure internal UnityEngine.SpookyHash class is initialized on the main thread before it is used anywhere
            var hash128 = new UnityEngine.Hash128();
            byte data = 0;
            HashUnsafeUtilities.ComputeHash128(UnsafeUtility.AddressOf(ref data), 1, &hash128);
            s_Initialized = true;
        }

        NativeList<byte> m_Bytes;

        public SpookyHashBuilder(Allocator allocator) => m_Bytes = new NativeList<byte>(allocator);

        public void Dispose()
        {
            if (m_Bytes.IsCreated)
                m_Bytes.Dispose();
        }

        public unsafe void Append<T>(ref T value) where T : unmanaged
        {
            var size = UnsafeUtility.SizeOf<T>();
            UnityEngine.Assertions.Assert.AreEqual(0, size % 4);
            m_Bytes.AddRange(UnsafeUtility.AddressOf(ref value), size);
        }

        public void Append<T>(in NativeArray<T> data) where T : unmanaged
        {
            if (!data.IsCreated)
                return;
            UnityEngine.Assertions.Assert.AreEqual(0, UnsafeUtility.SizeOf<T>() % 4);
            m_Bytes.AddRange(data.Reinterpret<T, byte>());
        }

        const string k_InitializedMessage =
            "SpookyHashBuilder was not initialized. Ensure you call SpookyHashBuilder.Initialize() from the main thread first.";

        public unsafe Hash128 Finish()
        {
            UnityEngine.Assertions.Assert.IsTrue(s_Initialized, k_InitializedMessage);
            var result = new UnityEngine.Hash128();
            HashUnsafeUtilities.ComputeHash128(m_Bytes.GetUnsafeReadOnlyPtr(), (ulong)m_Bytes.Length, &result);
            return new HashUnion { UnityEngine_Hash = result }.Entities_Hash;
        }

        [StructLayout(LayoutKind.Explicit)]
        struct HashUnion
        {
            [FieldOffset(0)]
            public UnityEngine.Hash128 UnityEngine_Hash;
            [FieldOffset(0)]
            public Hash128 Entities_Hash;
        }
    }
}
