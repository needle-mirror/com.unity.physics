using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics.Authoring
{
    // copied from upcoming improvements to UnityEngine.Hash128
    static class Hash128Ext
    {
        public static void Append(ref Hash128 hash128, uint val) => ShortHash4(ref hash128, val);
        public static void Append(ref Hash128 hash128, int val) => ShortHash4(ref hash128, (uint)val);
        public static unsafe void Append(ref Hash128 hash128, float val) => ShortHash4(ref hash128, *(uint*)&val);

        public static unsafe void Append<T>(ref Hash128 hash128, ref T val) where T : struct
        {
            var size = UnsafeUtility.SizeOf<T>();
            UnityEngine.Assertions.Assert.AreEqual(0, size % 4);
            var addr = (uint*)UnsafeUtility.AddressOf(ref val);
            var strides = size / 4;
            for (var i = 0; i < strides; ++i)
                Append(ref hash128, *(addr + i));
        }

        public static unsafe void Append<T>(ref Hash128 hash128, NativeArray<T> val) where T : struct
        {
            var size = UnsafeUtility.SizeOf<T>();
            UnityEngine.Assertions.Assert.AreEqual(0, size % 4);
            var addr = (uint*)val.GetUnsafeReadOnlyPtr();
            var strides = (size * val.Length) / 4;
            for (var i = 0; i < strides; ++i)
                Append(ref hash128, *(addr + i));
        }

        const ulong k_Const = 0xdeadbeefdeadbeefL;

        [StructLayout(LayoutKind.Explicit)]
        struct Union32_64
        {
            [FieldOffset(0)]
            public uint2 Values32;
            [FieldOffset(0)]
            public ulong Value64;
        }

        static void ShortHash4(ref Hash128 hash128, uint data)
        {
            ulong a = new Union32_64 { Values32 = hash128.Value.xy }.Value64;
            ulong b = new Union32_64 { Values32 = hash128.Value.zw }.Value64;
            ulong c = k_Const;
            ulong d = k_Const;
            d += 4ul << 56;
            c += data;
            ShortEnd(ref a, ref b, ref c, ref d);
            hash128.Value = new uint4(
                (uint)a,
                (uint)(a >> 32),
                (uint)b,
                (uint)(b >> 32)
            );
        }

        static void ShortEnd(ref ulong h0, ref ulong h1, ref ulong h2, ref ulong h3)
        {
            h3 ^= h2; Rot64(ref h2, 15); h3 += h2;
            h0 ^= h3; Rot64(ref h3, 52); h0 += h3;
            h1 ^= h0; Rot64(ref h0, 26); h1 += h0;
            h2 ^= h1; Rot64(ref h1, 51); h2 += h1;
            h3 ^= h2; Rot64(ref h2, 28); h3 += h2;
            h0 ^= h3; Rot64(ref h3, 9);  h0 += h3;
            h1 ^= h0; Rot64(ref h0, 47); h1 += h0;
            h2 ^= h1; Rot64(ref h1, 54); h2 += h1;
            h3 ^= h2; Rot64(ref h2, 32); h3 += h2;
            h0 ^= h3; Rot64(ref h3, 25); h0 += h3;
            h1 ^= h0; Rot64(ref h0, 63); h1 += h0;
        }

        static void Rot64(ref ulong x, int k) => x = (x << k) | (x >> (64 - k));
    }
}
