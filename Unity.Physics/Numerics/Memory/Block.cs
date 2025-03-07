using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Burst;
using Unity.Burst.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Numerics.Memory
{
    [GenerateTestsForBurstCompatibility]
    internal partial struct MemoryManager
    {
        [System.Flags]
        public enum Usage : long
        {
            Nothing = 0,
            Free = 0x1,
            Available = 0x2,
            Allocatable = Free | Available
        }

        [BurstCompile]
        [StructLayout(LayoutKind.Explicit, Size = 16)]
        [DebuggerTypeProxy(typeof(BlockDebugView))]
        public unsafe partial struct Block
        {
            /// <summary>
            /// Packed size and flags
            /// </summary>
            [FieldOffset(0)] public long content;

            /// <summary>
            /// The "this" pointer
            /// </summary>
            [FieldOffset(8)] private Block* self;

            public static Block* Construct(void* ptr)
            {
                var block = (Block*)ptr;
                block->self = (Block*)UnsafeUtility.AddressOf(ref block->content);
                return block;
            }

            public Block* Self => self;

            public readonly byte* Data => ((byte*)self) + HeaderSize;

            public unsafe Block* Next
            {
                get => (Block*)(Data + Size);
            }

            public bool Mark()
            {
                return Mark(Usage.Nothing);
            }

            public bool Mark(Usage required)
            {
                var cmp = content | (long)(required | Usage.Available);
                return Interlocked.CompareExchange(ref content, cmp & ~(long)Usage.Available, cmp) == cmp;
            }

            public bool Unmark()
            {
                var cmp = content & ~(long)Usage.Available;
                return Interlocked.CompareExchange(ref content, cmp | (long)Usage.Available, cmp) == cmp;
            }

            public bool IsValid
            {
                get => content != 0;
            }

            public bool IsFree { get => (content & (long)Usage.Free) != 0; }

            public bool IsLocked { get => (content & (long)Usage.Available) == 0; }

            public bool IsAllocatable { get => (content & (long)Usage.Allocatable) == (long)Usage.Allocatable; }

            public long Size
            {
                get => GetSize();
                set => SetSize(value);
            }

            public Usage Flags
            {
                get => (Usage)(content & 3);
                set => content = (content & ~3) | ((long)value & 3);
            }

            internal void Alloc([AssumeRange(0, long.MaxValue)] long size, Usage flags = 0)
            {
                Size = size;
                Flags = flags;
            }

            internal static readonly long HeaderSize = UnsafeUtility.SizeOf<Block>();

            public int UnsafeCount
            {
                get
                {
                    int count = 0;
                    for (var block = self; block->IsValid; block = block->Next)
                        count++;
                    return count;
                }
            }

            [return : AssumeRange(0, long.MaxValue)]
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private long GetSize()
            {
                return content & ~3;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private void SetSize([AssumeRange(0, long.MaxValue)] long size)
            {
                content = (content & 3) | (size & ~3);
            }
        }
    }
}
