#if !UNITY_ENTITIES_0_12_OR_NEWER
#if HAVOK_PHYSICS_EXISTS
using System.Reflection; // not compatible with UNITY_DOTSPLAYER
#endif
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnsafeUtility_Collections = Unity.Collections.LowLevel.Unsafe.UnsafeUtility;

namespace Unity.Physics
{
    struct ComponentTypeHandle<T> where T : struct, IComponentData
    {
        public ArchetypeChunkComponentType<T> Value;
    }

    struct EntityTypeHandle
    {
        public ArchetypeChunkEntityType Value;
    }

    static class ArchetypeChunkExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool DidChange<T>(this ArchetypeChunk chunk, ComponentTypeHandle<T> chunkComponentType, uint version) where T : struct, IComponentData => chunk.DidChange(chunkComponentType.Value, version);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static NativeArray<Entity> GetNativeArray(this ArchetypeChunk chunk, EntityTypeHandle archetypeChunkEntityType) => chunk.GetNativeArray(archetypeChunkEntityType.Value);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static NativeArray<T> GetNativeArray<T>(this ArchetypeChunk chunk, ComponentTypeHandle<T> chunkComponentType) where T : struct, IComponentData => chunk.GetNativeArray(chunkComponentType.Value);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Has<T>(this ArchetypeChunk chunk, ComponentTypeHandle<T> chunkComponentType) where T : struct, IComponentData => chunk.Has(chunkComponentType.Value);
    }

    static class NativeStreamExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Count(this NativeStream stream) => stream.ComputeItemCount();
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Count(this NativeStream.Reader reader) => reader.ComputeItemCount();
    }

    static class UnsafeUtility
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe ref T ArrayElementAsRef<T>(byte* offsetPtr, int index) where T : struct => ref UnsafeUtilityEx.ArrayElementAsRef<T>(offsetPtr, index);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe ref T AsRef<T>(void* ptr) where T : struct => ref UnsafeUtilityEx.AsRef<T>(ptr);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void* AddressOf<T>(ref T output) where T : struct => UnsafeUtility_Collections.AddressOf(ref output);
#if HAVOK_PHYSICS_EXISTS // not compatible with UNITY_DOTSPLAYER
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetFieldOffset(FieldInfo field) => UnsafeUtility_Collections.GetFieldOffset(field);
#endif
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Free(void* memory, Allocator allocator) => UnsafeUtility_Collections.Free(memory, allocator);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void* Malloc(long size, int alignment, Allocator allocator) => UnsafeUtility_Collections.Malloc(size, alignment, allocator);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void MemClear(void* destination, long size) => UnsafeUtility_Collections.MemClear(destination, size);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void MemCpy(void* destination, void* source, long size) => UnsafeUtility_Collections.MemCpy(destination, source, size);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int SizeOf<T>() where T : struct => UnsafeUtility_Collections.SizeOf<T>();
    }

    static class SystemExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ComponentTypeHandle<T> GetComponentTypeHandle<T>(this ComponentSystemBase system, bool isReadOnly = false) where T : struct, IComponentData => new ComponentTypeHandle<T> { Value = system.GetArchetypeChunkComponentType<T>(isReadOnly) };
    }
}
#endif
