using System;
using Unity.Mathematics;

namespace Unity.Physics
{
    // The input to AABB overlap queries
    public struct OverlapAabbInput
    {
        public Aabb Aabb;
        public CollisionFilter Filter;
    }

    // A hit from an overlap query
    public struct OverlapAabbHit
    {
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
    }

    // Interface for collecting hits from overlap queries
    public interface IOverlapCollector
    {
        unsafe void AddRigidBodyIndices(int* indices, int count);
        unsafe void AddColliderKeys(ColliderKey* keys, int count);
        void PushCompositeCollider(ColliderKeyPath compositeKey);
        void PopCompositeCollider(uint numCompositeKeyBits);
    }

    // Overlap query implementations
    public static class OverlapQueries
    {
        #region AABB vs colliders

        public static unsafe void AabbCollider<T>(OverlapAabbInput input, Collider* collider, ref T collector)
            where T : struct, IOverlapCollector
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, collider->Filter))
            {
                return;
            }

            switch (collider->Type)
            {
                case ColliderType.Mesh:
                    AabbMesh(input, (MeshCollider*)collider, ref collector);
                    break;
                case ColliderType.Compound:
                    AabbCompound(input, (CompoundCollider*)collider, ref collector);
                    break;
                default:
                    throw new NotImplementedException();
            }
        }

        // Mesh
        private unsafe struct MeshLeafProcessor : BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
        {
            readonly Mesh* m_Mesh;
            readonly uint m_NumColliderKeyBits;

            const int k_MaxKeys = 512;
            fixed uint m_Keys[k_MaxKeys];  // actually ColliderKeys, but C# doesn't allow fixed arrays of structs
            int m_NumKeys;

            public MeshLeafProcessor(MeshCollider* mesh)
            {
                m_Mesh = &mesh->Mesh;
                m_NumColliderKeyBits = mesh->NumColliderKeyBits;
                m_NumKeys = 0;
            }

            public void AabbLeaf<T>(OverlapAabbInput input, int primitiveKey, ref T collector) where T : struct, IOverlapCollector
            {
                fixed (uint* keys = m_Keys)
                {
                    keys[m_NumKeys++] = new ColliderKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1)).Value;

                    Mesh.PrimitiveFlags flags = m_Mesh->GetPrimitiveFlags(primitiveKey);
                    if (Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsTrianglePair) &&
                        !Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad))
                    {
                        keys[m_NumKeys++] = new ColliderKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1) | 1).Value;
                    }
                }

                if (m_NumKeys > k_MaxKeys - 8)
                {
                    Flush(ref collector);
                }
            }

            // Flush keys to collector
            internal void Flush<T>(ref T collector) where T : struct, IOverlapCollector
            {
                fixed (uint* keys = m_Keys)
                {
                    collector.AddColliderKeys((ColliderKey*)keys, m_NumKeys);
                }
                m_NumKeys = 0;
            }
        }

        private static unsafe void AabbMesh<T>(OverlapAabbInput input, MeshCollider* mesh, ref T collector)
            where T : struct, IOverlapCollector
        {
            var leafProcessor = new MeshLeafProcessor(mesh);
            mesh->Mesh.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref collector);
            leafProcessor.Flush(ref collector);
        }


        // Compound
        private unsafe struct CompoundLeafProcessor : BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
        {
            readonly CompoundCollider* m_CompoundCollider;
            readonly uint m_NumColliderKeyBits;

            const int k_MaxKeys = 512;
            fixed uint m_Keys[k_MaxKeys];  // actually ColliderKeys, but C# doesn't allow fixed arrays of structs
            int m_NumKeys;

            public CompoundLeafProcessor(CompoundCollider* compound)
            {
                m_CompoundCollider = compound;
                m_NumColliderKeyBits = compound->NumColliderKeyBits;
                m_NumKeys = 0;
            }

            public void AabbLeaf<T>(OverlapAabbInput input, int childIndex, ref T collector) where T : struct, IOverlapCollector
            {
                ColliderKey childKey = new ColliderKey(m_NumColliderKeyBits, (uint)(childIndex));

                // Recurse if child is a composite
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[childIndex];
                if (child.Collider->CollisionType == CollisionType.Composite)
                {
                    OverlapAabbInput childInput = input;
                    childInput.Aabb = Math.TransformAabb(math.inverse(child.CompoundFromChild), input.Aabb);

                    collector.PushCompositeCollider(new ColliderKeyPath(childKey, m_NumColliderKeyBits));
                    AabbCollider(childInput, child.Collider, ref collector);
                    collector.PopCompositeCollider(m_NumColliderKeyBits);
                }
                else
                {
                    m_Keys[m_NumKeys++] = childKey.Value;
                    if (m_NumKeys > k_MaxKeys - 8)
                    {
                        Flush(ref collector);
                    }
                }
            }

            // Flush keys to collector
            internal void Flush<T>(ref T collector) where T : struct, IOverlapCollector
            {
                fixed (uint* keys = m_Keys)
                {
                    collector.AddColliderKeys((ColliderKey*)keys, m_NumKeys);
                }
                m_NumKeys = 0;
            }
        }

        private static unsafe void AabbCompound<T>(OverlapAabbInput input, CompoundCollider* compound, ref T collector)
            where T : struct, IOverlapCollector
        {
            var leafProcessor = new CompoundLeafProcessor(compound);
            compound->BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref collector);
            leafProcessor.Flush(ref collector);
        }

        #endregion
    }
}
