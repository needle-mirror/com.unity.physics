using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Entities;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // A collider containing instances of other colliders
    public struct CompoundCollider : ICompositeCollider
    {
        private ColliderHeader m_Header;

        // A child collider, within the same blob as the compound collider.
        // Warning: This references the collider via a relative offset, so must always be passed by reference.
        public struct Child
        {
            public RigidTransform CompoundFromChild;
            internal int m_ColliderOffset;

            public unsafe Collider* Collider
            {
                get
                {
                    fixed (int* offsetPtr = &m_ColliderOffset)
                    {
                        return (Collider*)((byte*)offsetPtr + *offsetPtr);
                    }
                }
            }
        }

        // The array of child colliders
        private BlobArray m_ChildrenBlob;
        public int NumChildren => m_ChildrenBlob.Length;
        public BlobArray.Accessor<Child> Children => new BlobArray.Accessor<Child>(ref m_ChildrenBlob);

        // The bounding volume hierarchy
        // TODO: Store node filters array too, for filtering queries within the BVH
        private BlobArray m_BvhNodesBlob;
        public unsafe BoundingVolumeHierarchy BoundingVolumeHierarchy
        {
            get
            {
                fixed (BlobArray* blob = &m_BvhNodesBlob)
                {
                    var firstNode = (BoundingVolumeHierarchy.Node*)((byte*)&(blob->Offset) + blob->Offset);
                    return new BoundingVolumeHierarchy(firstNode, nodeFilters: null);
                }
            }
        }

        #region Construction

        // Input to Create()
        public struct ColliderBlobInstance
        {
            public RigidTransform CompoundFromChild;
            public BlobAssetReference<Collider> Collider;
        }

        // Create a compound collider containing an array of other colliders.
        // The source colliders are copied into the compound, so that it becomes one blob.
        public static unsafe BlobAssetReference<Collider> Create(NativeArray<ColliderBlobInstance> children)
        {
            if (children.Length == 0)
                throw new ArgumentException();

            // Get the total required memory size for the compound plus all its children,
            // and the combined filter of all children
            // TODO: Verify that the size is enough
            int totalSize = Math.NextMultipleOf16(UnsafeUtility.SizeOf<CompoundCollider>());
            CollisionFilter filter = children[0].Collider.Value.Filter;
            foreach (var child in children)
            {
                totalSize += Math.NextMultipleOf16(child.Collider.Value.MemorySize);
                filter = CollisionFilter.CreateUnion(filter, child.Collider.Value.Filter);
            }
            totalSize += (children.Length + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches) * UnsafeUtility.SizeOf<BoundingVolumeHierarchy.Node>();

            // Allocate the collider
            var compoundCollider = (CompoundCollider*)UnsafeUtility.Malloc(totalSize, 16, Allocator.Temp);
            UnsafeUtility.MemClear(compoundCollider, totalSize);
            compoundCollider->m_Header.Type = ColliderType.Compound;
            compoundCollider->m_Header.CollisionType = CollisionType.Composite;
            compoundCollider->m_Header.Version = 0;
            compoundCollider->m_Header.Magic = 0xff;
            compoundCollider->m_Header.Filter = filter;

            // Initialize children array
            Child* childrenPtr = (Child*)((byte*)compoundCollider + UnsafeUtility.SizeOf<CompoundCollider>());
            compoundCollider->m_ChildrenBlob.Offset = (int)((byte*)childrenPtr - (byte*)(&compoundCollider->m_ChildrenBlob.Offset));
            compoundCollider->m_ChildrenBlob.Length = children.Length;
            byte* end = (byte*)childrenPtr + UnsafeUtility.SizeOf<Child>() * children.Length;
            end = (byte*)Math.NextMultipleOf16((ulong)end);

            // Copy children
            for (int i = 0; i < children.Length; i++)
            {
                Collider* collider = (Collider*)children[i].Collider.GetUnsafePtr();
                UnsafeUtility.MemCpy(end, collider, collider->MemorySize);
                childrenPtr[i].m_ColliderOffset = (int)(end - (byte*)(&childrenPtr[i].m_ColliderOffset));
                childrenPtr[i].CompoundFromChild = children[i].CompoundFromChild;
                end += Math.NextMultipleOf16(collider->MemorySize);
            }

            // Build mass properties
            compoundCollider->MassProperties = compoundCollider->BuildMassProperties();

            // Build bounding volume
            int numNodes = compoundCollider->BuildBoundingVolume(out NativeArray<BoundingVolumeHierarchy.Node> nodes);
            int bvhSize = numNodes * UnsafeUtility.SizeOf<BoundingVolumeHierarchy.Node>();
            compoundCollider->m_BvhNodesBlob.Offset = (int)(end - (byte*)(&compoundCollider->m_BvhNodesBlob.Offset));
            compoundCollider->m_BvhNodesBlob.Length = numNodes;
            UnsafeUtility.MemCpy(end, nodes.GetUnsafeReadOnlyPtr(), bvhSize);
            end += bvhSize;
            nodes.Dispose();

            // Copy to blob asset
            int usedSize = (int)(end - (byte*)compoundCollider);
            UnityEngine.Assertions.Assert.IsTrue(usedSize < totalSize);
            compoundCollider->MemorySize = usedSize;
            var blob = BlobAssetReference<Collider>.Create(compoundCollider, usedSize);
            UnsafeUtility.Free(compoundCollider, Allocator.Temp);
            
            return blob;
        }

        private unsafe int BuildBoundingVolume(out NativeArray<BoundingVolumeHierarchy.Node> nodes)
        {
            // Create inputs
            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < NumChildren; ++i)
            {
                points[i] = new BoundingVolumeHierarchy.PointAndIndex { Position = Children[i].CompoundFromChild.pos, Index = i };
                aabbs[i] = Children[i].Collider->CalculateAabb(Children[i].CompoundFromChild);
            }

            // Build BVH
            // Todo: cleanup, better size of nodes array
            nodes = new NativeArray<BoundingVolumeHierarchy.Node>(2 + NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory)
            {
                [0] = BoundingVolumeHierarchy.Node.Empty,
                [1] = BoundingVolumeHierarchy.Node.Empty
            };

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodes);

            points.Dispose();
            aabbs.Dispose();

            return numNodes;
        }

        // Build mass properties representing a union of all the child collider mass properties.
        // This assumes a uniform density for all children, and returns a mass properties for a compound of unit mass.
        private unsafe MassProperties BuildMassProperties()
        {
            BlobArray.Accessor<Child> children = Children;

            // Calculate combined center of mass
            float3 combinedCenterOfMass = float3.zero;
            float combinedVolume = 0.0f;
            for (int i = 0; i < NumChildren; ++i)
            {
                ref Child child = ref children[i];
                var mp = child.Collider->MassProperties;

                // weight this contribution by its volume (=mass)
                combinedCenterOfMass += math.transform(child.CompoundFromChild, mp.MassDistribution.Transform.pos) * mp.Volume;
                combinedVolume += mp.Volume;
            }
            if (combinedVolume > 0.0f)
            {
                combinedCenterOfMass /= combinedVolume;
            }

            // Calculate combined inertia, relative to new center of mass
            float3x3 combinedOrientation;
            float3 combinedInertiaTensor;
            {
                float3x3 combinedInertiaMatrix = float3x3.zero;
                for (int i = 0; i < NumChildren; ++i)
                {
                    ref Child child = ref children[i];
                    var mp = child.Collider->MassProperties;

                    // rotate inertia into compound space
                    float3x3 temp = math.mul(mp.MassDistribution.InertiaMatrix, new float3x3(math.inverse(child.CompoundFromChild.rot)));
                    float3x3 inertiaMatrix = math.mul(new float3x3(child.CompoundFromChild.rot), temp);

                    // shift it to be relative to the new center of mass
                    float3 shift = math.transform(child.CompoundFromChild, mp.MassDistribution.Transform.pos) - combinedCenterOfMass;
                    float3 shiftSq = shift * shift;
                    var diag = new float3(shiftSq.y + shiftSq.z, shiftSq.x + shiftSq.z, shiftSq.x + shiftSq.y);
                    var offDiag = new float3(shift.x * shift.y, shift.y * shift.z, shift.z * shift.x) * -1.0f;
                    inertiaMatrix.c0 += new float3(diag.x, offDiag.x, offDiag.z);
                    inertiaMatrix.c1 += new float3(offDiag.x, diag.y, offDiag.y);
                    inertiaMatrix.c2 += new float3(offDiag.z, offDiag.y, diag.z);

                    // weight by its proportional volume (=mass)
                    inertiaMatrix *= mp.Volume / (combinedVolume + float.Epsilon);

                    combinedInertiaMatrix += inertiaMatrix;
                }

                // convert to box inertia
                Math.DiagonalizeSymmetricApproximation(
                    combinedInertiaMatrix, out combinedOrientation, out combinedInertiaTensor);
            }

            // Calculate combined angular expansion factor, relative to new center of mass
            float combinedAngularExpansionFactor = 0.0f;
            for (int i = 0; i < NumChildren; ++i)
            {
                ref Child child = ref children[i];
                var mp = child.Collider->MassProperties;

                float3 shift = math.transform(child.CompoundFromChild, mp.MassDistribution.Transform.pos) - combinedCenterOfMass;
                float expansionFactor = mp.AngularExpansionFactor + math.length(shift);
                combinedAngularExpansionFactor = math.max(combinedAngularExpansionFactor, expansionFactor);
            }
            
            return new MassProperties
            {
                MassDistribution = new MassDistribution
                {
                    Transform = new RigidTransform(combinedOrientation, combinedCenterOfMass),
                    InertiaTensor = combinedInertiaTensor
                },
                Volume = combinedVolume,
                AngularExpansionFactor = combinedAngularExpansionFactor
            };
        }

        #endregion

        #region ICompositeCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        public CollisionFilter Filter { get => m_Header.Filter; set => m_Header.Filter = value; }
        public MassProperties MassProperties { get; private set; }

        public Aabb CalculateAabb()
        {
            return CalculateAabb(RigidTransform.identity);
        }

        public unsafe Aabb CalculateAabb(RigidTransform transform)
        {
            // TODO: Store a convex hull wrapping all the children, and use that to calculate tighter AABBs?
            return Math.TransformAabb(transform, BoundingVolumeHierarchy.Domain);
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (CompoundCollider* target = &this)
            {
                return RaycastQueries.RayCollider(input, (Collider*)target, ref collector);
            }
        }

        // Cast another collider against this one.
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public unsafe bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            fixed (CompoundCollider* target = &this)
            {
                return ColliderCastQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (CompoundCollider* target = &this)
            {
                return DistanceQueries.PointCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from another collider to this one.
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (CompoundCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        public uint NumColliderKeyBits => (uint)(32 - math.lzcnt(NumChildren));

        public unsafe bool GetChild(ref ColliderKey key, out ChildCollider child)
        {
            if (key.PopSubKey(NumColliderKeyBits, out uint childIndex))
            {
                ref Child c = ref Children[(int)childIndex];
                child = new ChildCollider(c.Collider) { TransformFromChild = c.CompoundFromChild };
                return true;
            }

            child = new ChildCollider();
            return false;
        }

        public unsafe bool GetLeaf(ColliderKey key, out ChildCollider leaf)
        {
            fixed (CompoundCollider* root = &this)
            {
                return Collider.GetLeafCollider((Collider*)root, RigidTransform.identity, key, out leaf);
            }
        }

        public unsafe void GetLeaves<T>(ref T collector) where T : struct, ILeafColliderCollector
        {
            for (uint i = 0; i < NumChildren; i++)
            {
                ref Child c = ref Children[(int)i];
                ColliderKey childKey = new ColliderKey(NumColliderKeyBits, i);
                if (c.Collider->CollisionType == CollisionType.Composite)
                {
                    collector.PushCompositeCollider(new ColliderKeyPath(childKey, NumColliderKeyBits), new MTransform(c.CompoundFromChild), out MTransform worldFromCompound);
                    c.Collider->GetLeaves(ref collector);
                    collector.PopCompositeCollider(NumColliderKeyBits, worldFromCompound);
                }
                else
                {
                    var child = new ChildCollider(c.Collider) { TransformFromChild = c.CompoundFromChild };
                    collector.AddLeaf(childKey, ref child);
                }
            }
        }

        #endregion
    }
}
