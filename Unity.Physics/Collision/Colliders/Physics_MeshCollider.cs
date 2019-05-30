using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Entities;

namespace Unity.Physics
{
    // A collider representing a mesh comprised of triangles and quads.
    // Warning: This is just the header, it is followed by variable sized data in memory.
    // Therefore this struct must always be passed by reference, never by value.
    public struct MeshCollider : ICompositeCollider
    {
        private ColliderHeader m_Header;
        private Aabb m_Aabb;
        public Mesh Mesh;

        // followed by variable sized mesh data

        #region Construction

        // Create a mesh collider asset from a set of triangles
        public static unsafe BlobAssetReference<Collider> Create(float3[] vertices, int[] indices, CollisionFilter? filter = null, Material? material = null)
        {
            int numVertices = vertices.Length;
            int numIndices = indices.Length;
            int numTriangles = numIndices / 3;

            // Copy vertices
            float3[] tempVertices = new float3[numVertices];
            Array.Copy(vertices, tempVertices, numVertices);

            // Copy indices
            int[] tempIndices = new int[numIndices];
            for (int iTriangle = 0; iTriangle < numTriangles; iTriangle++)
            {
                int iIndex0 = iTriangle * 3;
                int iIndex1 = iIndex0 + 1;
                int iIndex2 = iIndex0 + 2;
                tempIndices[iIndex0] = indices[iIndex0];
                tempIndices[iIndex1] = indices[iIndex1];
                tempIndices[iIndex2] = indices[iIndex2];
            }

            // Build connectivity and primitives
            List<MeshConnectivityBuilder.Primitive> primitives = null;
            {
                MeshConnectivityBuilder.WeldVertices(tempIndices, ref tempVertices);
                var connectivity = new MeshConnectivityBuilder(tempIndices, tempVertices);
                primitives = connectivity.EnumerateQuadDominantGeometry(tempIndices, tempVertices);
            }

            // Build bounding volume hierarchy
            var nodes = new NativeArray<BoundingVolumeHierarchy.Node>(primitives.Count * 2 + 1, Allocator.Temp);
            int numNodes = 0;
            {
                // Prepare data for BVH
                var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(primitives.Count, Allocator.Temp);
                var aabbs = new NativeArray<Aabb>(primitives.Count, Allocator.Temp);

                for (int i = 0; i < primitives.Count; i++)
                {
                    MeshConnectivityBuilder.Primitive p = primitives[i];

                    // Skip degenerate triangles
                    if (MeshConnectivityBuilder.IsTriangleDegenerate(p.Vertices[0], p.Vertices[1], p.Vertices[2]))
                    {
                        continue;
                    }

                    aabbs[i] = Aabb.CreateFromPoints(p.Vertices);
                    points[i] = new BoundingVolumeHierarchy.PointAndIndex
                    {
                        Position = aabbs[i].Center,
                        Index = i
                    };
                }

                var bvh = new BoundingVolumeHierarchy(nodes);
                bvh.Build(points, aabbs, out numNodes, useSah: true);

                points.Dispose();
                aabbs.Dispose();
            }

            // Build mesh sections
            BoundingVolumeHierarchy.Node* nodesPtr = (BoundingVolumeHierarchy.Node*)nodes.GetUnsafePtr();
            List<MeshBuilder.TempSection> sections = MeshBuilder.BuildSections(nodesPtr, numNodes, primitives);

            // Allocate collider
            int meshDataSize = Mesh.CalculateMeshDataSize(numNodes, sections);
            int totalColliderSize = Math.NextMultipleOf(sizeof(MeshCollider), 16) + meshDataSize;
            MeshCollider* meshCollider = (MeshCollider*)UnsafeUtility.Malloc(totalColliderSize, 16, Allocator.Temp);

            // Initialize it
            {
                UnsafeUtility.MemClear(meshCollider, totalColliderSize);
                meshCollider->MemorySize = totalColliderSize;

                meshCollider->m_Header.Type = ColliderType.Mesh;
                meshCollider->m_Header.CollisionType = CollisionType.Composite;
                meshCollider->m_Header.Version += 1;
                meshCollider->m_Header.Magic = 0xff;

                ref var mesh = ref meshCollider->Mesh;

                mesh.Init(nodesPtr, numNodes, sections, filter ?? CollisionFilter.Default, material ?? Material.Default);

                // Calculate combined filter
                meshCollider->m_Header.Filter = mesh.Sections[0].Filters[0];
                for (int i = 0; i < mesh.Sections.Length; ++i)
                {
                    foreach (CollisionFilter f in mesh.Sections[i].Filters)
                    {
                        meshCollider->m_Header.Filter = CollisionFilter.CreateUnion(meshCollider->m_Header.Filter, f);
                    }
                }

                meshCollider->m_Aabb = meshCollider->Mesh.BoundingVolumeHierarchy.Domain;
                meshCollider->NumColliderKeyBits = meshCollider->Mesh.NumColliderKeyBits;
            }

            nodes.Dispose();

            // Copy collider into blob
            var blob = BlobAssetReference<Collider>.Create(meshCollider, totalColliderSize);
            UnsafeUtility.Free(meshCollider, Allocator.Temp);
            return blob;
        }

        #endregion

        #region ICompositeCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        public CollisionFilter Filter => m_Header.Filter;

        public MassProperties MassProperties
        {
            get
            {
                // Rough approximation based on AABB
                float3 size = m_Aabb.Extents;
                return new MassProperties
                {
                    MassDistribution = new MassDistribution
                    {
                        Transform = new RigidTransform(quaternion.identity, m_Aabb.Center),
                        InertiaTensor = new float3(
                            (size.y * size.y + size.z * size.z) / 12.0f,
                            (size.x * size.x + size.z * size.z) / 12.0f,
                            (size.x * size.x + size.y * size.y) / 12.0f)
                    },
                    Volume = 0,
                    AngularExpansionFactor = math.length(m_Aabb.Extents) * 0.5f
                };
            }
        }

        public Aabb CalculateAabb()
        {
            return m_Aabb;
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            // TODO: Store a convex hull wrapping the mesh, and use that to calculate tighter AABBs?
            return Math.TransformAabb(transform, m_Aabb);
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (MeshCollider* target = &this)
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
            fixed (MeshCollider* target = &this)
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
            fixed (MeshCollider* target = &this)
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
            fixed (MeshCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        public uint NumColliderKeyBits { get; private set; }

        public bool GetChild(ref ColliderKey key, out ChildCollider child)
        {
            if (key.PopSubKey(NumColliderKeyBits, out uint subKey))
            {
                int primitiveKey = (int)(subKey >> 1);
                int polygonIndex = (int)(subKey & 1);

                Mesh.GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter, out Material material);

                if (Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad))
                {
                    child = new ChildCollider(vertices[0], vertices[1], vertices[2], vertices[3], filter, material);
                }
                else
                {
                    child = new ChildCollider(vertices[0], vertices[1 + polygonIndex], vertices[2 + polygonIndex], filter, material);
                }

                return true;
            }

            child = new ChildCollider();
            return false;
        }

        public bool GetLeaf(ColliderKey key, out ChildCollider leaf)
        {
            return GetChild(ref key, out leaf);
        }

        public unsafe void GetLeaves<T>(ref T collector) where T : struct, ILeafColliderCollector
        {
            var polygon = new PolygonCollider();
            polygon.InitEmpty();
            if (Mesh.GetFirstPolygon(out uint meshKey, ref polygon))
            {
                do
                {
                    var leaf = new ChildCollider((Collider*)&polygon, RigidTransform.identity);
                    collector.AddLeaf(new ColliderKey(NumColliderKeyBits, meshKey), ref leaf);
                }
                while (Mesh.GetNextPolygon(meshKey, out meshKey, ref polygon));
            }
        }

        #endregion
    }
}
