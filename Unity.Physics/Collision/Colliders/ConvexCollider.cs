using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Entities;
using UnityEngine.Assertions;

namespace Unity.Physics
{
    // A collider in the shape of an arbitrary convex hull.
    // Warning: This is just the header, it is followed by variable sized data in memory.
    // Therefore this struct must always be passed by reference, never by value.
    public struct ConvexCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        public ConvexHull ConvexHull;

        // followed by variable sized convex hull data

        #region Construction

        // Create a convex collider from the given point cloud.
        public static unsafe BlobAssetReference<Collider> Create(
            NativeArray<float3> points, float convexRadius,
            float3? scale = null, CollisionFilter? filter = null, Material? material = null)
        {
            if (convexRadius < 0.0f || !math.isfinite(convexRadius))
            {
                throw new ArgumentException("Tried to create ConvexCollider with invalid convex radius");
            }

            // Build convex hull
            int verticesCapacity = points.Length;
            int triangleCapacity = 2 * verticesCapacity;
            var vertices = (ConvexHullBuilder.Vertex*)UnsafeUtility.Malloc(verticesCapacity * sizeof(ConvexHullBuilder.Vertex), 16, Allocator.Temp);
            var triangles = (ConvexHullBuilder.Triangle*)UnsafeUtility.Malloc(triangleCapacity * sizeof(ConvexHullBuilder.Triangle), 16, Allocator.Temp);
            var builder = new ConvexHullBuilder(vertices, verticesCapacity, triangles, triangleCapacity);
            float3 s = scale ?? new float3(1);

            // Build the points' AABB and validate them
            var domain = new Aabb();
            foreach (var point in points)
            {
                if (math.any(!math.isfinite(point)))
                {
                    throw new ArgumentException("Tried to create ConvexCollider with invalid points");
                }
                domain.Include(point * s);
            }

            // Add points to the hull
            builder.IntegerSpaceAabb = domain;
            foreach (float3 point in points)
            {
                builder.AddPoint(point * s);
            }

            // TODO: shrink by convex radius

            // Build face information
            float maxAngle = 0.1f * (float)math.PI / 180.0f;
            builder.BuildFaceIndices(maxAngle);

            // Simplify the hull until it's under the max vertices requirement
            // TODO.ma this is just a failsafe. We need to think about user-controlled simplification settings & how to warn the user if their shape is too complex.
            {
                const int maxVertices = 252;    // as per Havok

                float maxSimplificationError = 1e-3f;
                int iterations = 0;
                while (builder.Vertices.PeakCount > maxVertices)
                {
                    if (iterations++ > 10) // don't loop forever
                    {
                        Assert.IsTrue(false);
                        return new BlobAssetReference<Collider>();
                    }
                    builder.SimplifyVertices(maxSimplificationError);
                    builder.BuildFaceIndices();
                    maxSimplificationError *= 2.0f;
                }
            }

            // Convert hull to compact format
            var tempHull = new TempHull(ref builder);

            // Allocate collider
            int totalSize = UnsafeUtility.SizeOf<ConvexCollider>();
            totalSize += tempHull.Vertices.Count * sizeof(float3);
            totalSize = Math.NextMultipleOf16(totalSize);  // planes currently must be aligned for Havok
            totalSize += tempHull.Planes.Count * sizeof(Plane);
            totalSize += tempHull.Faces.Count * sizeof(ConvexHull.Face);
            totalSize += tempHull.FaceVertexIndices.Count * sizeof(short);
            totalSize += tempHull.VertexEdges.Count * sizeof(ConvexHull.Edge);
            totalSize += tempHull.FaceLinks.Count * sizeof(ConvexHull.Edge);
            ConvexCollider* collider = (ConvexCollider*)UnsafeUtility.Malloc(totalSize, 16, Allocator.Temp);

            // Initialize it
            {
                UnsafeUtility.MemClear(collider, totalSize);
                collider->MemorySize = totalSize;

                collider->m_Header.Type = ColliderType.Convex;
                collider->m_Header.CollisionType = CollisionType.Convex;
                collider->m_Header.Version = 0;
                collider->m_Header.Magic = 0xff;
                collider->m_Header.Filter = filter ?? CollisionFilter.Default;
                collider->m_Header.Material = material ?? Material.Default;

                ref var hull = ref collider->ConvexHull;

                hull.ConvexRadius = convexRadius;

                // Initialize blob arrays
                {
                    byte* end = (byte*)collider + UnsafeUtility.SizeOf<ConvexCollider>();

                    hull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.VerticesBlob);
                    hull.VerticesBlob.Length = tempHull.Vertices.Count;
                    end += sizeof(float3) * tempHull.Vertices.Count;

                    end = (byte*)Math.NextMultipleOf16((ulong)end); // planes currently must be aligned for Havok

                    hull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FacePlanesBlob);
                    hull.FacePlanesBlob.Length = tempHull.Planes.Count;
                    end += sizeof(Plane) * tempHull.Planes.Count;

                    hull.FacesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FacesBlob);
                    hull.FacesBlob.Length = tempHull.Faces.Count;
                    end += sizeof(ConvexHull.Face) * tempHull.Faces.Count;

                    hull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FaceVertexIndicesBlob);
                    hull.FaceVertexIndicesBlob.Length = tempHull.FaceVertexIndices.Count;
                    end += sizeof(byte) * tempHull.FaceVertexIndices.Count;

                    hull.VertexEdgesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.VertexEdgesBlob);
                    hull.VertexEdgesBlob.Length = tempHull.VertexEdges.Count;
                    end += sizeof(ConvexHull.Edge) * tempHull.VertexEdges.Count;

                    hull.FaceLinksBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FaceLinksBlob);
                    hull.FaceLinksBlob.Length = tempHull.FaceLinks.Count;
                    end += sizeof(ConvexHull.Edge) * tempHull.FaceLinks.Count;
                }

                // Fill blob arrays
                {
                    for (int i = 0; i < tempHull.Vertices.Count; i++)
                    {
                        hull.Vertices[i] = tempHull.Vertices[i];
                        hull.VertexEdges[i] = tempHull.VertexEdges[i];
                    }

                    for (int i = 0; i < tempHull.Faces.Count; i++)
                    {
                        hull.Planes[i] = tempHull.Planes[i];
                        hull.Faces[i] = tempHull.Faces[i];
                    }

                    for (int i = 0; i < tempHull.FaceVertexIndices.Count; i++)
                    {
                        hull.FaceVertexIndices[i] = tempHull.FaceVertexIndices[i];
                        hull.FaceLinks[i] = tempHull.FaceLinks[i];
                    }
                }

                // Fill mass properties
                {
                    var massProperties = builder.ComputeMassProperties();
                    Math.DiagonalizeSymmetricApproximation(massProperties.InertiaTensor, out float3x3 orientation, out float3 inertia);

                    float maxLengthSquared = 0.0f;
                    foreach (float3 vertex in hull.Vertices)
                    {
                        maxLengthSquared = math.max(maxLengthSquared, math.lengthsq(vertex - massProperties.CenterOfMass));
                    }

                    collider->MassProperties = new MassProperties
                    {
                        MassDistribution = new MassDistribution
                        {
                            Transform = new RigidTransform(orientation, massProperties.CenterOfMass),
                            InertiaTensor = inertia
                        },
                        Volume = massProperties.Volume,
                        AngularExpansionFactor = math.sqrt(maxLengthSquared)
                    };
                }
            }

            // Copy it into blob
            var asset = BlobAssetReference<Collider>.Create(collider, totalSize);

            UnsafeUtility.Free(collider, Allocator.Temp);
            UnsafeUtility.Free(vertices, Allocator.Temp);
            UnsafeUtility.Free(triangles, Allocator.Temp);

            return asset;
        }
        
        // Temporary hull of managed arrays, used during construction
        private struct TempHull
        {
            public readonly List<float3> Vertices;
            public readonly List<Plane> Planes;
            public readonly List<ConvexHull.Face> Faces;
            public readonly List<byte> FaceVertexIndices;
            public readonly List<ConvexHull.Edge> VertexEdges;
            public readonly List<ConvexHull.Edge> FaceLinks;

            public unsafe TempHull(ref ConvexHullBuilder builder)
            {
                Vertices = new List<float3>(builder.Vertices.PeakCount);
                Faces = new List<ConvexHull.Face>(builder.NumFaces);
                Planes = new List<Plane>(builder.NumFaces);
                FaceVertexIndices = new List<byte>(builder.NumFaceVertices);
                VertexEdges = new List<ConvexHull.Edge>(builder.Vertices.PeakCount);
                FaceLinks = new List<ConvexHull.Edge>(builder.NumFaceVertices);

                // Copy the vertices
                byte* vertexIndexMap = stackalloc byte[builder.Vertices.PeakCount];
                foreach (int i in builder.Vertices.Indices)
                {
                    vertexIndexMap[i] = (byte)Vertices.Count;
                    Vertices.Add(builder.Vertices[i].Position);
                    VertexEdges.Add(new ConvexHull.Edge());  // filled below
                }

                // Copy the faces
                var edgeMap = new NativeHashMap<ConvexHull.Edge, ConvexHull.Edge>(builder.NumFaceVertices, Allocator.Temp);
                for (ConvexHullBuilder.FaceEdge hullFace = builder.GetFirstFace(); hullFace.IsValid; hullFace = builder.GetNextFace(hullFace))
                {
                    // Store the plane
                    ConvexHullBuilder.Edge firstEdge = hullFace;
                    Plane facePlane = builder.ComputePlane(firstEdge.TriangleIndex);
                    Planes.Add(facePlane);

                    // Walk the face's outer vertices & edges
                    short firstVertexIndex = (short)FaceVertexIndices.Count;
                    byte numEdges = 0;
                    float maxCosAngle = -1.0f;
                    for (ConvexHullBuilder.FaceEdge edge = hullFace; edge.IsValid; edge = builder.GetNextFaceEdge(edge))
                    {
                        byte vertexIndex = vertexIndexMap[builder.StartVertex(edge)];
                        FaceVertexIndices.Add(vertexIndex);

                        var hullEdge = new ConvexHull.Edge { FaceIndex = (short)edge.Current.TriangleIndex, EdgeIndex = (byte)edge.Current.EdgeIndex }; // will be mapped to the output hull below
                        edgeMap.TryAdd(hullEdge, new ConvexHull.Edge { FaceIndex = (short)Faces.Count, EdgeIndex = numEdges });

                        VertexEdges[vertexIndex] = hullEdge;

                        ConvexHullBuilder.Edge linkedEdge = builder.GetLinkedEdge(edge);
                        FaceLinks.Add(new ConvexHull.Edge { FaceIndex = (short)linkedEdge.TriangleIndex, EdgeIndex = (byte)linkedEdge.EdgeIndex }); // will be mapped to the output hull below

                        Plane linkedPlane = builder.ComputePlane(linkedEdge.TriangleIndex);
                        maxCosAngle = math.max(maxCosAngle, math.dot(facePlane.Normal, linkedPlane.Normal));

                        numEdges++;
                    }
                    Assert.IsTrue(numEdges >= 3);

                    // Store the face
                    Faces.Add(new ConvexHull.Face
                    {
                        FirstIndex = firstVertexIndex,
                        NumVertices = numEdges,
                        MinHalfAngle = math.acos(maxCosAngle) * 0.5f
                    });
                }

                // Remap the edges
                {
                    for (int i = 0; i < VertexEdges.Count; i++)
                    {
                        edgeMap.TryGetValue(VertexEdges[i], out ConvexHull.Edge vertexEdge);
                        VertexEdges[i] = vertexEdge;
                    }

                    for (int i = 0; i < FaceLinks.Count; i++)
                    {
                        edgeMap.TryGetValue(FaceLinks[i], out ConvexHull.Edge faceLink);
                        FaceLinks[i] = faceLink;
                    }
                }
                edgeMap.Dispose();
            }
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        public CollisionFilter Filter { get => m_Header.Filter; set => m_Header.Filter = value; }
        public Material Material { get => m_Header.Material; set => m_Header.Material = value; }
        public MassProperties MassProperties { get; private set; }

        public Aabb CalculateAabb()
        {
            return CalculateAabb(RigidTransform.identity);
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            BlobArray.Accessor<float3> vertices = ConvexHull.Vertices;
            float3 min = math.rotate(transform, vertices[0]);
            float3 max = min;
            for (int i = 1; i < vertices.Length; ++i)
            {
                float3 v = math.rotate(transform, vertices[i]);
                min = math.min(min, v);
                max = math.max(max, v);
            }
            return new Aabb
            {
                Min = min + transform.pos - new float3(ConvexHull.ConvexRadius),
                Max = max + transform.pos + new float3(ConvexHull.ConvexRadius)
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }
}
