using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    // A collider in the shape of a box
    public struct BoxCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        // Convex hull data
        // Todo: would be nice to use the actual types here but C# only likes fixed arrays of builtin types..
        private unsafe fixed byte m_Vertices[sizeof(float) * 3 * 8];         // float3[8]
        private unsafe fixed byte m_FacePlanes[sizeof(float) * 4 * 6];       // Plane[6]
        private unsafe fixed byte m_Faces[4 * 6];                            // ConvexHull.Face[6]
        private unsafe fixed byte m_FaceVertexIndices[sizeof(byte) * 24];    // byte[24]
        private unsafe fixed byte m_VertexEdges[4 * 8];                      // ConvexHull.Edge[8]
        private unsafe fixed byte m_FaceLinks[4 * 24];                       // ConvexHull.Edge[24]

        // Box parameters
        private float3 m_Center;
        private quaternion m_Orientation;
        private float3 m_Size;

        public float3 Center { get => m_Center; set { m_Center = value; Update(); } }
        public quaternion Orientation { get => m_Orientation; set { m_Orientation = value; Update(); } }
        public float3 Size { get => m_Size; set { m_Size = value; Update(); } }
        public float ConvexRadius { get => ConvexHull.ConvexRadius; set { ConvexHull.ConvexRadius = value; Update(); } }

        #region Construction

        unsafe public static BlobAssetReference<Collider> Create(float3 center, quaternion orientation, float3 size, float convexRadius, CollisionFilter? filter = null, Material? material = null)
        {
            if (math.any(!math.isfinite(center)))
            {
                throw new System.ArgumentException("Tried to create BoxCollider with inf/nan center");
            }
            if (math.any(size <= 0) || math.any(!math.isfinite(size)))
            {
                throw new System.ArgumentException("Tried to create BoxCollider with a negative, zero, inf or nan size component");
            }
            if (convexRadius < 0 || !math.isfinite(convexRadius))
            {
                throw new System.ArgumentException("Tried to create BoxCollider with negative, zero, inf or nan convex radius");
            }
            if (math.any(convexRadius + convexRadius > size))
            {
                throw new System.ArgumentException("Tried to create BoxCollider with radius greater than half extent");
            }

            var collider = default(BoxCollider);
            collider.Init(center, orientation, size, convexRadius, filter ?? CollisionFilter.Default, material ?? Material.Default);
            return BlobAssetReference<Collider>.Create(&collider, sizeof(BoxCollider));
        }


        internal unsafe void Init(float3 center, quaternion orientation, float3 size, float convexRadius, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Box;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            // Build immutable convex data
            fixed (BoxCollider* collider = &this)
            {
                ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Vertices[0], ref ConvexHull.VerticesBlob);
                ConvexHull.VerticesBlob.Length = 8;

                ConvexHull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FacePlanes[0], ref ConvexHull.FacePlanesBlob);
                ConvexHull.FacePlanesBlob.Length = 6;

                ConvexHull.FacesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Faces[0], ref ConvexHull.FacesBlob.Offset);
                ConvexHull.FacesBlob.Length = 6;

                ConvexHull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceVertexIndices[0], ref ConvexHull.FaceVertexIndicesBlob);
                ConvexHull.FaceVertexIndicesBlob.Length = 24;

                ConvexHull.VertexEdgesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_VertexEdges[0], ref ConvexHull.VertexEdgesBlob);
                ConvexHull.VertexEdgesBlob.Length = 8;

                ConvexHull.FaceLinksBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceLinks[0], ref ConvexHull.FaceLinksBlob);
                ConvexHull.FaceLinksBlob.Length = 24;

                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                faces[0] = new ConvexHull.Face { FirstIndex = 0, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[1] = new ConvexHull.Face { FirstIndex = 4, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[2] = new ConvexHull.Face { FirstIndex = 8, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[3] = new ConvexHull.Face { FirstIndex = 12, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[4] = new ConvexHull.Face { FirstIndex = 16, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[5] = new ConvexHull.Face { FirstIndex = 20, NumVertices = 4, MinHalfAngleCompressed = 0x80 };

                byte* index = &collider->m_FaceVertexIndices[0];
                byte[] faceVertexIndices = new byte[24] { 2, 6, 4, 0, 1, 5, 7, 3, 1, 0, 4, 5, 7, 6, 2, 3, 3, 2, 0, 1, 7, 5, 4, 6 };
                for (int i = 0; i < 24; i++)
                {
                    *index++ = faceVertexIndices[i];
                }

                ConvexHull.Edge* vertexEdge = (ConvexHull.Edge*)(&collider->m_VertexEdges[0]);
                short[] vertexEdgeValuePairs = new short[16] { 4, 2, 2, 0, 4, 1, 4, 0, 5, 2, 5, 1, 0, 1, 5, 0 };
                for (int i = 0; i < 8; i++)
                {
                    *vertexEdge++ = new ConvexHull.Edge
                    {
                        FaceIndex = vertexEdgeValuePairs[2 * i],
                        EdgeIndex = (byte)vertexEdgeValuePairs[2 * i + 1]
                    };
                }

                ConvexHull.Edge* faceLink = (ConvexHull.Edge*)(&collider->m_FaceLinks[0]);
                short[] faceLinkValuePairs = new short[48] {
                    3, 1, 5, 2, 2, 1, 4, 1, 2, 3, 5, 0, 3, 3, 4, 3, 4, 2, 0, 2, 5, 1, 1, 0,
                    5, 3, 0, 0, 4, 0, 1, 2, 3, 2, 0, 3, 2, 0, 1, 3, 1, 1, 2, 2, 0, 1, 3, 0
                };
                for (int i = 0; i < 24; i++)
                {
                    *faceLink++ = new ConvexHull.Edge
                    {
                        FaceIndex = faceLinkValuePairs[2 * i],
                        EdgeIndex = (byte)faceLinkValuePairs[2 * i + 1]
                    };
                }
            }

            // Build mutable convex data
            m_Center = center;
            m_Orientation = orientation;
            m_Size = size;
            ConvexHull.ConvexRadius = convexRadius;
            Update();
        }

        // Update the vertices and planes to match the current box properties
        private unsafe void Update()
        {
            m_Header.Version++;

            fixed (BoxCollider* collider = &this)
            {
                var transform = new RigidTransform(m_Orientation, m_Center);

                // TODO: clamp to avoid extents <= 0
                float3 he = m_Size * 0.5f - ConvexHull.ConvexRadius; // half extents

                float3* vertices = (float3*)(&collider->m_Vertices[0]);
                vertices[0] = math.transform(transform, new float3(he.x, he.y, he.z));
                vertices[1] = math.transform(transform, new float3(-he.x, he.y, he.z));
                vertices[2] = math.transform(transform, new float3(he.x, -he.y, he.z));
                vertices[3] = math.transform(transform, new float3(-he.x, -he.y, he.z));
                vertices[4] = math.transform(transform, new float3(he.x, he.y, -he.z));
                vertices[5] = math.transform(transform, new float3(-he.x, he.y, -he.z));
                vertices[6] = math.transform(transform, new float3(he.x, -he.y, -he.z));
                vertices[7] = math.transform(transform, new float3(-he.x, -he.y, -he.z));

                Plane* planes = (Plane*)(&collider->m_FacePlanes[0]);
                planes[0] = Math.TransformPlane(transform, new Plane(new float3(1, 0, 0), -he.x));
                planes[1] = Math.TransformPlane(transform, new Plane(new float3(-1, 0, 0), -he.x));
                planes[2] = Math.TransformPlane(transform, new Plane(new float3(0, 1, 0), -he.y));
                planes[3] = Math.TransformPlane(transform, new Plane(new float3(0, -1, 0), -he.y));
                planes[4] = Math.TransformPlane(transform, new Plane(new float3(0, 0, 1), -he.z));
                planes[5] = Math.TransformPlane(transform, new Plane(new float3(0, 0, -1), -he.z));
            }
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<BoxCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set { m_Header.Filter = value; } }
        public Material Material { get => m_Header.Material; set { m_Header.Material = value; } }

        public MassProperties MassProperties => new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = new RigidTransform(m_Orientation, m_Center),
                InertiaTensor = new float3(
                    (m_Size.y * m_Size.y + m_Size.z * m_Size.z) / 12.0f,
                    (m_Size.x * m_Size.x + m_Size.z * m_Size.z) / 12.0f,
                    (m_Size.x * m_Size.x + m_Size.y * m_Size.y) / 12.0f)
            },
            Volume = m_Size.x * m_Size.y * m_Size.z,
            AngularExpansionFactor = math.length(m_Size * 0.5f - ConvexRadius)
        };

        public Aabb CalculateAabb()
        {
            return CalculateAabb(RigidTransform.identity);
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            float3 centerInB = math.transform(transform, m_Center);

            quaternion worldFromBox = math.mul(transform.rot, m_Orientation);
            float3 x = math.mul(worldFromBox, new float3(m_Size.x * 0.5f, 0, 0));
            float3 y = math.mul(worldFromBox, new float3(0, m_Size.y * 0.5f, 0));
            float3 z = math.mul(worldFromBox, new float3(0, 0, m_Size.z * 0.5f));
            float3 halfExtentsInB = math.abs(x) + math.abs(y) + math.abs(z);

            return new Aabb
            {
                Min = centerInB - halfExtentsInB,
                Max = centerInB + halfExtentsInB
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (BoxCollider* target = &this)
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
            fixed (BoxCollider* target = &this)
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
            fixed (BoxCollider* target = &this)
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
            fixed (BoxCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }
}
