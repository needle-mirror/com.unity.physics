using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Entities;

namespace Unity.Physics
{
    // A flat convex collider with either 3 or 4 coplanar vertices (ie, a triangle or a quad)
    public struct PolygonCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        // Convex hull data
        // Todo: would be nice to use the actual types here but C# only likes fixed arrays of builtin types
        private unsafe fixed byte m_Vertices[sizeof(float) * 3 * 4];     // float3[4]
        private unsafe fixed byte m_FacePlanes[sizeof(float) * 4 * 2];   // Plane[2]
        private unsafe fixed byte m_Faces[4 * 2];                        // ConvexHull.Face[2]
        private unsafe fixed byte m_FaceVertexIndices[sizeof(byte) * 8]; // byte[8]

        public bool IsTriangle => Vertices.Length == 3;
        public bool IsQuad => Vertices.Length == 4;

        public BlobArray.Accessor<float3> Vertices => ConvexHull.Vertices;
        public BlobArray.Accessor<Plane> Planes => ConvexHull.Planes;

        #region Construction

        unsafe public static BlobAssetReference<Collider> CreateTriangle(float3 vertex0, float3 vertex1, float3 vertex2, CollisionFilter? filter = null, Material? material = null)
        {
            if (math.any(!math.isfinite(vertex0)) || math.any(!math.isfinite(vertex1)) || math.any(!math.isfinite(vertex2)))
            {
                throw new System.ArgumentException("Tried to create triangle collider with nan/inf vertex");
            }

            var collider = new PolygonCollider();
            collider.InitAsTriangle(vertex0, vertex1, vertex2, filter ?? CollisionFilter.Default, material ?? Material.Default);
            
            return BlobAssetReference<Collider>.Create(&collider, UnsafeUtility.SizeOf<PolygonCollider>());
        }

        // Ray casting assumes vertices are presented in clockwise order
        unsafe public static BlobAssetReference<Collider> CreateQuad(float3 vertex0, float3 vertex1, float3 vertex2, float3 vertex3, CollisionFilter? filter = null, Material? material = null)
        {
            if (math.any(!math.isfinite(vertex0)) || math.any(!math.isfinite(vertex1)) || math.any(!math.isfinite(vertex2)) || math.any(!math.isfinite(vertex3)))
            {
                throw new System.ArgumentException("Tried to create triangle collider with nan/inf vertex");
            }

            // check if vertices are co-planar
            float3 normal = math.normalize(math.cross(vertex1 - vertex0, vertex2 - vertex0));
            if (math.abs(math.dot(normal, vertex3 - vertex0)) > 1e-3f)
            {
                throw new System.ArgumentException("Vertices for quad creation are not co-planar");
            }

            PolygonCollider collider = default(PolygonCollider);
            collider.InitAsQuad(vertex0, vertex1, vertex2, vertex3, filter ?? CollisionFilter.Default, material ?? Material.Default);
            return BlobAssetReference<Collider>.Create(&collider, UnsafeUtility.SizeOf<PolygonCollider>());
        }

        internal void InitEmpty()
        {
            Init(CollisionFilter.Default, Material.Default);
            ConvexHull.VerticesBlob.Length = 0;
        }

        internal void InitAsTriangle(float3 vertex0, float3 vertex1, float3 vertex2, CollisionFilter filter, Material material)
        {
            Init(filter, material);
            SetAsTriangle(vertex0, vertex1, vertex2);
        }

        internal void InitAsQuad(float3 vertex0, float3 vertex1, float3 vertex2, float3 vertex3, CollisionFilter filter, Material material)
        {
            Init(filter, material);
            SetAsQuad(vertex0, vertex1, vertex2, vertex3);
        }

        internal unsafe void SetAsTriangle(float3 v0, float3 v1, float3 v2)
        {
            m_Header.Type = ColliderType.Triangle;
            m_Header.Version += 1;

            ConvexHull.VerticesBlob.Length = 3;
            ConvexHull.FaceVertexIndicesBlob.Length = 6;

            fixed (PolygonCollider* collider = &this)
            {
                float3* vertices = (float3*)(&collider->m_Vertices[0]);
                vertices[0] = v0;
                vertices[1] = v1;
                vertices[2] = v2;

                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                faces[0] = new ConvexHull.Face { FirstIndex = 0, NumVertices = 3, MinHalfAngleCompressed = 0xff };
                faces[1] = new ConvexHull.Face { FirstIndex = 3, NumVertices = 3, MinHalfAngleCompressed = 0xff };

                byte* index = &collider->m_FaceVertexIndices[0];
                *index++ = 0; *index++ = 1; *index++ = 2;
                *index++ = 2; *index++ = 1; *index++ = 0;
            }

            SetPlanes();
        }

        internal unsafe void SetAsQuad(float3 v0, float3 v1, float3 v2, float3 v3)
        {
            m_Header.Type = ColliderType.Quad;
            m_Header.Version += 1;

            ConvexHull.VerticesBlob.Length = 4;
            ConvexHull.FaceVertexIndicesBlob.Length = 8;

            fixed (PolygonCollider* collider = &this)
            {
                float3* vertices = (float3*)(&collider->m_Vertices[0]);
                vertices[0] = v0;
                vertices[1] = v1;
                vertices[2] = v2;
                vertices[3] = v3;

                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                faces[0] = new ConvexHull.Face { FirstIndex = 0, NumVertices = 4, MinHalfAngleCompressed = 0xff };
                faces[1] = new ConvexHull.Face { FirstIndex = 4, NumVertices = 4, MinHalfAngleCompressed = 0xff };

                byte* index = &collider->m_FaceVertexIndices[0];
                *index++ = 0; *index++ = 1; *index++ = 2; *index++ = 3;
                *index++ = 3; *index++ = 2; *index++ = 1; *index++ = 0;
            }

            SetPlanes();
        }

        private unsafe void Init(CollisionFilter filter, Material material)
        {
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            ConvexHull.ConvexRadius = 0.0f;

            fixed (PolygonCollider* collider = &this)
            {
                ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Vertices[0], ref ConvexHull.VerticesBlob);
                ConvexHull.VerticesBlob.Length = 4;

                ConvexHull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FacePlanes[0], ref ConvexHull.FacePlanesBlob);
                ConvexHull.FacePlanesBlob.Length = 2;

                ConvexHull.FacesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Faces[0], ref ConvexHull.FacesBlob);
                ConvexHull.FacesBlob.Length = 2;

                ConvexHull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceVertexIndices[0], ref ConvexHull.FaceVertexIndicesBlob);
                ConvexHull.FaceVertexIndicesBlob.Length = 8;

                // No connectivity needed
                ConvexHull.VertexEdgesBlob.Offset = 0;
                ConvexHull.VertexEdgesBlob.Length = 0;
                ConvexHull.FaceLinksBlob.Offset = 0;
                ConvexHull.FaceLinksBlob.Length = 0;
            }
        }

        private void SetPlanes()
        {
            BlobArray.Accessor<float3> hullVertices = ConvexHull.Vertices;
            float3 cross = math.cross(
                hullVertices[1] - hullVertices[0],
                hullVertices[2] - hullVertices[0]);
            float dot = math.dot(cross, hullVertices[0]);
            float invLengthCross = math.rsqrt(math.lengthsq(cross));
            Plane plane = new Plane(cross * invLengthCross, -dot * invLengthCross);

            ConvexHull.Planes[0] = plane;
            ConvexHull.Planes[1] = plane.Flipped;
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<PolygonCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set { m_Header.Filter = value; } }
        public Material Material { get => m_Header.Material; set { m_Header.Material = value; } }

        public MassProperties MassProperties
        {
            get
            {
                // TODO - the inertia computed here is incorrect. Computing the correct inertia is expensive, so it probably ought to be cached.
                // Note this is only called for top level polygon colliders, not for polygons within a mesh.
                float3 center = (ConvexHull.Vertices[0] + ConvexHull.Vertices[1] + ConvexHull.Vertices[2]) / 3.0f;
                float radiusSq = math.max(math.max(
                    math.lengthsq(ConvexHull.Vertices[0] - center),
                    math.lengthsq(ConvexHull.Vertices[1] - center)),
                    math.lengthsq(ConvexHull.Vertices[2] - center));
                return new MassProperties
                {
                    MassDistribution = new MassDistribution
                    {
                        Transform = new RigidTransform(quaternion.identity, center),
                        InertiaTensor = new float3(2.0f / 5.0f * radiusSq)
                    },
                    Volume = 0,
                    AngularExpansionFactor = math.sqrt(radiusSq)
                };
            }
        }

        public Aabb CalculateAabb()
        {
            float3 min = math.min(math.min(ConvexHull.Vertices[0], ConvexHull.Vertices[1]), math.min(ConvexHull.Vertices[2], ConvexHull.Vertices[3]));
            float3 max = math.max(math.max(ConvexHull.Vertices[0], ConvexHull.Vertices[1]), math.max(ConvexHull.Vertices[2], ConvexHull.Vertices[3]));
            return new Aabb
            {
                Min = min - new float3(ConvexHull.ConvexRadius),
                Max = max + new float3(ConvexHull.ConvexRadius)
            };
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            float3 v0 = math.rotate(transform, ConvexHull.Vertices[0]);
            float3 v1 = math.rotate(transform, ConvexHull.Vertices[1]);
            float3 v2 = math.rotate(transform, ConvexHull.Vertices[2]);
            float3 v3 = IsQuad ? math.rotate(transform, ConvexHull.Vertices[3]) : v2;

            float3 min = math.min(math.min(v0, v1), math.min(v2, v3));
            float3 max = math.max(math.max(v0, v1), math.max(v2, v3));
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
            fixed (PolygonCollider* target = &this)
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
            fixed (PolygonCollider* target = &this)
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
            fixed (PolygonCollider* target = &this)
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
            fixed (PolygonCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }
}
