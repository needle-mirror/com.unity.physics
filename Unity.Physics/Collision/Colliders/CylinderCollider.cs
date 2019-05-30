using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    // A collider in the shape of a cylinder
    public struct CylinderCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        // Convex hull data, sized for the maximum allowed number of cylinder faces
        // Todo: would be nice to use the actual types here but C# only likes fixed arrays of builtin types..
        private const int k_MaxSideCount = 32;
        private unsafe fixed byte m_Vertices[sizeof(float) * 3 * 2 * k_MaxSideCount];
        private unsafe fixed byte m_FacePlanes[sizeof(float) * 4 * (2 + k_MaxSideCount)];
        private unsafe fixed byte m_Faces[4 * (2 + k_MaxSideCount)];
        private unsafe fixed byte m_FaceVertexIndices[sizeof(byte) * 6 * k_MaxSideCount];

        // Cylinder parameters
        private float3 m_Center;
        private quaternion m_Orientation;
        private float m_Height;
        private float m_Radius;
        private int m_SideCount;

        public float3 Center { get => m_Center; set { m_Center = value; Update(); } }
        public quaternion Orientation { get => m_Orientation; set { m_Orientation = value; Update(); } }
        public float Height { get => m_Height; set { m_Height = value; Update(); } }
        public float Radius { get => m_Radius; set { m_Radius = value; Update(); } }
        public int SideCount { get => m_SideCount; set { m_SideCount = math.clamp(value, 2, k_MaxSideCount); Update(); } }
        public float ConvexRadius { get => ConvexHull.ConvexRadius; set { ConvexHull.ConvexRadius = value; Update(); } }

        #region Construction

        unsafe public static BlobAssetReference<Collider> Create(
            float3 center, float height, float radius, quaternion orientation, float convexRadius,
            CollisionFilter? filter = null, Material? material = null)
        {
            if (height < 0 || !math.isfinite(height))
            {
                throw new System.ArgumentOutOfRangeException("Tried to create CylinderCollider with a negative, zero, inf or nan height");
            }
            if (radius < 0 || !math.isfinite(radius))
            {
                throw new System.ArgumentOutOfRangeException("Tried to create CylinderCollider with a negative, zero, inf or nan radius");
            }
            if (convexRadius < 0 || !math.isfinite(convexRadius))
            {
                throw new System.ArgumentOutOfRangeException("Tried to create CylinderCollider with negative, zero, inf or nan convex radius");
            }
            if (convexRadius > radius || convexRadius + convexRadius > height)
            {
                throw new System.ArgumentOutOfRangeException("Tried to create CylinderCollider with convex radius larger than shape dimensions");
            }

            var collider = default(CylinderCollider);
            collider.Init(center, height, radius, orientation, convexRadius, filter ?? CollisionFilter.Default, material ?? Material.Default);
            return BlobAssetReference<Collider>.Create(&collider, sizeof(CylinderCollider));
        }

        private unsafe void Init(float3 center, float height, float radius, quaternion orientation, float convexRadius, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Cylinder;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;
            MemorySize = UnsafeUtility.SizeOf<CylinderCollider>();

            // Build immutable convex data
            fixed (CylinderCollider* collider = &this)
            {
                ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Vertices[0], ref ConvexHull.VerticesBlob);
                ConvexHull.VerticesBlob.Length = 0;

                ConvexHull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FacePlanes[0], ref ConvexHull.FacePlanesBlob);
                ConvexHull.FacePlanesBlob.Length = 0;

                ConvexHull.FacesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Faces[0], ref ConvexHull.FacesBlob);
                ConvexHull.FacesBlob.Length = 0;

                ConvexHull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceVertexIndices[0], ref ConvexHull.FaceVertexIndicesBlob);
                ConvexHull.FaceVertexIndicesBlob.Length = 0;

                // No connectivity
                ConvexHull.VertexEdgesBlob.Offset = 0;
                ConvexHull.VertexEdgesBlob.Length = 0;
                ConvexHull.FaceLinksBlob.Offset = 0;
                ConvexHull.FaceLinksBlob.Length = 0;
            }

            // Build mutable convex data
            m_Center = center;
            m_Orientation = orientation;
            m_Height = height;
            m_Radius = radius;
            m_SideCount = 20;
            ConvexHull.ConvexRadius = convexRadius;
            Update();
        }

        // Update the vertices and faces to match the current cylinder properties
        private unsafe void Update()
        {
            m_Header.Version++;

            ConvexHull.VerticesBlob.Length = m_SideCount * 2;
            ConvexHull.FacePlanesBlob.Length = m_SideCount + 2;
            ConvexHull.FacesBlob.Length = m_SideCount + 2;
            ConvexHull.FaceVertexIndicesBlob.Length = m_SideCount * 6;

            var transform = new RigidTransform(m_Orientation, m_Center);
            var radius = math.max(m_Radius - ConvexRadius, 0);
            var halfHeight = math.max(m_Height * 0.5f - ConvexRadius, 0);
            if (m_SideCount > k_MaxSideCount)
                throw new System.ArgumentOutOfRangeException();
            
            fixed (CylinderCollider* collider = &this)
            {
                // vertices
                float3* vertices = (float3*)(&collider->m_Vertices[0]);
                var arcStep = 2f * (float)math.PI / m_SideCount;
                for (var i = 0; i < m_SideCount; i++)
                {
                    var x = math.cos(arcStep * i) * radius;
                    var y = math.sin(arcStep * i) * radius;
                    vertices[i] = math.transform(transform, new float3(x, y, -halfHeight));
                    vertices[i + m_SideCount] = math.transform(transform, new float3(x, y, halfHeight));
                }

                // planes
                Plane* planes = (Plane*)(&collider->m_FacePlanes[0]);
                planes[0] = Math.TransformPlane(transform, new Plane(new float3(0f, 0f, -1f), -halfHeight));
                planes[1] = Math.TransformPlane(transform, new Plane(new float3(0f, 0f, 1f), -halfHeight));
                float d = radius * math.cos((float)math.PI / m_SideCount);
                for (int i = 0; i < m_SideCount; ++i)
                {
                    float angle = 2.0f * (float)math.PI * (i + 0.5f) / m_SideCount;
                    planes[2 + i] = Math.TransformPlane(transform, new Plane(new float3(math.cos(angle), math.sin(angle), 0f), -d));
                }

                // faces
                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                byte* indices = (byte*)(&collider->m_FaceVertexIndices[0]);
                float halfAngle = (float)math.PI * 0.25f;
                {
                    faces[0].FirstIndex = 0;
                    faces[0].NumVertices = (byte)m_SideCount;
                    faces[0].MinHalfAngle = halfAngle;
                    for (int i = 0; i < m_SideCount; ++i)
                    {
                        indices[i] = (byte)(m_SideCount - 1 - i);
                    }

                    faces[1].FirstIndex = (short)m_SideCount;
                    faces[1].NumVertices = (byte)m_SideCount;
                    faces[1].MinHalfAngle = halfAngle;
                    for (int i = m_SideCount; i < 2 * m_SideCount; ++i)
                    {
                        indices[i] = (byte)(i);
                    }
                }
                halfAngle = (float)math.PI / m_SideCount;
                for (int i = 0; i < m_SideCount; ++i)
                {
                    int firstIndex = (2 * m_SideCount) + (4 * i);

                    faces[i + 2].FirstIndex = (short)firstIndex;
                    faces[i + 2].NumVertices = 4;
                    faces[i + 2].MinHalfAngle = halfAngle;

                    indices[firstIndex + 0] = (byte)i;
                    indices[firstIndex + 1] = (byte)((i + 1) % m_SideCount);
                    indices[firstIndex + 2] = (byte)((i + 1) % m_SideCount + m_SideCount);
                    indices[firstIndex + 3] = (byte)(i + m_SideCount);
                }
            }

            MassProperties = new MassProperties
            {
                MassDistribution = new MassDistribution
                {
                    Transform = transform,
                    InertiaTensor = new float3(
                        (m_Radius * m_Radius + m_Height * m_Height) / 12f,
                        (m_Radius * m_Radius + m_Height * m_Height) / 12f,
                        (m_Radius * m_Radius) * 0.5f)
                },
                Volume = (float)math.PI * m_Radius * m_Radius * m_Height,
                AngularExpansionFactor = math.sqrt(radius * radius + halfHeight * halfHeight)
            };
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

        public unsafe Aabb CalculateAabb(RigidTransform transform)
        {
            transform = math.mul(transform, new RigidTransform(m_Orientation, m_Center));
            float3 axis = math.rotate(transform, new float3(0, 0, 1));
            float3 v0 = transform.pos + axis * m_Height * 0.5f;
            float3 v1 = transform.pos - axis * m_Height * 0.5f;
            float3 e = m_Radius;
            return new Aabb
            {
                Min = math.min(v0, v1) - e,
                Max = math.max(v0, v1) + e
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (CylinderCollider* target = &this)
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
            fixed (CylinderCollider* target = &this)
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
            fixed (CylinderCollider* target = &this)
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
            fixed (CylinderCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }
}
