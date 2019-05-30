using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Entities;

namespace Unity.Physics
{
    // A collider in the shape of a capsule
    public struct CapsuleCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        private float3 m_Vertex0;
        private float3 m_Vertex1;

        public float3 Vertex0 { get => m_Vertex0; set { m_Vertex0 = value; Update(); } }
        public float3 Vertex1 { get => m_Vertex1; set { m_Vertex1 = value; Update(); } }
        public float Radius { get => ConvexHull.ConvexRadius; set => ConvexHull.ConvexRadius = value; }

        #region Construction

        unsafe public static BlobAssetReference<Collider> Create(float3 point0, float3 point1, float radius, CollisionFilter? filter = null, Material? material = null)
        {
            if (math.any(!math.isfinite(point0)))
            {
                throw new System.ArgumentException("Tried to create capsule collider with inf/nan point0");
            }
            if (math.any(!math.isfinite(point1)))
            {
                throw new System.ArgumentException("Tried to create capsule collider with inf/nan point1");
            }
            if (!math.isfinite(radius) || radius < 0.0f)
            {
                throw new System.ArgumentException("Tried to create capsule collider with zero/negative/inf/nan radius");
            }

            var collider = default(CapsuleCollider);
            collider.Init(point0, point1, radius, filter ?? CollisionFilter.Default, material ?? Material.Default);
            return BlobAssetReference<Collider>.Create(&collider, sizeof(CapsuleCollider));
        }

        internal unsafe void Init(float3 vertex0, float3 vertex1, float radius, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Capsule;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version += 1;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            ConvexHull.ConvexRadius = radius;
            ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref m_Vertex0, ref ConvexHull.VerticesBlob.Offset);
            ConvexHull.VerticesBlob.Length = 2;
            // note: no faces

            m_Vertex0 = vertex0;
            m_Vertex1 = vertex1;
            Update();
        }

        private void Update()
        {
            // Treat as sphere if degenerate
            ConvexHull.VerticesBlob.Length = m_Vertex0.Equals(m_Vertex1) ? 1 : 2;
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<CapsuleCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set { m_Header.Filter = value; } }
        public Material Material { get => m_Header.Material; set { m_Header.Material = value; } }

        public MassProperties MassProperties
        {
            get
            {
                float3 axis = m_Vertex1 - m_Vertex0;
                float length = math.length(axis);
                float cylinderMass = (float)math.PI * length * Radius * Radius;
                float sphereMass = (float)math.PI * (4.0f / 3.0f) * Radius * Radius * Radius;
                float totalMass = cylinderMass + sphereMass;
                cylinderMass /= totalMass;
                sphereMass /= totalMass;
                float onAxisInertia = (cylinderMass * 0.5f + sphereMass * 0.4f) * Radius * Radius;
                float offAxisInertia =
                    cylinderMass * (1.0f / 4.0f * Radius * Radius + 1.0f / 12.0f * length * length) +
                    sphereMass * (2.0f / 5.0f * Radius * Radius + 3.0f / 8.0f * Radius * length + 1.0f / 4.0f * length * length);

                float3 axisInMotion = new float3(0, 1, 0);
                quaternion orientation = length == 0 ? quaternion.identity :
                    Math.FromToRotation(axisInMotion, math.normalizesafe(Vertex1 - Vertex0, axisInMotion));
                
                return new MassProperties
                {
                    MassDistribution = new MassDistribution
                    {
                        Transform = new RigidTransform(orientation, (Vertex0 + Vertex1) * 0.5f),
                        InertiaTensor = new float3(offAxisInertia, onAxisInertia, offAxisInertia)
                    },
                    Volume = (float)math.PI * Radius * Radius * ((4.0f / 3.0f) * Radius + math.length(Vertex1-Vertex0)),
                    AngularExpansionFactor = math.length(m_Vertex1 - m_Vertex0) * 0.5f
                };
            }
        }

        public Aabb CalculateAabb()
        {
            return new Aabb
            {
                Min = math.min(m_Vertex0, m_Vertex1) - new float3(Radius),
                Max = math.max(m_Vertex0, m_Vertex1) + new float3(Radius)
            };
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            float3 v0 = math.transform(transform, m_Vertex0);
            float3 v1 = math.transform(transform, m_Vertex1);
            return new Aabb
            {
                Min = math.min(v0, v1) - new float3(Radius),
                Max = math.max(v0, v1) + new float3(Radius)
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (CapsuleCollider* target = &this)
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
            fixed (CapsuleCollider* target = &this)
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
            fixed (CapsuleCollider* target = &this)
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
            fixed (CapsuleCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }
}
