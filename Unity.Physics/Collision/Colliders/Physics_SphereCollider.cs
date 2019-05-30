using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Entities;

namespace Unity.Physics
{
    // A collider in the shape of a sphere
    public struct SphereCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        private float3 m_Vertex;

        public float3 Center { get => m_Vertex; set => m_Vertex = value; }
        public float Radius { get => ConvexHull.ConvexRadius; set => ConvexHull.ConvexRadius = value; }

        #region Construction

        unsafe public static BlobAssetReference<Collider> Create(float3 center, float radius, CollisionFilter? filter = null, Material? material = null)
        {
            if (math.any(!math.isfinite(center)))
            {
                throw new System.ArgumentException("Tried to create sphere collider with inf/nan center");
            }
            if (!math.isfinite(radius) || radius <= 0.0f)
            {
                throw new System.ArgumentException("Tried to create sphere collider with negative/inf/nan radius");
            }

            
            var collider = default(SphereCollider);
            collider.Init(center, radius, filter ?? CollisionFilter.Default, material ?? Material.Default);
            
            var sphereCollider = BlobAssetReference<Collider>.Create(&collider, sizeof(SphereCollider));
            return sphereCollider;
        }

        internal unsafe void Init(float3 center, float radius, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Sphere;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version += 1;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            ConvexHull.ConvexRadius = radius;
            ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref m_Vertex, ref ConvexHull.VerticesBlob);
            ConvexHull.VerticesBlob.Length = 1;
            // note: no faces

            m_Vertex = center;
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<SphereCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set => m_Header.Filter = value; }
        public Material Material { get => m_Header.Material; set => m_Header.Material = value; }

        public MassProperties MassProperties => new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = new RigidTransform(quaternion.identity, Center),
                InertiaTensor = new float3(2.0f / 5.0f * Radius * Radius)
            },
            Volume = (4.0f / 3.0f) * (float)math.PI * Radius * Radius * Radius,
            AngularExpansionFactor = 0.0f
        };

        public Aabb CalculateAabb()
        {
            return new Aabb
            {
                Min = Center - new float3(Radius),
                Max = Center + new float3(Radius)
            };
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            float3 centerInWorld = math.transform(transform, Center);
            return new Aabb
            {
                Min = centerInWorld - new float3(Radius),
                Max = centerInWorld + new float3(Radius)
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (SphereCollider* target = &this)
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
            fixed (SphereCollider* target = &this)
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
            fixed (SphereCollider* target = &this)
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
            fixed (SphereCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }
}
