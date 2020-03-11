using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    // An instance of a collider in a physics world.
    public struct RigidBody : ICollidable
    {
        // The rigid body's collider (allowed to be null)
        public BlobAssetReference<Collider> Collider;

        // The rigid body's transform in world space
        public RigidTransform WorldFromBody;

        // The entity that rigid body represents
        public Entity Entity;

        // Arbitrary custom tags.
        // These get copied into contact manifolds and can be used to inform contact modifiers.
        public byte CustomTags;

        public static readonly RigidBody Zero = new RigidBody
        {
            WorldFromBody = RigidTransform.identity,
            Collider = default,
            Entity = Entity.Null,
            CustomTags = 0
        };

        #region ICollidable implementation

        public Aabb CalculateAabb()
        {
            if (Collider.IsCreated)
            {
                return Collider.Value.CalculateAabb(WorldFromBody);
            }
            return new Aabb { Min = WorldFromBody.pos, Max = WorldFromBody.pos };
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            if (Collider.IsCreated)
            {
                return Collider.Value.CalculateAabb(math.mul(transform, WorldFromBody));
            }
            return new Aabb { Min = WorldFromBody.pos, Max = WorldFromBody.pos };
        }

        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return Collider.IsCreated && Collider.Value.CastRay(input, ref collector);
        }

        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return Collider.IsCreated && Collider.Value.CastCollider(input, ref collector);
        }

        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        #endregion

        #region Obsolete
        [Obsolete("HasCollider has been deprecated. Use Collider.IsCreated instead. (RemovedAfter 2020-03-18)")]
        public bool HasCollider => Collider.IsCreated;
        #endregion
    }

    // A pair of rigid body indices
    public struct BodyIndexPair
    {
        // B before A to match Havok
        public int BodyBIndex;
        public int BodyAIndex;

        public bool IsValid => BodyBIndex != -1 && BodyAIndex != -1;

        public static BodyIndexPair Invalid => new BodyIndexPair { BodyBIndex = -1, BodyAIndex = -1 };
    }

    // A pair of entities
    public struct EntityPair
    {
        // B before A for consistency with other pairs
        public Entity EntityB;
        public Entity EntityA;
    }

    // A pair of custom rigid body tags
    public struct CustomTagsPair
    {
        // B before A for consistency with other pairs
        public byte CustomTagsB;
        public byte CustomTagsA;
    }
}
