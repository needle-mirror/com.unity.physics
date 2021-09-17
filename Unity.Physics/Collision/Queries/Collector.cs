using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    public interface IQueryResult
    {
        // For casts this is fraction of the query at which the hit occurred.
        // For distance queries, this is a distance from the query object
        float Fraction { get; }

        // Index of the hit body in the CollisionWorld's rigid body array
        int RigidBodyIndex { get; }

        // ColliderKey of the hit leaf collider
        ColliderKey ColliderKey { get; }

        // Material of the hit leaf collider
        Material Material { get; }

        // Entity of the hit body
        Entity Entity { get; }
    }

    struct QueryContext
    {
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public Entity Entity;
        public uint NumColliderKeyBits;
        public MTransform WorldFromLocalTransform;
        public bool IsInitialized;

        // Needed only in ColliderCast queries with non convex input, where it is used to
        // handle penetration cases properly.
        public bool IsFlipped;

        public static QueryContext DefaultContext => new QueryContext
        {
            RigidBodyIndex = -1,
            ColliderKey = ColliderKey.Empty,
            Entity = Entity.Null,
            NumColliderKeyBits = 0,
            WorldFromLocalTransform = MTransform.Identity,
            IsInitialized = true,
            IsFlipped = false
        };

        public ColliderKey SetSubKey(uint childSubKeyNumOfBits, uint childSubKey)
        {
            var parentColliderKey = ColliderKey;
            parentColliderKey.PopSubKey(NumColliderKeyBits, out uint parentKey);

            var colliderKey = new ColliderKey(childSubKeyNumOfBits, childSubKey);
            colliderKey.PushSubKey(NumColliderKeyBits, parentKey);
            return colliderKey;
        }

        public ColliderKey PushSubKey(uint childSubKeyNumOfBits, uint childSubKey)
        {
            var colliderKey = SetSubKey(childSubKeyNumOfBits, childSubKey);
            NumColliderKeyBits += childSubKeyNumOfBits;
            return colliderKey;
        }
    }

    // Interface for collecting hits during a collision query
    public interface ICollector<T> where T : struct, IQueryResult
    {
        // Whether to exit the query as soon as any hit has been accepted
        bool EarlyOutOnFirstHit { get; }

        // The maximum fraction of the query within which to check for hits
        // For casts, this is a fraction along the ray
        // For distance queries, this is a distance from the query object
        float MaxFraction { get; }

        // The number of hits that have been collected
        int NumHits { get; }

        // Called when the query hits something
        // Return true to accept the hit, or false to ignore it
        bool AddHit(T hit);
    }

    // A collector which exits the query as soon as any hit is detected.
    public struct AnyHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnFirstHit => true;
        public float MaxFraction { get; }
        public int NumHits => 0;

        public AnyHitCollector(float maxFraction)
        {
            MaxFraction = maxFraction;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);
            return true;
        }

        #endregion
    }

    // A collector which stores only the closest hit.
    public struct ClosestHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnFirstHit => false;
        public float MaxFraction { get; private set; }
        public int NumHits { get; private set; }

        private T m_ClosestHit;
        public T ClosestHit => m_ClosestHit;

        public ClosestHitCollector(float maxFraction)
        {
            MaxFraction = maxFraction;
            m_ClosestHit = default(T);
            NumHits = 0;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);
            MaxFraction = hit.Fraction;
            m_ClosestHit = hit;
            NumHits = 1;
            return true;
        }

        #endregion
    }

    // A collector which stores every hit.
    public struct AllHitsCollector<T> : ICollector<T> where T : unmanaged, IQueryResult
    {
        public bool EarlyOutOnFirstHit => false;
        public float MaxFraction { get; }
        public int NumHits => AllHits.Length;

        public NativeList<T> AllHits;

        public AllHitsCollector(float maxFraction, ref NativeList<T> allHits)
        {
            MaxFraction = maxFraction;
            AllHits = allHits;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);

            AllHits.Add(hit);
            return true;
        }

        #endregion
    }

    // A collector used to provide filtering for QueryInteraction enum
    // This is a wrapper of the user provided collector, which serves to enable
    // filtering based on the QueryInteraction parameter.
    internal unsafe struct QueryInteractionCollector<T, C> : ICollector<T>
        where T : struct, IQueryResult
        where C : struct, ICollector<T>
    {
        public bool EarlyOutOnFirstHit => Collector.EarlyOutOnFirstHit;
        public float MaxFraction => Collector.MaxFraction;
        public int NumHits => Collector.NumHits;

        // Todo: have a QueryInteraction field here, and filter differently based on it in AddHit()
        // at the moment, this collector will only get constructed if IgnoreTriggers interaction is selected
        public ref C Collector
        {
            get => ref UnsafeUtility.AsRef<C>(m_CollectorPtr);
        }

        // This must be a void ptr, since C# doesn't allow generic type pointers
        private void* m_CollectorPtr;

        public QueryInteractionCollector(ref C collector)
        {
            m_CollectorPtr = UnsafeUtility.AddressOf(ref collector);
        }

        public bool AddHit(T hit)
        {
            if (hit.Material.CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents)
            {
                return false;
            }

            return Collector.AddHit(hit);
        }
    }

    // Collector used when flipping input and target of collider cast queries
    // It is just a wrapper around user provided collector (base collector)
    // All it does is restore the flipped query output to a non-flipped one
    // and passes the modified hit to the provided collector.
    internal unsafe struct FlippedColliderCastQueryCollector<C> : ICollector<ColliderCastHit>
        where C : struct, ICollector<ColliderCastHit>
    {
        public bool EarlyOutOnFirstHit => Collector.EarlyOutOnFirstHit;
        public float MaxFraction => Collector.MaxFraction;
        public int NumHits => Collector.NumHits;

        public ref C Collector
        {
            get => ref UnsafeUtility.AsRef<C>(m_CollectorPtr);
        }

        private ColliderKey m_TargetColliderKey;
        private Material m_TargetMaterial;
        private float3 m_CastDirectionWS;

        // This must be a void ptr, since C# doesn't allow generic type pointers
        private void* m_CollectorPtr;

        public FlippedColliderCastQueryCollector(ref C collector, float3 castDirectionWS, ColliderKey targetColliderKey, Material targetMaterial)
        {
            m_TargetColliderKey = targetColliderKey;
            m_TargetMaterial = targetMaterial;
            m_CastDirectionWS = castDirectionWS;
            m_CollectorPtr = UnsafeUtility.AddressOf(ref collector);
        }

        public bool AddHit(ColliderCastHit hit)
        {
            hit.Position = hit.Position + hit.Fraction * m_CastDirectionWS;
            hit.SurfaceNormal = -hit.SurfaceNormal;

            // Collider keys are in 'flipped' order, need to swap them.
            hit.QueryColliderKey = hit.ColliderKey;
            hit.ColliderKey = m_TargetColliderKey;

            // Material at this point represents the query collider material, which needs to be corrected to target collider material.
            hit.Material = m_TargetMaterial;

            return Collector.AddHit(hit);
        }
    }

    // Collector used when flipping input and target of collider distance queries
    // It is just a wrapper around user provided collector (base collector)
    // All it does is restore the flipped query output to a non-flipped one
    // and passes the modified hit to the provided collector.
    internal unsafe struct FlippedColliderDistanceQueryCollector<C> : ICollector<DistanceHit>
        where C : struct, ICollector<DistanceHit>
    {
        public bool EarlyOutOnFirstHit => Collector.EarlyOutOnFirstHit;
        public float MaxFraction => Collector.MaxFraction;
        public int NumHits => Collector.NumHits;

        public ref C Collector
        {
            get => ref UnsafeUtility.AsRef<C>(m_CollectorPtr);
        }

        private ColliderKey m_TargetColliderKey;
        private Material m_TargetMaterial;

        // This must be a void ptr, since C# doesn't allow generic type pointers
        private void* m_CollectorPtr;

        public FlippedColliderDistanceQueryCollector(ref C collector, ColliderKey targetColliderKey, Material targetMaterial)
        {
            m_TargetColliderKey = targetColliderKey;
            m_TargetMaterial = targetMaterial;
            m_CollectorPtr = UnsafeUtility.AddressOf(ref collector);
        }

        public bool AddHit(DistanceHit hit)
        {
            hit.Position = hit.Position + hit.SurfaceNormal * hit.Fraction;
            hit.SurfaceNormal = -hit.SurfaceNormal;

            // Collider keys are in 'flipped' order, need to swap them.
            hit.QueryColliderKey = hit.ColliderKey;
            hit.ColliderKey = m_TargetColliderKey;

            // Material at this point represents the query collider material, which needs to be corrected to target collider material.
            hit.Material = m_TargetMaterial;

            return Collector.AddHit(hit);
        }
    }
}
