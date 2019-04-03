using Unity.Collections;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    public interface IQueryResult
    {
        float Fraction { get; }

        void Transform(MTransform transform, uint parentSubKey, uint parentSubKeyBits);
        void Transform(MTransform transform, int rigidBodyIndex);
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

        void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, uint numSubKeyBits, uint subKey);
        void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, int rigidBodyIndex);
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
            Assert.IsTrue(hit.Fraction < MaxFraction);
            return true;
        }

        public void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, uint numSubKeyBits, uint subKey) { }
        public void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, int rigidBodyIndex) { }

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

        public void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, uint numSubKeyBits, uint subKey)
        {
            if (m_ClosestHit.Fraction < oldFraction)
            {
                m_ClosestHit.Transform(transform, numSubKeyBits, subKey);
            }
        }

        public void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, int rigidBodyIndex)
        {
            if (m_ClosestHit.Fraction < oldFraction)
            {
                m_ClosestHit.Transform(transform, rigidBodyIndex);
            }
        }

        #endregion
    }

    // A collector which stores every hit.
    public struct AllHitsCollector<T> : ICollector<T> where T : struct, IQueryResult
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

        #region

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction < MaxFraction);
            AllHits.Add(hit);
            return true;
        }

        public void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, uint numSubKeyBits, uint subKey)
        {
            for (int i = oldNumHits; i < NumHits; i++)
            {
                T hit = AllHits[i];
                hit.Transform(transform, numSubKeyBits, subKey);
                AllHits[i] = hit;
            }
        }

        public void TransformNewHits(int oldNumHits, float oldFraction, MTransform transform, int rigidBodyIndex)
        {
            for (int i = oldNumHits; i < NumHits; i++)
            {
                T hit = AllHits[i];
                hit.Transform(transform, rigidBodyIndex);
                AllHits[i] = hit;
            }
        }

        #endregion
    }
}
