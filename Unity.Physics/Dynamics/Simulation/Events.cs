using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics.LowLevel
{
    // An event raised when a pair of bodies have collided during solving.
    public struct CollisionEvent
    {
        public BodyIndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;
        public float3 Normal;
        public float4 AccumulatedImpulses;
    }

    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    public struct TriggerEvent
    {
        public BodyIndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;
    }

    // Collision event reader, for both Unity.Physics and Havok.Physics.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<CollisionEvent>).
    public unsafe struct CollisionEvents /* : IEnumerable<CollisionEvent> */
    {
        //@TODO: Unity should have a Allow null safety restriction
        [NativeDisableContainerSafetyRestriction]
        private readonly BlockStream m_EventStream;

        public CollisionEvents(BlockStream eventStream)
        {
            m_EventStream = eventStream;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_EventStream);
        }

        public struct Enumerator /* : IEnumerator<CollisionEvent> */
        {
            private BlockStream.Reader m_Reader;
            private int m_CurrentWorkItem;
            private readonly int m_NumWorkItems;

            private CollisionEvent m_Current;
            public CollisionEvent Current => m_Current;

            public Enumerator(BlockStream stream)
            {
                m_Reader = stream.IsCreated ? stream : new BlockStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;

                m_Current = default(CollisionEvent);

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    m_Current = m_Reader.Read<CollisionEvent>();
                    AdvanceReader();
                    return true;
                }
                return false;
            }

            public void Reset()
            {
                throw new NotImplementedException();
            }

            private void AdvanceReader()
            {
                while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_NumWorkItems)
                {
                    m_Reader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }
    }

    // Trigger event reader, for both Unity.Physics and Havok.Physics.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<TriggerEvent>).
    public unsafe struct TriggerEvents /* : IEnumerable<TriggerEvent> */
    {

        //@TODO: Unity should have a Allow null safety restriction
        [NativeDisableContainerSafetyRestriction]
        private readonly BlockStream m_EventStream;

        public TriggerEvents(BlockStream eventStream)
        {
            m_EventStream = eventStream;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_EventStream);
        }

        public struct Enumerator /* : IEnumerator<TriggerEvent> */
        {
            private BlockStream.Reader m_Reader;
            private int m_CurrentWorkItem;
            private readonly int m_NumWorkItems;

            private TriggerEvent m_Current;
            public TriggerEvent Current => m_Current;

            public Enumerator(BlockStream stream)
            {
                m_Reader = stream.IsCreated ? stream : new BlockStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;
                m_Current = default(TriggerEvent);

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    m_Current = m_Reader.Read<TriggerEvent>();
                    AdvanceReader();
                    return true;
                }
                return false;
            }

            public void Reset()
            {
                throw new NotImplementedException();
            }

            private void AdvanceReader()
            {
                while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_NumWorkItems)
                {
                    m_Reader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }
    }
}
