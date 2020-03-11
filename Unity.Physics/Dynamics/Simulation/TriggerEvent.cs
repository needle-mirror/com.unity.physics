using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Physics
{
    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    public struct TriggerEvent
    {
        internal TriggerEventData EventData;
        public EntityPair Entities { get; internal set; }

        public BodyIndexPair BodyIndices => EventData.BodyIndices;
        public ColliderKeyPair ColliderKeys => EventData.ColliderKeys;
    }

    // A stream of trigger events.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<TriggerEvent>).
    public struct TriggerEvents /* : IEnumerable<TriggerEvent> */
    {
        //@TODO: Unity should have a Allow null safety restriction
        [NativeDisableContainerSafetyRestriction]
        private readonly NativeStream m_EventDataStream;

        private readonly NativeSlice<RigidBody> m_Bodies;

        internal TriggerEvents(NativeStream eventDataStream, NativeSlice<RigidBody> bodies)
        {
            m_EventDataStream = eventDataStream;
            m_Bodies = bodies;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_EventDataStream, m_Bodies);
        }

        public struct Enumerator /* : IEnumerator<TriggerEvent> */
        {
            private NativeStream.Reader m_Reader;
            private int m_CurrentWorkItem;
            private readonly int m_NumWorkItems;
            private readonly NativeSlice<RigidBody> m_Bodies;

            public TriggerEvent Current { get; private set; }

            internal Enumerator(NativeStream stream, NativeSlice<RigidBody> bodies)
            {
                m_Reader = stream.IsCreated ? stream.AsReader() : new NativeStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;
                Current = default;

                m_Bodies = bodies;

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    var eventData = m_Reader.Read<TriggerEventData>();

                    Current = eventData.CreateTriggerEvent(m_Bodies);

                    AdvanceReader();
                    return true;
                }
                return false;
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

    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    struct TriggerEventData
    {
        public BodyIndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;

        internal TriggerEvent CreateTriggerEvent(NativeSlice<RigidBody> bodies)
        {
            return new TriggerEvent
            {
                EventData = this,
                Entities = new EntityPair
                {
                    EntityA = bodies[BodyIndices.BodyAIndex].Entity,
                    EntityB = bodies[BodyIndices.BodyBIndex].Entity
                }
            };
        }
    }
}
