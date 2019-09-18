using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Physics.LowLevel
{
    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    struct TriggerEvent
    {
        public BodyIndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;
    }

    // A stream of trigger events.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<TriggerEvent>).
    struct TriggerEvents /* : IEnumerable<TriggerEvent> */
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
            public TriggerEvent Current { get; private set; }

            public Enumerator(BlockStream stream)
            {
                m_Reader = stream.IsCreated ? stream : new BlockStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;
                Current = default;

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    Current = m_Reader.Read<TriggerEvent>();
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
}
