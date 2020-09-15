using System.Threading;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    internal struct Unit
    {
        internal int m_Begin;
        internal int m_Next;
        internal int m_End;

        internal Unit AllocateAtomic(int count)
        {
            var begin = m_Next;
            while (true)
            {
                var end = math.min(begin + count, m_End);
                if (begin == end)
                    return default;
                var found = Interlocked.CompareExchange(ref m_Next, end, begin);
                if (found == begin)
                    return new Unit { m_Begin = begin, m_Next = begin, m_End = end };
                begin = found;
            }
        }

        internal int AllocateAtomic()
        {
            return AllocateAtomic(1).m_Begin;
        }

/*
        internal Unit Allocate(int count)
        {
            var end = math.min(m_next + count, m_end);
            var unit = new Unit { m_begin = m_next, m_next = m_next, m_end = end };
            m_next = end;
            return unit;
        }

        internal int Allocate()
        {
            return Allocate(1).m_begin;
        }
*/

        internal Unit(int count)
        {
            m_Begin = m_Next = 0;
            m_End = count;
        }

        internal Unit(int me, int writers, int writableBegin, int writableEnd)
        {
            var writables = writableEnd - writableBegin;
            m_Begin = writableBegin + (me * writables) / writers;
            m_End = writableBegin + ((me + 1) * writables) / writers;
            if (m_Begin > writableEnd)
                m_Begin = writableEnd;
            if (m_End > writableEnd)
                m_End = writableEnd;
            m_Next = m_Begin;
        }

        internal void Fill()
        {
            m_Next = m_End;
        }

        internal int Length => m_End - m_Begin;
        internal int Filled => m_Next - m_Begin;
        internal int Remaining => m_End - m_Next;
    }
}
