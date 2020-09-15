using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    internal struct Window
    {
        internal int2 m_begin;
        internal int2 m_end;
        internal int2 Size => math.max(0, m_end - m_begin);
        internal int Width => math.max(0, m_end.x - m_begin.x);
        internal int Height => math.max(0, m_end.y - m_begin.y);
        internal int Cells => Width * Height;
        internal bool HasCells => (Width | Height) != 0;
        internal Window Shrink => new Window {m_begin = m_begin + 1, m_end = m_end - 1};
        internal Window Expand => new Window {m_begin = m_begin - 1, m_end = m_end + 1};

        internal Window(int x, int y, int w, int h)
        {
            m_begin = new int2(x, y);
            m_end = new int2(x + w, y + h);
        }
    }
}
