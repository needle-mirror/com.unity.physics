using System;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    internal struct LogBuffer : IDisposable
    {
        internal CellSurface m_Surface;
        internal int2 m_Cursor;
        internal ColorIndex m_Fg;
        internal ColorIndex m_Bg;

        internal LogBuffer(int2 size)
        {
            m_Surface = new CellSurface(size);
            m_Cursor = default;
            m_Fg = ColorIndex.White;
            m_Bg = ColorIndex.Black;
            CellSurface.Clear(ref m_Surface, new Cell {fg = m_Fg, bg = m_Bg, rune = new Unicode.Rune {value = ' '}});
        }

        internal void WriteLine(in FixedString128 f)
        {
            var y = m_Cursor.y & (m_Surface.m_Size.y - 1);
            CellSurface.Clear(ref m_Surface, new Cell {fg = m_Fg, bg = m_Bg, rune = new Unicode.Rune {value = ' '}}, new Window {m_begin = new int2(0, y), m_end = new int2(m_Surface.m_Size.x, y + 1)});
            var temp = m_Cursor;
            temp.y = y;
            m_Surface.PutChars(ref temp, f, m_Fg, m_Bg);
            m_Cursor.x = 0;
            ++m_Cursor.y;
        }

        internal void CopyTo(ref CellSurface dst, Window dstWindow)
        {
            var lineCursorIsOn = m_Cursor.y & (m_Surface.m_Size.y - 1); // how far back can we look from cursor in src buffer, without rolling past start of buffer?
            if (lineCursorIsOn >= dstWindow.Height) // if as many or more than we plan to copy anyway,
            {
                Window srcWindow = new Window
                {
                    m_begin = new int2(0, lineCursorIsOn - dstWindow.Height),
                    m_end = new int2(m_Surface.m_Size.x, lineCursorIsOn)
                };
                CellSurface.Blit(ref dst, dstWindow, ref m_Surface, srcWindow); // just copy them all at once.
            }
            else
            {
                var remainingLines = dstWindow.Height - lineCursorIsOn;
                dstWindow.m_end.y = dstWindow.m_begin.y + remainingLines;
                {
                    Window srcWindow = new Window // first, render end of src buffer into beginning of dst buffer
                    {
                        m_begin = new int2(0, m_Surface.m_Size.y - remainingLines),
                        m_end = new int2(m_Surface.m_Size.x, m_Surface.m_Size.y)
                    };
                    CellSurface.Blit(ref dst, dstWindow, ref m_Surface, srcWindow);
                }
                dstWindow.m_begin.y = dstWindow.m_end.y;
                dstWindow.m_end.y = dstWindow.m_begin.y + lineCursorIsOn;
                {
                    Window srcWindow = new Window // then, render beginning of src buffer into end of dst buffer
                    {
                        m_begin = new int2(0, 0),
                        m_end = new int2(m_Surface.m_Size.x, lineCursorIsOn)
                    };
                    CellSurface.Blit(ref dst, dstWindow, ref m_Surface, srcWindow);
                }
            }
        }

        internal void CopyToWithFrame(ref CellSurface dest, Window dstWindow)
        {
            dest.PutFrame(dstWindow, ColorIndex.Yellow, ColorIndex.Blue, 7);
            dstWindow = dstWindow.Shrink;
            CopyTo(ref dest, dstWindow);
        }

        public void Dispose()
        {
            m_Surface.Dispose();
        }
    }
}
