using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace Unity.DebugDisplay
{
    internal struct CellSurface : IDisposable
    {
        internal int2 m_Size;
        internal UnsafeArray<Cell> m_Cell;

        internal CellSurface(int2 size)
        {
            Assert.AreEqual(0, size.x & (size.x - 1));
            Assert.AreEqual(0, size.y & (size.y - 1));
            m_Size = size;
            m_Cell = new UnsafeArray<Cell>(size.x * size.y);
        }

        internal unsafe void Clear()
        {
            UnsafeUtility.MemClear(m_Cell.GetUnsafePtr(), m_Cell.Length * sizeof(Cell));
        }

        public void Dispose()
        {
            m_Cell.Dispose();
        }

        void PutChar(int2 xy, Cell t)
        {
            if (xy.x < 0 || xy.x >= m_Size.x || xy.y < 0 || xy.y >= m_Size.y)
                return;
            var offset = xy.y * m_Size.x + xy.x;
            m_Cell[offset] = t;
        }

        void PutChar(ref int2 xy, Unicode.Rune rune, ColorIndex fg, ColorIndex bg)
        {
            Cell t = new Cell {rune = rune, fg = fg, bg = bg};
            PutChar(xy, t);
            ++xy.x;
            if (isWide(t.rune))
            {
                t.side = true;
                PutChar(xy, t);
                ++xy.x;
            }
        }

        void ParseAnsiColor(ref FixedListInt64 numbers, ref ColorIndex fg, ref ColorIndex bg)
        {
            for (var i = 0; i < numbers.Length; ++i)
            {
                var code = numbers[i];
                switch (code)
                {
                    case 0:
                        fg.value = 15;
                        bg.value = 0;
                        break;
                    case 1:
                        fg.value |= 8;
                        break;
                    case 30:
                    case 31:
                    case 32:
                    case 33:
                    case 34:
                    case 35:
                    case 36:
                    case 37:
                        fg.value &= 8;
                        fg.value |= code - 30;
                        break;
                    case 40:
                    case 41:
                    case 42:
                    case 43:
                    case 44:
                    case 45:
                    case 46:
                    case 47:
                        bg.value = code - 40;
                        break;
                    case 90:
                    case 91:
                    case 92:
                    case 93:
                    case 94:
                    case 95:
                    case 96:
                    case 97:
                        fg.value = 8 | (code - 90);
                        break;
                    case 100:
                    case 101:
                    case 102:
                    case 103:
                    case 104:
                    case 105:
                    case 106:
                    case 107:
                        bg.value = code - 100;
                        break;
                }
            }
        }

        void ParseAnsi(ref FixedListByte32 escape, ref ColorIndex fg, ref ColorIndex bg)
        {
            if (escape.Length < 4)
                return;
            if (escape[0] != 0x1b || escape[1] != '[')
                return;
            int offset = 2;
            FixedListInt64 numbers = default;
            int number = 0;
            while (offset < escape.Length)
            {
                switch ((char)escape[offset])
                {
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        number *= 10;
                        number += escape[offset] - '0';
                        break;
                    case ';':
                        numbers.Add(number);
                        number = 0;
                        break;
                    case 'm':
                        numbers.Add(number);
                        ParseAnsiColor(ref numbers, ref fg, ref bg);
                        return;
                }
                ++offset;
            }
        }

        internal unsafe void PutChars(ref int2 xy, in FixedString128 f, ColorIndex fg, ColorIndex bg)
        {
            fixed(FixedString128* ff = &f)
            {
                FixedListByte32 escape = default;
                int offset = 0;
                while (offset < f.Length)
                {
                    var error = Unicode.Utf8ToUcs(out Unicode.Rune rune, (byte*)ff + 2, ref offset, f.Length);
                    if (error != 0)
                        return;
                    if (rune.value == 0x1b)
                    {
                        escape.Length = 0;
                        escape.Add((byte)rune.value);
                    }
                    else
                    {
                        if (escape.Length == 0)
                            PutChar(ref xy, rune, fg, bg);
                        else
                        {
                            escape.Add((byte)rune.value);
                            if (((rune.value | 32) >= 'a' && (rune.value | 32) <= 'z') || escape.Length == FixedString32.UTF8MaxLengthInBytes)
                            {
                                if (rune.value == 'm')
                                    ParseAnsi(ref escape, ref fg, ref bg);
                                escape.Length = 0;
                            }
                        }
                    }
                }
            }
        }

        static readonly char[] FrameStyle =
        {
            /*0*/ '┌', '┐', '└', '┘', '─', '─', '│', '│',
            /*1*/ '╔', '╗', '╚', '╝', '═', '═', '║', '║',
            /*2*/ '╒', '╕', '╘', '╛', '═', '═', '│', '│',
            /*3*/ '╓', '╖', '╙', '╜', '─', '─', '║', '║',
            /*4*/ '▛', '▜', '▙', '▟', '▀', '▄', '▌', '▐',
            /*5*/ '▗', '▖', '▝', '▘', '▄', '▀', '▐', '▌',
            /*6*/ '╭', '╮', '╰', '╯', '─', '─', '│', '│',
            /*7*/ '┌', '╖', '╘', '╝', '─', '═', '│', '║',
        };

        internal unsafe void PutFrame(in Window window, ColorIndex fg, ColorIndex bg, int style = 0)
        {
            Assert.IsTrue(style < FrameStyle.Length >> 3);
            if (!window.HasCells)
                return;
            Cell c0 = new Cell {fg = fg, bg = bg};
            Cell c1 = new Cell {fg = fg, bg = bg};
            c0.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 0]};
            PutChar(new int2(window.m_begin.x, window.m_begin.y), c0);
            c0.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 1]};
            PutChar(new int2(window.m_end.x - 1, window.m_begin.y), c0);
            c0.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 2]};
            PutChar(new int2(window.m_begin.x, window.m_end.y - 1), c0);
            c0.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 3]};
            PutChar(new int2(window.m_end.x - 1, window.m_end.y - 1), c0);
            if (window.Width > 2)
            {
                c0.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 4]};
                c1.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 5]};
                for (var x = window.m_begin.x + 1; x < window.m_end.x - 1; ++x)
                {
                    PutChar(new int2(x, window.m_begin.y), c0);
                    PutChar(new int2(x, window.m_end.y - 1), c1);
                }
            }
            if (window.Height > 2)
            {
                c0.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 6]};
                c1.rune = new Unicode.Rune {value = FrameStyle[(style << 3) + 7]};
                for (var y = window.m_begin.y + 1; y < window.m_end.y - 1; ++y)
                {
                    PutChar(new int2(window.m_begin.x, y), c0);
                    PutChar(new int2(window.m_end.x - 1, y), c1);
                }
            }
        }

        internal static void Clip(ref CellSurface dst, ref Window dstWindow)
        {
            dstWindow.m_begin = math.max(0, dstWindow.m_begin);
            dstWindow.m_end = math.min(dstWindow.m_end, dst.m_Size);
        }

        internal static void Clip(ref CellSurface dst, ref Window dstWindow, ref CellSurface src, ref Window srcWindow)
        {
            srcWindow.m_begin -= math.min(0, dstWindow.m_begin);
            dstWindow.m_begin = math.max(0, dstWindow.m_begin);
            dstWindow.m_begin -= math.min(0, srcWindow.m_begin);
            srcWindow.m_begin = math.max(0, srcWindow.m_begin);
            srcWindow.m_end = math.min(srcWindow.m_end, src.m_Size);
            dstWindow.m_end = math.min(dstWindow.m_end, dst.m_Size);
            srcWindow.m_end -= math.max(0, (dstWindow.m_begin + srcWindow.Size) - dstWindow.m_end);
        }

        internal static unsafe void Clear(ref CellSurface dst, Cell cell, Window dstWindow)
        {
            Clip(ref dst, ref dstWindow);
            Cell* d = (Cell*)dst.m_Cell.GetUnsafePtr() + dstWindow.m_begin.y * dst.m_Size.x + dstWindow.m_begin.x;
            for (var dy = 0; dy < dstWindow.Size.y; ++dy)
            {
                for (var dx = 0; dx < dstWindow.Size.x; ++dx)
                    d[dx] = cell;
                d += dst.m_Size.x;
            }
        }

        internal static void Clear(ref CellSurface dst, Cell cell)
        {
            Clear(ref dst, cell, new Window {m_end = dst.m_Size});
        }

        internal static unsafe void Blit(ref CellSurface dst, Window dstWindow, ref CellSurface src, Window srcWindow)
        {
            Clip(ref dst, ref dstWindow, ref src, ref srcWindow);
            Cell* d = (Cell*)dst.m_Cell.GetUnsafePtr() + dstWindow.m_begin.y * dst.m_Size.x + dstWindow.m_begin.x;
            Cell* s = (Cell*)src.m_Cell.GetUnsafePtr() + srcWindow.m_begin.y * src.m_Size.x + srcWindow.m_begin.x;
            for (var dy = 0; dy < dstWindow.Size.y; ++dy)
            {
                UnsafeUtility.MemCpy(d, s, srcWindow.Width * sizeof(Cell));
                d += dst.m_Size.x;
                s += src.m_Size.x;
            }
        }

        internal static void Blit(ref CellSurface dst, Window dstWindow, ref CellSurface src)
        {
            Blit(ref dst, dstWindow, ref src, new Window {m_end = src.m_Size});
        }

        internal static void Blit(ref CellSurface dst, int2 dstPos, ref CellSurface src)
        {
            Blit(ref dst, new Window {m_begin = dstPos, m_end = dstPos + src.m_Size}, ref src);
        }

        internal static unsafe bool isWide(Unicode.Rune rune)
        {
            if (rune.value > 65535)
                return true;
            fixed(EightK* e = &Unmanaged.Instance.Data.m_EightK)
            {
                var bits = ((byte*)e)[rune.value >> 3];
                var mask = 1 << (rune.value & 7);
                return (bits & mask) != 0;
            }
        }

        internal static int WidthInCells(in FixedString128 f)
        {
            int widthInCells = 0;
            var offset = 0;
            while (offset < f.Length)
            {
                unsafe
                {
                    fixed(FixedString128* ff = &f)
                    {
                        var error = Unicode.Utf8ToUcs(out Unicode.Rune rune, (byte*)ff + 2, ref offset, f.Length);
                        if (error != 0)
                            return -1;
                        widthInCells += isWide(rune) ? 2 : 1;
                    }
                }
            }
            return widthInCells;
        }
    }
}
