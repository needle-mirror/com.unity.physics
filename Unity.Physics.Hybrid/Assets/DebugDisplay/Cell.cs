using System.Runtime.InteropServices;
using Unity.Collections;

namespace Unity.DebugDisplay
{
    [StructLayout(LayoutKind.Explicit)]
    internal struct Cell
    {
        [FieldOffset(0)] internal byte runelo;
        [FieldOffset(1)] internal byte color;
        [FieldOffset(2)] internal ushort runehi;

        internal const int kMaxDisplayPixelsWide = 4096; // no support for 8K or 16K, yet
        internal const int kMaxDisplayPixelsTall = 4096;
        internal const int kPixelsWide = 8;
        internal const int kPixelsTall = 16;
        internal const int kMaxWide = (kMaxDisplayPixelsWide + kPixelsWide - 1) / kPixelsWide;
        internal const int kMaxTall = (kMaxDisplayPixelsTall + kPixelsTall - 1) / kPixelsTall;
        internal const int kMax = kMaxWide * kMaxTall;

        internal Unicode.Rune rune
        {
            get
            {
                uint value = (uint)(runelo & 0x7f);
                if ((runelo & 0x80) == 0x80)
                    value |= (uint)runehi << 7;
                return new Unicode.Rune {value = (int)value};
            }
            set
            {
                runelo = (byte)(value.value & 0x7f);
                if (value.value > 0x7f)
                    runelo |= 0x80;
                runehi = (ushort)(value.value >> 7);
            }
        }

        internal ColorIndex fg
        {
            get => new ColorIndex {value = (color >> 0) & 0xf};
            set => color = (byte)((color & 0xF0) | ((value.value & 0xF) << 0));
        }

        internal ColorIndex bg
        {
            get => new ColorIndex {value = (color >> 4) & 0x7};
            set => color = (byte)((color & 0x8F) | ((value.value & 0x7) << 4));
        }

        internal bool side
        {
            get => (color & 0x80) == 0x80;
            set
            {
                color &= 0x7F;
                color = (byte)(color | (value ? 0x80 : 0x00));
            }
        }
    }
}
