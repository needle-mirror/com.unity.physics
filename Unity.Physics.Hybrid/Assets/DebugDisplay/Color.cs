namespace Unity.DebugDisplay
{
    internal struct ColorIndex
    {
        internal int value;
        internal static readonly ColorIndex Black = new ColorIndex{value = 0};
        internal static readonly ColorIndex Red = new ColorIndex{value = 1};
        internal static readonly ColorIndex Green = new ColorIndex{value = 2};
        internal static readonly ColorIndex Yellow = new ColorIndex{value = 3};
        internal static readonly ColorIndex Blue = new ColorIndex{value = 4};
        internal static readonly ColorIndex Magenta = new ColorIndex{value = 5};
        internal static readonly ColorIndex Cyan = new ColorIndex{value = 6};
        internal static readonly ColorIndex White = new ColorIndex{value = 7};
        internal static readonly ColorIndex BrightBlack = new ColorIndex{value = 8};
        internal static readonly ColorIndex BrightRed = new ColorIndex{value = 9};
        internal static readonly ColorIndex BrightGreen = new ColorIndex{value = 10};
        internal static readonly ColorIndex BrightYellow = new ColorIndex{value = 11};
        internal static readonly ColorIndex BrightBlue = new ColorIndex{value = 12};
        internal static readonly ColorIndex BrightMagenta = new ColorIndex{value = 13};
        internal static readonly ColorIndex BrightCyan = new ColorIndex{value = 14};
        internal static readonly ColorIndex BrightWhite = new ColorIndex{value = 15};

        internal static ColorIndex Foreground(int value)
        {
            return new ColorIndex {value = (value >> 0) & 0xf};
        }

        internal static ColorIndex Background(int value)
        {
            return new ColorIndex {value = (value >> 4) & 0x7};
        }
    }
}
