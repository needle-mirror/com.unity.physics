using System;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    internal struct TextBuffer : IDisposable
    {
        internal struct Instance
        {
            internal float3 worldPosition;
            internal float2 firstCell;
            internal float2 cellSize;
            internal uint useWorldMatrix;
        }

        internal const int kMaxInstances = 128;
        internal const int kMaxColors = 16;

        internal UnsafeArray<Instance> m_Instance;
        internal CellSurface m_Screen;

        internal unsafe void ClearCells()
        {
            m_Screen.Clear();
        }

        internal void Initialize()
        {
            m_Instance = new UnsafeArray<Instance>(kMaxInstances);
            m_Screen = new CellSurface(new int2(Cell.kMaxWide, Cell.kMaxTall));
        }

        internal void ClearTextBox(int index)
        {
            m_Instance[index] = new Instance();
        }

        internal void SetTextBox(int2 cellXY, int2 cellWH, int index)
        {
            m_Instance[index] = new Instance
            {
                worldPosition = new float3(cellXY.x * Cell.kPixelsWide, cellXY.y * Cell.kPixelsTall, 0.0f),
                firstCell = new float2(cellXY),
                cellSize = new float2(cellWH),
                useWorldMatrix = 0
            };
        }

        internal void SetTextBoxSmooth(float3 position, int2 cellXY, int2 cellWH, int index)
        {
            m_Instance[index] = new Instance
            {
                worldPosition = position,
                firstCell = new float2(cellXY),
                cellSize = new float2(cellWH),
                useWorldMatrix = 1
            };
        }

        internal void SetLabel(float3 position, int2 cellXY, in FixedString128 f, ColorIndex fg, ColorIndex bg,
            int index)
        {
            SetTextBoxSmooth(position, cellXY, new int2(CellSurface.WidthInCells(f), 1), index);
            m_Screen.PutChars(ref cellXY, f, fg, bg);
        }

        public void Dispose()
        {
            m_Instance.Dispose();
            m_Screen.Dispose();
        }

        internal Unit AllocateAll()
        {
            return new Unit(0, 1, 0, m_Instance.Length);
        }
    }
}
