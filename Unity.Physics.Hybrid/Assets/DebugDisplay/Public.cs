using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    internal struct Color
    {
        internal static ColorIndex Quantize(float4 rgba)
        {
            var oldi = 0;
            var oldd = math.lengthsq(rgba - Unmanaged.Instance.Data.m_ColorData[0]);
            for (var i = 1; i < Unmanaged.kMaxColors; ++i)
            {
                var newd = math.lengthsq(rgba - Unmanaged.Instance.Data.m_ColorData[0]);
                if (newd < oldd)
                {
                    oldi = i;
                    oldd = newd;
                }
            }
            return new ColorIndex {value = oldi};
        }
    }

    internal struct Arrows : IDisposable
    {
        private Lines m_Lines;
        internal Arrows(int count)
        {
            m_Lines = new Lines(count * 5);
        }

        internal void Draw(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            var X0 = x;
            var X1 = x + v;

            m_Lines.Draw(X0, X1, color);

            float3 dir;
            float length = Physics.Math.NormalizeWithLength(v, out dir);
            float3 perp, perp2;
            Physics.Math.CalculatePerpendicularNormalized(dir, out perp, out perp2);
            float3 scale = length * 0.2f;

            m_Lines.Draw(X1, X1 + (perp - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp + dir) * scale, color);
            m_Lines.Draw(X1, X1 + (perp2 - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp2 + dir) * scale, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Arrow
    {
        internal static void Draw(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            new Arrows(1).Draw(x, v, color);
        }
    }

    internal struct Planes : IDisposable
    {
        private Lines m_Lines;
        internal Planes(int count)
        {
            m_Lines = new Lines(count * 9);
        }

        internal void Draw(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            var X0 = x;
            var X1 = x + v;

            m_Lines.Draw(X0, X1, color);

            float3 dir;
            float length = Physics.Math.NormalizeWithLength(v, out dir);
            float3 perp, perp2;
            Physics.Math.CalculatePerpendicularNormalized(dir, out perp, out perp2);
            float3 scale = length * 0.2f;

            m_Lines.Draw(X1, X1 + (perp - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp + dir) * scale, color);
            m_Lines.Draw(X1, X1 + (perp2 - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp2 + dir) * scale, color);

            perp *= length;
            perp2 *= length;

            m_Lines.Draw(X0 + perp + perp2, X0 + perp - perp2, color);
            m_Lines.Draw(X0 + perp - perp2, X0 - perp - perp2, color);
            m_Lines.Draw(X0 - perp - perp2, X0 - perp + perp2, color);
            m_Lines.Draw(X0 - perp + perp2, X0 + perp + perp2, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Plane
    {
        internal static void Draw(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            new Planes(1).Draw(x, v, color);
        }
    }

    internal struct Arcs : IDisposable
    {
        private Lines m_Lines;
        const int res = 16;

        internal Arcs(int count)
        {
            m_Lines = new Lines(count * (2 + res));
        }

        internal void Draw(float3 center, float3 normal, float3 arm, float angle, Unity.DebugDisplay.ColorIndex color)
        {
            quaternion q = quaternion.AxisAngle(normal, angle / res);
            float3 currentArm = arm;
            m_Lines.Draw(center, center + currentArm, color);
            for (int i = 0; i < res; i++)
            {
                float3 nextArm = math.mul(q, currentArm);
                m_Lines.Draw(center + currentArm, center + nextArm, color);
                currentArm = nextArm;
            }
            m_Lines.Draw(center, center + currentArm, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Arc
    {
        internal static void Draw(float3 center, float3 normal, float3 arm, float angle,
            Unity.DebugDisplay.ColorIndex color)
        {
            new Arcs(1).Draw(center, normal, arm, angle, color);
        }
    }

    internal struct Boxes : IDisposable
    {
        private Lines m_Lines;

        internal Boxes(int count)
        {
            m_Lines = new Lines(count * 12);
        }

        internal void Draw(float3 Size, float3 Center, quaternion Orientation, Unity.DebugDisplay.ColorIndex color)
        {
            float3x3 mat = math.float3x3(Orientation);
            float3 x = mat.c0 * Size.x * 0.5f;
            float3 y = mat.c1 * Size.y * 0.5f;
            float3 z = mat.c2 * Size.z * 0.5f;
            float3 c0 = Center - x - y - z;
            float3 c1 = Center - x - y + z;
            float3 c2 = Center - x + y - z;
            float3 c3 = Center - x + y + z;
            float3 c4 = Center + x - y - z;
            float3 c5 = Center + x - y + z;
            float3 c6 = Center + x + y - z;
            float3 c7 = Center + x + y + z;

            m_Lines.Draw(c0, c1, color); // ring 0
            m_Lines.Draw(c1, c3, color);
            m_Lines.Draw(c3, c2, color);
            m_Lines.Draw(c2, c0, color);

            m_Lines.Draw(c4, c5, color); // ring 1
            m_Lines.Draw(c5, c7, color);
            m_Lines.Draw(c7, c6, color);
            m_Lines.Draw(c6, c4, color);

            m_Lines.Draw(c0, c4, color); // between rings
            m_Lines.Draw(c1, c5, color);
            m_Lines.Draw(c2, c6, color);
            m_Lines.Draw(c3, c7, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Box
    {
        internal static void Draw(float3 Size, float3 Center, quaternion Orientation, Unity.DebugDisplay.ColorIndex color)
        {
            new Boxes(1).Draw(Size, Center, Orientation, color);
        }
    }

    internal struct Cones : IDisposable
    {
        private Lines m_Lines;
        const int res = 16;

        internal Cones(int count)
        {
            m_Lines = new Lines(count * res * 2);
        }

        internal void Draw(float3 point, float3 axis, float angle, Unity.DebugDisplay.ColorIndex color)
        {
            float3 dir;
            float scale = Physics.Math.NormalizeWithLength(axis, out dir);
            float3 arm;
            {
                float3 perp1, perp2;
                Physics.Math.CalculatePerpendicularNormalized(dir, out perp1, out perp2);
                arm = math.mul(quaternion.AxisAngle(perp1, angle), dir) * scale;
            }
            quaternion q = quaternion.AxisAngle(dir, 2.0f * (float)math.PI / res);

            for (int i = 0; i < res; i++)
            {
                float3 nextArm = math.mul(q, arm);
                m_Lines.Draw(point, point + arm, color);
                m_Lines.Draw(point + arm, point + nextArm, color);
                arm = nextArm;
            }
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Cone
    {
        internal static void Draw(float3 point, float3 axis, float angle, Unity.DebugDisplay.ColorIndex color)
        {
            new Cones(1).Draw(point, axis, angle, color);
        }
    }

    internal struct Lines : IDisposable
    {
        Unit m_Unit;
        internal Lines(int count)
        {
            DebugDisplay.Instantiate();
            m_Unit = Unmanaged.Instance.Data.m_LineBufferAllocations.AllocateAtomic(count);
        }

        internal void Draw(float3 begin, float3 end, ColorIndex color)
        {
            if (m_Unit.m_Next < m_Unit.m_End)
                Unmanaged.Instance.Data.m_LineBuffer.SetLine(begin, end, color, m_Unit.m_Next++);
        }

        public void Dispose()
        {
            while (m_Unit.m_Next < m_Unit.m_End)
                Unmanaged.Instance.Data.m_LineBuffer.ClearLine(m_Unit.m_Next++);
        }
    }
    internal struct Line
    {
        internal static void Draw(float3 begin, float3 end, ColorIndex color)
        {
            new Lines(1).Draw(begin, end, color);
        }
    }

    internal struct TextBox
    {
        internal static void Draw(int2 xy, int2 wh)
        {
            DebugDisplay.Instantiate();
            var unit = Unmanaged.Instance.Data.m_TextBufferAllocations.AllocateAtomic(1);
            if (unit.Length == 0)
                return;
            var text = Unmanaged.Instance.Data.m_TextBuffer;
            text.SetTextBox(xy, wh, unit.m_Next);
        }
    }

    internal struct Label
    {
        internal static void Draw(float3 position, in FixedString128 s, ColorIndex fg, ColorIndex bg)
        {
            DebugDisplay.Instantiate();
            var unit = Unmanaged.Instance.Data.m_TextBufferAllocations.AllocateAtomic(1);
            if (unit.Length == 0)
                return;
            var text = Unmanaged.Instance.Data.m_TextBuffer;
            text.SetLabel(position, new int2(0, text.m_Screen.m_Size.y + unit.m_Next), s, fg, bg, unit.m_Next);
        }
    }

    internal struct Text
    {
        internal static void Draw(int x, int y, byte color, in FixedString128 f)
        {
            DebugDisplay.Instantiate();
            var xy = new int2(x, y);
            Unmanaged.Instance.Data.m_TextBuffer.m_Screen.PutChars(ref xy, f, ColorIndex.Foreground(color), ColorIndex.Background(color));
        }
    }

    internal struct Log
    {
        internal static void Info(in FixedString128 f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Debug(in FixedString128 f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Error(in FixedString128 f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Warn(in FixedString128 f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Draw(Window w)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.CopyToWithFrame(ref Unmanaged.Instance.Data.m_TextBuffer.m_Screen, w);
        }
    }

    internal struct GraphData
    {
        Unit m_unit;

        internal int Length => m_unit.Length;

        internal GraphData(int count)
        {
            DebugDisplay.Instantiate();
            m_unit = Unmanaged.Instance.Data.m_GraphDataBufferAllocations.AllocateAtomic(count);
        }

        internal float GetValue(int offset)
        {
            return Unmanaged.Instance.Data.m_GraphBuffer.m_Data[m_unit.m_Begin + (offset & (m_unit.Length - 1))];
        }

        internal void SetValue(int offset, float value)
        {
            Unmanaged.Instance.Data.m_GraphBuffer.m_Data[m_unit.m_Begin + (offset & (m_unit.Length - 1))] = value;
        }

        internal unsafe ref float this[int offset] =>
            ref ((float*)Unmanaged.Instance.Data.m_GraphBuffer.m_Data.GetUnsafePtr())[m_unit.m_Begin + (offset & (m_unit.Length - 1))];

        internal void AddValue(float value)
        {
            if (m_unit.m_Next >= m_unit.m_End)
                m_unit.m_Next = m_unit.m_Begin;
            Unmanaged.Instance.Data.m_GraphBuffer.m_Data[m_unit.m_Next++] = value;
        }

        internal Data GetData()
        {
            return new Data { offset = m_unit.m_Begin, length = m_unit.Length };
        }

        internal unsafe void CalcMinMaxMean(out float minValue, out float maxValue, out float mean)
        {
            float* f = (float*)Unmanaged.Instance.Data.m_GraphBuffer.m_Data.GetUnsafePtr();
            minValue = f[m_unit.m_Begin];
            maxValue = f[m_unit.m_Begin];
            float sum = f[m_unit.m_Begin];
            for (var i = m_unit.m_Begin + 1; i < m_unit.m_End; i++)
            {
                var x = f[i];
                sum += x;
                if (x < minValue) minValue = x;
                if (x > maxValue) maxValue = x;
            }

            mean = sum / m_unit.Length;
        }

        internal unsafe void CalcStatistics(out float mean, out float variance, out float minValue,
            out float maxValue)
        {
            CalcMinMaxMean(out minValue, out maxValue, out mean);
            float* f = (float*)Unmanaged.Instance.Data.m_GraphBuffer.m_Data.GetUnsafePtr();
            float sum2 = 0;
            for (var i = m_unit.m_Begin; i < m_unit.m_End; i++)
            {
                float d = f[i] - mean;
                sum2 += d * d;
            }

            variance = sum2 / m_unit.Length;
        }
    }

    internal struct Graph
    {
        internal int Color;
        internal Unit DataSeries0;
        internal Unit DataSeries1;
        internal float YMin;
        internal float YMax;

        internal Graph(int color, float yMin, float yMax, int Length0, int Length1 = 0)
        {
            DebugDisplay.Instantiate();
            Color = color;
            DataSeries0 = Unmanaged.Instance.Data.m_GraphDataBufferAllocations.AllocateAtomic(Length0);
            DataSeries1 = Unmanaged.Instance.Data.m_GraphDataBufferAllocations.AllocateAtomic(Length1);
            YMin = yMin;
            YMax = yMax;
        }

        internal void Draw(Window window, int frame)
        {
            var unit = Unmanaged.Instance.Data.m_GraphBufferAllocations.AllocateAtomic(1);
            if (unit.Length == 0)
                return;
            var x = window.m_begin.x;
            var y = window.m_begin.y;
            var w = window.Size.x;
            var h = window.Size.y;
            if (DataSeries1.Length > 0)
                Unmanaged.Instance.Data.m_GraphBuffer.SetGraph(x, y, w, h, new GraphBuffer.Sample
                {
                    data = DataSeries0, ColorIndex = ColorIndex.Foreground(Color), xMin = frame - 1 - w * 8, xMax = frame - 1,
                    yMin = YMin, yMax = YMax
                }, new GraphBuffer.Sample
                    {
                        data = DataSeries1, ColorIndex = ColorIndex.Background(Color), xMin = frame - 1 - w * 8, xMax = frame - 1,
                        yMin = YMin, yMax = YMax
                    }, unit.m_Next);
            else
                Unmanaged.Instance.Data.m_GraphBuffer.SetGraph(x, y, w, h, new GraphBuffer.Sample
                {
                    data = DataSeries0, ColorIndex = ColorIndex.Foreground(Color), xMin = frame - 1 - w * 8, xMax = frame - 1,
                    yMin = YMin, yMax = YMax
                }, unit.m_Next);
        }
    }

    internal class DebugDisplay
    {
        internal static void Render()
        {
            Managed.Instance.CopyFromCpuToGpu();
            Managed.Instance.Render();
        }

        internal static void Clear()
        {
            Managed.Instance.Clear();
        }

        [BurstDiscard]
        internal static void Instantiate()
        {
            if (Managed.Instance == null)
                Managed.Instance = new Managed();
        }
    }
}
