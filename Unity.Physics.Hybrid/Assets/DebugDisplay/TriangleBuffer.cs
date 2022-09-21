using System;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    unsafe internal struct TriangleBuffer : IDisposable
    {
        const int kMaxLines = 99999;

        internal struct Instance
        {
            internal float4 m_vertex0;
            internal float4 m_vertex1;
            internal float4 m_vertex2;
            internal float3 normal;
        }

        internal UnsafeArray<Instance> m_Instance;

        internal void Initialize()
        {
            m_Instance = new UnsafeArray<Instance>(kMaxLines);
        }

        internal void SetTriangle(float3 vertex0, float3 vertex1, float3 vertex2, float3 normal, Unity.DebugDisplay.ColorIndex colorIndex, int index)
        {
            m_Instance[index] = new Instance
            {
                m_vertex0 = new float4(vertex0, colorIndex.value),
                m_vertex1 = new float4(vertex1, colorIndex.value),
                m_vertex2 = new float4(vertex2, colorIndex.value),
                normal = new float3(normal.x, normal.y, normal.z)
            };
        }

        internal void ClearTriangle(int index)
        {
            m_Instance[index] = new Instance {};
        }

        public void Dispose()
        {
            m_Instance.Dispose();
        }

        internal Unit AllocateAll()
        {
            return new Unit(m_Instance.Length);
        }
    }
}
