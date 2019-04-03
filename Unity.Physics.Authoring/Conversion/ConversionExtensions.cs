using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    static class ConversionExtensions
    {
        internal static void AddOrSetComponent<T>(this EntityManager manager, Entity entity, T value)
            where T : struct, IComponentData
        {
            if (!manager.HasComponent<T>(entity))
                manager.AddComponentData(entity, value);
            else if (!TypeManager.IsZeroSized(TypeManager.GetTypeIndex<T>()))
                manager.SetComponentData(entity, value);
        }

        internal static float3[] GetScaledVertices(this UnityEngine.Mesh mesh, float3 scale)
        {
            var source = mesh.vertices;
            var vertices = new float3[mesh.vertices.Length]; 
            for (int i = 0, count = vertices.Length; i < count; ++i)
                vertices[i] = source[i] * scale;
            return vertices;
        }
    }    
}

