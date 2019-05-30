using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
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

        internal static void RemoveParentAndSetWorldTranslationAndRotation(this EntityManager manager, Entity entity, Transform worldTransform)
        {
            manager.RemoveComponent<Parent>(entity);
            manager.RemoveComponent<LocalToParent>(entity);
            manager.AddOrSetComponent(entity, new Translation { Value = worldTransform.position });
            manager.AddOrSetComponent(entity, new Rotation { Value = worldTransform.rotation });
            if (math.lengthsq((float3)worldTransform.lossyScale - new float3(1f)) > 0f)
            {
                // bake in composite scale
                var compositeScale = math.mul(
                    math.inverse(float4x4.TRS(worldTransform.position, worldTransform.rotation, 1f)),
                    worldTransform.localToWorldMatrix
                );
                manager.AddOrSetComponent(entity, new CompositeScale { Value = compositeScale });
            }
            // TODO: revisit whether or not NonUniformScale/Scale should be preserved along with ParentScaleInverse instead
            manager.RemoveComponent<NonUniformScale>(entity);
            manager.RemoveComponent<Scale>(entity);
        }
    }    
}

