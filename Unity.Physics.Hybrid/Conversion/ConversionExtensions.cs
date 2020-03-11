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


        internal static void PostProcessTransformComponents(
            this EntityManager manager, Entity entity, Transform worldTransform, BodyMotionType motionType
        )
        {
            var transformParent     = manager.HasComponent<Parent>(entity) ? worldTransform.parent : null;
            var haveParentEntity    = transformParent != null; // TODO: revisit what to do in subscenes where everything is implicitly converted
            var haveBakedTransform  = worldTransform.GetComponent<StaticOptimizeEntity>() != null;
            var unparent            = motionType != BodyMotionType.Static || !haveParentEntity || haveBakedTransform;
            
            // ensure dynamic and kinematic bodies translation/rotation are in world space
            // ensure static optimized entities have translation/rotation so they can be processed more efficiently
            if (!unparent)
                return;
            
            var rigidBodyTransform = Math.DecomposeRigidBodyTransform(worldTransform.localToWorldMatrix);

            manager.RemoveComponent<Parent>(entity);
            manager.RemoveComponent<LocalToParent>(entity);
            manager.AddOrSetComponent(entity, new Translation { Value = rigidBodyTransform.pos });
            manager.AddOrSetComponent(entity, new Rotation { Value = rigidBodyTransform.rot });

            if (math.lengthsq((float3)worldTransform.lossyScale - new float3(1f)) > 0f)
            {
                // bake in composite scale
                var compositeScale = math.mul(
                    math.inverse(new float4x4(rigidBodyTransform)),
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

