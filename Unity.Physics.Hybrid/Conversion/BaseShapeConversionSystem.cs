using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    public abstract class BaseShapeConversionSystem<T> : GameObjectConversionSystem where T : Component
    {
        struct ComparableEntity : IEquatable<ComparableEntity>, IComparable<ComparableEntity>
        {
            public Entity Entity;

            public bool Equals(ComparableEntity other) => Entity.Equals(other.Entity);

            public int CompareTo(ComparableEntity other) => Entity.Index - other.Entity.Index;
        }

        struct LeafShapeData
        {
            public CompoundCollider.ColliderBlobInstance ColliderBlobInstance;
            public byte CustomFlags;
        }

        protected abstract bool ShouldConvertShape(T shape);
        protected abstract GameObject GetPrimaryBody(T shape);
        protected abstract BlobAssetReference<Collider> ProduceColliderBlob(T shape);
        protected abstract byte GetCustomFlags(T shape);

        void ConvertShape(T shape)
        {
            if (!ShouldConvertShape(shape))
                return;

            var body = GetPrimaryBody(shape);
            var entity = GetPrimaryEntity(body);

            var colliderBlob = ProduceColliderBlob(shape);

            if (!DstEntityManager.HasComponent<PhysicsCollider>(entity))
                DstEntityManager.AddComponentData(entity, new PhysicsCollider());

            var collider = DstEntityManager.GetComponentData<PhysicsCollider>(entity);
            var customFlags = GetCustomFlags(shape);
            if (!collider.IsValid && body == shape.gameObject)
            {
                // Shape is on same object as body, therefore has no relative transform.
                // Set it directly into the entity component.
                collider.Value = colliderBlob;
                DstEntityManager.SetComponentData(entity, collider);
                if (customFlags != 0)
                    DstEntityManager.AddOrSetComponent(entity, new PhysicsCustomData { Value = customFlags });

                DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(entity, body.transform);
            }
            else
            {
                // Body has multiple shapes or may have a relative transform.
                // Store it for building compound colliders in the second pass.
                // Note: Not including relative scale, since that is baked into the colliders
                var worldFromBody = new RigidTransform(body.transform.rotation, body.transform.position);
                var worldFromShape = new RigidTransform(shape.transform.rotation, shape.transform.position);
                var compoundFromChild = math.mul(math.inverse(worldFromBody), worldFromShape);
                m_ExtraColliders.Add(
                    new ComparableEntity { Entity = entity },
                    new LeafShapeData
                    {
                        ColliderBlobInstance = new CompoundCollider.ColliderBlobInstance
                        {
                            CompoundFromChild = compoundFromChild,
                            Collider = colliderBlob
                        },
                        CustomFlags = customFlags
                    }
                );
            }
        }

        NativeMultiHashMap<ComparableEntity, LeafShapeData> m_ExtraColliders;

        protected override void OnUpdate()
        {
            // A map from entities to arrays of colliders that were not applied to the body during the first pass.
            m_ExtraColliders = new NativeMultiHashMap<ComparableEntity, LeafShapeData>(64, Allocator.Temp); // TODO - add custom data to Physics.Collider?

            // First pass.
            // Convert all editor shape components into colliders, and either apply them to their parent rigid body
            // or store them for building compound colliders in the second pass.
            Entities.ForEach<T>(ConvertShape);

            // Second pass.
            // Merge any leftover colliders into their parent rigid bodies, as compound colliders.
            if (m_ExtraColliders.Length > 0)
            {
                var keys = m_ExtraColliders.GetUniqueKeyArray(Allocator.Temp);
                using (keys.Item1)
                {
                    for (var k = 0; k < keys.Item2; ++k)
                    {
                        ComparableEntity entity = keys.Item1[k];
                        var collider = DstEntityManager.GetComponentData<PhysicsCollider>(entity.Entity);
                        var children = new NativeList<CompoundCollider.ColliderBlobInstance>(16, Allocator.Temp);

                        if (collider.IsValid)
                        {
                            // Include the already assigned collider as a child
                            children.Add(new CompoundCollider.ColliderBlobInstance { Collider = collider.Value, CompoundFromChild = RigidTransform.identity });
                        }

                        byte customData = 0;
                        if (m_ExtraColliders.TryGetFirstValue(entity, out var child, out var iterator))
                        {
                            do
                            {
                                children.Add(child.ColliderBlobInstance);
                                customData &= child.CustomFlags; // TODO: Should be |= ??
                            } while (m_ExtraColliders.TryGetNextValue(out child, ref iterator));
                        }

                        collider.Value = CompoundCollider.Create(children);

                        children.Dispose();

                        DstEntityManager.SetComponentData(entity.Entity, collider);

                        if (customData != 0)
                            DstEntityManager.AddOrSetComponent(entity.Entity, new PhysicsCustomData { Value = customData });
                    }
                }
            }
            m_ExtraColliders.Dispose();
        }
    }
}

