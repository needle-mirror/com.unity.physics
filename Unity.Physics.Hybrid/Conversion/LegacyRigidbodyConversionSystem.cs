using System;
using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(PhysicsBodyConversionSystem))]
    [DisableAutoCreation]
    [Obsolete("SecondPassLegacyRigidbodyConversionSystem has been deprecated. Use LegacyRigidbodyConversionSystem instead. (RemovedAfter 2019-08-28) (UnityUpgradable) -> LegacyRigidbodyConversionSystem", true)]
    public class SecondPassLegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }

    [UpdateAfter(typeof(PhysicsBodyConversionSystem))]
    public class LegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach(
                (Rigidbody body) =>
                {
                    var entity = GetPrimaryEntity(body.gameObject);

                    DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(entity, body.transform);

                    // prefer conversions from non-legacy data if they have already been performed
                    if (DstEntityManager.HasComponent<PhysicsVelocity>(entity))
                        return;

                    if (body.gameObject.isStatic)
                        return;

                    // Build mass component
                    var massProperties = MassProperties.UnitSphere;
                    if (DstEntityManager.HasComponent<PhysicsCollider>(entity))
                    {
                        // Build mass component
                        massProperties = DstEntityManager.GetComponentData<PhysicsCollider>(entity).MassProperties;
                    }
                    // n.b. no way to know if CoM was manually adjusted, so all legacy Rigidbody objects use auto CoM
                    DstEntityManager.AddOrSetComponent(entity, !body.isKinematic ?
                        PhysicsMass.CreateDynamic(massProperties, body.mass) :
                        PhysicsMass.CreateKinematic(massProperties));

                    DstEntityManager.AddOrSetComponent(entity, new PhysicsVelocity());

                    if (!body.isKinematic)
                    {
                        DstEntityManager.AddOrSetComponent(entity, new PhysicsDamping
                        {
                            Linear = body.drag,
                            Angular = body.angularDrag
                        });
                        if (!body.useGravity)
                            DstEntityManager.AddOrSetComponent(entity, new PhysicsGravityFactor { Value = 0f });
                    }
                    else
                        DstEntityManager.AddOrSetComponent(entity, new PhysicsGravityFactor { Value = 0 });
                }
            );
        }
    }
}
