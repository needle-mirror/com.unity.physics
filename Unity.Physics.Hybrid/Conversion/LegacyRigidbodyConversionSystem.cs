#if LEGACY_PHYSICS
using Unity.Entities;
using Unity.Physics.GraphicsIntegration;
using UnityEngine;
using LegacyRigidBody = UnityEngine.Rigidbody;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(PhysicsBodyConversionSystem))]
    public sealed class LegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach(
                (LegacyRigidBody body) =>
                {
                    var entity = GetPrimaryEntity(body.gameObject);

                    // prefer conversions from non-legacy data if they have already been performed
                    if (DstEntityManager.HasComponent<PhysicsVelocity>(entity))
                        return;

                    DstEntityManager.PostProcessTransformComponents(
                        entity, body.transform,
                        body.isKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic
                    );

                    if (body.gameObject.isStatic)
                        return;

                    if (body.interpolation != RigidbodyInterpolation.None)
                    {
                        DstEntityManager.AddOrSetComponent(entity, new PhysicsGraphicalSmoothing());
                        if (body.interpolation == RigidbodyInterpolation.Interpolate)
                        {
                            DstEntityManager.AddComponentData(entity, new PhysicsGraphicalInterpolationBuffer
                            {
                                PreviousTransform = Math.DecomposeRigidBodyTransform(body.transform.localToWorldMatrix)
                            });
                        }
                    }

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
#endif
