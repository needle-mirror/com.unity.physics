using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(LegacyBoxColliderConversionSystem))]
    [UpdateAfter(typeof(LegacyCapsuleColliderConversionSystem))]
    [UpdateAfter(typeof(LegacySphereColliderConversionSystem))]
    [UpdateAfter(typeof(LegacyMeshColliderConversionSystem))]
    [UpdateAfter(typeof(PhysicsShapeConversionSystem))]
    [UpdateBefore(typeof(LegacyRigidbodyConversionSystem))]
    [DisableAutoCreation]
    [Obsolete("SecondPassPhysicsBodyConversionSystem has been deprecated. Use PhysicsBodyConversionSystem instead. (RemovedAfter 2019-08-28) (UnityUpgradable) -> PhysicsBodyConversionSystem", true)]
    public class SecondPassPhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }

    [UpdateAfter(typeof(LegacyBoxColliderConversionSystem))]
    [UpdateAfter(typeof(LegacyCapsuleColliderConversionSystem))]
    [UpdateAfter(typeof(LegacySphereColliderConversionSystem))]
    [UpdateAfter(typeof(LegacyMeshColliderConversionSystem))]
    [UpdateAfter(typeof(PhysicsShapeConversionSystem))]
    [UpdateBefore(typeof(LegacyRigidbodyConversionSystem))]
    public class PhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach(
                (PhysicsBody body) =>
                {
                    var entity = GetPrimaryEntity(body.gameObject);

                    DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(entity, body.transform);

                    if (body.MotionType == BodyMotionType.Static)
                        return;

                    var massProperties = MassProperties.UnitSphere;
                    if (DstEntityManager.HasComponent<PhysicsCollider>(entity))
                    {
                        // Build mass component
                        massProperties = DstEntityManager.GetComponentData<PhysicsCollider>(entity).MassProperties;
                    }
                    if (body.OverrideDefaultMassDistribution)
                    {
                        massProperties.MassDistribution = body.CustomMassDistribution;
                        // Increase the angular expansion factor to account for the shift in center of mass
                        massProperties.AngularExpansionFactor += math.length(massProperties.MassDistribution.Transform.pos - body.CustomMassDistribution.Transform.pos);
                    }
                    DstEntityManager.AddOrSetComponent(entity, body.MotionType == BodyMotionType.Dynamic ?
                        PhysicsMass.CreateDynamic(massProperties, body.Mass) :
                        PhysicsMass.CreateKinematic(massProperties));
                    
                    DstEntityManager.AddOrSetComponent(entity, new PhysicsVelocity
                    {
                        Linear = body.InitialLinearVelocity,
                        Angular = body.InitialAngularVelocity
                    });

                    if (body.MotionType == BodyMotionType.Dynamic)
                    {
                        // TODO make these optional in editor?
                        DstEntityManager.AddOrSetComponent(entity, new PhysicsDamping
                        {
                            Linear = body.LinearDamping,
                            Angular = body.AngularDamping
                        });
                        if (body.GravityFactor != 1)
                        {
                            DstEntityManager.AddOrSetComponent(entity, new PhysicsGravityFactor
                            {
                                Value = body.GravityFactor
                            });
                        }
                    }
                    else if (body.MotionType == BodyMotionType.Kinematic)
                    {
                        DstEntityManager.AddOrSetComponent(entity, new PhysicsGravityFactor
                        {
                            Value = 0
                        });
                    }
                }
            );
        }
    }
}
