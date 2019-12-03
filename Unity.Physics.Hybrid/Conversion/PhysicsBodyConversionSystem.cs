using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(EndColliderConversionSystem))]
    public sealed class PhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach(
                (StaticOptimizeEntity staticOptimized) =>
                {
                    var entity = GetPrimaryEntity(staticOptimized.gameObject);
                    if (DstEntityManager.HasComponent<PhysicsCollider>(entity))
                        DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(entity, staticOptimized.transform);
                }
            );
            Entities.ForEach(
                (PhysicsBodyAuthoring body) =>
                {
                    var entity = GetPrimaryEntity(body.gameObject);

                    DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(entity, body.transform);

                    var customTags = body.CustomTags;
                    if (!customTags.Equals(CustomPhysicsBodyTags.Nothing))
                        DstEntityManager.AddOrSetComponent(entity, new PhysicsCustomTags { Value = customTags.Value });

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
