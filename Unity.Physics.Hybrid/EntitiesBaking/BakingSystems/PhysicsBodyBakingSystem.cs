using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.GraphicsIntegration;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [TemporaryBakingType]
    public struct PhysicsBodyAuthoringData : IComponentData
    {
        public bool IsDynamic;
        public float Mass;
        public bool OverrideDefaultMassDistribution;
        public MassDistribution CustomMassDistribution;
    }

    class PhysicsBodyAuthoringBaker : BasePhysicsBaker<PhysicsBodyAuthoring>
    {
        public static List<UnityEngine.Collider> colliderComponents = new List<UnityEngine.Collider>();
        public static List<PhysicsShapeAuthoring> physicsShapeComponents = new List<PhysicsShapeAuthoring>();

        public override void Bake(PhysicsBodyAuthoring authoring)
        {
            // To process later in the Baking System
            AddComponent(new PhysicsBodyAuthoringData
            {
                IsDynamic = (authoring.MotionType == BodyMotionType.Dynamic),
                Mass = authoring.Mass,
                OverrideDefaultMassDistribution = authoring.OverrideDefaultMassDistribution,
                CustomMassDistribution = authoring.CustomMassDistribution
            });

            AddSharedComponent(new PhysicsWorldIndex(authoring.WorldIndex));

            var bodyTransform = GetComponent<Transform>();

            var motionType = authoring.MotionType;
            PostProcessTransform(bodyTransform, motionType);

            var customTags = authoring.CustomTags;
            if (!customTags.Equals(CustomPhysicsBodyTags.Nothing))
                AddComponent(new PhysicsCustomTags { Value = customTags.Value });

            // Check that there is at least one collider in the hierarchy to add these three
            GetComponentsInChildren(colliderComponents);
            GetComponentsInChildren(physicsShapeComponents);
            if (colliderComponents.Count > 0 || physicsShapeComponents.Count > 0)
            {
                AddComponent(new PhysicsCompoundData()
                {
                    AssociateBlobToBody = false,
                    ConvertedBodyInstanceID = authoring.GetInstanceID(),
                    Hash = default,
                });
                AddComponent<PhysicsRootBaked>();
                AddComponent<PhysicsCollider>();
                AddBuffer<PhysicsColliderKeyEntityPair>();
            }

            if (authoring.MotionType == BodyMotionType.Static || IsStatic())
                return;

            var massProperties = MassProperties.UnitSphere;

            AddComponent(authoring.MotionType == BodyMotionType.Dynamic ?
                PhysicsMass.CreateDynamic(massProperties, authoring.Mass) :
                PhysicsMass.CreateKinematic(massProperties));

            var physicsVelocity = new PhysicsVelocity
            {
                Linear = authoring.InitialLinearVelocity,
                Angular = authoring.InitialAngularVelocity
            };
            AddComponent(physicsVelocity);

            if (authoring.MotionType == BodyMotionType.Dynamic)
            {
                // TODO make these optional in editor?
                AddComponent(new PhysicsDamping
                {
                    Linear = authoring.LinearDamping,
                    Angular = authoring.AngularDamping
                });
                if (authoring.GravityFactor != 1)
                {
                    AddComponent(new PhysicsGravityFactor
                    {
                        Value = authoring.GravityFactor
                    });
                }
            }
            else if (authoring.MotionType == BodyMotionType.Kinematic)
            {
                AddComponent(new PhysicsGravityFactor
                {
                    Value = 0
                });
            }

            if (authoring.Smoothing != BodySmoothing.None)
            {
                AddComponent(new PhysicsGraphicalSmoothing());
                if (authoring.Smoothing == BodySmoothing.Interpolation)
                {
                    AddComponent(new PhysicsGraphicalInterpolationBuffer
                    {
                        PreviousTransform = Math.DecomposeRigidBodyTransform(bodyTransform.localToWorldMatrix),
                        PreviousVelocity = physicsVelocity,
                    });
                }
            }
        }
    }

    [RequireMatchingQueriesForUpdate]
    [UpdateAfter(typeof(EndColliderBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class PhysicsBodyBakingSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            // Fill in the MassProperties based on the potential calculated value by BuildCompoundColliderBakingSystem
            Entities
                .ForEach(
                (ref PhysicsMass physicsMass, in PhysicsBodyAuthoringData bodyData, in PhysicsCollider collider) =>
                {
                    // Build mass component
                    var massProperties = collider.MassProperties;
                    if (bodyData.OverrideDefaultMassDistribution)
                    {
                        massProperties.MassDistribution = bodyData.CustomMassDistribution;
                        // Increase the angular expansion factor to account for the shift in center of mass
                        massProperties.AngularExpansionFactor += math.length(massProperties.MassDistribution.Transform.pos - bodyData.CustomMassDistribution.Transform.pos);
                    }

                    physicsMass = bodyData.IsDynamic ?
                        PhysicsMass.CreateDynamic(massProperties, bodyData.Mass) :
                        PhysicsMass.CreateKinematic(massProperties);
                }).WithEntityQueryOptions(EntityQueryOptions.IncludePrefab).Run();
        }
    }
}
