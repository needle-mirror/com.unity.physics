#if LEGACY_PHYSICS
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.GraphicsIntegration;
using Unity.Transforms;
using UnityEngine;
using LegacyRigidBody = UnityEngine.Rigidbody;
using LegacyCollider = UnityEngine.Collider;
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;

namespace Unity.Physics.Authoring
{
    [TemporaryBakingType]
    public struct LegacyRigidBodyBakingData : IComponentData
    {
        public bool isKinematic;
        public float mass;
    }

    class LegacyRigidBodyBaker : BasePhysicsBaker<LegacyRigidBody>
    {
        public static List<UnityEngine.Collider> colliderComponents = new List<UnityEngine.Collider>();
        public static List<PhysicsShapeAuthoring> physicsShapeComponents = new List<PhysicsShapeAuthoring>();

        public override void Bake(LegacyRigidBody authoring)
        {
            AddComponent(new LegacyRigidBodyBakingData
            {
                isKinematic = authoring.isKinematic,
                mass = authoring.mass
            });

            // PhysicsBodyAuthoring has priority, ignore the LegacyRigidBody
            if (GetComponent<PhysicsBodyAuthoring>() != null)
                return;

            AddSharedComponent(new PhysicsWorldIndex());

            var bodyTransform = GetComponent<Transform>();

            var motionType = authoring.isKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic;
            PostProcessTransform(bodyTransform, motionType);

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

            // Ignore the rest if the object is static
            if (IsStatic())
                return;

            if (authoring.interpolation != RigidbodyInterpolation.None)
            {
                AddComponent(new PhysicsGraphicalSmoothing());
                if (authoring.interpolation == RigidbodyInterpolation.Interpolate)
                {
                    AddComponent(new PhysicsGraphicalInterpolationBuffer
                    {
                        PreviousTransform = Math.DecomposeRigidBodyTransform(bodyTransform.localToWorldMatrix)
                    });
                }
            }

            // We add the component here with default values, so it can be reverted when the baker rebakes
            // The values will be rewritten in the system if needed
            var massProperties = MassProperties.UnitSphere;
            AddComponent(!authoring.isKinematic ?
                PhysicsMass.CreateDynamic(massProperties, authoring.mass) :
                PhysicsMass.CreateKinematic(massProperties));

            AddComponent(new PhysicsVelocity());

            if (!authoring.isKinematic)
            {
                AddComponent(new PhysicsDamping
                {
                    Linear = authoring.drag,
                    Angular = authoring.angularDrag
                });
                if (!authoring.useGravity)
                    AddComponent(new PhysicsGravityFactor { Value = 0f });
            }
            else
                AddComponent(new PhysicsGravityFactor { Value = 0 });
        }
    }

    [RequireMatchingQueriesForUpdate]
    [UpdateAfter(typeof(PhysicsBodyBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class LegacyRigidbodyBakingSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            // Fill in the MassProperties based on the potential calculated value by BuildCompoundColliderBakingSystem
            Entities
                .ForEach(
                (ref PhysicsMass physicsMass, in LegacyRigidBodyBakingData bodyData, in PhysicsCollider collider) =>
                {
                    // Build mass component
                    var massProperties = collider.MassProperties;

                    // n.b. no way to know if CoM was manually adjusted, so all legacy Rigidbody objects use auto CoM
                    physicsMass = !bodyData.isKinematic ?
                        PhysicsMass.CreateDynamic(massProperties, bodyData.mass) :
                        PhysicsMass.CreateKinematic(massProperties);
                }).WithEntityQueryOptions(EntityQueryOptions.IncludePrefab).Run();
        }
    }
}
#endif
