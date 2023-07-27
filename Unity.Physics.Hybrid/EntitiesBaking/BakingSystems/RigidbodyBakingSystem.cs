using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.GraphicsIntegration;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Component that specifies if a rigid body is a kinematic body and what its mass is.
    /// </summary>
    [TemporaryBakingType]
    public struct RigidbodyBakingData : IComponentData
    {
        /// <summary> Sets rigid body as kinematic body. </summary>
        public bool isKinematic;

        /// <summary> Mass of the rigid body. </summary>
        public float mass;

        /// <summary> Center of mass of the rigid body is automatically calculated if this is true. </summary>
        public bool automaticCenterOfMass;

        /// <summary> Center of mass of the rigid body. Used if automaticCenterOfMass is false. </summary>
        public float3 centerOfMass;

        /// <summary> Inertia tensor of the rigid body is automatically calculated if this is true. </summary>
        public bool automaticInertiaTensor;

        /// <summary> Inertia tensor of the rigid body. Used if automaticInertiaTensor is false. </summary>
        public float3 inertiaTensor;

        /// <summary> Rotation of the inertia tensor. Used if automaticInertiaTensor is false. </summary>
        public quaternion inertiaTensorRotation;
    }

    /// <summary>
    /// A baker for a Rigidbody component
    /// </summary>
    class RigidbodyBaker : BasePhysicsBaker<Rigidbody>
    {
        static List<UnityEngine.Collider> colliderComponents = new List<UnityEngine.Collider>();

        public override void Bake(Rigidbody authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var bakingData = new RigidbodyBakingData
            {
                isKinematic = authoring.isKinematic,
                mass = authoring.mass,
                automaticCenterOfMass = authoring.automaticCenterOfMass,
                centerOfMass = authoring.centerOfMass,
                automaticInertiaTensor = authoring.automaticInertiaTensor,
                inertiaTensor = authoring.inertiaTensor,
                inertiaTensorRotation = authoring.inertiaTensorRotation
            };
            AddComponent(entity, bakingData);

            AddSharedComponent(entity, new PhysicsWorldIndex());

            var bodyTransform = GetComponent<Transform>();

            var motionType = authoring.isKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic;
            var hasInterpolation = authoring.interpolation != RigidbodyInterpolation.None;
            PostProcessTransform(bodyTransform, motionType);

            // Check that there is at least one collider in the hierarchy to add these three
            GetComponentsInChildren(colliderComponents);
            if (colliderComponents.Count > 0)
            {
                AddComponent(entity, new PhysicsCompoundData()
                {
                    AssociateBlobToBody = false,
                    ConvertedBodyInstanceID = authoring.GetInstanceID(),
                    Hash = default,
                });
                AddComponent<PhysicsRootBaked>(entity);
                AddComponent<PhysicsCollider>(entity);
            }

            // Ignore the rest if the object is static
            if (IsStatic())
                return;

            if (hasInterpolation)
            {
                AddComponent(entity, new PhysicsGraphicalSmoothing());

                if (authoring.interpolation == RigidbodyInterpolation.Interpolate)
                {
                    AddComponent(entity, new PhysicsGraphicalInterpolationBuffer
                    {
                        PreviousTransform = Math.DecomposeRigidBodyTransform(bodyTransform.localToWorldMatrix)
                    });
                }
            }

            // Add default PhysicsMass component. The actual mass properties values will be set by the RigidbodyBakingSystem.
            var massProperties = MassProperties.UnitSphere;
            AddComponent(entity, !authoring.isKinematic ?
                PhysicsMass.CreateDynamic(massProperties, authoring.mass) :
                PhysicsMass.CreateKinematic(massProperties));

            AddComponent(entity, new PhysicsVelocity());

            if (!authoring.isKinematic)
            {
                AddComponent(entity, new PhysicsDamping
                {
                    Linear = authoring.drag,
                    Angular = authoring.angularDrag
                });
                if (!authoring.useGravity)
                    AddComponent(entity, new PhysicsGravityFactor { Value = 0f });
            }
            else
                AddComponent(entity, new PhysicsGravityFactor { Value = 0 });
        }
    }

    /// <summary>
    /// Represents a rigidbody baking system.
    /// </summary>
    [RequireMatchingQueriesForUpdate]
    [UpdateAfter(typeof(EndColliderBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class RigidbodyBakingSystem : SystemBase
    {
        static SimulationMode k_InvalidSimulationMode = (SimulationMode) ~0;
        SimulationMode m_SavedSmulationMode = k_InvalidSimulationMode;
        bool m_ProcessSimulationModeChange;

        protected override void OnCreate()
        {
            m_ProcessSimulationModeChange = Application.isPlaying;
        }

        protected override void OnDestroy()
        {
            // Unless no legacy physics step data is available, restore previously stored legacy physics simulation mode
            // when leaving play-mode.
            if (m_SavedSmulationMode != k_InvalidSimulationMode)
            {
                UnityEngine.Physics.simulationMode = m_SavedSmulationMode;
                m_SavedSmulationMode = k_InvalidSimulationMode;
            }
        }

        protected override void OnUpdate()
        {
            if (m_ProcessSimulationModeChange)
            {
                // When entering playmode, cache and override legacy physics simulation mode, disabling legacy physics and this way
                // preventing it from running while playing with open sub-scenes. Otherwise, the legacy physics simulation
                // will overwrite the last known edit mode state of game objects in the sub-scene with its simulation results.
                m_SavedSmulationMode = UnityEngine.Physics.simulationMode;
                UnityEngine.Physics.simulationMode = SimulationMode.Script;

                m_ProcessSimulationModeChange = false;
            }

            // Set world index for bodies with world index baking data
            var ecb = new EntityCommandBuffer(WorldUpdateAllocator);
            foreach (var(rigidBodyData, worldIndexData, entity) in
                     SystemAPI.Query<RefRO<RigidbodyBakingData>, RefRO<PhysicsWorldIndexBakingData>>()
                         .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities).WithEntityAccess())
            {
                ecb.SetSharedComponent(entity, new PhysicsWorldIndex(worldIndexData.ValueRO.WorldIndex));
            }

            // Set mass properties for rigid bodies without collider
            foreach (var(physicsMass, bodyData) in
                     SystemAPI.Query<RefRW<PhysicsMass>, RefRO<RigidbodyBakingData>>().WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities))
            {
                physicsMass.ValueRW = CreatePhysicsMass(bodyData.ValueRO, MassProperties.UnitSphere);
            }

            // Set mass properties for rigid bodies with collider
            foreach (var(physicsMass, bodyData, collider) in
                     SystemAPI.Query<RefRW<PhysicsMass>, RefRO<RigidbodyBakingData>, RefRO<PhysicsCollider>>().WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities))
            {
                physicsMass.ValueRW = CreatePhysicsMass(bodyData.ValueRO, collider.ValueRO.MassProperties);
            }

            ecb.Playback(EntityManager);
            ecb.Dispose();
        }

        private PhysicsMass CreatePhysicsMass(in RigidbodyBakingData inBodyData, in MassProperties inMassProperties)
        {
            var massProperties = inMassProperties;
            if (!inBodyData.automaticCenterOfMass)
            {
                massProperties.MassDistribution.Transform.pos = inBodyData.centerOfMass;
            }

            if (!inBodyData.automaticInertiaTensor)
            {
                massProperties.MassDistribution.InertiaTensor = inBodyData.inertiaTensor;
                massProperties.MassDistribution.Transform.rot = inBodyData.inertiaTensorRotation;
            }

            return !inBodyData.isKinematic ?
                PhysicsMass.CreateDynamic(massProperties, inBodyData.mass) :
                PhysicsMass.CreateKinematic(massProperties);
        }
    }
}
