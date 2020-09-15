using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using static Unity.Physics.PhysicsStep;

namespace Unity.Physics.Authoring
{
    [AddComponentMenu("DOTS/Physics/Physics Step")]
    [DisallowMultipleComponent]
    [HelpURL(HelpURLs.PhysicsStepAuthoring)]
    public sealed class PhysicsStepAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        PhysicsStepAuthoring() { }

        public SimulationType SimulationType
        {
            get => m_SimulationType;
            set => m_SimulationType = value;
        }
        [SerializeField]
        [Tooltip("Specifies the type of the physics simulation to be executed.")]
        SimulationType m_SimulationType = Default.SimulationType;

        public float3 Gravity
        {
            get => m_Gravity;
            set => m_Gravity = value;
        }
        [SerializeField]
        float3 m_Gravity = Default.Gravity;

        public int SolverIterationCount
        {
            get => m_SolverIterationCount;
            set => m_SolverIterationCount = value;
        }
        [SerializeField]
        [Tooltip("Specifies the number of solver iterations the physics engine will perform. Higher values mean more stability, but also worse performance.")]
        int m_SolverIterationCount = Default.SolverIterationCount;

        public bool EnableSolverStabilizationHeuristic
        {
            get => m_EnableSolverStabilizationHeuristic;
            set => m_EnableSolverStabilizationHeuristic = value;
        }
        [SerializeField]
        bool m_EnableSolverStabilizationHeuristic = Default.SolverStabilizationHeuristicSettings.EnableSolverStabilization;

        public int ThreadCountHint
        {
            get => m_ThreadCountHint;
            set => m_ThreadCountHint = value;
        }
        [SerializeField]
        [Tooltip("Specifies the hint to the physics engine about the number of threads the simulation will be executed on. " +
            "Invalid values (<=0) will result in a simulation with very small number of single threaded jobs.")]
        int m_ThreadCountHint = Default.ThreadCountHint;

        public bool SynchronizeCollisionWorld
        {
            get => m_SynchronizeCollisionWorld;
            set => m_SynchronizeCollisionWorld = value;
        }
        [SerializeField]
        [Tooltip("Specifies whether to update the collision world after the step for more precise queries.")]
        bool m_SynchronizeCollisionWorld = Default.SynchronizeCollisionWorld > 0 ? true : false;

        PhysicsStep AsComponent => new PhysicsStep
        {
            SimulationType = SimulationType,
            Gravity = Gravity,
            SolverIterationCount = SolverIterationCount,
            SolverStabilizationHeuristicSettings = EnableSolverStabilizationHeuristic ?
                new Solver.StabilizationHeuristicSettings
                {
                    EnableSolverStabilization = true,
                    EnableFrictionVelocities = Default.SolverStabilizationHeuristicSettings.EnableFrictionVelocities,
                    VelocityClippingFactor = Default.SolverStabilizationHeuristicSettings.VelocityClippingFactor,
                    InertiaScalingFactor = Default.SolverStabilizationHeuristicSettings.InertiaScalingFactor
                } :
                Solver.StabilizationHeuristicSettings.Default,
            ThreadCountHint = ThreadCountHint,
            SynchronizeCollisionWorld = (byte)(SynchronizeCollisionWorld ? 1 : 0)
        };

        Entity m_ConvertedEntity = Entity.Null;
        EntityManager m_ConvertedEntityManager;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(entity, AsComponent);

            m_ConvertedEntity = entity;
            m_ConvertedEntityManager = dstManager;
        }

        void OnValidate()
        {
            SolverIterationCount = math.max(1, SolverIterationCount);

            if (!enabled) return;
            if (gameObject.scene.isSubScene) return;
            if (m_ConvertedEntity == Entity.Null) return;

            // This requires Entity Conversion mode to be 'Convert And Inject Game Object'
            if (m_ConvertedEntityManager.HasComponent<Physics.PhysicsStep>(m_ConvertedEntity))
            {
                m_ConvertedEntityManager.SetComponentData(m_ConvertedEntity, AsComponent);
            }
        }
    }
}
