using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using static Unity.Physics.PhysicsStep;

namespace Unity.Physics.Authoring
{
    [AddComponentMenu("DOTS/Physics/Physics Step")]
    [DisallowMultipleComponent]
    [RequiresEntityConversion]
    public sealed class PhysicsStepAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        PhysicsStepAuthoring() { }

        public SimulationType SimulationType = Default.SimulationType;
        public float3 Gravity = Default.Gravity;
        public int SolverIterationCount = Default.SolverIterationCount;
        public int ThreadCountHint = Default.ThreadCountHint;

        private Physics.PhysicsStep AsComponent => new Physics.PhysicsStep
        {
            SimulationType = SimulationType,
            Gravity = Gravity,
            SolverIterationCount = SolverIterationCount,
            ThreadCountHint = ThreadCountHint
        };

        private Entity m_ConvertedEntity = Entity.Null;
        private EntityManager m_ConvertedEntityManager = null;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(entity, AsComponent);

            m_ConvertedEntity = entity;
            m_ConvertedEntityManager = dstManager;
        }

        void OnValidate()
        {
            SolverIterationCount = math.max(1, SolverIterationCount);
            ThreadCountHint = math.max(1, ThreadCountHint);

            if (!enabled) return;
            if (m_ConvertedEntity == Entity.Null) return;

            // This requires Entity Conversion mode to be 'Convert And Inject Game Object'
            if (m_ConvertedEntityManager.HasComponent<Physics.PhysicsStep>(m_ConvertedEntity))
            {
                m_ConvertedEntityManager.SetComponentData(m_ConvertedEntity, AsComponent);
            }
        }
    }
}
