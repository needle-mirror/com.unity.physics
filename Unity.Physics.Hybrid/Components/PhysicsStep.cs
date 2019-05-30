using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using static Unity.Physics.PhysicsStep;

namespace Unity.Physics.Authoring
{
    [AddComponentMenu("DOTS/Physics/Physics Step")]
    [DisallowMultipleComponent]
    [RequiresEntityConversion]
    public class PhysicsStep : MonoBehaviour, IConvertGameObjectToEntity
    {
        public SimulationType SimulationType = Default.SimulationType;
        public float3 Gravity = Default.Gravity;
        public int SolverIterationCount = Default.SolverIterationCount;
        public int ThreadCountHint = Default.ThreadCountHint;

        private Entity convertedEntity = Entity.Null;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            var componentData = new Physics.PhysicsStep
            {
                SimulationType = SimulationType,
                Gravity = Gravity,
                SolverIterationCount = SolverIterationCount,
                ThreadCountHint = ThreadCountHint
            };
            dstManager.AddComponentData(entity, componentData);

            convertedEntity = entity;
        }

        void OnValidate()
        {
            SolverIterationCount = math.max(1, SolverIterationCount);
            ThreadCountHint = math.max(1, ThreadCountHint);

            if (!enabled) return;
            if (convertedEntity == Entity.Null) return;

            // This requires Entity Conversion mode to be 'Convert And Inject Game Object'
            var entityManager = World.Active.EntityManager;
            if (entityManager.HasComponent<Physics.PhysicsStep>(convertedEntity))
            {
                var component = entityManager.GetComponentData<Physics.PhysicsStep>(convertedEntity);
                component.SimulationType = SimulationType;
                component.Gravity = Gravity;
                component.SolverIterationCount = SolverIterationCount;
                component.ThreadCountHint = ThreadCountHint;
                entityManager.SetComponentData<Physics.PhysicsStep>(convertedEntity, component);
            }
        }
    }
}
