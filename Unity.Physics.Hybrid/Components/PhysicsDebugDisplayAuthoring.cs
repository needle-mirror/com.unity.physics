using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    public struct PhysicsDebugDisplayData : IComponentData
    {
        public int DrawColliders;
        public int DrawColliderEdges;
        public int DrawColliderAabbs;
        public int DrawBroadphase;
        public int DrawMassProperties;
        public int DrawContacts;
        public int DrawCollisionEvents;
        public int DrawTriggerEvents;
        public int DrawJoints;
    }

    [AddComponentMenu("DOTS/Physics/Physics Debug Display")]
    [DisallowMultipleComponent]
    [RequiresEntityConversion]
    public sealed class PhysicsDebugDisplayAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        PhysicsDebugDisplayAuthoring() { }

        public bool DrawColliders = false;
        public bool DrawColliderEdges = false;
        public bool DrawColliderAabbs = false;
        public bool DrawBroadphase = false;
        public bool DrawMassProperties = false;
        public bool DrawContacts = false;
        public bool DrawCollisionEvents = false;
        public bool DrawTriggerEvents = false;
        public bool DrawJoints = false;

        private PhysicsDebugDisplayData AsComponent => new PhysicsDebugDisplayData
        {
            DrawColliders = DrawColliders ? 1 : 0,
            DrawColliderEdges = DrawColliderEdges ? 1 : 0,
            DrawColliderAabbs = DrawColliderAabbs ? 1 : 0,
            DrawBroadphase = DrawBroadphase ? 1 : 0,
            DrawMassProperties = DrawMassProperties ? 1 : 0,
            DrawContacts = DrawContacts ? 1 : 0,
            DrawCollisionEvents = DrawCollisionEvents ? 1 : 0,
            DrawTriggerEvents = DrawTriggerEvents ? 1 : 0,
            DrawJoints = DrawJoints ? 1 : 0
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
