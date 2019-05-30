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
    public class PhysicsDebugDisplay : MonoBehaviour, IConvertGameObjectToEntity
    {
        public bool DrawColliders = false;
        public bool DrawColliderEdges = false;
        public bool DrawColliderAabbs = false;
        public bool DrawBroadphase = false;
        public bool DrawMassProperties = false;
        public bool DrawContacts = false;
        public bool DrawCollisionEvents = false;
        public bool DrawTriggerEvents = false;
        public bool DrawJoints = false;

        private Entity convertedEntity = Entity.Null;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            var componentData = new PhysicsDebugDisplayData
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
            dstManager.AddComponentData<PhysicsDebugDisplayData>(entity, componentData);

            convertedEntity = entity;
        }

        void OnValidate()
        {
            if (!enabled) return;
            if (convertedEntity == Entity.Null) return;

            // This requires Entity Conversion mode to be 'Convert And Inject Game Object'
            var entityManager = World.Active.EntityManager;
            if (entityManager.HasComponent<PhysicsDebugDisplayData>(convertedEntity))
            {
                var component = entityManager.GetComponentData<PhysicsDebugDisplayData>(convertedEntity);
                component.DrawColliders = DrawColliders ? 1 : 0;
                component.DrawColliderEdges = DrawColliderEdges ? 1 : 0;
                component.DrawColliderAabbs = DrawColliderAabbs ? 1 : 0;
                component.DrawBroadphase = DrawBroadphase ? 1 : 0;
                component.DrawMassProperties = DrawMassProperties ? 1 : 0;
                component.DrawContacts = DrawContacts ? 1 : 0;
                component.DrawCollisionEvents = DrawCollisionEvents ? 1 : 0;
                component.DrawTriggerEvents = DrawTriggerEvents ? 1 : 0;
                component.DrawJoints = DrawJoints ? 1 : 0;
                entityManager.SetComponentData<PhysicsDebugDisplayData>(convertedEntity, component);
            }
        }
    }
}
