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

    [AddComponentMenu("Entities/Physics/Physics Debug Display")]
    [DisallowMultipleComponent]
    [HelpURL(HelpURLs.PhysicsDebugDisplayAuthoring)]
    public sealed class PhysicsDebugDisplayAuthoring : MonoBehaviour
    {
        PhysicsDebugDisplayAuthoring() {}

        public bool DrawColliders = false;
        public bool DrawColliderEdges = false;
        public bool DrawColliderAabbs = false;
        public bool DrawBroadphase = false;
        public bool DrawMassProperties = false;
        public bool DrawContacts = false;
        public bool DrawCollisionEvents = false;
        public bool DrawTriggerEvents = false;
        public bool DrawJoints = false;

        internal PhysicsDebugDisplayData AsComponent => new PhysicsDebugDisplayData
        {
            DrawColliders = DrawColliders ? 1 : 0,
            DrawColliderEdges = DrawColliderEdges ? 1 : 0,
            DrawColliderAabbs = DrawColliderAabbs ? 1 : 0,
            DrawBroadphase = DrawBroadphase ? 1 : 0,
            DrawMassProperties = DrawMassProperties ? 1 : 0,
            DrawContacts = DrawContacts ? 1 : 0,
            DrawCollisionEvents = DrawCollisionEvents ? 1 : 0,
            DrawTriggerEvents = DrawTriggerEvents ? 1 : 0,
            DrawJoints = DrawJoints ? 1 : 0,
        };
    }

    public class PhysicsDebugDisplayDataBaker : Baker<PhysicsDebugDisplayAuthoring>
    {
        public override void Bake(PhysicsDebugDisplayAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, authoring.AsComponent);
        }
    }
}
