using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Physics Debug Display Data.<para/>
    ///
    /// Component data containing physics debug display settings.
    /// </summary>
    public struct PhysicsDebugDisplayData : IComponentData
    {
        /// <summary>
        /// Enable or disable collider debug display.
        /// </summary>
        public int DrawColliders;

        /// <summary>
        /// Enable or disable debug display of collider edges.
        /// </summary>
        public int DrawColliderEdges;

        /// <summary>
        /// Enable or disable debug display of the colliders' axis-aligned bounding boxes.
        /// </summary>
        public int DrawColliderAabbs;

        /// <summary>
        /// Enable or disable debug display of the colliders' bounding volume used during the Broadphase.
        /// </summary>
        public int DrawBroadphase;

        /// <summary>
        /// Enable or disable debug display of the mass properties of rigid bodies.
        /// </summary>
        public int DrawMassProperties;

        /// <summary>
        /// Enable or disable debug display of the contacts detected in the Narrowphase.
        /// </summary>
        public int DrawContacts;

        /// <summary>
        /// Enable or disable the collision events debug display.
        /// </summary>
        public int DrawCollisionEvents;

        /// <summary>
        /// Enable or disable the trigger events debug display.
        /// </summary>
        public int DrawTriggerEvents;

        /// <summary>
        /// Enable or disable the joints debug display.
        /// </summary>
        public int DrawJoints;
    }

    /// <summary>
    /// Physics Debug Display Authoring.<para/>
    ///
    /// GameObject component containing physics debug display settings.
    /// </summary>
    [AddComponentMenu("Entities/Physics/Physics Debug Display")]
    [DisallowMultipleComponent]
    [HelpURL(HelpURLs.PhysicsDebugDisplayAuthoring)]
    public sealed class PhysicsDebugDisplayAuthoring : MonoBehaviour
    {
        PhysicsDebugDisplayAuthoring() {}

        /// <summary>
        /// Enable or disable collider debug display.
        /// </summary>
        public bool DrawColliders = false;

        /// <summary>
        /// Enable or disable debug display of collider edges.
        /// </summary>
        public bool DrawColliderEdges = false;

        /// <summary>
        /// Enable or disable debug display of the colliders' axis-aligned bounding boxes.
        /// </summary>
        public bool DrawColliderAabbs = false;

        /// <summary>
        /// Enable or disable debug display of the colliders' bounding volume used during the Broadphase.
        /// </summary>
        public bool DrawBroadphase = false;

        /// <summary>
        /// Enable or disable debug display of the mass properties of rigid bodies.
        /// </summary>
        public bool DrawMassProperties = false;

        /// <summary>
        /// Enable or disable debug display of the contacts detected in the Narrowphase.
        /// </summary>
        public bool DrawContacts = false;

        /// <summary>
        /// Enable or disable the collision events debug display.
        /// </summary>
        public bool DrawCollisionEvents = false;

        /// <summary>
        /// Enable or disable the trigger events debug display.
        /// </summary>
        public bool DrawTriggerEvents = false;

        /// <summary>
        /// Enable or disable the joints debug display.
        /// </summary>
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

    /// <summary>
    /// Baker for the <see cref="PhysicsDebugDisplayAuthoring>"/>
    /// </summary>
    internal class PhysicsDebugDisplayDataBaker : Baker<PhysicsDebugDisplayAuthoring>
    {
        /// <summary>
        /// Baking method to create the <see cref="PhysicsDebugDisplayAuthoring"/> component data.
        /// </summary>
        public override void Bake(PhysicsDebugDisplayAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, authoring.AsComponent);
        }
    }
}
