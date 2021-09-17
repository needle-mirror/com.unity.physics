using Unity.Entities;

namespace Unity.Physics.GraphicsIntegration
{
    /// <summary>
    /// Stores a direct association to another Entity holding a graphical representation
    /// of a physics shape referenced by this Entity.
    /// This component is usually added where the edit time structure has separate hierarchies
    /// for the graphics and physical representations. i.e. a node that held a <see cref="PhysicsShape"/>
    /// component but no <see cref="MeshRenderer"/> component.
    /// </summary>
    public struct PhysicsRenderEntity : IComponentData
    {
        /// <summary>
        /// An Entity containing the graphical representation of a physics shape.
        /// </summary>
        public Entity Entity;
    }
}
