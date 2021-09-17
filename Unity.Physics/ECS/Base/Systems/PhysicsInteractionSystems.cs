using Unity.Entities;

namespace Unity.Physics.Systems
{
    // Represents runtime data of physics (all data stored in PhysicsWorld) - used only to declare reading/writing the data.
    internal struct PhysicsSystemRuntimeData : IComponentData
    {
        byte DummyValue;
    }

    public static class PhysicsRuntimeExtensions
    {
        /// <summary>
        /// Call in your system's OnStartRunning() method if you only want to read physics runtime data from the default physics world
        /// </summary>
        public static void RegisterPhysicsRuntimeSystemReadOnly(this SystemBase system)
        {
            system.GetSingleton<PhysicsSystemRuntimeData>();
        }

        /// <summary>
        /// Call in your system's OnStartRunning() method if you want to read and write physics runtime data from the default physics world
        /// </summary>
        public static void RegisterPhysicsRuntimeSystemReadWrite(this SystemBase system)
        {
            var cData = system.GetSingleton<PhysicsSystemRuntimeData>();
            system.SetSingleton(cData);
        }

        /// <summary>
        /// Call in your system's OnStartRunning() method if you only want to read physics runtime data from a non-default physics world.
        /// Each non-default PhysicsWorld should have its own ComponentData type passed in.
        /// </summary>
        public static void RegisterPhysicsRuntimeSystemReadOnly<T>(this SystemBase system) where T : struct, IComponentData
        {
            system.GetSingleton<T>();
        }

        /// <summary>
        /// Call in your system's OnStartRunning() method if you want to read and write physics runtime data from a non-default physics world.
        /// Each non-default PhysicsWorld should have its own ComponentData type passed in.
        /// </summary>
        public static void RegisterPhysicsRuntimeSystemReadWrite<T>(this SystemBase system) where T : struct, IComponentData
        {
            var cData = system.GetSingleton<T>();
            system.SetSingleton(cData);
        }
    }
}
