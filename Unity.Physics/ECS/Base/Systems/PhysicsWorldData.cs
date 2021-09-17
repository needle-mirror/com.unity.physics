using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// Structure containing PhysicsWorld and other data and queries that are necessary for simulating a physics world.
    /// </summary>
    public struct PhysicsWorldData : IDisposable
    {
        public PhysicsWorld PhysicsWorld;

        public NativeArray<int> HaveStaticBodiesChanged;

        // Entity group queries
        public EntityQuery DynamicEntityGroup;
        public EntityQuery StaticEntityGroup;
        public EntityQuery JointEntityGroup;

        public PhysicsWorldData(EntityManager EntityManager, in PhysicsWorldIndex worldIndex)
        {
            PhysicsWorld = new PhysicsWorld(0, 0, 0);
            HaveStaticBodiesChanged = new NativeArray<int>(1, Allocator.Persistent);

            DynamicEntityGroup = EntityManager.CreateEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(Translation),
                    typeof(Rotation),
                    typeof(PhysicsWorldIndex)
                }
            });
            DynamicEntityGroup.SetSharedComponentFilter(worldIndex);

            StaticEntityGroup = EntityManager.CreateEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsCollider),
                    typeof(PhysicsWorldIndex)
                },
                Any = new ComponentType[]
                {
                    typeof(LocalToWorld),
                    typeof(Translation),
                    typeof(Rotation)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsVelocity)
                }
            });
            StaticEntityGroup.SetSharedComponentFilter(worldIndex);

            JointEntityGroup = EntityManager.CreateEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsConstrainedBodyPair),
                    typeof(PhysicsJoint),
                    typeof(PhysicsWorldIndex)
                }
            });
            JointEntityGroup.SetSharedComponentFilter(worldIndex);
        }

        public void Dispose()
        {
            PhysicsWorld.Dispose();
            HaveStaticBodiesChanged.Dispose();
        }
    }
}
