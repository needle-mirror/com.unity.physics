using System;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Physics
{
    // A collection of rigid bodies and joints.
    public struct PhysicsWorld : ICollidable, IDisposable, ICloneable
    {
        public CollisionWorld CollisionWorld;   // stores rigid bodies and broadphase
        public DynamicsWorld DynamicsWorld;     // stores motions and joints

        public int NumBodies => CollisionWorld.NumBodies;
        public int NumDynamicBodies => DynamicsWorld.NumMotions;
        public int NumStaticBodies => CollisionWorld.NumBodies - DynamicsWorld.NumMotions;
        public int NumJoints => DynamicsWorld.NumJoints;

        public NativeSlice<RigidBody> Bodies => CollisionWorld.Bodies;
        public NativeSlice<RigidBody> StaticBodies => new NativeSlice<RigidBody>(CollisionWorld.Bodies, DynamicsWorld.NumMotions, CollisionWorld.NumBodies - DynamicsWorld.NumMotions);
        public NativeSlice<RigidBody> DynamicBodies => new NativeSlice<RigidBody>(CollisionWorld.Bodies, 0, DynamicsWorld.NumMotions);
        public NativeSlice<MotionData> MotionDatas => DynamicsWorld.MotionDatas;
        public NativeSlice<MotionVelocity> MotionVelocities => DynamicsWorld.MotionVelocities;
        public NativeSlice<Joint> Joints => DynamicsWorld.Joints;

        public float CollisionTolerance => 0.1f; // todo - make this configurable?

        // Construct a world with the given number of uninitialized bodies and joints
        public PhysicsWorld(int numStaticBodies, int numDynamicBodies, int numJoints)
        {
            CollisionWorld = new CollisionWorld(numDynamicBodies + numStaticBodies);
            DynamicsWorld = new DynamicsWorld(numDynamicBodies, numJoints);
        }

        // Reset the number of bodies and joints in the world
        public void Reset(int numStaticBodies, int numDynamicBodies, int numJoints)
        {
            CollisionWorld.NumBodies = numDynamicBodies + numStaticBodies;
            DynamicsWorld.NumMotions = numDynamicBodies;
            DynamicsWorld.NumJoints = numJoints;
        }

        // Free internal memory
        public void Dispose()
        {
            CollisionWorld.Dispose();
            DynamicsWorld.Dispose();
        }

        // Clone the world
        public object Clone() => new PhysicsWorld
        {
            CollisionWorld = (CollisionWorld)CollisionWorld.Clone(),
            DynamicsWorld = (DynamicsWorld)DynamicsWorld.Clone()
        };

        #region ICollidable implementation

        public Aabb CalculateAabb()
        {
            return CollisionWorld.CalculateAabb();
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            return CollisionWorld.CalculateAabb(transform);
        }

        // Cast ray
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return CollisionWorld.CastRay(input, ref collector);
        }

        // Cast collider
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return CollisionWorld.CastCollider(input, ref collector);
        }

        // Point distance
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return CollisionWorld.CalculateDistance(input, ref collector);
        }

        // Collider distance
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return CollisionWorld.CalculateDistance(input, ref collector);
        }

        #endregion
    }
}
