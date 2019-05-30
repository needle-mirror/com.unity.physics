using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // A collection of rigid bodies wrapped by a bounding volume hierarchy.
    // This allows to do collision queries such as raycasting, overlap testing, etc.
    public struct CollisionWorld : ICollidable, IDisposable, ICloneable
    {
        private NativeArray<RigidBody> m_Bodies;    // storage for the rigid bodies
        private int m_NumBodies;                    // number of rigid bodies currently in use

        public Broadphase Broadphase;               // a bounding volume hierarchy around the rigid bodies

        public NativeSlice<RigidBody> Bodies => new NativeSlice<RigidBody>(m_Bodies, 0, m_NumBodies);

        public int NumBodies
        {
            get => m_NumBodies;
            set
            {
                m_NumBodies = value;
                if (m_Bodies.Length < m_NumBodies)
                {
                    m_Bodies.Dispose();
                    m_Bodies = new NativeArray<RigidBody>(m_NumBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                }
            }
        }

        // Construct a collision world with the given number of uninitialized rigid bodies
        public CollisionWorld(int numBodies)
        {
            m_Bodies = new NativeArray<RigidBody>(numBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_NumBodies = numBodies;
            Broadphase = new Broadphase();
            Broadphase.Init();
        }

        // Free internal memory
        public void Dispose()
        {
            m_Bodies.Dispose();
            Broadphase.Dispose();
        }

        // Clone the world (except the colliders)
        public object Clone()
        {
            CollisionWorld clone = new CollisionWorld
            {
                m_Bodies = new NativeArray<RigidBody>(m_Bodies.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_NumBodies = m_NumBodies,
                Broadphase = (Broadphase)Broadphase.Clone()
            };
            clone.m_Bodies.CopyFrom(m_Bodies);
            return clone;
        }

        #region Jobs

        internal struct DiposeArrayJob : IJob
        {
            [DeallocateOnJobCompletion] public NativeArray<int> Array;
            public void Execute() {}
        }

        // Schedule a set of jobs to synchronize the collision world with the dynamics world.
        public JobHandle ScheduleUpdateDynamicLayer(ref PhysicsWorld world, float timeStep, int numThreadsHint, JobHandle inputDeps)
        {
            JobHandle handle = new UpdateRigidBodyTransformsJob
            {
                MotionDatas = world.MotionDatas,
                MotionVelocities = world.MotionVelocities,
                RigidBodies = m_Bodies
            }.Schedule(world.MotionDatas.Length, 32, inputDeps);

            StaticLayerChangeInfo staticLayerChangeInfo = new StaticLayerChangeInfo();
            staticLayerChangeInfo.Init(Allocator.TempJob);

            // TODO: Instead of a full build we could probably incrementally update the existing broadphase,
            // since the number of bodies will be the same and their positions should be similar.
            handle = Broadphase.ScheduleBuildJobs(ref world, timeStep, numThreadsHint, ref staticLayerChangeInfo, inputDeps: handle);

            return JobHandle.CombineDependencies(
                new DiposeArrayJob { Array = staticLayerChangeInfo.HaveStaticBodiesChangedArray }.Schedule(handle),
                new DiposeArrayJob { Array = staticLayerChangeInfo.NumStaticBodiesArray }.Schedule(handle));
        }

        [BurstCompile]
        private struct UpdateRigidBodyTransformsJob : IJobParallelFor
        {
            [ReadOnly] public NativeSlice<MotionData> MotionDatas;
            [ReadOnly] public NativeSlice<MotionVelocity> MotionVelocities;
            public NativeSlice<RigidBody> RigidBodies;

            public void Execute(int i)
            {
                RigidBody rb = RigidBodies[i];
                rb.WorldFromBody = math.mul(MotionDatas[i].WorldFromMotion, math.inverse(MotionDatas[i].BodyFromMotion));
                RigidBodies[i] = rb;
            }
        }

        #endregion

        #region ICollidable implementation

        public Aabb CalculateAabb()
        {
            return Broadphase.Domain;
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            return TransformAabb(transform, Broadphase.Domain);
        }

        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return Broadphase.CastRay(input, m_Bodies, ref collector);
        }

        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return Broadphase.CastCollider(input, m_Bodies, ref collector);
        }

        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Broadphase.CalculateDistance(input, m_Bodies, ref collector);
        }

        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Broadphase.CalculateDistance(input, m_Bodies, ref collector);
        }

        #endregion
        
        // Test input against the broadphase tree, filling allHits with the body indices of every overlap.
        // Returns true if there was at least overlap.
        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits)
        {
            int hitsBefore = allHits.Length;
            Broadphase.OverlapAabb(input, m_Bodies, ref allHits);
            return allHits.Length > hitsBefore;
        }
    }
}
