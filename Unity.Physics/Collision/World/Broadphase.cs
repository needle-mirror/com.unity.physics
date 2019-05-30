
using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.BoundingVolumeHierarchy;
using static Unity.Physics.Math;


namespace Unity.Physics
{
    // A bounding volume around a collection of rigid bodies
    public struct Broadphase : IDisposable, ICloneable
    {
        private Tree m_StaticTree;  // The tree of static rigid bodies
        private Tree m_DynamicTree; // The tree of dynamic rigid bodies
        private NativeArray<CollisionFilter> m_BodyFilters; // A copy of the rigid body's filters

        public Tree StaticTree => m_StaticTree;
        public Tree DynamicTree => m_DynamicTree;
        public Aabb Domain => Aabb.Union(m_StaticTree.BoundingVolumeHierarchy.Domain, m_DynamicTree.BoundingVolumeHierarchy.Domain);

        public void Init()
        {
            m_StaticTree.Init();
            m_DynamicTree.Init();
            m_BodyFilters = new NativeArray<CollisionFilter>(0, Allocator.Persistent, NativeArrayOptions.ClearMemory);
        }

        public void Dispose()
        {
            m_StaticTree.Dispose();
            m_DynamicTree.Dispose();
            if (m_BodyFilters.IsCreated)
            {
                m_BodyFilters.Dispose();
            }
        }

        public object Clone()
        {
            var clone = new Broadphase
            {
                m_StaticTree = (Tree)m_StaticTree.Clone(),
                m_DynamicTree = (Tree)m_DynamicTree.Clone(),
                m_BodyFilters = new NativeArray<CollisionFilter>(m_BodyFilters.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory)
            };
            clone.m_BodyFilters.CopyFrom(m_BodyFilters);
            return clone;
        }

        /// <summary>
        /// Schedule building of the static layer in broadphase.
        /// Note: If param staticLayerChangeInfo.HaveStaticBodiesChanged indicates static layer needs to be rebuilt,
        /// than Adjust* jobs will early out and Build* jobs will be executed.
        /// Otherwise if staticLayerChangeInfo indicates that rebuild is not required,
        /// Adjust* jobs will be executed and Build* will early out in runtime.
        /// </summary>        
        public unsafe JobHandle ScheduleStaticTreeBuildJobs(
            ref PhysicsWorld world, int numThreadsHint, ref StaticLayerChangeInfo staticLayerChangeInfo,
            NativeArray<CollisionFilter> previousFrameBodyFilters, JobHandle inputDeps)
        {
            JobHandle handle = inputDeps;

            m_StaticTree.NodeCount = world.NumStaticBodies + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;
            int dynamicBodyDiffFromPreviousFrame = world.NumDynamicBodies - m_DynamicTree.BodyCount;

            // Fix up the static tree.
            JobHandle adjustBodyIndices = new AdjustBodyIndicesJob
            {
                StaticNodes = m_StaticTree.Nodes,
                NumStaticNodes = m_StaticTree.Nodes.Length,
                NumDynamicBodiesDiff = dynamicBodyDiffFromPreviousFrame,
                HaveStaticBodiesChanged = staticLayerChangeInfo.HaveStaticBodiesChangedArray
            }.Schedule(handle);

            // Fix up body filters for static bodies.
            JobHandle adjustStaticBodyFilters = new AdjustStaticBodyFilters
            {
                NumDynamicBodiesDiff = dynamicBodyDiffFromPreviousFrame,
                PreviousFrameBodyFilters = previousFrameBodyFilters,
                BodyFilters = m_BodyFilters,
                NumStaticBodies = world.NumStaticBodies,
                NumDynamicBodies = world.NumDynamicBodies,
                HaveStaticBodiesChanged = staticLayerChangeInfo.HaveStaticBodiesChangedArray
            }.Schedule(handle);

            handle = JobHandle.CombineDependencies(adjustBodyIndices, adjustStaticBodyFilters);

            var aabbs = new NativeArray<Aabb>(world.NumBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var lookup = new NativeArray<PointAndIndex>(world.NumStaticBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            handle = new BuildStaticBodyDataJob
            {
                RigidBodies = world.StaticBodies,
                Aabbs = aabbs,
                FiltersOut = new NativeSlice<CollisionFilter>(m_BodyFilters, world.NumDynamicBodies, world.NumStaticBodies),
                Lookup = lookup,
                Offset = world.NumDynamicBodies,
                AabbMargin = world.CollisionTolerance / 2.0f // each body contributes half,
            }.ScheduleUnsafeIndex0(staticLayerChangeInfo.NumStaticBodiesArray, 32, handle);

            return m_StaticTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                lookup, aabbs, m_BodyFilters, staticLayerChangeInfo.HaveStaticBodiesChangedArray, numThreadsHint, handle,
                m_StaticTree.NodeCount, m_StaticTree.Ranges, m_StaticTree.m_BranchCount);
        }

        internal struct DisposeArrayJob : IJob
        {
            [DeallocateOnJobCompletion] public NativeArray<int> Array;
            public void Execute() {}
        }

        private JobHandle ScheduleDynamicTreeBuildJobs(ref PhysicsWorld world, float timeStep, int numThreadsHint, JobHandle inputDeps)
        {
            JobHandle handle = inputDeps;
            var aabbs = new NativeArray<Aabb>(world.NumBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var lookup = new NativeArray<PointAndIndex>(world.NumDynamicBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var shouldDoWork = new NativeArray<int>(1, Allocator.TempJob);
            shouldDoWork[0] = 1;

            handle = new BuildDynamicBodyDataJob
            {
                RigidBodies = world.DynamicBodies,
                MotionVelocities = world.MotionVelocities,
                Aabbs = aabbs,
                FiltersOut = new NativeSlice<CollisionFilter>(m_BodyFilters, 0, world.NumDynamicBodies),
                Lookup = lookup,
                AabbMargin = world.CollisionTolerance / 2.0f, // each body contributes half
                TimeStep = timeStep
            }.Schedule(world.NumDynamicBodies, 32, handle);

            m_DynamicTree.NodeCount = world.NumDynamicBodies + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;

            handle = m_DynamicTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                lookup, aabbs, m_BodyFilters, shouldDoWork, numThreadsHint, handle,
                m_DynamicTree.NodeCount, m_DynamicTree.Ranges, m_DynamicTree.m_BranchCount);

            return new DisposeArrayJob { Array = shouldDoWork }.Schedule(handle);
        }

        /// <summary>
        /// Schedule a set of jobs to build the broadphase based on the given world.
        /// </summary>       
        public JobHandle ScheduleBuildJobs(ref PhysicsWorld world, float timeStep, int numThreadsHint, ref StaticLayerChangeInfo staticLayerChangeInfo, JobHandle inputDeps)
        {
            var previousFrameBodyFilters = new NativeArray<CollisionFilter>(m_BodyFilters, Allocator.TempJob);

            if (world.NumBodies > m_BodyFilters.Length)
            {
                m_BodyFilters.Dispose();
                m_BodyFilters = new NativeArray<CollisionFilter>(world.NumBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            m_StaticTree.BodyCount = world.NumStaticBodies;
            JobHandle staticTree = ScheduleStaticTreeBuildJobs(ref world, numThreadsHint, ref staticLayerChangeInfo, previousFrameBodyFilters, inputDeps);

            m_DynamicTree.BodyCount = world.NumDynamicBodies;
            JobHandle dynamicTree = inputDeps;
            if (world.NumDynamicBodies > 0)
            {
                dynamicTree = ScheduleDynamicTreeBuildJobs(ref world, timeStep, numThreadsHint, inputDeps);
            }

            return JobHandle.CombineDependencies(staticTree, dynamicTree);
        }


        // Schedule a set of jobs which will write all overlapping body pairs to the given steam,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public JobHandle ScheduleFindOverlapsJobs(out BlockStream dynamicVsDynamicPairsStream, out BlockStream staticVsDynamicPairsStream, ref Simulation.Context context, JobHandle inputDeps)
        {
            var dynamicVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);
            var staticVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);

            var allocateDeps = new AllocateDynamicVsStaticNodePairs
            {
                dynamicVsDynamicNodePairIndices = dynamicVsDynamicNodePairIndices,
                staticVsDynamicNodePairIndices = staticVsDynamicNodePairIndices,
                dynamicBranchCount = m_DynamicTree.m_BranchCount,
                staticBranchCount = m_StaticTree.m_BranchCount
            }.Schedule(inputDeps);

            // Build pairs of branch node indices
            JobHandle dynamicVsDynamicPairs = new DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = m_DynamicTree.Ranges,
                NumBranches = m_DynamicTree.m_BranchCount,
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(allocateDeps);

            JobHandle staticVsDynamicPairs = new StaticVsDynamicBuildBranchNodePairsJob
            {
                DynamicRanges = m_DynamicTree.Ranges,
                StaticRanges = m_StaticTree.Ranges,
                NumStaticBranches = m_StaticTree.m_BranchCount,
                NumDynamicBranches = m_DynamicTree.m_BranchCount,
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(allocateDeps);

            //@TODO: We only need a dependency on allocateDeps, but the safety system doesn't understand that we can not change length list in DynamicVsDynamicBuildBranchNodePairsJob & StaticVsDynamicBuildBranchNodePairsJob
            //       if this is a performance issue we can use [NativeDisableContainerSafetyRestriction] on DynamicVsDynamicBuildBranchNodePairsJob & StaticVsDynamicBuildBranchNodePairsJob 
            JobHandle dynamicConstruct = BlockStream.ScheduleConstruct(out dynamicVsDynamicPairsStream, dynamicVsDynamicNodePairIndices, 0x0a542b34, dynamicVsDynamicPairs);
            JobHandle staticConstruct = BlockStream.ScheduleConstruct(out staticVsDynamicPairsStream, staticVsDynamicNodePairIndices, 0x0a542666, staticVsDynamicPairs);

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            JobHandle dynamicVsDynamicHandle = new DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicNodes = m_DynamicTree.Nodes,
                BodyFilters = m_BodyFilters,
                DynamicNodeFilters = m_DynamicTree.NodeFilters,
                PairWriter = dynamicVsDynamicPairsStream,
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(dynamicVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(dynamicVsDynamicPairs, dynamicConstruct));

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            JobHandle staticVsDynamicHandle = new StaticVsDynamicFindOverlappingPairsJob
            {
                StaticNodes = m_StaticTree.Nodes,
                DynamicNodes = m_DynamicTree.Nodes,
                BodyFilters = m_BodyFilters,
                StaticNodeFilters = m_StaticTree.NodeFilters,
                DynamicNodeFilters = m_DynamicTree.NodeFilters,
                PairWriter = staticVsDynamicPairsStream,
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(staticVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(staticVsDynamicPairs, staticConstruct));

            // Dispose node pair lists
            context.DisposeOverlapPairs0 = NativeListUtilityTemp.DisposeHotFix(ref dynamicVsDynamicNodePairIndices, dynamicVsDynamicHandle);
            context.DisposeOverlapPairs1 = NativeListUtilityTemp.DisposeHotFix(ref staticVsDynamicNodePairIndices, staticVsDynamicHandle);
            
            return JobHandle.CombineDependencies(dynamicVsDynamicHandle, staticVsDynamicHandle);
        }

        #region Tree

        // A tree of rigid bodies
        public struct Tree : IDisposable, ICloneable
        {
            public NativeArray<Node> Nodes; // The nodes of the bounding volume
            public NativeArray<CollisionFilter> NodeFilters;        // The collision filter for each node (a union of all its children)
            public NativeArray<Builder.Range> Ranges;
            public int BodyCount;
            internal NativeArray<int> m_BranchCount;

            public BoundingVolumeHierarchy BoundingVolumeHierarchy => new BoundingVolumeHierarchy(Nodes, NodeFilters);

            public int NodeCount
            {
                get => Nodes.Length;
                set
                {
                    if (value != Nodes.Length)
                    {
                        Nodes.Dispose();
                        Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(value, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                        NodeFilters.Dispose();
                        NodeFilters = new NativeArray<CollisionFilter>(value, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                    }
                }
            }

            public int BranchCount { get => m_BranchCount[0]; set => m_BranchCount[0] = value; }

            public void Init()
            {
                // Need minimum of 2 empty nodes, to gracefully return from queries on an empty tree
                Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(2, Allocator.Persistent, NativeArrayOptions.ClearMemory)
                {
                    [0] = BoundingVolumeHierarchy.Node.Empty,
                    [1] = BoundingVolumeHierarchy.Node.Empty
                };
                NodeFilters = new NativeArray<CollisionFilter>(2, Allocator.Persistent, NativeArrayOptions.ClearMemory)
                {
                    [0] = CollisionFilter.Default,
                    [1] = CollisionFilter.Default
                };
                Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(
                    BoundingVolumeHierarchy.Constants.MaxNumTreeBranches, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

                m_BranchCount = new NativeArray<int>(1, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                BodyCount = 0;
            }

            public object Clone()
            {
                var clone = new Tree();
                clone.Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(Nodes.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                clone.Nodes.CopyFrom(Nodes);
                clone.NodeFilters = new NativeArray<CollisionFilter>(NodeFilters.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                clone.NodeFilters.CopyFrom(NodeFilters);
                clone.Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(Ranges.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                clone.Ranges.CopyFrom(Ranges);
                clone.m_BranchCount = new NativeArray<int>(1, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                clone.BranchCount = BranchCount;
                clone.BodyCount = BodyCount;
                return clone;
            }

            public void Dispose()
            {
                if (Nodes.IsCreated)
                {
                    Nodes.Dispose();
                }

                if (NodeFilters.IsCreated)
                {
                    NodeFilters.Dispose();
                }

                if (Ranges.IsCreated)
                {
                    Ranges.Dispose();
                }

                if (m_BranchCount.IsCreated)
                {
                    m_BranchCount.Dispose();
                }
            }
        }

        #endregion

        #region Queries

        private struct RigidBodyOverlapsCollector : IOverlapCollector
        {
            public NativeList<int> RigidBodyIndices;
            
            public unsafe void AddRigidBodyIndices(int* indices, int count)
            {
                RigidBodyIndices.AddRange(indices, count);
            }

            public unsafe void AddColliderKeys(ColliderKey* keys, int count)
            {
                throw new NotSupportedException();
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey)
            {
                throw new NotSupportedException();
            }

            public void PopCompositeCollider(uint numCompositeKeyBits)
            {
                throw new NotSupportedException();
            }
        }

        // Test broadphase nodes against the aabb in input. For any overlapping
        // tree leaves, put the body indices into the output leafIndices.
        public void OverlapAabb(OverlapAabbInput input, NativeSlice<RigidBody> rigidBodies, ref NativeList<int> rigidBodyIndices)
        {
            Assert.IsTrue(input.Filter.IsValid);
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            var leafCollector = new RigidBodyOverlapsCollector { RigidBodyIndices = rigidBodyIndices };
            m_StaticTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);
            m_DynamicTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);
        }

        public bool CastRay<T>(RaycastInput input, NativeSlice<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<RaycastHit>
        {
            Assert.IsTrue(input.Filter.IsValid);
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public unsafe bool CastCollider<T>(ColliderCastInput input, NativeSlice<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            Assert.IsTrue(input.Collider != null && input.Collider->Filter.IsValid);
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public bool CalculateDistance<T>(PointDistanceInput input, NativeSlice<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            Assert.IsTrue(input.Filter.IsValid);
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, NativeSlice<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            Assert.IsTrue(input.Collider != null && input.Collider->Filter.IsValid);
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        private struct BvhLeafProcessor :
            BoundingVolumeHierarchy.IRaycastLeafProcessor,
            BoundingVolumeHierarchy.IColliderCastLeafProcessor,
            BoundingVolumeHierarchy.IPointDistanceLeafProcessor,
            BoundingVolumeHierarchy.IColliderDistanceLeafProcessor,
            BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
        {
            [Obsolete("Do not call this method. It is only included to hint AOT compilation", true)]
            static void AOTHint()
            {
                var p = new BvhLeafProcessor();
                var collector = new ClosestHitCollector<RaycastHit>();
                p.RayLeaf(default(RaycastInput), 0, ref collector);
            }

            private readonly NativeSlice<RigidBody> m_Bodies;

            public BvhLeafProcessor(NativeSlice<RigidBody> bodies)
            {
                m_Bodies = bodies;
            }

            public bool AabbOverlap(int rigidBodyIndex, ref NativeList<int> allHits)
            {
                allHits.Add(rigidBodyIndex);
                return true;
            }

            public bool RayLeaf<T>(RaycastInput input, int rigidBodyIndex, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                RigidBody body = m_Bodies[rigidBodyIndex];

                var worldFromBody = new MTransform(body.WorldFromBody);

                // Transform the ray into body space
                RaycastInput inputLs = input;
                {
                    MTransform bodyFromWorld = Inverse(worldFromBody);
                    inputLs.Ray.Origin = Mul(bodyFromWorld, input.Ray.Origin);
                    inputLs.Ray.Displacement = math.mul(bodyFromWorld.Rotation, input.Ray.Displacement);
                }

                float fraction = collector.MaxFraction;
                int numHits = collector.NumHits;

                if (body.CastRay(inputLs, ref collector))
                {
                    // Transform results back into world space
                    collector.TransformNewHits(numHits, fraction, worldFromBody, rigidBodyIndex);
                    return true;
                }
                return false;
            }

            public unsafe bool ColliderCastLeaf<T>(ColliderCastInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                RigidBody body = m_Bodies[rigidBodyIndex];

                // Transform the input into body space
                MTransform worldFromBody = new MTransform(body.WorldFromBody);
                MTransform bodyFromWorld = Inverse(worldFromBody);
                ColliderCastInput inputLs = new ColliderCastInput
                {
                    Collider = input.Collider,
                    Orientation = math.mul(math.inverse(body.WorldFromBody.rot), input.Orientation),
                };
                inputLs.Ray.Origin = Mul(bodyFromWorld, input.Ray.Origin);
                inputLs.Ray.Displacement = math.mul(bodyFromWorld.Rotation, input.Ray.Displacement);


                float fraction = collector.MaxFraction;
                int numHits = collector.NumHits;

                if (body.CastCollider(inputLs, ref collector))
                {
                    // Transform results back into world space
                    collector.TransformNewHits(numHits, fraction, worldFromBody, rigidBodyIndex);
                    return true;
                }
                return false;
            }

            public bool DistanceLeaf<T>(PointDistanceInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                RigidBody body = m_Bodies[rigidBodyIndex];

                // Transform the input into body space
                MTransform worldFromBody = new MTransform(body.WorldFromBody);
                MTransform bodyFromWorld = Inverse(worldFromBody);
                PointDistanceInput inputLs = new PointDistanceInput
                {
                    Position = Mul(bodyFromWorld, input.Position),
                    MaxDistance = input.MaxDistance,
                    Filter = input.Filter
                };

                float fraction = collector.MaxFraction;
                int numHits = collector.NumHits;

                if (body.CalculateDistance(inputLs, ref collector))
                {
                    // Transform results back into world space
                    collector.TransformNewHits(numHits, fraction, worldFromBody, rigidBodyIndex);
                    return true;
                }
                return false;
            }

            public unsafe bool DistanceLeaf<T>(ColliderDistanceInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                RigidBody body = m_Bodies[rigidBodyIndex];

                // Transform the input into body space
                MTransform worldFromBody = new MTransform(body.WorldFromBody);
                MTransform bodyFromWorld = Inverse(worldFromBody);
                ColliderDistanceInput inputLs = new ColliderDistanceInput
                {
                    Collider = input.Collider,
                    Transform = new RigidTransform(
                        math.mul(math.inverse(body.WorldFromBody.rot), input.Transform.rot),
                        Mul(bodyFromWorld, input.Transform.pos)),
                    MaxDistance = input.MaxDistance
                };

                float fraction = collector.MaxFraction;
                int numHits = collector.NumHits;

                if (body.CalculateDistance(inputLs, ref collector))
                {
                    // Transform results back into world space
                    collector.TransformNewHits(numHits, fraction, worldFromBody, rigidBodyIndex);
                    return true;
                }
                return false;
            }

            public unsafe void AabbLeaf<T>(OverlapAabbInput input, int rigidBodyIndex, ref T collector)
                where T : struct, IOverlapCollector
            {
                RigidBody body = m_Bodies[rigidBodyIndex];
                if (body.Collider != null && CollisionFilter.IsCollisionEnabled(input.Filter, body.Collider->Filter))
                {
                    collector.AddRigidBodyIndices(&rigidBodyIndex, 1);
                }
            }
        }

        #endregion

        #region Jobs

        // Allocate memory for pair indices
        [BurstCompile]
        struct AllocateDynamicVsStaticNodePairs : IJob
        {
            public NativeList<int2> dynamicVsDynamicNodePairIndices;
            public NativeList<int2> staticVsDynamicNodePairIndices;

            [ReadOnly]
            public NativeArray<int> dynamicBranchCount;
            [ReadOnly]
            public NativeArray<int> staticBranchCount;

            public void Execute()
            {
                int numDynamicVsDynamicBranchOverlapPairs = dynamicBranchCount[0] * (dynamicBranchCount[0] + 1) / 2;
                dynamicVsDynamicNodePairIndices.ResizeUninitialized(numDynamicVsDynamicBranchOverlapPairs);

                int numStaticVsDynamicBranchOverlapPairs = staticBranchCount[0] * dynamicBranchCount[0];
                staticVsDynamicNodePairIndices.ResizeUninitialized(numStaticVsDynamicBranchOverlapPairs);
            }
        }

        
        // Reads broadphase data from dynamic rigid bodies
        [BurstCompile]
        internal struct BuildDynamicBodyDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeSlice<RigidBody> RigidBodies;
            [ReadOnly] public NativeSlice<MotionVelocity> MotionVelocities;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public float AabbMargin;

            public NativeArray<PointAndIndex> Lookup;
            public NativeArray<Aabb> Aabbs;
            [NativeDisableContainerSafetyRestriction]
            public NativeSlice<CollisionFilter> FiltersOut;

            public unsafe void Execute(int index)
            {
                RigidBody body = RigidBodies[index];

                Aabb aabb;
                if (body.Collider != null)
                {
                    MotionExpansion expansion = MotionVelocities[index].CalculateExpansion(TimeStep);
                    aabb = expansion.ExpandAabb(body.Collider->CalculateAabb(body.WorldFromBody));
                    aabb.Expand(AabbMargin);

                    FiltersOut[index] = body.Collider->Filter;
                }
                else
                {
                    aabb.Min = body.WorldFromBody.pos;
                    aabb.Max = body.WorldFromBody.pos;

                    FiltersOut[index] = CollisionFilter.Zero;
                }

                Aabbs[index] = aabb;
                Lookup[index] = new BoundingVolumeHierarchy.PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }

        // Reads broadphase data from static rigid bodies
        [BurstCompile]
        internal struct BuildStaticBodyDataJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeSlice<RigidBody> RigidBodies;
            [ReadOnly] public int Offset;
            [ReadOnly] public float AabbMargin;

            [NativeDisableParallelForRestriction]
            public NativeArray<Aabb> Aabbs;
            public NativeArray<PointAndIndex> Lookup;
            
            [NativeDisableContainerSafetyRestriction]
            public NativeSlice<CollisionFilter> FiltersOut;

            public unsafe void Execute(int index)
            {
                int staticBodyIndex = index + Offset;
                RigidBody body = RigidBodies[index];

                Aabb aabb;
                if (body.Collider != null)
                {
                    aabb = body.Collider->CalculateAabb(body.WorldFromBody);
                    aabb.Expand(AabbMargin);

                    FiltersOut[index] = RigidBodies[index].Collider->Filter;
                }
                else
                {
                    aabb.Min = body.WorldFromBody.pos;
                    aabb.Max = body.WorldFromBody.pos;

                    FiltersOut[index] = CollisionFilter.Default;
                }

                Aabbs[staticBodyIndex] = aabb;
                Lookup[index] = new BoundingVolumeHierarchy.PointAndIndex
                {
                    Position = aabb.Center,
                    Index = staticBodyIndex
                };
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        public struct DynamicVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> NumBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                var numBranches = NumBranches[0];

                int arrayIndex = 0;

                // First add all branch self overlaps.
                // Start with largest branch
                for (int i = 0; i < numBranches; i++)
                {
                    NodePairIndices[arrayIndex++] = new int2(Ranges[i].Root, Ranges[i].Root);
                }

                for (int i = 0; i < numBranches; i++)
                {
                    for (int j = i + 1; j < numBranches; j++)
                    {
                        int2 pair = new int2 { x = Ranges[i].Root, y = Ranges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        public struct StaticVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> StaticRanges;
            [ReadOnly] public NativeArray<Builder.Range> DynamicRanges;
            [ReadOnly] public NativeArray<int> NumStaticBranches;
            [ReadOnly] public NativeArray<int> NumDynamicBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                var numStaticBranches = NumStaticBranches[0];
                var numDynamicBranches = NumDynamicBranches[0];

                int arrayIndex = 0;
                for (int i = 0; i < numStaticBranches; i++)
                {
                    for (int j = 0; j < numDynamicBranches; j++)
                    {
                        int2 pair = new int2 { x = StaticRanges[i].Root, y = DynamicRanges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // An implementation of IOverlapCollector which filters and writes body pairs to a block stream
        public unsafe struct BodyPairWriter : BoundingVolumeHierarchy.ITreeOverlapCollector
        {
            const int k_Capacity = 256;
            const int k_Margin = 64;
            const int k_Threshold = k_Capacity - k_Margin;

            fixed int m_PairsLeft[k_Capacity];
            fixed int m_PairsRight[k_Capacity];

            private BlockStream.Writer* m_CollidingPairs;
            private CollisionFilter* m_BodyFilters;
            private int m_Count;

            public BodyPairWriter(BlockStream.Writer* collidingPairs, CollisionFilter* bodyFilters)
            {
                m_CollidingPairs = collidingPairs;
                m_BodyFilters = bodyFilters;
                m_Count = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int4 pairsLeft, int4 pairsRight, int count)
            {
                fixed (int* l = m_PairsLeft)
                {
                    *((int4*)(l + m_Count)) = pairsLeft;
                }

                fixed (int* r = m_PairsRight)
                {
                    *((int4*)(r + m_Count)) = pairsRight;
                }

                m_Count += count;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int pairLeft, int4 pairsRight, int countR)
            {
                fixed (int* l = m_PairsLeft)
                {
                    *((int4*)(l + m_Count)) = new int4(pairLeft);
                }

                fixed (int* r = m_PairsRight)
                {
                    *((int4*)(r + m_Count)) = pairsRight;
                }

                m_Count += countR;
            }

            public void Close()
            {
                Flush();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void FlushIfNeeded()
            {
                if (m_Count >= k_Threshold)
                {
                    Flush();
                }
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            private void Flush()
            {
                if (m_Count != 0)
                {
                    fixed (int* l = m_PairsLeft)
                    {
                        fixed (int* r = m_PairsRight)
                        {
                            for (int i = 0; i < m_Count; i++)
                            {
                                var bodyIndexPair = new BodyIndexPair { BodyAIndex = l[i], BodyBIndex = r[i] };

                                if (CollisionFilter.IsCollisionEnabled(m_BodyFilters[bodyIndexPair.BodyAIndex], m_BodyFilters[bodyIndexPair.BodyBIndex]))
                                {
                                    m_CollidingPairs->Write(bodyIndexPair);
                                }
                            }
                        }
                    }

                    m_Count = 0;
                }
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        public unsafe struct DynamicVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<Node> DynamicNodes;
            [ReadOnly] public NativeArray<CollisionFilter> DynamicNodeFilters;
            [ReadOnly] public NativeArray<CollisionFilter> BodyFilters;
            [ReadOnly] public NativeArray<int2> NodePairIndices;

            public BlockStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                int2 pair = NodePairIndices[index];

                CollisionFilter* bodyFiltersPtr = (CollisionFilter*)BodyFilters.GetUnsafeReadOnlyPtr();
                var bufferedPairs = new BodyPairWriter((BlockStream.Writer*)UnsafeUtility.AddressOf(ref PairWriter), bodyFiltersPtr);

                new BoundingVolumeHierarchy(DynamicNodes, DynamicNodeFilters).SelfBvhOverlap(ref bufferedPairs, pair.x, pair.y);

                bufferedPairs.Close();
                PairWriter.EndForEachIndex();
            }
        }

        [BurstCompile]
        public unsafe struct StaticVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<BoundingVolumeHierarchy.Node> StaticNodes;
            [ReadOnly] public NativeArray<BoundingVolumeHierarchy.Node> DynamicNodes;

            [ReadOnly] public NativeArray<CollisionFilter> StaticNodeFilters;
            [ReadOnly] public NativeArray<CollisionFilter> DynamicNodeFilters;
            [ReadOnly] public NativeArray<CollisionFilter> BodyFilters;

            [ReadOnly] public NativeArray<int2> NodePairIndices;

            public BlockStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                int2 pair = NodePairIndices[index];

                CollisionFilter* bodyFiltersPtr = (CollisionFilter*)BodyFilters.GetUnsafeReadOnlyPtr();
                var bufferedPairs = new BodyPairWriter((BlockStream.Writer*)UnsafeUtility.AddressOf(ref PairWriter), bodyFiltersPtr);

                var staticBvh = new BoundingVolumeHierarchy(StaticNodes, StaticNodeFilters);
                var dynamicBvh = new BoundingVolumeHierarchy(DynamicNodes, DynamicNodeFilters);

                staticBvh.BvhOverlap(ref bufferedPairs, dynamicBvh, pair.x, pair.y);

                bufferedPairs.Close();
                PairWriter.EndForEachIndex();
            }

        }

        // In case static bodies haven't changed, but dynamic bodies did
        // we need to move body filter info by an offset defined by 
        // number of dynamic bodies count diff.
        [BurstCompile]
        public struct AdjustStaticBodyFilters : IJob
        {
            [ReadOnly] public int NumDynamicBodiesDiff;
            [ReadOnly] public int NumStaticBodies;
            [ReadOnly] public int NumDynamicBodies;
            [ReadOnly] public NativeArray<int> HaveStaticBodiesChanged;
            [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<CollisionFilter> PreviousFrameBodyFilters;

            // Static & dynamic BodyFilters can be written to in parallel
            [NativeDisableContainerSafetyRestriction]  public NativeArray<CollisionFilter> BodyFilters;

            public void Execute()
            {
                if (HaveStaticBodiesChanged[0] == 1 || NumDynamicBodiesDiff == 0)
                {
                    return;
                }

                int previousFrameNumDynamicBodies = NumDynamicBodies - NumDynamicBodiesDiff;

                for (int i = 0; i < NumStaticBodies; i++)
                {
                    BodyFilters[NumDynamicBodies + i] = PreviousFrameBodyFilters[previousFrameNumDynamicBodies + i];
                }
            }
        }

        // In case static bodies haven't changed, but dynamic bodies did
        // we can keep the static tree as it is, we just need to update
        // indices of static rigid bodies by fixed offset in all leaf nodes.
        [BurstCompile]
        public unsafe struct AdjustBodyIndicesJob : IJob
        {
            [ReadOnly] public int NumDynamicBodiesDiff;
            [ReadOnly] public int NumStaticNodes;
            [ReadOnly] public NativeArray<int> HaveStaticBodiesChanged;

            [NativeDisableContainerSafetyRestriction]
            public NativeArray<Node> StaticNodes;

            public void Execute()
            {
                if (HaveStaticBodiesChanged[0] == 1 || NumDynamicBodiesDiff == 0)
                {
                    return;
                }

                Node* staticNodesPtr = (Node*)StaticNodes.GetUnsafePtr();

                for (int nodeIndex = 0; nodeIndex < NumStaticNodes; nodeIndex++)
                {
                    if (staticNodesPtr[nodeIndex].IsLeaf)
                    {
                        staticNodesPtr[nodeIndex].Data[0] += NumDynamicBodiesDiff;

                        for (int i = 1; i < 4; i++)
                        {
                            if (staticNodesPtr[nodeIndex].IsLeafValid(i))
                            {
                                staticNodesPtr[nodeIndex].Data[i] += NumDynamicBodiesDiff;
                            }
                        }
                    }
                }
            }
        }

        #endregion
    }
}
