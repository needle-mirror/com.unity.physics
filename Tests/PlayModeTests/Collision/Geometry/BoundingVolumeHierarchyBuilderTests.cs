using NUnit.Framework;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.PerformanceTesting;
using UnityEngine;
using static Unity.Physics.BoundingVolumeHierarchy;
using static Unity.Physics.BoundingVolumeHierarchy.Builder;
using Assert = UnityEngine.Assertions.Assert;
using Random = UnityEngine.Random;

namespace Unity.Physics.Tests.Collision.Geometry
{
    public class BoundingVolumeHierarchyBuilderTests
    {
        public void InitInputArrays(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> filters)
        {
            Random.InitState(1234);

            const int posRange = 1000;
            const int radiusRangeMin = 1;
            const int radiusRangeMax = 10;

            for (int i = 0; i < points.Length; i++)
            {
                float3 pos;
                pos.x = Random.Range(-posRange, posRange);
                pos.y = Random.Range(-posRange, posRange);
                pos.z = Random.Range(-posRange, posRange);
                points[i] = new PointAndIndex { Position = pos, Index = i };

                float3 radius = new float3(Random.Range(radiusRangeMin, radiusRangeMax));
                aabbs[i] = new Aabb { Min = pos - radius, Max = pos + radius };

                filters[i] = CollisionFilter.Default;
            }
        }

        public void InitInputWithCopyArrays(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> filters)
        {
            Random.InitState(1234);

            const int posRange = 1000;
            const int radiusRangeMin = 1;
            const int radiusRangeMax = 10;

            for (int i = 0; i < points.Length; i++)
            {
                float3 pos;
                pos.x = Random.Range(-posRange, posRange);
                pos.y = Random.Range(-posRange, posRange);
                pos.z = Random.Range(-posRange, posRange);
                points[i] = new PointAndIndex { Position = pos, Index = i };

                float3 radius = new float3(Random.Range(radiusRangeMin, radiusRangeMax));
                aabbs[i] = new Aabb { Min = pos - radius, Max = pos + radius };

                points[i + 1] = new PointAndIndex { Position = pos, Index = i + 1 };

                aabbs[i + 1] = new Aabb { Min = pos - radius, Max = pos + radius };

                filters[i] = new CollisionFilter
                {
                    GroupIndex = 0,
                    CategoryBits = (uint)Random.Range(0, 16),
                    MaskBits = (uint)Random.Range(0, 16)
                };

                filters[i + 1] = new Physics.CollisionFilter
                {
                    GroupIndex = 0,
                    CategoryBits = (uint)Random.Range(0, 16),
                    MaskBits = (uint)Random.Range(0, 16)
                };

                i++;
            }
        }

        [Test]
        public void BuildTree([Values(2, 10, 100, 1000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitInputArrays(points, aabbs, filters);

            var nodes = new NativeArray<Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodesOut);
            bvh.CheckIntegrity();

            points.Dispose();
            filters.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
        }

        [Test]
        public void BuildTreeByBranches([Values(2, 10, 33, 100, 1000)] int elementCount)
        {
            const int threadCount = 8;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            InitInputArrays(points, aabbs, filters);

            var nodes = new NativeArray<Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var ranges = new NativeArray<Range>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var branchNodeOffsets = new NativeArray<int>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);

            bvh.BuildFirstNLevels(points, ranges, branchNodeOffsets, threadCount, out int branchCount);

            int minBranchNodeIndex = branchNodeOffsets[0];
            for (int i = 0; i < branchCount; i++)
            {
                bvh.BuildBranch(points, aabbs, ranges[i], branchNodeOffsets[i]);
                minBranchNodeIndex = math.min(branchNodeOffsets[i], minBranchNodeIndex);
            }

            bvh.Refit(aabbs, 1, minBranchNodeIndex);

            bvh.CheckIntegrity();

            points.Dispose();
            filters.Dispose();
            aabbs.Dispose();
            nodes.Dispose();

            ranges.Dispose();
            branchNodeOffsets.Dispose();
        }

        [Test]
        public unsafe void BuildTreeTasks([Values(2, 10, 33, 100, 1000)] int elementCount)
        {
            const int threadCount = 8;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            InitInputArrays(points, aabbs, filters);

            var nodes = new NativeArray<Node>(numNodes, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var ranges = new NativeArray<Range>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchNodeOffset = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount,
                ThreadCount = threadCount
            }.Schedule();

            var task2 = new BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = filters,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                NodeFilters = null,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount
            };

            var task3 = new FinalizeTreeJob
            {
                Aabbs = aabbs,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                BranchNodeOffsets = branchNodeOffset,
                NumNodes = nodes.Length,
                LeafFilters = filters,
                BranchCount = branchCount
            };

            handle = task2.Schedule(Constants.MaxNumTreeBranches, 1, handle);
            handle = task3.Schedule(handle);
            handle.Complete();

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.CheckIntegrity();

            filters.Dispose();
            nodes.Dispose();
            ranges.Dispose();
            branchCount.Dispose();
        }

        [Test]
        public unsafe void BuildTreeAndOverlapTasks([Values(2, 10, 33, 100)] int elementCount)
        {
            const int threadCount = 8;
            elementCount *= 2;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var nodefilters = new NativeArray<CollisionFilter>(numNodes, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitInputWithCopyArrays(points, aabbs, filters);

            // Override filter data with default filters.
            for (int i = 0; i < filters.Length; i++)
            {
                filters[i] = CollisionFilter.Default;
            }

            for (int i = 0; i < nodefilters.Length; i++)
            {
                nodefilters[i] = CollisionFilter.Default;
            }

            var nodes = new NativeArray<Node>(numNodes, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var ranges = new NativeArray<Range>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchNodeOffset = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            JobHandle handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount,
                ThreadCount = threadCount,
            }.Schedule();

            handle = new BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = filters,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount
            }.Schedule(Constants.MaxNumTreeBranches, 1, handle);

            handle = new FinalizeTreeJob
            {
                Aabbs = aabbs,
                LeafFilters = filters,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                BranchNodeOffsets = branchNodeOffset,
                NumNodes = numNodes,
                BranchCount = branchCount
            }.Schedule(handle);

            handle.Complete();

            int numBranchOverlapPairs = branchCount[0] * (branchCount[0] + 1) / 2;
            var nodePairIndices = new NativeArray<int2>(numBranchOverlapPairs, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var collisionPairs = new BlockStream(numBranchOverlapPairs, 0xb08c3d78);

            handle = new Broadphase.DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = ranges,
                NumBranches = branchCount[0],
                NodePairIndices = nodePairIndices
            }.Schedule();

            handle = new Broadphase.DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicNodes = nodes,
                PairWriter = collisionPairs,
                BodyFilters = filters,
                NodePairIndices = nodePairIndices,
                DynamicNodeFilters = nodefilters,
            }.Schedule(numBranchOverlapPairs, numBranchOverlapPairs, handle);

            handle.Complete();

            int numPairs = collisionPairs.ComputeItemCount();

            Debug.Log($"Num colliding pairs: {numPairs}");

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.CheckIntegrity();

            Assert.IsTrue(elementCount / 2 == numPairs);

            filters.Dispose();
            nodefilters.Dispose();
            nodes.Dispose();
            ranges.Dispose();
            collisionPairs.Dispose();
            branchCount.Dispose();
        }

        // Util writer which saves every body pair to an HashSet.
        struct EverythingWriter : BoundingVolumeHierarchy.ITreeOverlapCollector
        {
            public void AddPairs(int l, int4 r, int countR) { AddPairs(new int4(l, l, l, l), r, countR); }
            public void AddPairs(int4 pairLeft, int4 r, int count)
            {
                for (int i = 0; i < count; i++)
                {
                    SeenPairs.Add(new BodyIndexPair { BodyAIndex = pairLeft[0], BodyBIndex = r[0] });
                }
            }
            public void FlushIfNeeded() { }
            public HashSet<BodyIndexPair> SeenPairs;
        }

        [Test]
        public unsafe void OverlapTaskFilteringTest([Values(2, 10, 33, 100)] int elementCount)
        {
            elementCount *= 2;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var bodyFilters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            InitInputWithCopyArrays(points, aabbs, bodyFilters);

            var nodes = new NativeArray<Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            Node* nodesPtr = (Node*)nodes.GetUnsafePtr();

            var seenUnfiltered = new HashSet<BodyIndexPair>();
            {
                var bvhUnfiltered = new BoundingVolumeHierarchy(nodes);
                bvhUnfiltered.Build(points, aabbs, out int numNodesOut);
                bvhUnfiltered.CheckIntegrity();

                EverythingWriter pairWriter = new EverythingWriter { SeenPairs = seenUnfiltered };
                BoundingVolumeHierarchy.TreeOverlap(ref pairWriter, nodesPtr, nodesPtr);
            }

            var nodeFilters = new NativeArray<CollisionFilter>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var bvhFiltered = new BoundingVolumeHierarchy(nodes, nodeFilters);
            int numNodesFilteredTree;
            bvhFiltered.Build(points, aabbs, out numNodesFilteredTree);
            bvhFiltered.BuildCombinedCollisionFilter(bodyFilters, 0, numNodesFilteredTree - 1);

            var filteredCollisionPairs = new BlockStream(1, 0xec87b613);
            BlockStream.Writer filteredPairWriter = filteredCollisionPairs;
            filteredPairWriter.BeginForEachIndex(0);
            CollisionFilter* bodyFiltersPtr = (CollisionFilter*)bodyFilters.GetUnsafePtr();
            var bufferedPairs = new Broadphase.BodyPairWriter(ref filteredPairWriter, bodyFiltersPtr);

            CollisionFilter* nodeFiltersPtr = (CollisionFilter*)nodeFilters.GetUnsafePtr();
            BoundingVolumeHierarchy.TreeOverlap(ref bufferedPairs, nodesPtr, nodesPtr, nodeFiltersPtr, nodeFiltersPtr);
            bufferedPairs.Close();
            filteredPairWriter.EndForEachIndex();

            BlockStream.Reader filteredPairReader = filteredCollisionPairs;
            filteredPairReader.BeginForEachIndex(0);

            // Check that every pair in our filtered set also appears in the unfiltered set
            while (filteredPairReader.RemainingItemCount > 0)
            {
                var pair = filteredPairReader.Read<BodyIndexPair>();

                Assert.IsTrue(seenUnfiltered.Contains(pair));
                seenUnfiltered.Remove(pair); // Remove the pair
            }

            // Pairs were removed, so the only remaining ones should be filtered
            foreach (BodyIndexPair pair in seenUnfiltered)
            {
                bool shouldCollide = CollisionFilter.IsCollisionEnabled(bodyFilters[pair.BodyAIndex], bodyFilters[pair.BodyBIndex]);
                Assert.IsFalse(shouldCollide);
            }

            nodeFilters.Dispose();
            nodes.Dispose();
            bodyFilters.Dispose();
            aabbs.Dispose();
            points.Dispose();
            filteredCollisionPairs.Dispose();
        }

        struct PairBuffer : BoundingVolumeHierarchy.ITreeOverlapCollector
        {
            public List<BodyIndexPair> Pairs; // TODO: use NativeList?

            public void AddPairs(int l, int4 r, int countR)
            {
                for (int i = 0; i < countR; i++)
                {
                    Pairs.Add(new BodyIndexPair { BodyAIndex = l, BodyBIndex = r[i] });
                }
            }

            public void AddPairs(int4 pairLeft, int4 r, int count)
            {
                for (int i = 0; i < count; i++)
                {
                    Pairs.Add(new BodyIndexPair { BodyAIndex = pairLeft[i], BodyBIndex = r[i] });
                }
            }

            public void FlushIfNeeded()
            {
            }

            public int MaxId;
        }

        [Test]
        public unsafe void BuildTreeAndOverlap([Values(2, 10, 33, 100)] int elementCount)
        {
            elementCount *= 2;
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            InitInputWithCopyArrays(points, aabbs, filters);

            var nodes = new NativeArray<Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodesOut);
            bvh.CheckIntegrity();

            var buffer = new PairBuffer { Pairs = new List<BodyIndexPair>() };
            buffer.MaxId = elementCount - 1;

            Node* nodesPtr = (Node*)nodes.GetUnsafePtr();
            BoundingVolumeHierarchy.TreeOverlap(ref buffer, nodesPtr, nodesPtr);

            int numCollidingPairs = buffer.Pairs.Count;
            Debug.Log($"Num colliding pairs: {buffer.Pairs.Count}");
            Assert.IsTrue(elementCount / 2 == numCollidingPairs);

            filters.Dispose();
            points.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
        }

        [BurstCompile(CompileSynchronously=true)]
        struct TestTreeOverlapJob : IJob
        {
            public BlockStream.Writer CollisionPairWriter;
            public NativeArray<Node> Nodes;
            public NativeArray<CollisionFilter> Filter;
            public int NumObjects;
            // If true, do no work in Execute() - allows us to get timings for a BurstCompiled
            // run without profiling the overhead of the compiler
            public bool DummyRun;

            public unsafe void Execute()
            {
                if (DummyRun)
                {
                    return;
                }

                CollisionPairWriter.BeginForEachIndex(0);

                CollisionFilter* bodyFilters = (CollisionFilter*)Filter.GetUnsafePtr();
                var pairBuffer = new Broadphase.BodyPairWriter(ref CollisionPairWriter, bodyFilters);

                Node* nodesPtr = (Node*)Nodes.GetUnsafePtr();
                BoundingVolumeHierarchy.TreeOverlap(ref pairBuffer, nodesPtr, nodesPtr);

                pairBuffer.Close();

                CollisionPairWriter.EndForEachIndex();
            }
        }

#if UNITY_2019_2_OR_NEWER
        [Test, Performance]
#else
        [PerformanceTest]
#endif
        [TestCase(100, true, TestName = "TreeOverlapPerfTest 200")]
        [TestCase(1000, true, TestName = "TreeOverlapPerfTest 2000")]
        public void TreeOverlapPerfTest(int elementCount, bool newOverlap)
        {
            // Execute dummy job just to get Burst compilation out of the way.
            {
                var dummyBlockStream = new BlockStream(1, 0, Allocator.TempJob);
                var dummyNodes = new NativeArray<Node>(0, Allocator.TempJob);
                var dummyFilters = new NativeArray<CollisionFilter>(0, Allocator.TempJob);
                new TestTreeOverlapJob
                {
                    CollisionPairWriter = dummyBlockStream,
                    Nodes = dummyNodes,
                    Filter = dummyFilters,
                    NumObjects = 0,
                    DummyRun = true
                }.Run();
                dummyBlockStream.Dispose();
                dummyNodes.Dispose();
                dummyFilters.Dispose();
            }

            elementCount *= 2;
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            // Override filter data with default filters.
            for (int i = 0; i < filters.Length; i++)
            {
                filters[i] = CollisionFilter.Default;
            }

            InitInputWithCopyArrays(points, aabbs, filters);

            var nodes = new NativeArray<Node>(numNodes, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodesOut);
            bvh.CheckIntegrity();

            var collisionPairs = new BlockStream(1, 0xd586fc6e);

            var job = new TestTreeOverlapJob
            {
                Nodes = nodes,
                Filter = filters,
                NumObjects = elementCount,
                CollisionPairWriter = collisionPairs,
                DummyRun = false
            };

            Measure.Method(() =>
            {
                job.Run();
            }).Definition(sampleUnit: SampleUnit.Millisecond)
              .MeasurementCount(1)
              .Run();

            points.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
            collisionPairs.Dispose();
            filters.Dispose();
        }
    }
}
