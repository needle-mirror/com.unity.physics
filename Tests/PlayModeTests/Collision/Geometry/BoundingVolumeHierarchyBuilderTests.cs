using System.Collections.Generic;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
#if PHYSICS_ENABLE_PERF_TESTS
using Unity.PerformanceTesting;
#endif
using static Unity.Physics.BoundingVolumeHierarchy;
using static Unity.Physics.BoundingVolumeHierarchy.Builder;
using Assert = UnityEngine.Assertions.Assert;
using Random = UnityEngine.Random;

namespace Unity.Physics.Tests.Collision.Geometry
{
    [BurstCompile]
    class BoundingVolumeHierarchyBuilderTests
    {
        void InitInputArrays(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> filters, int worldSize, DimensionType dim)
        {
            var random = new Mathematics.Random(1234);

            int posRange = worldSize / 2;
            const int dimensionRangeMin = 1;
            const int dimensionRangeMax = 10;

            for (int i = 0; i < points.Length; i++)
            {
                float3 pos;
                pos.x = random.NextInt(-posRange, posRange);
                pos.y = random.NextInt(-posRange, posRange);
                pos.z = random.NextInt(-posRange, posRange);
                points[i] = new PointAndIndex { Position = pos, Index = i };

                float3 dimensions;
                switch (dim)
                {
                    case DimensionType.XZEqual:
                        var xz = random.NextInt(dimensionRangeMin, dimensionRangeMax);
                        dimensions = new float3(
                            xz,
                            random.NextInt(dimensionRangeMin, dimensionRangeMax),
                            xz);
                        break;
                    case DimensionType.NoneEqual:
                        dimensions = new float3(
                            random.NextInt(dimensionRangeMin, dimensionRangeMax),
                            random.NextInt(dimensionRangeMin, dimensionRangeMax),
                            random.NextInt(dimensionRangeMin, dimensionRangeMax));
                        break;
                    case DimensionType.XYZEqual:
                    default:
                        dimensions = new float3(random.NextInt(dimensionRangeMin, dimensionRangeMax));
                        break;
                }

                aabbs[i] = new Aabb { Min = pos - dimensions, Max = pos + dimensions };

                filters[i] = CollisionFilter.Default;
            }
        }

        private enum DimensionType
        {
            XYZEqual, // for spherical/cube-like objects
            XZEqual,  // for cylindrical-like objects
            NoneEqual // for objects with random dimensions
        }

        // All of the output NativeArray have structures such that data within index i and i+1 are equal. The length of
        // these NativeArray are twice the number of elements. Note that for the filters NativeArray, that Random is
        // called separately for each index.
        void InitInputWithCopyArrays(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs,
            NativeArray<CollisionFilter> filters, NativeArray<bool> respondsToCollision)
        {
            var random = new Mathematics.Random(1234);

            const int posRange = 1000;
            const int radiusRangeMin = 1;
            const int radiusRangeMax = 10;

            for (int i = 0; i < points.Length; i++)
            {
                float3 pos;
                pos.x = random.NextInt(-posRange, posRange);
                pos.y = random.NextInt(-posRange, posRange);
                pos.z = random.NextInt(-posRange, posRange);
                points[i] = new PointAndIndex { Position = pos, Index = i };

                float3 radius = new float3(Random.Range(radiusRangeMin, radiusRangeMax));
                aabbs[i] = new Aabb { Min = pos - radius, Max = pos + radius };

                points[i + 1] = new PointAndIndex { Position = pos, Index = i + 1 };

                aabbs[i + 1] = new Aabb { Min = pos - radius, Max = pos + radius };

                filters[i] = new CollisionFilter
                {
                    GroupIndex = 0,
                    BelongsTo = (uint)random.NextInt(0, 16),
                    CollidesWith = (uint)random.NextInt(0, 16)
                };

                filters[i + 1] = new CollisionFilter
                {
                    GroupIndex = 0,
                    BelongsTo = (uint)random.NextInt(0, 16),
                    CollidesWith = (uint)random.NextInt(0, 16)
                };

                respondsToCollision[i] = true;
                respondsToCollision[i + 1] = true;

                i++;
            }
        }

        [Test]
        public unsafe void BuildTree([Values(2, 10, 100, 1000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

            var nodes = new NativeArray<Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodesOut);

            bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());

            points.Dispose();
            filters.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
        }

        [Test]
        public unsafe void BuildTreeByBranches([Values(2, 10, 33, 100, 1000)] int elementCount)
        {
            const int threadCount = 8;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

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

            bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());

            points.Dispose();
            filters.Dispose();
            aabbs.Dispose();
            nodes.Dispose();

            ranges.Dispose();
            branchNodeOffsets.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        // Note: we need to use ref arguments here since otherwise this function can not be burst-compiled.
        // Burst does not support passing native containers by value.
        static unsafe void DoBuildBvhIncrementally(ref BoundingVolumeHierarchy bvh, ref NativeArray<Aabb> aabbs,
            ref NativeArray<PointAndIndex> points, ref NativeArray<CollisionFilter> filters, IncrementalInsertionContext* insertionContext = null)
        {
            for (int i = 0; i < aabbs.Length; ++i)
            {
                var pointAndIndex = points[i];
                var result = bvh.Insert(aabbs[pointAndIndex.Index], points[i], filters[pointAndIndex.Index]);
                if (insertionContext != null)
                {
                    insertionContext->Insert(ref bvh, result);
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        static void RemoveElementsFromBvh(ref BoundingVolumeHierarchy bvh, ref NativeArray<CollisionFilter> bodyFilters,
            ref IncrementalInsertionContext insertionContext, int removeCount)
        {
            var random = new Mathematics.Random(42);
            var removals = new NativeArray<RemovalData>(removeCount, Allocator.Temp);
            using var insertionData = new NativeList<ElementLocationData>(bodyFilters.Length, Allocator.Temp);
            using var indexSet = new NativeHashSet<int>(removeCount, Allocator.Temp);
            insertionContext.CopyTo(insertionData);
            for (int i = 0; i < removeCount; ++i)
            {
                int index = -1;
                bool indexAlreadyPicked = false;
                do
                {
                    // rejection sampling to pick an index that hasn't been picked yet
                    index = random.NextInt(0, insertionData.Length - 1);
                    indexAlreadyPicked = !indexSet.Add(index);
                }
                while (indexAlreadyPicked);

                var data = insertionData[index];
                removals[i] = new RemovalData
                {
                    NodeIndex = data.NodeIndex,
                    LeafSlotIndex = data.LeafSlotIndex
                };
            }
            bvh.Remove(removals, bodyFilters);

            removals.Dispose();
        }

        [Test]
        public unsafe void BuildTreeIncrementally([Values(2, 10, 1000, 100000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

            using var nodes = new NativeList<Node>(numNodes, Allocator.Temp);
            using var nodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator.Temp);
            var bvh = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodeFilters.GetUnsafeList());
            Assert.IsTrue(bvh.IsIncremental);

            DoBuildBvhIncrementally(ref bvh, ref aabbs, ref points, ref filters);

            bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());

            aabbs.Dispose();
            points.Dispose();
            filters.Dispose();
        }

        [Test]
        public unsafe void BuildTreeIncrementallyAndRemove([Values(2, 10, 1000, 100000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

            using var nodes = new NativeList<Node>(numNodes, Allocator.Temp);
            using var nodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator.Temp);
            var bvh = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodeFilters.GetUnsafeList());
            Assert.IsTrue(bvh.IsIncremental);

            var insertionContext = new IncrementalInsertionContext(elementCount, Allocator.Temp);
            DoBuildBvhIncrementally(ref bvh, ref aabbs, ref points, ref filters, &insertionContext);

            bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());

            // remove a subset of randomly selected elements:

            // remove a quarter of elements
            var removeCount = math.max(1, elementCount / 4);
            RemoveElementsFromBvh(ref bvh, ref filters, ref insertionContext, removeCount);

            bvh.CheckIntegrity(-1, (CollisionFilter*)filters.GetUnsafePtr());

            insertionContext.Dispose();
            aabbs.Dispose();
            points.Dispose();
            filters.Dispose();
        }

        [Test]
        public unsafe void BuildTreeTasks([Values(2, 10, 33, 100, 1000)] int elementCount)
        {
            const int threadCount = 8;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

            var nodes = new NativeArray<Node>(numNodes, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var ranges = new NativeArray<Range>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchNodeOffset = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchNodeCounts = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var shouldDoWork = new NativeReference<int>(1, Allocator.Persistent);
            NativeArray<int> oldBranchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                MaxNodeCount = nodes.Length,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount,
                OldBranchCount = oldBranchCount,
                ThreadCount = threadCount,
                ShouldDoWork = shouldDoWork
            }.Schedule();

            handle = new BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = filters,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                MaxNodeCount = nodes.Length,
                NodeFilters = null,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                NodeCounts = branchNodeCounts
            }.ScheduleUnsafeIndex0(branchCount, 1, handle);

            new FinalizeTreeJob
            {
                Aabbs = aabbs,
                Nodes = (Node*)nodes.GetUnsafePtr(),
                BranchNodeOffsets = branchNodeOffset,
                NodeCounts = branchNodeCounts,
                LeafFilters = filters,
                BranchCount = branchCount,
                OldBranchCount = oldBranchCount,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle).Complete();

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());

            filters.Dispose();
            nodes.Dispose();
            ranges.Dispose();
            branchCount.Dispose();
            shouldDoWork.Dispose();
        }

        [Test]
        public unsafe void BuildTreeAndOverlapTasks([Values(2, 10, 33, 100)] int elementCount)
        {
            const int threadCount = 8;
            elementCount *= 2;

            var tree = new Broadphase.Tree(elementCount);

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            // Fill aabbs and points array with random pairs of identical elements.
            // With this data set, we expect to get elementCount / 2 overlaps in a broad phase tree overlap.
            InitInputWithCopyArrays(points, aabbs, tree.BodyFilters.AsArray(), tree.RespondsToCollision.AsArray());

            // Override filter data with default filters.
            for (int i = 0; i < tree.BodyFilters.Length; i++)
            {
                tree.BodyFilters[i] = CollisionFilter.Default;
            }

            // for (int i = 0; i < tree.NodeFilters.Length; i++)
            // {
            //     tree.NodeFilters[i] = CollisionFilter.Default;
            // }

            var branchNodeOffset = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchNodeCounts = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var shouldDoWork = new NativeReference<int>(1, Allocator.Persistent);
            NativeArray<int> oldBranchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            JobHandle handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = tree.Nodes.GetUnsafePtr(),
                MaxNodeCount = tree.Nodes.Capacity,
                NodeFilters = tree.NodeFilters.GetUnsafePtr(),
                Ranges = tree.Ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount,
                OldBranchCount = oldBranchCount,
                ThreadCount = threadCount,
                ShouldDoWork = shouldDoWork
            }.Schedule();

            handle = new BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = tree.BodyFilters.AsArray(),
                Nodes = tree.Nodes.GetUnsafePtr(),
                MaxNodeCount = tree.Nodes.Capacity,
                NodeFilters = tree.NodeFilters.GetUnsafePtr(),
                Ranges = tree.Ranges,
                BranchNodeOffsets = branchNodeOffset,
                NodeCounts = branchNodeCounts
            }.ScheduleUnsafeIndex0(branchCount, 1, handle);

            new FinalizeTreeJob
            {
                Aabbs = aabbs,
                LeafFilters = tree.BodyFilters.AsArray(),
                Nodes = tree.Nodes.GetUnsafePtr(),
                NodeFilters = tree.NodeFilters.GetUnsafePtr(),
                BranchNodeOffsets = branchNodeOffset,
                NodeCounts = branchNodeCounts,
                BranchCount = branchCount,
                ShouldDoWork = shouldDoWork,
                OldBranchCount = oldBranchCount
            }.Schedule(handle).Complete();

            int numBranchOverlapPairs = branchCount[0] * (branchCount[0] + 1) / 2;
            var nodePairIndices = new NativeList<int2>(Allocator.TempJob);
            nodePairIndices.ResizeUninitialized(numBranchOverlapPairs);
            var collisionPairs = new NativeStream(numBranchOverlapPairs, Allocator.TempJob);

            handle = new Broadphase.DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = tree.Ranges,
                NumBranches = branchCount,
                NodePairIndices = nodePairIndices.AsArray()
            }.Schedule();

            handle = new Broadphase.DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicTree = tree,
                NodePairIndices = nodePairIndices.AsArray(),
                PairWriter = collisionPairs.AsWriter()
            }.Schedule(nodePairIndices, numBranchOverlapPairs, handle);

            handle.Complete();

            int numPairs = collisionPairs.Count();

            Assert.AreEqual(elementCount / 2, numPairs);
            //Debug.Log($"Num colliding pairs: {numPairs}");

            tree.BoundingVolumeHierarchy.CheckIntegrity(elementCount, tree.BodyFilters.GetUnsafePtr());

            nodePairIndices.Dispose();
            tree.Dispose();
            collisionPairs.Dispose();
            branchCount.Dispose();
            shouldDoWork.Dispose();
        }

        [Test]
        public unsafe void BuildIncrementalTreeAndOverlapTasks([Values(2, 10, 33, 100)] int elementCount)
        {
            const int threadCount = 8;
            elementCount *= 2;

            var tree = new Broadphase.Tree(elementCount);

            using var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            using var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            // Fill aabbs and points array with random pairs of identical elements.
            // With this data set, we expect to get elementCount / 2 overlaps in a broad phase tree overlap.
            InitInputWithCopyArrays(points, aabbs, tree.BodyFilters.AsArray(), tree.RespondsToCollision.AsArray());

            // Override filter data with default filters to enable all collisions.
            for (int i = 0; i < tree.BodyFilters.Length; i++)
            {
                tree.BodyFilters[i] = CollisionFilter.Default;
            }

            for (int i = 0; i < tree.NodeFilters.Length; i++)
            {
                tree.NodeFilters[i] = CollisionFilter.Default;
            }

            var insertBodyDataStream = new NativeStream(1, Allocator.Temp);
            var updatedElementLocationDataList = new NativeList<ElementLocationData>(elementCount, Allocator.Temp);
            tree.InsertBodyDataStream = insertBodyDataStream;
            tree.UpdatedElementLocationDataList = updatedElementLocationDataList;

            // fill insertion data stream
            var insertionWriter = insertBodyDataStream.AsWriter();
            insertionWriter.BeginForEachIndex(0);
            for (int i = 0; i < aabbs.Length; ++i)
            {
                insertionWriter.Write(new Broadphase.InsertionData
                {
                    Aabb = aabbs[i],
                    PointAndIndex = points[i],
                    Filter = CollisionFilter.Default
                });
            }
            insertionWriter.EndForEachIndex();

            tree.BuildIncremental();
            tree.BoundingVolumeHierarchy.CheckIntegrity(elementCount, tree.BodyFilters.GetUnsafePtr());

            new Broadphase.CalculateTreeSplitDataJob
            {
                Tree = tree,
                ThreadCount = threadCount
            }.Run();

            int numBranchOverlapPairs = tree.BranchCount[0] * (tree.BranchCount[0] + 1) / 2;
            using var nodePairIndices = new NativeList<int2>(Allocator.TempJob);
            nodePairIndices.ResizeUninitialized(numBranchOverlapPairs);
            using var collisionPairs = new NativeStream(numBranchOverlapPairs, Allocator.TempJob);

            new Broadphase.DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = tree.Ranges,
                NumBranches = tree.BranchCount,
                NodePairIndices = nodePairIndices.AsArray()
            }.Run();

            var handle = new Broadphase.DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicTree = tree,
                NodePairIndices = nodePairIndices.AsArray(),
                PairWriter = collisionPairs.AsWriter()
            }.Schedule(nodePairIndices, numBranchOverlapPairs);

            handle.Complete();

            // check if we are getting the expected number of overlaps
            Assert.AreEqual(elementCount / 2, collisionPairs.Count());

            tree.Dispose();
        }

        // Util writer which saves every body pair to an HashSet.
        struct EverythingWriter : BoundingVolumeHierarchy.ITreeOverlapCollector
        {
            public void AddPairs(int l, int4 r, int countR) { AddPairs(new int4(l, l, l, l), r, countR); }
            public void AddPairs(int4 pairLeft, int4 r, int count, bool swapped = false)
            {
                for (int i = 0; i < count; i++)
                {
                    SeenPairs.Add(new BodyIndexPair { BodyIndexA = pairLeft[0], BodyIndexB = r[0] });
                }
            }

            public void FlushIfNeeded() {}
            public HashSet<BodyIndexPair> SeenPairs;
        }

        [Test]
        public unsafe void OverlapTaskFilteringTest([Values(2, 10, 33, 100)] int elementCount, [Values] bool incremental)
        {
            elementCount *= 2;
            int numNodes = elementCount + Constants.MaxNumTreeBranches;

            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var bodyFilters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var bodyRespondsToCollision = new NativeArray<bool>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            InitInputWithCopyArrays(points, aabbs, bodyFilters, bodyRespondsToCollision);

            var nodes = new NativeList<Node>(numNodes, Allocator.Temp);
            var nodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator.Temp);

            var seenUnfiltered = new HashSet<BodyIndexPair>();
            {
                nodes.Resize(numNodes, NativeArrayOptions.UninitializedMemory);
                var bvhUnfiltered = new BoundingVolumeHierarchy(nodes.AsArray());
                bvhUnfiltered.Build(points, aabbs, out int numNodesOut);
                bvhUnfiltered.CheckIntegrity(elementCount, null);

                EverythingWriter pairWriter = new EverythingWriter { SeenPairs = seenUnfiltered };
                BoundingVolumeHierarchy.TreeOverlap(ref pairWriter, nodes.GetUnsafePtr(), nodes.GetUnsafePtr());
            }

            nodes.Clear();

            if (incremental)
            {
                var bvhFiltered = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodeFilters.GetUnsafeList());
                DoBuildBvhIncrementally(ref bvhFiltered, ref aabbs, ref points, ref bodyFilters);
                bvhFiltered.CheckIntegrity(elementCount, (CollisionFilter*)bodyFilters.GetUnsafePtr());
            }
            else
            {
                nodes.Resize(numNodes, NativeArrayOptions.UninitializedMemory);
                nodeFilters.Resize(numNodes, NativeArrayOptions.UninitializedMemory);
                var bvhFiltered = new BoundingVolumeHierarchy(nodes.AsArray(), nodeFilters.AsArray());
                bvhFiltered.Build(points, aabbs, out int numNodesFilteredTree);
                bvhFiltered.BuildCombinedCollisionFilter(bodyFilters, 0, numNodesFilteredTree - 1);
                bvhFiltered.CheckIntegrity(elementCount, (CollisionFilter*)bodyFilters.GetUnsafePtr());
            }

            var filteredCollisionPairs = new NativeStream(1, Allocator.TempJob);
            NativeStream.Writer filteredPairWriter = filteredCollisionPairs.AsWriter();
            filteredPairWriter.BeginForEachIndex(0);
            CollisionFilter* bodyFiltersPtr = (CollisionFilter*)bodyFilters.GetUnsafePtr();
            bool* bodyRespondsToCollisionPtr = (bool*)bodyRespondsToCollision.GetUnsafePtr();
            var bufferedPairs = new Broadphase.BodyPairWriter(&filteredPairWriter, bodyFiltersPtr, bodyFiltersPtr, bodyRespondsToCollisionPtr, bodyRespondsToCollisionPtr, 0, 0);

            CollisionFilter* nodeFiltersPtr = nodeFilters.GetUnsafePtr();
            BoundingVolumeHierarchy.TreeOverlap(ref bufferedPairs, nodes.GetUnsafePtr(), nodes.GetUnsafePtr(), nodeFiltersPtr, nodeFiltersPtr);
            bufferedPairs.Close();
            filteredPairWriter.EndForEachIndex();

            NativeStream.Reader filteredPairReader = filteredCollisionPairs.AsReader();
            filteredPairReader.BeginForEachIndex(0);

            // Check that every pair in our filtered set also appears in the unfiltered set
            while (filteredPairReader.RemainingItemCount > 0)
            {
                var pair = filteredPairReader.Read<BodyIndexPair>();

                bool found = false;
                if (seenUnfiltered.Contains(pair))
                {
                    found = true;
                    seenUnfiltered.Remove(pair); // Remove the pair
                }
                else
                {
                    // Try the reverse
                    var reversePair = new BodyIndexPair { BodyIndexA = pair.BodyIndexB, BodyIndexB = pair.BodyIndexA };
                    if (seenUnfiltered.Contains(reversePair))
                    {
                        found = true;
                        seenUnfiltered.Remove(reversePair); // Remove the pair
                    }
                }
                Assert.IsTrue(found);
            }

            // Pairs were removed, so the only remaining ones should be filtered
            foreach (BodyIndexPair pair in seenUnfiltered)
            {
                bool shouldCollide = CollisionFilter.IsCollisionEnabled(bodyFilters[pair.BodyIndexA], bodyFilters[pair.BodyIndexB]);
                Assert.IsFalse(shouldCollide);
            }

            nodeFilters.Dispose();
            nodes.Dispose();
            bodyFilters.Dispose();
            bodyRespondsToCollision.Dispose();
            aabbs.Dispose();
            points.Dispose();
            filteredCollisionPairs.Dispose();
        }

        struct PairBuffer : BoundingVolumeHierarchy.ITreeOverlapCollector
        {
            public List<BodyIndexPair> Pairs;

            public void AddPairs(int l, int4 r, int countR)
            {
                for (int i = 0; i < countR; i++)
                {
                    Pairs.Add(new BodyIndexPair { BodyIndexA = l, BodyIndexB = r[i] });
                }
            }

            public void AddPairs(int4 pairLeft, int4 r, int count, bool swapped = false)
            {
                for (int i = 0; i < count; i++)
                {
                    Pairs.Add(new BodyIndexPair { BodyIndexA = pairLeft[i], BodyIndexB = r[i] });
                }
            }

            public void FlushIfNeeded()
            {
            }
        }

        [Test]
        public unsafe void BuildTreeAndOverlap([Values] bool incremental, [Values(2, 10, 33, 100)] int elementCount)
        {
            elementCount *= 2;
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            using var respondsToCollision = new NativeArray<bool>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            using var nodes = new NativeList<Node>(numNodes, Allocator.Temp);
            using var nodesFilters = new NativeList<CollisionFilter>(numNodes, Allocator.Temp);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp);

            InitInputWithCopyArrays(points, aabbs, filters, respondsToCollision);

            if (incremental)
            {
                var bvh = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodesFilters.GetUnsafeList());
                Assert.IsTrue(bvh.IsIncremental);
                DoBuildBvhIncrementally(ref bvh, ref aabbs, ref points, ref filters);
                bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());
            }
            else
            {
                nodes.Resize(numNodes, NativeArrayOptions.UninitializedMemory);
                var bvh = new BoundingVolumeHierarchy(nodes.AsArray());
                bvh.Build(points, aabbs, out int numNodesOut);
                bvh.CheckIntegrity(elementCount, null);
            }

            var nodesPtr = nodes.GetUnsafePtr();
            var buffer = new PairBuffer { Pairs = new List<BodyIndexPair>() };

            BoundingVolumeHierarchy.TreeOverlap(ref buffer, nodesPtr, nodesPtr);

            int numCollidingPairs = buffer.Pairs.Count;
            //Debug.Log($"Test {elementCount}: Num colliding pairs: {buffer.Pairs.Count}");
            Assert.AreEqual(elementCount / 2, numCollidingPairs);

            aabbs.Dispose();
            points.Dispose();
            filters.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        struct TestTreeOverlapJob : IJob
        {
            public NativeStream.Writer CollisionPairWriter;
            public NativeArray<Node> Nodes;
            public NativeArray<CollisionFilter> Filter;
            public NativeArray<bool> RespondsToCollision;
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
                bool* bodyRespondsToCollision = (bool*)RespondsToCollision.GetUnsafePtr();
                var pairBuffer = new Broadphase.BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref CollisionPairWriter),
                    bodyFilters, bodyFilters, bodyRespondsToCollision, bodyRespondsToCollision, 0, 0);

                Node* nodesPtr = (Node*)Nodes.GetUnsafePtr();
                BoundingVolumeHierarchy.TreeOverlap(ref pairBuffer, nodesPtr, nodesPtr);

                pairBuffer.Close();

                CollisionPairWriter.EndForEachIndex();
            }
        }

#if PHYSICS_ENABLE_PERF_TESTS
        [Test, Performance]
        public unsafe void BuildTreeIncrementallyPerformance([Values(1000, 10000, 100000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

            using var nodes = new NativeList<Node>(numNodes, Allocator.Temp);
            using var nodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator.Temp);
            var bvh = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodeFilters.GetUnsafeList());
            Assert.IsTrue(bvh.IsIncremental);

            Measure.Method(() =>
            {
                bvh.Clear();
                DoBuildBvhIncrementally(ref bvh, ref aabbs, ref points, ref filters);
            }).MeasurementCount(2).Run();

            aabbs.Dispose();
            points.Dispose();
            filters.Dispose();
        }

        [Test, Performance]
        public unsafe void BuildTreeIncrementallyAndRemovePerformance([Values(1000, 10000, 100000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            InitInputArrays(points, aabbs, filters, 1000, DimensionType.XYZEqual);

            using var nodes = new NativeList<Node>(numNodes, Allocator.Temp);
            using var nodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator.Temp);
            var bvh = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodeFilters.GetUnsafeList());
            Assert.IsTrue(bvh.IsIncremental);

            Measure.Method(() =>
            {
                bvh.Clear();

                var insertionContext = new IncrementalInsertionContext(elementCount, Allocator.Temp);
                DoBuildBvhIncrementally(ref bvh, ref aabbs, ref points, ref filters, &insertionContext);

                // remove a quarter of elements
                var removeCount = math.max(1, elementCount / 4);
                RemoveElementsFromBvh(ref bvh, ref filters, ref insertionContext, removeCount);

                insertionContext.Dispose();
            }).MeasurementCount(2).Run();

            aabbs.Dispose();
            points.Dispose();
            filters.Dispose();
        }

        [Test, Performance]
        [TestCase(100, false, TestName = "TreeOverlapPerfTest 200")]
        [TestCase(1000, false, TestName = "TreeOverlapPerfTest 2000")]
        [TestCase(100, true, TestName = "TreeOverlapPerfTest 200 (incremental)")]
        [TestCase(1000, true, TestName = "TreeOverlapPerfTest 2000 (incremental)")]
        public unsafe void TreeOverlapPerfTest(int elementCount, bool incremental)
        {
            // Execute dummy job just to get Burst compilation out of the way.
            {
                var dummyStream = new NativeStream(1, Allocator.TempJob);
                var dummyNodes = new NativeArray<Node>(0, Allocator.TempJob);
                var dummyFilters = new NativeArray<CollisionFilter>(0, Allocator.TempJob);
                var dummyRespondsToCollision = new NativeArray<bool>(0, Allocator.TempJob);
                new TestTreeOverlapJob
                {
                    CollisionPairWriter = dummyStream.AsWriter(),
                    Nodes = dummyNodes,
                    Filter = dummyFilters,
                    RespondsToCollision = dummyRespondsToCollision,
                    DummyRun = true
                }.Run();
                dummyStream.Dispose();
                dummyNodes.Dispose();
                dummyFilters.Dispose();
                dummyRespondsToCollision.Dispose();
            }

            elementCount *= 2;
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var respondsToCollision = new NativeArray<bool>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitInputWithCopyArrays(points, aabbs, filters, respondsToCollision);

            // Override filter data with default filters.
            for (int i = 0; i < filters.Length; i++)
            {
                filters[i] = CollisionFilter.Default;
            }

            var nodes = new NativeList<Node>(numNodes, Allocator.TempJob);
            var nodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator.TempJob);

            if (incremental)
            {
                var bvh = new BoundingVolumeHierarchy(nodes.GetUnsafeList(), nodeFilters.GetUnsafeList());
                DoBuildBvhIncrementally(ref bvh, ref aabbs, ref points, ref filters);
                bvh.CheckIntegrity(elementCount, (CollisionFilter*)filters.GetUnsafePtr());
            }
            else
            {
                nodes.Resize(numNodes, NativeArrayOptions.UninitializedMemory);
                var bvh = new BoundingVolumeHierarchy(nodes.AsArray());
                bvh.Build(points, aabbs, out int numNodesOut);
                bvh.CheckIntegrity(elementCount, null);
            }

            var collisionPairs = new NativeStream(1, Allocator.TempJob);

            var job = new TestTreeOverlapJob
            {
                Nodes = nodes.AsArray(),
                Filter = filters,
                RespondsToCollision = respondsToCollision,
                CollisionPairWriter = collisionPairs.AsWriter(),
                DummyRun = false
            };

            Measure.Method(() =>
            {
                job.Run();
            }).MeasurementCount(1)
                .Run();

            points.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
            nodeFilters.Dispose();
            collisionPairs.Dispose();
            filters.Dispose();
            respondsToCollision.Dispose();
        }

#endif
    }
}
