using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // Utilities for building bounding volume hierarchies
    internal partial struct BoundingVolumeHierarchy
    {
        public struct Constants
        {
            public const int MaxNumTreeBranches = 64;
            public const int SmallRangeSize = 32;
            public const int UnaryStackSize = 256;
            public const int BinaryStackSize = 512;
        }

        public struct PointAndIndex
        {
            public float3 Position;
            public int Index;
        }

        /// <summary>
        /// Builder for a BVH.
        /// </summary>
        public unsafe struct Builder
        {
            // Resultant bounding volume hierarchy (BVH)
            public BoundingVolumeHierarchy Bvh;
            // Points and indices of the objects hashed in the tree
            public NativeArray<PointAndIndex> Points;
            // Bounds of the objects hashed in the tree
            public NativeArray<Aabb> Aabbs;
            // Index of the next available node
            public int FreeNodeIndex;
            // Indicates whether the tree is build using the surface area heuristic (SAH)
            public bool UseSah;

            // Scratch data arrays when using SAH for BVH building.
            private NativeArray<float> ScratchScores;
            private NativeArray<float4> ScratchPointsX;
            private NativeArray<float4> ScratchPointsY;
            private NativeArray<float4> ScratchPointsZ;

            /// <summary>
            /// Represents a Range of 'Length' many objects that encompasses an AABB domain. The objects included
            /// begin at the 'Start' index of the Builder's 'Points', and will be inserted in the subtree
            /// located at the node with the index 'Root'.
            /// </summary>
            public struct Range
            {
                /// <summary>
                /// Constructs a starting range for the BoundingVolumeHierarchy builder, inserting
                /// objects at the root node (index 1) of the resultant hierarchy.
                /// </summary>
                public Range(int start, int length, Aabb domain)
                {
                    Start = start;
                    Length = length;
                    Root = 1;   // 1 is the index of the root node in a BoundingVolumeHierarchy
                    Parent = 0; // default parent index 0, indicating an invalid node in the BoundingVolumeHierarchy
                    Domain = domain;
                }

                /// <summary> The start index in the points array of the sequence of points in this range </summary>
                public int Start;

                /// <summary>
                /// The length of the sequence of points in this Range. This is the number of elements that will
                /// be inserted in the subtree located at the provided Root node.
                /// </summary>
                public int Length;

                /// <summary> The index of the node where the range will be inserted. </summary>
                public int Root;

                /// <summary> Index of the parent node of `Root`, the node at which this Range will be inserted. </summary>
                public int Parent;

                /// <summary> The unioned AABB encompassing all objects in the Range </summary>
                public Aabb Domain;
            }

            /// <summary>
            /// Sorts the objects in the provided range along the axis with the specified index using the points of the
            /// objects in the range.
            /// </summary>
            void SortRange(int axis, ref Range range)
            {
                for (int i = range.Start; i < range.Start + range.Length; ++i)
                {
                    PointAndIndex value = Points[i];
                    float key = value.Position[axis];
                    int j = i;
                    while (j > range.Start && key < Points[j - 1].Position[axis])
                    {
                        Points[j] = Points[j - 1];
                        j--;
                    }
                    Points[j] = value;
                }
            }

            /// <summary>
            /// Compute axis and pivot of a given range.
            /// </summary>
            /// <param name="range"></param>
            /// <param name="axis"></param>
            /// <param name="pivot"></param>
            static void ComputeAxisAndPivot(ref Range range, out int axis, out float pivot)
            {
                // Compute axis and pivot.
                axis = IndexOfMaxComponent(range.Domain.Extents);
                pivot = ((range.Domain.Min + range.Domain.Max) / 2)[axis];
            }

            static void SplitRange(ref Range range, int size, ref Range lRange, ref Range rRange)
            {
                lRange.Start = range.Start;
                lRange.Length = size;
                rRange.Start = lRange.Start + lRange.Length;
                rRange.Length = range.Length - lRange.Length;
            }

            struct CompareVertices : IComparer<float4>
            {
                public int Compare(float4 x, float4 y)
                {
                    return x[SortAxis].CompareTo(y[SortAxis]);
                }

                public int SortAxis;
            }

            void ProcessAxis(int rangeLength, int axis, NativeArray<float> scores, NativeArray<float4> points, ref int bestAxis, ref int pivot, ref float minScore)
            {
                CompareVertices comparator;
                comparator.SortAxis = axis;
                NativeSortExtension.Sort((float4*)points.GetUnsafePtr(), rangeLength, comparator);

                PointAndIndex* p = (PointAndIndex*)points.GetUnsafePtr();

                Aabb runningAabb = Aabb.Empty;

                for (int i = 0; i < rangeLength; i++)
                {
                    runningAabb.Include(Aabbs[p[i].Index]);
                    scores[i] = (i + 1) * runningAabb.SurfaceArea;
                }

                runningAabb = Aabb.Empty;

                for (int i = rangeLength - 1, j = 1; i > 0; --i, ++j)
                {
                    runningAabb.Include(Aabbs[p[i].Index]);
                    float sum = scores[i - 1] + j * runningAabb.SurfaceArea;
                    if (sum < minScore)
                    {
                        pivot = i;
                        bestAxis = axis;
                        minScore = sum;
                    }
                }
            }

            void SegregateSah3(Range range, int minItems, ref Range lRange, ref Range rRange)
            {
                if (!ScratchScores.IsCreated)
                {
                    ScratchScores = new NativeArray<float>(Aabbs.Length, Allocator.Temp);
                    ScratchPointsX = new NativeArray<float4>(Aabbs.Length, Allocator.Temp);
                    ScratchPointsY = new NativeArray<float4>(Aabbs.Length, Allocator.Temp);
                    ScratchPointsZ = new NativeArray<float4>(Aabbs.Length, Allocator.Temp);
                }

                // This code relies on range.length always being less than or equal to the number of primitives, which
                // happens to be Aabbs.length.  If that ever becomes not true then scratch memory size should be increased.
                Assert.IsTrue(range.Length <= ScratchScores.Length /*, "Aabbs.Length isn't a large enough scratch memory size for SegregateSah3"*/);

                float4* p = PointsAsFloat4 + range.Start;

                for (int i = 0; i < range.Length; i++)
                {
                    ScratchPointsX[i] = p[i];
                    ScratchPointsY[i] = p[i];
                    ScratchPointsZ[i] = p[i];
                }

                int bestAxis = -1, pivot = -1;
                float minScore = float.MaxValue;

                ProcessAxis(range.Length, 0, ScratchScores, ScratchPointsX, ref bestAxis, ref pivot, ref minScore);
                ProcessAxis(range.Length, 1, ScratchScores, ScratchPointsY, ref bestAxis, ref pivot, ref minScore);
                ProcessAxis(range.Length, 2, ScratchScores, ScratchPointsZ, ref bestAxis, ref pivot, ref minScore);

                // Build sub-ranges.
                int lSize = pivot;
                int rSize = range.Length - lSize;
                if (lSize < minItems || rSize < minItems)
                {
                    // Make sure sub-ranges contains at least minItems nodes, in these rare cases (i.e. all points at the same position), we just split the set in half regardless of positions.
                    SplitRange(ref range, range.Length / 2, ref lRange, ref rRange);
                }
                else
                {
                    SplitRange(ref range, lSize, ref lRange, ref rRange);
                }

                float4* sortedPoints;

                if (bestAxis == 0)
                {
                    sortedPoints = (float4*)ScratchPointsX.GetUnsafePtr();
                }
                else if (bestAxis == 1)
                {
                    sortedPoints = (float4*)ScratchPointsY.GetUnsafePtr();
                }
                else // bestAxis == 2
                {
                    sortedPoints = (float4*)ScratchPointsZ.GetUnsafePtr();
                }

                // Write back sorted points.
                for (int i = 0; i < range.Length; i++)
                {
                    p[i] = sortedPoints[i];
                }
            }

            private static void Swap<T>(ref T a, ref T b) where T : struct { T t = a; a = b; b = t; }

            void Segregate(int axis, float pivot, Range range, int minItems, ref Range lRange, ref Range rRange)
            {
                Assert.IsTrue(range.Length > 1 /*, "Range length must be greater than 1."*/);

                Aabb lDomain = Aabb.Empty;
                Aabb rDomain = Aabb.Empty;

                float4* p = PointsAsFloat4;
                float4* start = p + range.Start;
                float4* end = start + range.Length - 1;

                do
                {
                    // Consume left.

                    while (start <= end && (*start)[axis] < pivot)
                    {
                        lDomain.Include((*(start++)).xyz);
                    }

                    // Consume right.
                    while (end > start && (*end)[axis] >= pivot)
                    {
                        rDomain.Include((*(end--)).xyz);
                    }

                    if (start >= end) goto FINISHED;

                    lDomain.Include((*end).xyz);
                    rDomain.Include((*start).xyz);

                    Swap(ref *(start++), ref *(end--));
                }
                while (true);
            FINISHED:
                // Build sub-ranges.
                int lSize = (int)(start - p);
                int rSize = range.Length - lSize;
                if (lSize < minItems || rSize < minItems)
                {
                    // Make sure sub-ranges contains at least minItems nodes, in these rare cases (i.e. all points at the same position), we just split the set in half regardless of positions.
                    SplitRange(ref range, range.Length / 2, ref lRange, ref rRange);

                    SetAabbFromPoints(ref lDomain, PointsAsFloat4 + lRange.Start, lRange.Length);
                    SetAabbFromPoints(ref rDomain, PointsAsFloat4 + rRange.Start, rRange.Length);
                }
                else
                {
                    SplitRange(ref range, lSize, ref lRange, ref rRange);
                }

                lRange.Domain = lDomain;
                rRange.Domain = rDomain;
            }

            void CreateChildren(Range* subRanges, int numSubRanges, Range range, ref int freeNodeIndex, Range* rangeStack, ref int stackSize)
            {
                int4 parentData = int4.zero;

                for (int i = 0; i < numSubRanges; i++)
                {
                    // Add child node.
                    int childNodeIndex = freeNodeIndex++;
                    parentData[i] = childNodeIndex;

                    if (subRanges[i].Length > 4)
                    {
                        // Keep splitting the range, push it on the stack.
                        var subRange = subRanges[i];
                        subRange.Root = childNodeIndex;
                        subRange.Parent = range.Root;
                        rangeStack[stackSize++] = subRange;
                    }
                    else
                    {
                        ref Node childNode = ref Bvh.GetNode(childNodeIndex);
                        childNode = Node.EmptyLeaf;
                        childNode.Parent = range.Root;
                        childNode.IsLeaf = true;

                        for (int pointIndex = 0; pointIndex < subRanges[i].Length; pointIndex++)
                        {
                            childNode.Data[pointIndex] = Points[subRanges[i].Start + pointIndex].Index;
                        }

                        for (int j = subRanges[i].Length; j < 4; j++)
                        {
                            childNode.ClearLeafData(j);
                        }
                    }
                }

                ref var parentNode = ref Bvh.GetNode(range.Root);
                parentNode = Node.Empty;
                parentNode.Parent = range.Parent;
                parentNode.Data = parentData;
                parentNode.IsInternal = true;
            }

            float4* PointsAsFloat4 => (float4*)Points.GetUnsafePtr();

            void ProcessSmallRange(Range baseRange, ref int freeNodeIndex)
            {
                Range range = baseRange;

                ComputeAxisAndPivot(ref range, out int axis, out float pivot);
                SortRange(axis, ref range);

                Range* subRanges = stackalloc Range[4];
                int hasLeftOvers;
                do
                {
                    int numSubRanges = 0;
                    while (range.Length > 4 && numSubRanges < 3)
                    {
                        subRanges[numSubRanges].Start = range.Start;
                        subRanges[numSubRanges].Length = 4;
                        numSubRanges++;

                        range.Start += 4;
                        range.Length -= 4;
                    }

                    if (range.Length > 0)
                    {
                        subRanges[numSubRanges].Start = range.Start;
                        subRanges[numSubRanges].Length = range.Length;

                        numSubRanges++;
                    }

                    hasLeftOvers = 0;
                    CreateChildren(subRanges, numSubRanges, range, ref freeNodeIndex, &range, ref hasLeftOvers);

                    Assert.IsTrue(hasLeftOvers <= 1 /*, "Internal error"*/);
                }
                while (hasLeftOvers > 0);
            }

            public void ProcessLargeRange(Range range, Range* subRanges)
            {
                Range* temps = stackalloc Range[2];

                if (!UseSah)
                {
                    // split range in two temporary ranges:
                    ComputeAxisAndPivot(ref range, out int axis, out float pivot);
                    Segregate(axis, pivot, range, 2, ref temps[0], ref temps[1]);

                    // split both temporary ranges again in two, producing the final four sub-ranges:
                    ComputeAxisAndPivot(ref temps[0], out int lAxis, out float lPivot);
                    Segregate(lAxis, lPivot, temps[0], 1, ref subRanges[0], ref subRanges[1]);

                    ComputeAxisAndPivot(ref temps[1], out int rAxis, out float rPivot);
                    Segregate(rAxis, rPivot, temps[1], 1, ref subRanges[2], ref subRanges[3]);
                }
                else
                {
                    // split range in two temporary ranges using the surface area heuristic (SAH):
                    SegregateSah3(range, 2, ref temps[0], ref temps[1]);

                    // split temporary ranges again in 2, producing the final four sub-ranges:
                    SegregateSah3(temps[0], 1, ref subRanges[0], ref subRanges[1]);
                    SegregateSah3(temps[1], 1, ref subRanges[2], ref subRanges[3]);
                }
            }

            public void CreateInternalNodes(Range* subRanges, int numSubRanges, Range range, Range* rangeStack, ref int stackSize, ref int freeNodeIndex)
            {
                int4 rootData = int4.zero;

                for (int i = 0; i < numSubRanges; ++i)
                {
                    rootData[i] = freeNodeIndex++;
                    var subRange = subRanges[i];
                    subRange.Root = rootData[i];
                    subRange.Parent = range.Root;
                    rangeStack[stackSize++] = subRange;
                }

                ref Node rootNode = ref Bvh.GetNode(range.Root);
                rootNode = Node.Empty;
                rootNode.Parent = range.Parent;
                rootNode.Data = rootData;
                rootNode.IsInternal = true;
            }

            /// <summary>
            /// Builds a 4-way bounding volume hierarchy from the specified Range of data.
            /// </summary>
            public void Build(Range baseRange)
            {
                Range* ranges = stackalloc Range[Constants.UnaryStackSize];
                int rangeStackSize = 1;
                ranges[0] = baseRange;
                Range* subRanges = stackalloc Range[4];

                if (baseRange.Length > 4)
                {
                    do
                    {
                        Range range = ranges[--rangeStackSize];

                        if (range.Length <= Constants.SmallRangeSize)
                        {
                            ProcessSmallRange(range, ref FreeNodeIndex);
                        }
                        else
                        {
                            ProcessLargeRange(range, subRanges);
                            CreateChildren(subRanges, 4, range, ref FreeNodeIndex, ranges, ref rangeStackSize);
                        }
                    }
                    while (rangeStackSize > 0);
                }
                else // no need to further split the range if there are 4 or less objects in the range
                {
                    // Creates a single leaf node as child and attach it to the root node represented by the baseRange
                    CreateChildren(ranges, 1, baseRange, ref FreeNodeIndex, ranges, ref rangeStackSize);
                }
            }
        }

        public unsafe JobHandle ScheduleBuildJobs(
            NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> bodyFilters, NativeReference<int>.ReadOnly shouldDoWork,
            int numThreadsHint, JobHandle inputDeps, NativeArray<Builder.Range> ranges, NativeArray<int> numBranches)
        {
            JobHandle handle = inputDeps;

            var branchNodeOffsets = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var nodeCounts = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var oldNumBranches = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            // Build initial branches
            handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = m_Nodes,
                MaxNodeCount = m_MaxNodeCount,
                NodeFilters = m_NodeFilters,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches,
                OldBranchCount = oldNumBranches,
                ThreadCount = numThreadsHint,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle);

            // Build branches (Note: deallocates Points array on completion)
            handle = new BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = bodyFilters,
                Nodes = m_Nodes,
                MaxNodeCount = m_MaxNodeCount,
                NodeFilters = m_NodeFilters,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffsets,
                NodeCounts = nodeCounts
            }.ScheduleUnsafeIndex0(numBranches, 1, handle);

            // Note: This job also deallocates the aabbs and lookup arrays on completion
            handle = new FinalizeTreeJob
            {
                Aabbs = aabbs,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                NodesList = m_NodesList,
                NodeFiltersList = m_NodeFiltersList,
                LeafFilters = bodyFilters,
                BranchNodeOffsets = branchNodeOffsets,
                NodeCounts = nodeCounts,
                BranchCount = numBranches,
                OldBranchCount = oldNumBranches,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle);

            return handle;
        }

        /// <summary>
        /// <para>Clears the tree and adds a single empty node at the root (<see cref="GetNode">node</see> with index 1).</para>
        /// <para>Can only be called on an <see cref="IsIncremental">incremental</see> tree.</para>
        /// </summary>
        public unsafe void Clear()
        {
            SafetyChecks.CheckAreEqualAndThrow(true, IsIncremental);
            SafetyChecks.CheckAreEqualAndThrow(true, GetNodeCapacity() > 1);

            m_Nodes[0] = Node.Empty;
            m_Nodes[1] = Node.Empty;

            if (m_NodeFilters != null)
            {
                m_NodeFilters[0] = CollisionFilter.Zero;
                m_NodeFilters[1] = CollisionFilter.Zero;
            }

            NodeCount = 2;
        }

        /// <summary>
        /// Builds a bounding volume hierarchy for a set of objects with bounds given by the aabbs array.
        /// Each object is identified via an index and approximately located at some point, both being
        /// provided via the points array.
        /// </summary>
        public unsafe void Build(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, out int nodeCount, bool useSah = false)
        {
            m_Nodes[0] = Node.Empty;

            if (aabbs.Length > 0)
            {
                var builder = new Builder
                {
                    Bvh = this,
                    Points = points,
                    Aabbs = aabbs,
                    FreeNodeIndex = 2,
                    UseSah = useSah
                };

                Aabb aabb = new Aabb();
                SetAabbFromPoints(ref aabb, (float4*)points.GetUnsafePtr(), points.Length);
                builder.Build(new Builder.Range(0, points.Length, aabb)); // the range of data we want to include when building this tree
                // The number of nodes in the constructed tree corresponds to the next available node index, indicated by the FreeNodeIndex in the builder.
                nodeCount = builder.FreeNodeIndex;

                Refit(aabbs, 1, builder.FreeNodeIndex - 1);
            }
            else // equivalent to calling Clear() in an incremental tree
            {
                // No input AABBs - building a tree for no nodes.
                // Make an empty node for the root
                m_Nodes[1] = Node.Empty;
                nodeCount = 2;
            }

            // Make sure to set the correct node count in the lists if present
            if (m_NodesList != null)
            {
                SafetyChecks.CheckAreEqualAndThrow(true, m_NodesList->Capacity >= nodeCount);
                m_NodesList->Length = nodeCount;
            }

            if (m_NodeFiltersList != null)
            {
                SafetyChecks.CheckAreEqualAndThrow(true, m_NodeFiltersList->Capacity >= nodeCount);
                m_NodeFiltersList->Length = nodeCount;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe CollisionFilter BuildCombinedCollisionFilterForLeafNode(CollisionFilter* leafFilters, ref Node leafNode)
        {
            SafetyChecks.CheckAreEqualAndThrow(true, leafNode.IsLeaf);

            CollisionFilter combinedFilter = CollisionFilter.Zero;

            bool first = true;
            for (int j = 0; j < 4; ++j)
            {
                if (leafNode.IsLeafValid(j))
                {
                    CollisionFilter leafFilter = leafFilters[leafNode.Data[j]];
                    combinedFilter = first ? leafFilter : CollisionFilter.CreateUnion(combinedFilter, leafFilter);
                    first = false;
                }
            }

            return combinedFilter;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe CollisionFilter BuildCombinedCollisionFilterForInternalNode(ref Node internalNode)
        {
            SafetyChecks.CheckAreEqualAndThrow(true, internalNode.IsInternal);
            CollisionFilter combinedFilter = CollisionFilter.Zero;

            bool first = true;
            for (int j = 0; j < 4; j++)
            {
                if (internalNode.IsInternalValid(j))
                {
                    CollisionFilter nodeFilter = m_NodeFilters[internalNode.Data[j]];
                    combinedFilter = first ? nodeFilter : CollisionFilter.CreateUnion(combinedFilter, nodeFilter);
                    first = false;
                }
            }

            return combinedFilter;
        }

        // For each node between nodeStartIndex and nodeEnd index, set the collision filter info to the combination of the node's children
        internal unsafe void BuildCombinedCollisionFilter(NativeArray<CollisionFilter> leafFilterInfo, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            SafetyChecks.CheckIndexAndThrow(nodeEndIndex, m_MaxNodeCount);
            Node* currentNode = baseNode + nodeEndIndex;
            CollisionFilter* leafFilters = (CollisionFilter*)leafFilterInfo.GetUnsafeReadOnlyPtr();

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                SafetyChecks.CheckIndexAndThrow(i, m_MaxNodeCount);
                m_NodeFilters[i] = currentNode->IsLeaf ? BuildCombinedCollisionFilterForLeafNode(leafFilters, ref UnsafeUtility.AsRef<Node>(currentNode))
                    : BuildCombinedCollisionFilterForInternalNode(ref UnsafeUtility.AsRef<Node>(currentNode));
            }
        }

        // Set the collision filter on nodeIndex to the combination of all its child filters. Node must not be a leaf.
        unsafe void BuildCombinedCollisionFilter(int nodeIndex)
        {
            ref Node currentNode = ref GetNode(nodeIndex);

            Assert.IsTrue(currentNode.IsInternal);

            CollisionFilter combinedFilter = new CollisionFilter();
            for (int j = 0; j < 4; j++)
            {
                combinedFilter = CollisionFilter.CreateUnion(combinedFilter, m_NodeFilters[currentNode.Data[j]]);
            }

            m_NodeFilters[nodeIndex] = combinedFilter;
        }

        public unsafe void Refit(NativeArray<Aabb> aabbs, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            SafetyChecks.CheckIndexAndThrow(nodeEndIndex, m_MaxNodeCount);
            Node* currentNode = baseNode + nodeEndIndex;

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                SafetyChecks.CheckIndexAndThrow(i, m_MaxNodeCount);
                if (currentNode->IsLeaf)
                {
                    ushort numFreeLeafSlots = 0;
                    uint numElements = 0;
                    for (int j = 0; j < 4; ++j)
                    {
                        Aabb aabb;
                        if (currentNode->IsLeafValid(j))
                        {
                            aabb = aabbs[currentNode->Data[j]];
                            ++numElements;
                        }
                        else
                        {
                            aabb = Aabb.Empty;
                            ++numFreeLeafSlots;
                        }

                        currentNode->Bounds.SetAabb(j, aabb);
                    }
                    currentNode->NumFreeSlotsInLeaf = numFreeLeafSlots;
                    currentNode->NumElements = numElements;
                }
                else
                {
                    uint numElements = 0;
                    for (int j = 0; j < 4; j++)
                    {
                        Aabb aabb;
                        if (currentNode->IsInternalValid(j))
                        {
                            ref var childNode = ref baseNode[currentNode->Data[j]];
                            aabb = childNode.Bounds.GetCompoundAabb();

                            currentNode->NumFreeLeafSlots[j] = (ushort)childNode.NumFreeLeafSlotsTotal;
                            numElements += childNode.NumElements;
                        }
                        else
                        {
                            aabb = Aabb.Empty;

                            // could add a new leaf node here which would provide 4 available leaf slots
                            currentNode->NumFreeLeafSlots[j] = 4;
                        }

                        currentNode->Bounds.SetAabb(j, aabb);
                        currentNode->NumElements = numElements;
                    }
                }
            }
        }

        unsafe void RefitNode(int nodeIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeIndex;

            Assert.IsTrue(currentNode->IsInternal);

            for (int j = 0; j < 4; j++)
            {
                Aabb compoundAabb = baseNode[currentNode->Data[j]].Bounds.GetCompoundAabb();
                currentNode->Bounds.SetAabb(j, compoundAabb);
            }
        }

        private struct RangeSizeAndIndex
        {
            public int RangeIndex;
            public int RangeSize;
            public int RangeFirstNodeOffset;
        }

        unsafe void SortRangeMap(RangeSizeAndIndex* rangeMap, int numElements)
        {
            for (int i = 0; i < numElements; i++)
            {
                RangeSizeAndIndex value = rangeMap[i];
                int key = rangeMap[i].RangeSize;
                int j = i;
                while (j > 0 && key > rangeMap[j - 1].RangeSize)
                {
                    rangeMap[j] = rangeMap[j - 1];
                    j--;
                }

                rangeMap[j] = value;
            }
        }

        internal unsafe void BuildFirstNLevels(
            NativeArray<PointAndIndex> points,
            NativeArray<Builder.Range> branchRanges, NativeArray<int> branchNodeOffset,
            int threadCount, out int branchCount)
        {
            Builder.Range* level0 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            Builder.Range* level1 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            int level0Size = 1; // start with root as level0. Size therefore only 1.
            int level1Size = 0;

            Aabb aabb = new Aabb();
            SetAabbFromPoints(ref aabb, (float4*)points.GetUnsafePtr(), points.Length);
            // construct initial range for root node, containing all elements
            level0[0] = new Builder.Range(0, points.Length, aabb);

            int largestAllowedRange = math.max(level0[0].Length / threadCount, Constants.SmallRangeSize);
            int smallRangeThreshold = math.max(largestAllowedRange / threadCount, Constants.SmallRangeSize);
            int largestRangeInLastLevel;
            int maxNumBranchesMinusOneSplit = Constants.MaxNumTreeBranches - 3;
            int freeNodeIndex = 2;

            var builder = new Builder { Bvh = this, Points = points, UseSah = false };

            Builder.Range* subRanges = stackalloc Builder.Range[4];

            do
            {
                largestRangeInLastLevel = 0;

                for (int i = 0; i < level0Size; ++i)
                {
                    if (level0[i].Length > smallRangeThreshold && freeNodeIndex < maxNumBranchesMinusOneSplit)
                    {
                        // Split range in up to 4 sub-ranges.

                        builder.ProcessLargeRange(level0[i], subRanges);

                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[0].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[1].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[2].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[3].Length);

                        // Create nodes for the sub-ranges and append level 1 sub-ranges.
                        builder.CreateInternalNodes(subRanges, 4, level0[i], level1, ref level1Size, ref freeNodeIndex);
                    }
                    else
                    {
                        // Too small, ignore.
                        level1[level1Size++] = level0[i];
                    }
                }

                // swap new and old level stacks to descent deeper into the tree in the next iteration
                Builder.Range* tmp = level0;
                level0 = level1;
                level1 = tmp;

                level0Size = level1Size;
                level1Size = 0;
                smallRangeThreshold = largestAllowedRange;
            }
            while (level0Size < Constants.MaxNumTreeBranches && largestRangeInLastLevel > largestAllowedRange);

            RangeSizeAndIndex* rangeMapBySize = stackalloc RangeSizeAndIndex[Constants.MaxNumTreeBranches];

            int nodeOffset = freeNodeIndex;
            for (int i = 0; i < level0Size; i++)
            {
                rangeMapBySize[i] = new RangeSizeAndIndex { RangeIndex = i, RangeSize = level0[i].Length, RangeFirstNodeOffset = nodeOffset };

                // The nodeOffset below defines the starting point in the final tree's nodes array for each subtree that we will
                // build in parallel (see BuildBranchesJob).
                // So that we don't create data races, we need to leave enough space for each job.
                // Choosing the node offset for the different subtrees as level0[i].Length is conservative as we will
                // show below. This also means that we can have untouched nodes in the nodes array between the subtrees
                // once the tree is fully built.
                //
                // Proof:
                // In a perfectly balanced b-ary tree of height h, the total number of nodes n can be computed as
                //      n = (b^(h+1)-1) / (b-1).
                // The height required for a tree with l leaf nodes is h=log_b(l). Here, we need to insert level0[i].Length
                // many elements into leaf nodes. Considering that we can insert up to 4 elements into a leaf node, we have
                //      l = L/4,
                // where L is level0[i].Length.
                // The total number of nodes n required in a perfectly balanced tree containing l leaves is then given by
                //      n = (b^(h+1)-1) / (b-1) = (b^(log_b(l)+1)-1) / (b-1) = (l * b - 1) / (b-1).
                // With b=4, we have
                //      n = (l * 4 - 1) / 3 = L/4 * 4/3 - 1/3 = L/3 - 1/3 < L.
                // Using L = level0[i].Length as the node offset is therefore sufficient and conservative.
                //
                nodeOffset += level0[i].Length;
            }

            SortRangeMap(rangeMapBySize, level0Size);

            for (int i = 0; i < level0Size; i++)
            {
                branchRanges[i] = level0[rangeMapBySize[i].RangeIndex];
                branchNodeOffset[i] = rangeMapBySize[i].RangeFirstNodeOffset;
            }

            for (int i = level0Size; i < Constants.MaxNumTreeBranches; i++)
            {
                branchNodeOffset[i] = -1;
            }

            branchCount = level0Size;

            m_Nodes[0] = Node.Empty;
        }

        // Build the branch for range. Returns the index of the last built node in the range
        internal int BuildBranch(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabb, Builder.Range range, int firstNodeIndex)
        {
            var builder = new Builder
            {
                Bvh = this,
                Points = points,
                FreeNodeIndex = firstNodeIndex,
                UseSah = false
            };

            builder.Build(range);

            Refit(aabb, firstNodeIndex, builder.FreeNodeIndex - 1);
            RefitNode(range.Root);
            return builder.FreeNodeIndex - 1;
        }

        // helper
        private static unsafe void SetAabbFromPoints(ref Aabb aabb, float4* points, int length)
        {
            aabb.Min = Math.Constants.Max3F;
            aabb.Max = Math.Constants.Min3F;
            for (int i = 0; i < length; i++)
            {
                aabb.Min = math.min(aabb.Min, points[i].xyz);
                aabb.Max = math.max(aabb.Max, points[i].xyz);
            }
        }

        [BurstCompile]
        internal unsafe struct BuildFirstNLevelsJob : IJob
        {
            public NativeArray<PointAndIndex> Points;
            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            public int MaxNodeCount;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;
            public NativeArray<Builder.Range> Ranges;
            public NativeArray<int> BranchNodeOffsets;
            public NativeArray<int> BranchCount;
            public NativeArray<int> OldBranchCount;
            public NativeReference<int>.ReadOnly ShouldDoWork;

            public int ThreadCount;

            public void Execute()
            {
                // Save old branch count for finalize tree job
                OldBranchCount[0] = BranchCount[0];

                if (ShouldDoWork.Value == 0)
                {
                    // If we need to to skip tree building tasks, than set BranchCount to zero so
                    // that BuildBranchesJob also gets early out in runtime.
                    BranchCount[0] = 0;
                    return;
                }

                var bvh = new BoundingVolumeHierarchy(Nodes, MaxNodeCount, NodeFilters);
                bvh.BuildFirstNLevels(Points, Ranges, BranchNodeOffsets, ThreadCount, out int branchCount);
                BranchCount[0] = branchCount;
            }
        }

        [BurstCompile]
        internal unsafe struct BuildBranchesJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<Aabb> Aabbs;
            [ReadOnly] public NativeArray<CollisionFilter> BodyFilters;
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> BranchNodeOffsets;

            [NativeDisableParallelForRestriction]
            public NativeArray<int> NodeCounts;

            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            public int MaxNodeCount;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;

            [NativeDisableContainerSafetyRestriction]
            [DeallocateOnJobCompletion] public NativeArray<PointAndIndex> Points;

            public void Execute(int index)
            {
                Assert.IsTrue(BranchNodeOffsets[index] >= 0);
                var bvh = new BoundingVolumeHierarchy(Nodes, MaxNodeCount, NodeFilters);
                int lastNode = bvh.BuildBranch(Points, Aabbs, Ranges[index], BranchNodeOffsets[index]);

                NodeCounts[index] = lastNode + 1;

                if (NodeFilters != null)
                {
                    bvh.BuildCombinedCollisionFilter(BodyFilters, BranchNodeOffsets[index], lastNode);
                    bvh.BuildCombinedCollisionFilter(Ranges[index].Root);
                }
            }
        }

        [BurstCompile]
        internal unsafe struct FinalizeTreeJob : IJob
        {
            [ReadOnly][DeallocateOnJobCompletion] public NativeArray<Aabb> Aabbs;
            [ReadOnly][DeallocateOnJobCompletion] public NativeArray<int> BranchNodeOffsets;
            [ReadOnly][DeallocateOnJobCompletion] public NativeArray<int> NodeCounts;
            [ReadOnly] public NativeArray<CollisionFilter> LeafFilters;
            public NativeReference<int>.ReadOnly ShouldDoWork;
            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;
            [NativeDisableUnsafePtrRestriction]
            public UnsafeList<Node>* NodesList;
            [NativeDisableUnsafePtrRestriction]
            public UnsafeList<CollisionFilter>* NodeFiltersList;
            [DeallocateOnJobCompletion][ReadOnly] public NativeArray<int> OldBranchCount;
            public NativeArray<int> BranchCount;

            public void Execute()
            {
                if (ShouldDoWork.Value == 0)
                {
                    // Restore original branch count
                    BranchCount[0] = OldBranchCount[0];
                    return;
                }

                int minBranchNodeIndex = BranchNodeOffsets[0] - 1;
                int maxNodeCount = NodeCounts[0];
                for (int i = 1; i < BranchCount[0]; i++)
                {
                    maxNodeCount = math.max(NodeCounts[i], maxNodeCount);
                    minBranchNodeIndex = math.min(BranchNodeOffsets[i] - 1, minBranchNodeIndex);
                }

                // make sure to set the correct length of the lists if present
                if (NodesList != null)
                {
                    SafetyChecks.CheckAreEqualAndThrow(true, NodesList->Capacity >= maxNodeCount);
                    NodesList->Length = maxNodeCount;
                }

                if (NodeFiltersList != null)
                {
                    SafetyChecks.CheckAreEqualAndThrow(true, NodeFiltersList->Capacity >= maxNodeCount);
                    NodeFiltersList->Length = maxNodeCount;
                }

                var bvh = new BoundingVolumeHierarchy(Nodes, maxNodeCount , NodeFilters);
                bvh.Refit(Aabbs, 1, minBranchNodeIndex);

                if (NodeFilters != null)
                {
                    bvh.BuildCombinedCollisionFilter(LeafFilters, 1, minBranchNodeIndex);
                }
            }
        }

        internal unsafe void CheckIntegrity(int numElements, CollisionFilter* filters)
        {
            if (filters != null && m_NodeFilters != null)
            {
                CheckLeafNodeFilterIntegrity(filters);
            }

            CheckLeafNodeElementIntegrity(numElements);
            CheckIntegrity();
        }

        // Verifies that the tree is built correctly in that:
        //  a) child nodes are leaves and have data
        //  b) all AABBs are valid (min < max for all axes)
        //  c) ensures that if a node is invalid, that the AABB is also invalid
        //  d) verifies the parent node does not contain AABB data
        //  e) validates free slot count and number of element count in the nodes
        //  f) validates inner node collision filter integrity
        internal unsafe void CheckIntegrity(int nodeIndex = 1, int parentIndex = 0, byte childIndex = 0)
        {
            Node parent = GetNode(parentIndex);
            Node node = GetNode(nodeIndex);
            // the 0'th node is reserved as an empty invalid node. The tree starts at index 1.
            var parentValid = parentIndex != 0;

            if (parentValid)
            {
                // check if child node specifies correct parent node index
                if (node.Parent != parentIndex)
                {
                    SafetyChecks.ThrowInvalidOperationException("Parent node index in child does not match actual parent node index.");
                    return;
                }

                // check parent's collision filter
                if (m_NodeFilters != null)
                {
                    var combinedFilter = BuildCombinedCollisionFilterForInternalNode(ref parent);
                    if (!combinedFilter.Equals(m_NodeFilters[parentIndex]))
                    {
                        SafetyChecks.ThrowInvalidOperationException("Parent filter does not match combination of child filters.");
                        return;
                    }
                }
            }

            var expectedNumFreeSlots = parent.NumFreeLeafSlots[childIndex];
            uint expectedNumElements = node.NumElements;

            // check if the expected number of free slots and the number of elements match the actual numbers in the children.
            if (node.IsLeaf) // leaf node
            {
                var numElements = node.NumValidChildren();
                var numFreeSlots = 4 - numElements;

                if (numElements != expectedNumElements)
                {
                    SafetyChecks.ThrowInvalidOperationException("Expected number of elements in leaf node does not match actual number of elements.");
                    return;
                }

                if ((parentValid && numFreeSlots != expectedNumFreeSlots)
                    || numFreeSlots != node.NumFreeLeafSlotsTotal)
                {
                    SafetyChecks.ThrowInvalidOperationException("Actually available number of free slots in leaf node does not match expected number of free slots.");
                    return;
                }
            }
            else // internal node
            {
                // check free slot count
                int specifiedNumFreeSlotsInChildrenTotal = 0;
                uint specifiedNumElementsInChildrenTotal = 0;
                for (byte i = 0; i < 4; ++i)
                {
                    specifiedNumFreeSlotsInChildrenTotal += node.NumFreeLeafSlots[i];

                    if (!node.IsChildValid(i)) // no child node present
                    {
                        // if the node has no child at this index, we expect the number of free slots to be exactly 4, since we could
                        // attach a leaf here.
                        if (node.NumFreeLeafSlots[i] != 4)
                        {
                            SafetyChecks.ThrowInvalidOperationException("Internal node has no leaf at this index, but number of free slots is not 4.");
                            return;
                        }
                    }
                    else
                    {
                        var childNode = m_Nodes[node.Data[i]];
                        specifiedNumElementsInChildrenTotal += childNode.NumElements;
                    }
                }

                if (parentValid)
                {
                    if (specifiedNumFreeSlotsInChildrenTotal != expectedNumFreeSlots)
                    {
                        SafetyChecks.ThrowInvalidOperationException(
                            "Expected number of free slots in parent does not match total number of free slots specified in children.");
                    }
                }

                if (specifiedNumElementsInChildrenTotal != expectedNumElements)
                {
                    SafetyChecks.ThrowInvalidOperationException(
                        "Expected number of elements in nodes does not match total number of elements specified in children.");
                }
            }

            // check bounds and recurse
            Aabb parentAabb = parent.Bounds.GetAabb(childIndex);
            for (byte i = 0; i < 4; ++i)
            {
                int data = node.Data[i];
                Aabb aabb = node.Bounds.GetAabb(i);

                bool validData = node.IsChildValid(i);

                bool validAabb = aabb.IsValid;

                if (!validData && validAabb)
                {
                    SafetyChecks.ThrowInvalidOperationException("An invalid node should have an empty AABB.");
                    return;
                }

                if (validData) // child node present
                {
                    if (parentValid)
                    {
                        if (validAabb && !parentAabb.Contains(aabb))
                        {
                            SafetyChecks.ThrowInvalidOperationException("Parent AABB does not contain child AABB.");
                            return;
                        }
                    }

                    if (node.IsInternal)
                    {
                        CheckIntegrity(data, nodeIndex, i);
                    }
                }
            }
        }

        internal void CheckLeafNodeElementIntegrity(int numElements = -1)
        {
            // make sure that all element indices from 0 to numElements - 1 are present in the tree
            var elementIndexSet = new NativeHashSet<int>(numElements, Allocator.Temp);

            var queue = new NativeQueue<int>(Allocator.Temp);

            int duplicateEntries = 0;
            int missingEntries = 0;

            // start with root node (node at index 1)
            queue.Enqueue(1);
            while (!queue.IsEmpty())
            {
                var nodeIndex = queue.Dequeue();
                var node = GetNode(nodeIndex);
                if (node.IsInternal)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        if (node.IsChildValid(i))
                        {
                            queue.Enqueue(node.Data[i]);
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        if (node.IsChildValid(i))
                        {
                            bool result = elementIndexSet.Add(node.Data[i]);
                            // check if we have a duplicate entry (which is when result is false, meaning that the
                            // element was not inserted since it was already in the set).
                            duplicateEntries += math.select(1, 0, result);
                        }
                    }
                }
            }

            bool success = true;
            if (duplicateEntries > 0)
            {
                success = false;
                Debug.LogError($"There are {duplicateEntries} duplicate elements in the tree.");
            }

            if (numElements >= 0)
            {
                // check if all elements are represented
                for (int i = 0; i < numElements; ++i)
                {
                    bool result = elementIndexSet.Contains(i);
                    // check if the element is missing
                    missingEntries += math.select(1, 0, result);
                }

                if (missingEntries > 0)
                {
                    success = false;
                    Debug.LogError($"There are {missingEntries} elements missing in the tree.");
                }

                if (elementIndexSet.Count != numElements)
                {
                    success = false;
                    Debug.LogError(
                        $"There are {elementIndexSet.Count} elements in the tree, but expected are {numElements} elements.");
                }
            }

            Assert.IsTrue(success);
        }

        internal unsafe void CheckLeafNodeFilterIntegrity(CollisionFilter* filters)
        {
            // make sure that all element indices from 0 to numBodies - 1 are present in the tree
            var queue = new NativeQueue<int>(Allocator.Temp);
            // start with root node (node at index 1)
            queue.Enqueue(1);
            while (!queue.IsEmpty())
            {
                var nodeIndex = queue.Dequeue();
                var node = GetNode(nodeIndex);
                if (node.IsInternal)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        if (node.IsChildValid(i))
                        {
                            queue.Enqueue(node.Data[i]);
                        }
                    }
                }
                else
                {
                    var combinedFilter = BuildCombinedCollisionFilterForLeafNode(filters, ref node);
                    if (!combinedFilter.Equals(m_NodeFilters[nodeIndex]))
                    {
                        SafetyChecks.ThrowInvalidOperationException("Leaf filter does not match combination of child filters.");
                    }
                }
            }
        }

        // Once a tree has been built, have a look at it and get some metrics to assess it's quality.
        // This method should run after CheckIntegrity as it is working under the assumption that the tree is valid.
        internal unsafe void WalkTreeAndAssess(ref int elementCount, int nodeIndex = 1)
        {
            Node node = m_Nodes[nodeIndex];

            int shortestPath = int.MaxValue; //find the data in the shortest path through the tree
            int longestPath = 0;  //find the data in the longest path through the tree
            int pathCount = 0;
            for (int i = 0; i < 4; ++i)
            {
                pathCount = 0; //count the number of nodes in the path to the data
                int data = node.Data[i];
                Aabb aabb = node.Bounds.GetAabb(i);

                if (aabb.IsValid)
                {
                    if (node.IsInternal)
                    {
                        WalkTreeAndAssess(ref elementCount, data);
                    }
                    else if (node.IsLeaf)
                    {
                        if (node.IsLeafValid(i))
                        {
                            // This is the end of the line. How far have we travelled
                            elementCount++;
                            pathCount++;
                            shortestPath = math.min(shortestPath, pathCount);
                            longestPath = math.max(longestPath, pathCount);
                        }
                    }
                }
            }

            Debug.Log("Walk found " + elementCount + " elements in the tree. Path count: " + pathCount + " Shortest path: " + shortestPath + ". Longest path: " + longestPath);
        }
    }
}
