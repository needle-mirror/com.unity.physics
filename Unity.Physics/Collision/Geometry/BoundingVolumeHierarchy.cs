//#define BVH_CHECK_INTEGRITY

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics
{
    // A 4-way bounding volume hierarchy
    internal partial struct BoundingVolumeHierarchy
    {
        private unsafe UnsafeList<Node>* m_NodesList;
        private unsafe UnsafeList<CollisionFilter>* m_NodeFiltersList;

        private unsafe Node* m_Nodes;
        private int m_MaxNodeCount;
        private unsafe CollisionFilter* m_NodeFilters;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private readonly int GetNodeCapacity()
        {
            return m_MaxNodeCount;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SetNodeCapacity(int capacity)
        {
            unsafe
            {
                SafetyChecks.CheckAreEqualAndThrow(true, IsIncremental);
                m_NodesList->Capacity = capacity;
                m_Nodes = m_NodesList->Ptr;
                m_MaxNodeCount = capacity;
                m_NodeFiltersList->Capacity = capacity;
                m_NodeFilters = m_NodeFiltersList->Ptr;
            }
        }

        /// <summary>
        ///  Returns true if the tree is in incremental mode, allowing use of the functions to add and remove nodes incrementally.
        /// </summary>
        public readonly bool IsIncremental
        {
            get
            {
                unsafe
                {
                    return m_NodesList != null && m_NodeFiltersList != null;
                }
            }
        }

        /// <summary>
        /// <para>The number of occupied nodes in the tree.</para>
        /// <para>Only valid when the tree is <see cref="IsIncremental">incremental</see>.</para>
        /// </summary>
        public int NodeCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            readonly get
            {
                unsafe
                {
                    SafetyChecks.CheckAreEqualAndThrow(true, IsIncremental);
                    return m_NodesList->Length;
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private set
            {
                unsafe
                {
                    m_NodesList->m_length = value;
                    m_NodeFiltersList->m_length = value;
                }
                SafetyChecks.CheckAreEqualAndThrow(true, IsIncremental);
            }
        }

        public readonly unsafe Node* Nodes => m_Nodes;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Node GetNode(int index)
        {
            unsafe
            {
                SafetyChecks.CheckIndexAndThrow(index, m_MaxNodeCount);
                return ref *(m_Nodes + index);
            }
        }

        /// <summary> Get the AABB that includes everything in this tree (compound AABB of first valid node in the tree). </summary>
        public unsafe Aabb Domain => m_Nodes[1].Bounds.GetCompoundAabb();

        /// <summary>
        /// <para>Create a new bounding volume hierarchy from the nodes and node filters located at the provided memory location.</para>
        /// <para>The tree will not be <see cref="IsIncremental">incremental</see>.</para>
        /// </summary>
        public unsafe BoundingVolumeHierarchy(Node* nodes, int maxNodeCount, CollisionFilter* nodeFilters)
        {
            m_Nodes = nodes;
            m_MaxNodeCount = maxNodeCount;
            m_NodeFilters = nodeFilters;
            m_NodesList = null;
            m_NodeFiltersList = null;
        }

        /// <summary>
        /// <para>Create a new <see cref="IsIncremental">incremental</see> bounding volume hierarchy using the provided
        /// node and node filter lists as storage.</para>
        /// <para> If required, the lists will be resized when <see cref="Insert">inserting</see> new nodes. The caller is responsible
        /// for disposing the lists when required.</para>
        /// </summary>
        public unsafe BoundingVolumeHierarchy(UnsafeList<Node>* nodes, UnsafeList<CollisionFilter>* nodeFilters, bool clear = true)
        {
            SafetyChecks.CheckAreEqualAndThrow(nodes->Length, nodeFilters->Length);
            SafetyChecks.CheckAreEqualAndThrow(true, nodes->IsCreated && nodeFilters->IsCreated);

            m_NodesList = nodes;
            m_NodeFiltersList = nodeFilters;
            m_Nodes = nodes->Ptr;
            m_MaxNodeCount = m_NodesList->Capacity;
            m_NodeFilters = nodeFilters->Ptr;

            if (clear)
            {
                Clear();
            }
        }

        /// <summary>
        /// <para>Create a new bounding volume hierarchy from the provided nodes and node filters.</para>
        /// <para>The tree will not be <see cref="IsIncremental">incremental</see>.</para>
        /// </summary>
        public unsafe BoundingVolumeHierarchy(NativeArray<Node> nodes, NativeArray<CollisionFilter> nodeFilters)
        {
            SafetyChecks.CheckAreEqualAndThrow(nodes.Length, nodeFilters.Length);

            m_Nodes = (Node*)nodes.GetUnsafeReadOnlyPtr();
            m_MaxNodeCount = nodes.Length;
            m_NodeFilters = (CollisionFilter*)nodeFilters.GetUnsafeReadOnlyPtr();
            m_NodesList = null;
            m_NodeFiltersList = null;
        }

        public unsafe BoundingVolumeHierarchy(NativeArray<Node> nodes)
        {
            m_Nodes = (Node*)nodes.GetUnsafeReadOnlyPtr();
            m_MaxNodeCount = nodes.Length;
            m_NodeFilters = null;
            m_NodesList = null;
            m_NodeFiltersList = null;
        }

        // A node in the hierarchy
        [DebuggerDisplay("{IsLeaf?\"Leaf\":\"Internal\"}, [ {Data[0]}, {Data[1]}, {Data[2]}, {Data[3]} ]")]
        [StructLayout(LayoutKind.Explicit, Size = 128)]
        public struct Node
        {
            [FieldOffset(0)]
            public FourTransposedAabbs Bounds;              // 96 bytes
            [FieldOffset(96)]
            public int4 Data;                               // 16 bytes
            [FieldOffset(112)]
            // Packed node data: 24 bits for parent node index followed by 8 bits for leaf/internal node type flag.
            // Warning: the layout must ensure that the leaf/internal node type flag appears exactly in the last
            // byte of the 4 byte m_NodeData member for correct mapping to Havok's BVH node struct.
            private uint m_NodeData;                        // 4 bytes
            [FieldOffset(116)]
            public unsafe fixed ushort NumFreeLeafSlots[4]; // 8 bytes
            [FieldOffset(124)]
            public uint NumElements;                        // 4 bytes

            static readonly int k_ParentShift = 8;
            static readonly uint k_ParentMask = 0xFFFFFF00;

            /// <summary> A flag to indicate the type of node, where it is set to one for leaves and zero for internal nodes </summary>
            private byte Flags
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => (byte)m_NodeData;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => m_NodeData = (m_NodeData & k_ParentMask) | value;
            }

            /// <summary>
            /// Index of the parent node of this node.
            /// </summary>
            public int Parent
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => (int)((m_NodeData & k_ParentMask) >> k_ParentShift);
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => m_NodeData = (m_NodeData & ~k_ParentMask) | (uint)(value << k_ParentShift);
            }

            /// <summary>
            /// Returns a Node such that the Data = 0 and IsLeaf = False. This can be used to initialize internal nodes.
            /// </summary>
            public static Node Empty
            {
                get
                {
                    unsafe
                    {
                        var node = new Node
                        {
                            Bounds = FourTransposedAabbs.Empty,
                            Data = int4.zero,
                            NumElements = 0,
                            Parent = 0,
                            IsLeaf = false
                        };
                        InitFreeSlotsArray(node.NumFreeLeafSlots, 4);
                        return node;
                    }
                }
            }

            /// <summary>
            /// Returns a Node such that the Data = -1 and IsLeaf = True. This can be used to initialize leaf nodes.
            /// </summary>
            public static Node EmptyLeaf
            {
                get
                {
                    unsafe
                    {
                        var node = new Node
                        {
                            Bounds = FourTransposedAabbs.Empty,
                            Data = new int4(-1),
                            NumElements = 0,
                            Parent = 0,
                            IsLeaf = true
                        };
                        // Note: children store their free leaf slots count in the first entry.
                        node.NumFreeLeafSlots[0] = 4;
                        return node;
                    }
                }
            }

            /// <summary> Is set to true if the node is internal. Sets Flags = 0 if true. </summary>
            public bool IsInternal
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => Flags == 0;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => Flags = (byte)(value ? 0 : 1);
            }

            /// <summary> Is set to true if the node is a leaf. Sets Flags = 1 if true. </summary>
            public bool IsLeaf
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => Flags != 0;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => Flags = (byte)(value ? 1 : 0);
            }

            /// <summary> Returns a bool4 for a node. Leaves are considered not valid if Data == -1 </summary>
            public bool4 AreLeavesValid => (Data != new int4(-1));

            /// <summary> Returns a bool4 for a node. Internal nodes are considered not valid if Data == 0 </summary>
            public bool4 AreInternalsValid => (Data != int4.zero);

            /// <summary>
            /// For a child to be considered valid, it must be a leaf and the Data != -1.
            /// </summary>
            /// <param name="index"> The index of the child in the node. </param>
            /// <returns></returns>
            public bool IsChildValid(int index)
            {
                if (IsLeaf && Data[index] == -1) return false;
                if (!IsLeaf && Data[index] == 0) return false;
                return true;
            }

            /// <summary> Counts the number of valid children within a node. </summary>
            public int NumValidChildren()
            {
                int cnt = 0;
                for (int i = 0; i < 4; i++)
                {
                    cnt += IsChildValid(i) ? 1 : 0;
                }

                return cnt;
            }

            public int NumFreeLeafSlotsTotal
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    unsafe
                    {
                        return IsLeaf
                            ? NumFreeLeafSlots[0]
                            : NumFreeLeafSlots[0] + NumFreeLeafSlots[1] + NumFreeLeafSlots[2] + NumFreeLeafSlots[3];
                    }
                }
            }

            public ushort NumFreeSlotsInLeaf
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    SafetyChecks.CheckAreEqualAndThrow(true, IsLeaf);
                    unsafe
                    {
                        return NumFreeLeafSlots[0];
                    }
                }
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set
                {
                    SafetyChecks.CheckAreEqualAndThrow(true, IsLeaf);
                    unsafe
                    {
                        NumFreeLeafSlots[0] = value;
                    }
                }
            }

            /// <summary> Returns a bool for a specified index in a node. Leaves are considered not valid if Data == -1 </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool IsLeafValid(int index) => Data[index] != -1;

            /// <summary> Returns a bool for a node. Internal nodes are considered not valid if Data == 0 </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool IsInternalValid(int index) => Data[index] != 0;

            /// <summary> Invalidates the data (sets to -1) for a specified index in a node </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ClearLeafData(int index) => Data[index] = -1;

            /// <summary> Clears the data (sets to 0) for a specified index in a node </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ClearInternalData(int index) => Data[index] = 0;

            /// <summary> Finds offset to child node containing the specified node index. </summary>
            public int FindChildOffset(int nodeIndex)
            {
                for (byte i = 0; i < 4; ++i)
                {
                    if (Data[i] == nodeIndex)
                    {
                        return i;
                    }
                }

                return -1;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static unsafe void InitFreeSlotsArray(ushort* array, ushort init)
            {
                array[0] = array[1] = array[2] = array[3] = init;
            }
        }

        #region Incremental updates

        public struct IncrementalInsertionContext : IDisposable
        {
            NativeParallelMultiHashMap<int, ElementLocationData> m_ElementLocationDataHashMap;

            public IncrementalInsertionContext(int capacity, Allocator allocator)
            {
                m_ElementLocationDataHashMap = new NativeParallelMultiHashMap<int, ElementLocationData>(capacity, allocator);
            }

            public bool IsCreated => m_ElementLocationDataHashMap.IsCreated;

            public void Clear()
            {
                m_ElementLocationDataHashMap.Clear();
            }

            public void Insert(ref BoundingVolumeHierarchy bvh, Aabb aabb, PointAndIndex point, CollisionFilter filter)
            {
                var result = bvh.Insert(aabb, point, filter);
                SafetyChecks.CheckAreEqualAndThrow(true, bvh.GetNode(result.NodeIndex).IsLeaf);

                Insert(ref bvh, result);
            }

            public void Insert(ref BoundingVolumeHierarchy bvh, InsertionResult result)
            {
                m_ElementLocationDataHashMap.Add(result.NodeIndex, new ElementLocationData
                {
                    ElementIndex = result.ElementIndex,
                    NodeIndex = result.NodeIndex,
                    LeafSlotIndex = result.ChildIndex
                });

                if (result.LeafNodeWasMigrated)
                {
                    // remove outdated migration data
                    m_ElementLocationDataHashMap.Remove(result.MigratedLeafNodeIndexOld);

                    // add new migration data for the migrated node
                    var migratedNode = bvh.GetNode(result.MigratedLeafNodeIndexNew);
                    SafetyChecks.CheckAreEqualAndThrow(true, migratedNode.IsLeaf);
                    for (byte j = 0; j < 4; ++j)
                    {
                        if (migratedNode.IsChildValid(j))
                        {
                            m_ElementLocationDataHashMap.Add(result.MigratedLeafNodeIndexNew,
                                new ElementLocationData
                                {
                                    ElementIndex = migratedNode.Data[j],
                                    NodeIndex = result.MigratedLeafNodeIndexNew,
                                    LeafSlotIndex = j
                                });
                        }
                    }
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void CopyTo(NativeList<ElementLocationData> list)
            {
                foreach (var data in m_ElementLocationDataHashMap)
                {
                    list.Add(data.Value);
                }
            }

            public void Dispose()
            {
                m_ElementLocationDataHashMap.Dispose();
            }
        }

        public struct RemovalData : IComparable<RemovalData>
        {
            public int NodeIndex;
            public byte LeafSlotIndex;

            public int CompareTo(RemovalData other)
            {
                if (NodeIndex == other.NodeIndex)
                {
                    return LeafSlotIndex - other.LeafSlotIndex;
                }
                // else:

                return NodeIndex - other.NodeIndex;
            }
        }

        public struct UpdateData : IComparable<UpdateData>
        {
            public enum CommandFlags
            {
                None = 0,
                UpdateAabb = 1 << 0,
                UpdateFilter = 1 << 1
            }

            public Aabb Aabb;
            public int NodeIndex;
            public byte LeafSlotIndex;
            public byte UpdateCommandFlags;

            public int CompareTo(UpdateData other)
            {
                if (NodeIndex == other.NodeIndex)
                {
                    return LeafSlotIndex - other.LeafSlotIndex;
                }
                // else:

                return NodeIndex - other.NodeIndex;
            }
        }

        public void Remove(NativeArray<RemovalData> removals, NativeArray<CollisionFilter> leafFiltersInfo)
            => RemoveOrUpdate(removals, new NativeArray<UpdateData>(), leafFiltersInfo);

        public unsafe void RemoveOrUpdate(NativeArray<RemovalData> removals, NativeArray<UpdateData> updates, NativeArray<CollisionFilter> leafFiltersInfo)
        {
            var elementsToProcess = removals.Length + updates.Length;
            SafetyChecks.CheckAreEqualAndThrow(true, elementsToProcess > 0);

            // For now we support incremental removal only in trees with collision filters.
            SafetyChecks.CheckAreEqualAndThrow(true, m_NodeFilters != null);

            var leafFilters = (CollisionFilter*)leafFiltersInfo.GetUnsafeReadOnlyPtr();
            var nodesToProcess = new NativePriorityHeap<int>(elementsToProcess, Allocator.Temp, HeapType.Max);
            var addedNodesSet = new NativeHashSet<int>(elementsToProcess, Allocator.Temp);

            // Fill nodesToProcess with parents of removed nodes.
            // Process from back to front assuming that removals array is sorted by increasing node index
            // for chance of adding parents with higher node index into priority queue first,
            // requiring less sort operations.
            for (int i = removals.Length - 1; i >= 0; --i)
            {
                var removalData = removals[i];
                var slotIndex = removalData.LeafSlotIndex;
                ref var leafNode = ref GetNode(removalData.NodeIndex);
                SafetyChecks.CheckAreEqualAndThrow(true, leafNode.IsLeaf);
                SafetyChecks.CheckAreEqualAndThrow(true, leafNode.IsChildValid(slotIndex));

                leafNode.ClearLeafData(slotIndex);
                leafNode.Bounds.SetAabb(slotIndex, Aabb.Empty);
                SafetyChecks.CheckAreEqualAndThrow(true, leafNode.NumFreeSlotsInLeaf <= 3);
                leafNode.NumFreeSlotsInLeaf++;
                SafetyChecks.CheckAreEqualAndThrow(true, leafNode.NumElements >= 1);
                leafNode.NumElements--;

                var parentIndex = leafNode.Parent;
                if (addedNodesSet.Add(parentIndex))
                {
                    nodesToProcess.Push(parentIndex);
                }

                if (leafNode.NumFreeSlotsInLeaf == 4 && removalData.NodeIndex == NodeCount - 1)
                {
                    // no more entries in leaf and leaf is last node in tree. Free it.
                    --NodeCount;

                    // Move up the tree and disconnect removed leaf node
                    ref var parent = ref GetNode(parentIndex);
                    var childOffset = parent.FindChildOffset(removalData.NodeIndex);
                    SafetyChecks.CheckAreEqualAndThrow(true, childOffset != -1);
                    parent.ClearInternalData(childOffset);
                }
                else
                {
                    // update combined collision filter for this node

                    // Note: we are aware that we are recalculating the combined collision filters for the leaf nodes here
                    // potentially multiple times. However, the processing and memory access cost is hopefully low enough that
                    // the potential duplication of work does not matter.
                    var nodeFilter = m_NodeFilters + removalData.NodeIndex;
                    *nodeFilter = BuildCombinedCollisionFilterForLeafNode(leafFilters, ref leafNode);
                }
            }

            // Fill nodesToProcess with parents of updated nodes.
            for (int i = updates.Length - 1; i >= 0; --i)
            {
                var updateData = updates[i];
                SafetyChecks.CheckAreEqualAndThrow(true, updateData.UpdateCommandFlags != (byte)UpdateData.CommandFlags.None);
                var slotIndex = updateData.LeafSlotIndex;
                ref var leafNode = ref GetNode(updateData.NodeIndex);
                SafetyChecks.CheckAreEqualAndThrow(true, leafNode.IsLeaf);
                SafetyChecks.CheckAreEqualAndThrow(true, leafNode.IsChildValid(slotIndex));

                var parentIndex = leafNode.Parent;
                if (addedNodesSet.Add(parentIndex))
                {
                    nodesToProcess.Push(parentIndex);
                }

                if ((updateData.UpdateCommandFlags & (byte)UpdateData.CommandFlags.UpdateAabb) != 0)
                {
                    leafNode.Bounds.SetAabb(slotIndex, updateData.Aabb);
                }

                if ((updateData.UpdateCommandFlags & (byte)UpdateData.CommandFlags.UpdateFilter) != 0)
                {
                    // update combined collision filter for this node

                    // Note: we are aware that we are recalculating the combined collision filters for the leaf nodes here
                    // potentially multiple times. However, the processing and memory access cost is hopefully low enough that
                    // the potential duplication of work does not matter.
                    var nodeFilter = m_NodeFilters + updateData.NodeIndex;
                    *nodeFilter = BuildCombinedCollisionFilterForLeafNode(leafFilters, ref leafNode);
                }
            }

            // in a breadth first manner, update the aabbs, free leaf slots, number of elements and collision filters
            // of all nodes up the tree until the root.
            var lastParentIndex = 0;
            do
            {
                var nodeIndex = nodesToProcess.Pop();
                ref var node = ref GetNode(nodeIndex);
                SafetyChecks.CheckAreEqualAndThrow(true, node.IsInternal);

                uint numElements = 0;
                int childNodeCount = 0;
                for (int j = 0; j < 4; j++)
                {
                    Aabb aabb;
                    if (node.IsInternalValid(j))
                    {
                        ++childNodeCount;
                        ref var childNode = ref GetNode(node.Data[j]);
                        aabb = childNode.Bounds.GetCompoundAabb();

                        node.NumFreeLeafSlots[j] = (ushort)childNode.NumFreeLeafSlotsTotal;
                        numElements += childNode.NumElements;
                    }
                    else
                    {
                        aabb = Aabb.Empty;

                        // could add a new leaf node here which would provide 4 available leaf slots
                        node.NumFreeLeafSlots[j] = 4;
                    }

                    node.Bounds.SetAabb(j, aabb);
                }

                // set final element count
                node.NumElements = numElements;

                if (nodeIndex > 1 && childNodeCount == 0 && nodeIndex == NodeCount - 1)
                {
                    // We haven't reached the root (1), there are no more child nodes
                    // in this inner node and this node is the last node in tree. Free it.
                    --NodeCount;

                    // Move up the tree and disconnect removed leaf node
                    ref var parent = ref GetNode(node.Parent);
                    var childOffset = parent.FindChildOffset(nodeIndex);
                    SafetyChecks.CheckAreEqualAndThrow(true, childOffset != -1);
                    parent.ClearInternalData(childOffset);
                }
                else
                {
                    // update combined collision filter for this node
                    var nodeFilter = m_NodeFilters + nodeIndex;
                    *nodeFilter = BuildCombinedCollisionFilterForInternalNode(ref node);
                }

                var parentIndex = node.Parent;
                if (addedNodesSet.Add(parentIndex))
                {
                    nodesToProcess.Push(parentIndex);
                }

                lastParentIndex = parentIndex;
            }
            while (lastParentIndex != 0);   // Note: The parent of the root of the tree is the invalid node with
                                            // index 0. Once we have reached this node we are done processing the root
                                            // (index 1) and are done.
#if BVH_CHECK_INTEGRITY
            CheckIntegrity();
#endif
        }

        struct NodeInsertionData : IComparable<NodeInsertionData>
        {
            public unsafe Node* NodePtr;
            public float Score;
            public readonly char NodeOffset;

            public NodeInsertionData(int nodeOffset)
            {
                unsafe
                {
                    NodePtr = null;
                    NodeOffset = (char)nodeOffset;
                    Score = -1;
                }
            }

            public int CompareTo(NodeInsertionData other)
            {
                return Score.CompareTo(other.Score);
            }
        }

        public struct InsertionResult
        {
            public int ElementIndex;
            public int NodeIndex;
            public int MigratedLeafNodeIndexOld;
            public int MigratedLeafNodeIndexNew;
            public byte ChildIndex;

            public InsertionResult(int elementIndex, int nodeIndex, byte childIndex)
            {
                ElementIndex = elementIndex;
                NodeIndex = nodeIndex;
                ChildIndex = childIndex;
                MigratedLeafNodeIndexOld = -1;
                MigratedLeafNodeIndexNew = -1;
            }

            public bool LeafNodeWasMigrated => MigratedLeafNodeIndexNew != -1;
        }

        public struct ElementLocationData : IComparable<ElementLocationData>
        {
            public int ElementIndex;
            public int NodeIndex;
            public byte LeafSlotIndex;

            public int CompareTo(ElementLocationData other)
            {
                if (ElementIndex == other.ElementIndex)
                {
                    return NodeIndex - other.NodeIndex;
                }
                // else:
                return ElementIndex - other.ElementIndex;
            }
        }

        public unsafe InsertionResult Insert(Aabb aabb, PointAndIndex point, CollisionFilter filter)
        {
            // For now we only support incremental insertion in trees with collision filters.
            SafetyChecks.CheckAreEqualAndThrow(true, m_NodeFilters != null);

            // node index stack, with pairs of global index and local-to-child offset,
            // indicating the path taken in the descent of the tree.
            var nodeIndexStack = stackalloc int2[Constants.UnaryStackSize];
            int nodeIndexStackCount = 0;

            const float kNoFreeSlotPenaltyFactor = 1.618f;
            // Always start at the root since the way we update the AABBs and other data
            // elements along the path to the leaf requires it.
            const int kStartNodeIndex = 1;
            var currentNode = m_Nodes + kStartNodeIndex;
            var currentNodeIndex = kStartNodeIndex;

            var insertionData = stackalloc NodeInsertionData[4];
            NodeInsertionData bestNodeData = default;

            while (!currentNode->IsLeaf)
            {
                nodeIndexStack[nodeIndexStackCount++] = new int2(currentNodeIndex, -1);

                // Simply traverse down where the aabb fits best and where there is space.
                // Find some acceptable balance between the two with some heuristic.
                // The heuristic should be based on the number of free slots in the leaf nodes and the depth of the tree.
                // We could also use the number of free slots in the leaf nodes as a heuristic for the depth of the tree.
                // Alternatively we can maintain the depth of the tree.
                // When calculating the increase in volume of the new aabb after insertion, we can use the heuristic to decide whether to insert the new aabb
                // at the optimal position or at a position that is not optimal but has still free slots.
                // If the optimal position has no free slots, we need to come up with a heuristic to decide whether it is still worth inserting the new aabb
                // at the optimal position or whether we should insert it at a position that is not optimal but has free slots.
                // The heuristic could be based on the number of comparison operations that will need to be done when using the BVH during queries.
                // If there will be more comparison operations when inserting the new aabb at the optimal position because we have to add another level to the tree,
                // then we should insert it at a position that is not optimal but has free slots.
                // If the optimal position is available, we should insert the new aabb there. This is a no-brainer.

                var done = false;
                for (int i = 0; i < 4; ++i)
                {
                    var data = new NodeInsertionData(i);
                    if (currentNode->IsChildValid(i))
                    {
                        var union = Aabb.Union(aabb, currentNode->Bounds.GetAabb(i));
                        var surfaceArea = union.SurfaceArea;

                        var childNode = m_Nodes + currentNode->Data[i];
                        data.Score = math.select(surfaceArea, surfaceArea * kNoFreeSlotPenaltyFactor,
                            currentNode->NumFreeLeafSlots[i] == 0);
                        data.NodePtr = childNode;
                    }
                    else
                    {
                        // We have a free slot in the inner node.
                        // In this case we add a new leaf node and insert there to broaden the tree as much as
                        // possible.

                        // todo: this can be sub-optimal since we might miss some good clustering and
                        // create overlapping aabbs in neighboring sub trees.

                        // store local to child offset for last node on the stack
                        nodeIndexStack[nodeIndexStackCount - 1].y = data.NodeOffset;

                        bestNodeData = data;
                        done = true;
                        break;
                    }

                    insertionData[i] = data;
                }

                if (done)
                {
                    break;
                }

                // sort the insertion data by score
                NativeSortExtension.Sort(insertionData, 4);

                bestNodeData = insertionData[0];
                var bestNodeIndex = currentNode->Data[bestNodeData.NodeOffset];

                // update aabb
                var innerAabb = currentNode->Bounds.GetAabb(bestNodeData.NodeOffset);
                currentNode->Bounds.SetAabb(bestNodeData.NodeOffset, Aabb.Union(innerAabb, aabb));
                // update free slots
                currentNode->NumFreeLeafSlots[bestNodeData.NodeOffset]--;
                // update number of elements
                currentNode->NumElements++;
                // update collision filter
                CollisionFilter* currentFilter = m_NodeFilters + currentNodeIndex;
                *currentFilter = CollisionFilter.CreateUnion(*currentFilter, filter);

                currentNode = bestNodeData.NodePtr;

                currentNodeIndex = bestNodeIndex;
                nodeIndexStack[nodeIndexStackCount - 1].y = bestNodeData.NodeOffset;
            }

            // insert in node
            if (currentNode->IsLeaf)
            {
                // todo: here we need to be careful. If we insert here blindly, we might create a very large leaf node.
                // we should have a heuristic that ensures that when inserting in this leaf node, we will not increase it by a given factor.
                // if we do, we should split the tree.
                // we might need to consider this already above when navigating down.
                // Potential approach: When slots available and aabb reasonably contained within path on which lies a slot, continue moving this way and if
                // we reach the slot without causing a too large expansion (some max expansion factor?) insert in that leaf.
                // Otherwise, split the tree! Insert a new inner node.
                // Problem: we can only do that at a leaf. Otherwise we will screw up the refit algorithm (leafs must be later in the node array).
                // If we had a back-pointer that wouldn't be a problem, BUT then the refit algorithm would have worse cache locality, which is bad for performance.
                // So, we can walk all the way down to the best fitting leaf, and if in that best fitting leaf there is no more room, insert a new leaf node and keep the new
                // entry separate from the previous leaf.
                // At that point, do we even need some sort of "max expansion" factor? Maybe only for cases where we encounter an inner node with a free slot, in which case,
                // if the expansion would be too large, we would  split the tree right there. And that could be done safely since the newly created node would be after the
                // inner node in the array to which we attach it.

                if (currentNode->NumFreeSlotsInLeaf > 0)
                {
                    // find the first free entry
                    for (byte i = 0; i < 4; ++i)
                    {
                        if (currentNode->Data[i] == -1)
                        {
                            currentNode->Bounds.SetAabb(i, aabb);
                            currentNode->Data[i] = point.Index;
                            currentNode->NumFreeSlotsInLeaf--;
                            currentNode->NumElements++;

                            // update combined collision filter for this node after inserting the new element
                            CollisionFilter* currentFilter = m_NodeFilters + currentNodeIndex;
                            *currentFilter = CollisionFilter.CreateUnion(*currentFilter, filter);

#if BVH_CHECK_INTEGRITY
                            CheckIntegrity();
#endif
                            return new InsertionResult(point.Index, currentNodeIndex, i);
                        }
                    }
                    // We should never get here since if we have free leaf slots within the leaf node, we should
                    // be able to find at least one and return above.
                    SafetyChecks.ThrowInvalidOperationException("No free leaf slot found in leaf node.");
                }
                // else:

                // Turn this leaf node into an internal node, add the old leaf node, and a new empty leaf node where we insert the new element.

                if (GetNodeCapacity() < NodeCount + 2)
                {
                    SetNodeCapacity(math.max(GetNodeCapacity() * 2, 64));
                    currentNode = m_Nodes + currentNodeIndex;
                }

                // @todo: rather than just migrating the old, full leaf node, subdivide the set of old leaf node entries and the new entry
                // into two subsets separating them by the longest axis of extent (or using the SAH heuristic), and add them to the two leaf nodes.

                // migrate old leaf node to a new leaf node location
                int migratedLeafNodeIndexOld = currentNodeIndex;
                int migratedLeafNodeIndexNew = NodeCount++;

                Node* migratedLeafNode = m_Nodes + migratedLeafNodeIndexNew;
                *migratedLeafNode = *currentNode;

                // copy the collision filter of the migrated leaf node from the old to the new leaf node location
                CollisionFilter* currentNodeFilter = m_NodeFilters + currentNodeIndex; // node filter of old, now migrated leaf node
                CollisionFilter* migratedLeafNodeFilter = m_NodeFilters + migratedLeafNodeIndexNew;
                *migratedLeafNodeFilter = *currentNodeFilter;

                // Set collision filter of new inner node to old, migrated leaf node's filter combined with the incoming
                // element's filter. The latter will be used as filter for the new leaf node.
                *currentNodeFilter = CollisionFilter.CreateUnion(*currentNodeFilter, filter);

                // turn old leaf node into an inner node while preserving its parent
                int oldParentIndex = currentNode->Parent;
                *currentNode = Node.Empty;
                currentNode->Parent = oldParentIndex;

                SafetyChecks.CheckAreEqualAndThrow(true, currentNode->IsInternal);

                // attach old, migrated leaf node to new inner node
                currentNode->Data[0] = migratedLeafNodeIndexNew;
                migratedLeafNode->Parent = currentNodeIndex;

                // create a new leaf node
                int newLeafNodeIndex = NodeCount++;
                Node* newLeafNode = m_Nodes + newLeafNodeIndex;
                *newLeafNode = Node.EmptyLeaf;

                // attach new leaf node to new inner node
                currentNode->Data[1] = newLeafNodeIndex;
                newLeafNode->Parent = currentNodeIndex;

                // insert the new element into the new leaf node
                newLeafNode->Data[0] = point.Index;
                newLeafNode->Bounds.SetAabb(0, aabb);

                // set the free slots count to 3 since we only add one element to the new node
                newLeafNode->NumFreeSlotsInLeaf = 3;
                newLeafNode->NumElements = 1;

                // set collision filter of new leaf node to incoming element's filter
                *(m_NodeFilters + newLeafNodeIndex) = filter;

                // set bounds of new inner node
                currentNode->Bounds.SetAabb(0, migratedLeafNode->Bounds.GetCompoundAabb());
                currentNode->Bounds.SetAabb(1, aabb);

                // Set free leaf node counts for the current node (the new inner node) directly.
                // The old, migrated leaf node at location 0 has 0 free slots and the newly added leaf node at
                // location 1 has 3 free slots.
                currentNode->NumFreeLeafSlots[0] = 0;
                currentNode->NumFreeLeafSlots[1] = 3;

                // Set number of elements for the current node (the new inner node) directly.
                currentNode->NumElements = 5;

                // Recalculate free leaf node count all the way to the top, starting at the parent of the current node.
                // +16 for the new inner node to which we attached the old leaf node.
                // -4 for the old, full leaf node that uses 4 slots.
                // -1 for the new leaf node that uses only 1 slot.
                // Total: +11
                // However, since we already previously updated all the nodes on the path to the root by -1,
                // in order to create the new and correct count, we need to update them by +12.
                UpdateNumFreeLeafSlots(m_Nodes, nodeIndexStack, nodeIndexStackCount, 12);

#if BVH_CHECK_INTEGRITY
                CheckIntegrity();
#endif

                // Note: this operation will move the entries in this leaf into new nodes. Any coherence data for these
                // entries will need to be updated. Return the corresponding migration information.
                return new InsertionResult
                {
                    ElementIndex = point.Index,
                    NodeIndex = newLeafNodeIndex,
                    ChildIndex = 0,
                    MigratedLeafNodeIndexNew = migratedLeafNodeIndexNew,
                    MigratedLeafNodeIndexOld = migratedLeafNodeIndexOld
                };
            }
            // else: inner node

            // The node is an internal node with a free slot.
            // Insert new leaf node at the free slot location.

            if (GetNodeCapacity() < NodeCount + 1)
            {
                SetNodeCapacity(math.max(GetNodeCapacity() * 2, 64));
                currentNode = m_Nodes + currentNodeIndex;
            }

            // @todo: Rather than just adding a new leaf node here, take all other leaf nodes on the same level (if any) and redistribute their
            // content according to the SAH heuristic or the longest axis of extent heuristic.

            {
                int freeSlotOffset = bestNodeData.NodeOffset;
                int newLeafNodeIndex = NodeCount++;

                Node* newLeafNode = m_Nodes + newLeafNodeIndex;
                *newLeafNode = Node.EmptyLeaf;

                currentNode->Data[freeSlotOffset] = newLeafNodeIndex;
                newLeafNode->Parent = currentNodeIndex;

                newLeafNode->Data[0] = point.Index;
                newLeafNode->Bounds.SetAabb(0, aabb);
                newLeafNode->NumFreeSlotsInLeaf = 3;
                newLeafNode->NumElements = 1;
                currentNode->NumFreeLeafSlots[freeSlotOffset] = 3;
                currentNode->NumElements++;

                currentNode->Bounds.SetAabb(freeSlotOffset, aabb);

                // update collision filters
                CollisionFilter* currentFilter = m_NodeFilters + currentNodeIndex;
                // old inner node filter must be combined with filter of incoming element, which will correspond to the
                // node filter of the newly added leaf node.
                *currentFilter = CollisionFilter.CreateUnion(*currentFilter, filter);
                // new leaf node filter is simply the filter of the incoming element
                *(m_NodeFilters + newLeafNodeIndex) = filter;

#if BVH_CHECK_INTEGRITY
                CheckIntegrity();
#endif
                return new InsertionResult(point.Index, newLeafNodeIndex, 0);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void UpdateNumFreeLeafSlots(Node* nodes, int2* nodeIndexStack, int nodeIndexStackCount, ushort slotIncrement)
        {
            for (int i = nodeIndexStackCount - 1; i >= 0; --i)
            {
                var nodeIndexPair = nodeIndexStack[i];
                Node* node = nodes + nodeIndexPair.x;
                node->NumFreeLeafSlots[nodeIndexPair.y] += slotIncrement;
            }
        }

        /// <summary>
        /// Calculates tree split data for use in multi-threaded overlap pair search.
        /// <seealso cref="Broadphase.CalculateTreeSplitDataJob"/>
        /// <seealso cref="Broadphase.ScheduleFindOverlapsJobs"/>
        /// </summary>
        public unsafe void CalculateTreeSplitData(NativeArray<Builder.Range> branchRanges, int threadCount, out int branchCount)
        {
            // Note: this function is a modified version of BoundingVolumeHierarchy.BuildFirstNLevels, containing
            // only the portions relevant to the calculation of the tree split data, which is reported as the number of branches
            // via the branchCount output parameter and the root nodes of the subtrees via the Range.Root index in the branchRanges
            // array.

            // It is assumed that the branchRanges array is large enough to hold the data for all branches, that is,
            // its length is at least as large as the maximum number of branches allowed in the split.
            SafetyChecks.CheckAreEqualAndThrow(true, branchRanges.Length >= Constants.MaxNumTreeBranches);

            Builder.Range* level0 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            Builder.Range* level1 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            int level0Size = 1; // start with root as level0. Size therefore only 1.
            int level1Size = 0;

            var rootNode = m_Nodes + 1;
            level0[0] = new Builder.Range(0, (int)rootNode->NumElements, Aabb.Empty);

            int largestAllowedRange = math.max(level0[0].Length / threadCount, Constants.SmallRangeSize);
            int smallRangeThreshold = math.max(largestAllowedRange / threadCount, Constants.SmallRangeSize);
            int largestRangeInLastLevel;
            int maxNumBranchesMinusOneSplit = Constants.MaxNumTreeBranches - 3;
            int freeNodeIndex = 2;

            do
            {
                largestRangeInLastLevel = 0;

                for (int i = 0; i < level0Size; ++i)
                {
                    var node = m_Nodes + level0[i].Root;
                    if (node->IsInternal && level0[i].Length > smallRangeThreshold && freeNodeIndex < maxNumBranchesMinusOneSplit)
                    {
                        var validChildren = 0;
                        for (int j = 0; j < 4; ++j)
                        {
                            if (node->IsChildValid(j))
                            {
                                ++validChildren;
                                var childIndex = node->Data[j];
                                var child = m_Nodes + childIndex;
                                var numElements = (int)child->NumElements;

                                var range = new Builder.Range(0, numElements, Aabb.Empty);
                                range.Root = childIndex;

                                largestRangeInLastLevel = math.max(largestRangeInLastLevel, numElements);

                                level1[level1Size++] = range;
                            }
                        }

                        freeNodeIndex += validChildren;
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
                nodeOffset += level0[i].Length;
            }

            SortRangeMap(rangeMapBySize, level0Size);

            for (int i = 0; i < level0Size; i++)
            {
                branchRanges[i] = level0[rangeMapBySize[i].RangeIndex];
            }

            branchCount = level0Size;
        }

        #endregion

        #region Self overlap query

        public interface ITreeOverlapCollector
        {
            void AddPairs(int l, int4 r, int countR);
            void AddPairs(int4 l, int4 r, int count, bool swapped = false);
            void FlushIfNeeded();
        }

        public unsafe void BvhOverlap<T>(ref T pairWriter, BoundingVolumeHierarchy other, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            TreeOverlap(ref pairWriter, m_Nodes, other.m_Nodes, m_NodeFilters, other.m_NodeFilters, rootA, rootB);
        }

        public unsafe void SelfBvhOverlap<T>(ref T pairWriter, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            TreeOverlap(ref pairWriter, m_Nodes, m_Nodes, m_NodeFilters, m_NodeFilters, rootA, rootB);
        }

        public static unsafe void TreeOverlap<T>(
            ref T pairWriter,
            Node* treeA, Node* treeB,
            CollisionFilter* collisionFilterA = null, CollisionFilter* collisionFilterB = null,
            int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            int* binaryStackA = stackalloc int[Constants.BinaryStackSize];
            int* binaryStackB = stackalloc int[Constants.BinaryStackSize];
            int* stackA = binaryStackA;
            int* stackB = binaryStackB;

            int4* compressedDataBuffer = stackalloc int4[4];

            if (treeA == treeB && rootA == rootB)
            {
                int* unaryStack = stackalloc int[Constants.UnaryStackSize];
                int* stack = unaryStack;
                *stack++ = rootA;

                do
                {
                    int nodeIndex = *(--stack);
                    if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndex], collisionFilterB[nodeIndex]))
                    {
                        ProcessAA(ref treeA[nodeIndex], compressedDataBuffer, ref stack, ref stackA, ref stackB, ref pairWriter);
                    }
                    pairWriter.FlushIfNeeded();

                    while (stackA > binaryStackA)
                    {
                        int nodeIndexA = *(--stackA);
                        int nodeIndexB = *(--stackB);

                        if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
                        {
                            ProcessAB(&treeA[nodeIndexA], &treeA[nodeIndexB], treeA, treeA, compressedDataBuffer, ref stackA, ref stackB, ref pairWriter);
                        }
                        pairWriter.FlushIfNeeded();
                    }
                }
                while (stack > unaryStack);
            }
            else
            {
                *stackA++ = rootA;
                *stackB++ = rootB;

                do
                {
                    int nodeIndexA = *(--stackA);
                    int nodeIndexB = *(--stackB);
                    if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
                    {
                        ProcessAB(&treeA[nodeIndexA], &treeB[nodeIndexB], treeA, treeB, compressedDataBuffer, ref stackA, ref stackB, ref pairWriter);
                    }

                    pairWriter.FlushIfNeeded();
                }
                while (stackA > binaryStackA);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void ProcessAA<T>(ref Node node, int4* compressedData, ref int* stack, ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        {
            int4 nodeData = node.Data;
            FourTransposedAabbs nodeBounds = node.Bounds;

            FourTransposedAabbs* aabbT = stackalloc FourTransposedAabbs[3];
            aabbT[0] = nodeBounds.GetAabbT(0);
            aabbT[1] = nodeBounds.GetAabbT(1);
            aabbT[2] = nodeBounds.GetAabbT(2);

            bool4* masks = stackalloc bool4[3];
            masks[0] = new bool4(false, true, true, true);
            masks[1] = new bool4(false, false, true, true);
            masks[2] = new bool4(false, false, false, true);

            int3 compressedCounts = int3.zero;
            compressedCounts[0] = math.compress((int*)(compressedData + 0), 0, nodeData, aabbT[0].Overlap1Vs4(ref nodeBounds) & masks[0]);
            compressedCounts[1] = math.compress((int*)(compressedData + 1), 0, nodeData, aabbT[1].Overlap1Vs4(ref nodeBounds) & masks[1]);
            compressedCounts[2] = math.compress((int*)(compressedData + 2), 0, nodeData, aabbT[2].Overlap1Vs4(ref nodeBounds) & masks[2]);

            if (node.IsLeaf)
            {
                for (int i = 0; i < 3; i++)
                {
                    pairWriter.AddPairs(nodeData[i], compressedData[i], compressedCounts[i]);
                }
            }
            else
            {
                int4 internalNodes;
                int numInternals = math.compress((int*)&internalNodes, 0, nodeData, node.AreInternalsValid);
                *((int4*)stack) = internalNodes;
                stack += numInternals;

                for (int i = 0; i < 3; i++)
                {
                    *((int4*)stackA) = new int4(nodeData[i]);
                    *((int4*)stackB) = compressedData[i];
                    stackA += compressedCounts[i];
                    stackB += compressedCounts[i];
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void ProcessAB<T>(
            Node* nodeA, Node* nodeB,
            Node* treeA, Node* treeB,
            int4* compressedData,
            ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        {
            bool swapped = false;
            if (nodeA->IsInternal && nodeB->IsLeaf)
            {
                Node* tmp = nodeA;
                nodeA = nodeB;
                nodeB = tmp;

                treeB = treeA;

                swapped = true;
            }

            bool4* overlapMask = stackalloc bool4[4];
            FourTransposedAabbs aabbTA = nodeA->Bounds.GetAabbT(0);
            overlapMask[0] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
            aabbTA = nodeA->Bounds.GetAabbT(1);
            overlapMask[1] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
            aabbTA = nodeA->Bounds.GetAabbT(2);
            overlapMask[2] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
            aabbTA = nodeA->Bounds.GetAabbT(3);
            overlapMask[3] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);

            int4 compressedCount = int4.zero;

            compressedCount[0] = math.compress((int*)&compressedData[0], 0, nodeB->Data, overlapMask[0]);
            compressedCount[1] = math.compress((int*)&compressedData[1], 0, nodeB->Data, overlapMask[1]);
            compressedCount[2] = math.compress((int*)&compressedData[2], 0, nodeB->Data, overlapMask[2]);
            compressedCount[3] = math.compress((int*)&compressedData[3], 0, nodeB->Data, overlapMask[3]);

            if (nodeA->IsLeaf && nodeB->IsLeaf)
            {
                for (int i = 0; i < 4; i++)
                {
                    pairWriter.AddPairs(nodeA->Data[i], compressedData[i], compressedCount[i]);
                }
            }
            else if (nodeA->IsInternal && nodeB->IsInternal)
            {
                for (int i = 0; i < 4; i++)
                {
                    *((int4*)stackA) = new int4(nodeA->Data[i]);
                    *((int4*)stackB) = compressedData[i];
                    stackA += compressedCount[i];
                    stackB += compressedCount[i];
                }
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    if (compressedCount[i] > 0)
                    {
                        *((int4*)stackA) = compressedData[i];
                        int* internalStack = stackA + compressedCount[i];
                        FourTransposedAabbs aabbT = nodeA->Bounds.GetAabbT(i);
                        int4 leafA = new int4(nodeA->Data[i]);

                        do
                        {
                            Node* internalNode = treeB + *(--internalStack);
                            int4 internalCompressedData;
                            int internalCount = math.compress((int*)&internalCompressedData, 0, internalNode->Data, aabbT.Overlap1Vs4(ref internalNode->Bounds));
                            if (internalCount > 0)
                            {
                                if (internalNode->IsLeaf)
                                {
                                    pairWriter.AddPairs(leafA, internalCompressedData, internalCount, swapped);
                                    pairWriter.FlushIfNeeded();
                                }
                                else
                                {
                                    *((int4*)internalStack) = internalCompressedData;
                                    internalStack += internalCount;
                                }
                            }
                        }
                        while (internalStack != stackA);
                    }
                }
            }
        }

        #endregion

        #region AABB overlap query

        public interface IAabbOverlapLeafProcessor
        {
            // Called when the query overlaps a leaf node of the bounding volume hierarchy
            void AabbLeaf<T>(OverlapAabbInput input, int leafData, ref T collector) where T : struct, IOverlapCollector;
        }

        public unsafe void AabbOverlap<TProcessor, TCollector>(OverlapAabbInput input, ref TProcessor processor, ref TCollector collector, int root = 1)
            where TProcessor : struct, IAabbOverlapLeafProcessor
            where TCollector : struct, IOverlapCollector
        {
            int* binaryStack = stackalloc int[Constants.BinaryStackSize];
            int* stack = binaryStack;
            *stack++ = root;

            FourTransposedAabbs aabbT;
            (&aabbT)->SetAllAabbs(input.Aabb);
            do
            {
                int nodeIndex = *(--stack);
                Node* node = m_Nodes + nodeIndex;
                bool4 overlap = aabbT.Overlap1Vs4(ref node->Bounds);
                int4 compressedValues;
                int compressedCount = math.compress((int*)(&compressedValues), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (int i = 0; i < compressedCount; i++)
                    {
                        processor.AabbLeaf(input, compressedValues[i], ref collector);
                    }
                }
                else
                {
                    *((int4*)stack) = compressedValues;
                    stack += compressedCount;
                }
            }
            while (stack > binaryStack);
        }

        #endregion

        #region Ray cast query

        public interface IRaycastLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool RayLeaf<T>(RaycastInput input, int leafData, ref T collector) where T : struct, ICollector<RaycastHit>;
        }

        public unsafe bool Raycast<TProcessor, TCollector>(RaycastInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IRaycastLeafProcessor
            where TCollector : struct, ICollector<RaycastHit>
        {
            bool hadHit = false;
            int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
            *top++ = 1;
            do
            {
                Node* node = m_Nodes + *(--top);
                bool4 hitMask = node->Bounds.Raycast(input.Ray, collector.MaxFraction, out float4 hitFractions);
                int4 hitData;
                int hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

                if (node->IsLeaf)
                {
                    for (int i = 0; i < hitCount; i++)
                    {
                        hadHit |= leafProcessor.RayLeaf(input, hitData[i], ref collector);
                        if (collector.EarlyOutOnFirstHit && hadHit)
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    *((int4*)top) = hitData;
                    top += hitCount;
                }
            }
            while (top > stack);

            return hadHit;
        }

        #endregion

        #region Collider cast query

        public interface IColliderCastLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool ColliderCastLeaf<T>(ColliderCastInput input, int leafData, ref T collector) where T : struct, ICollector<ColliderCastHit>;
        }

        public unsafe bool ColliderCast<TProcessor, TCollector>(ColliderCastInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IColliderCastLeafProcessor
            where TCollector : struct, ICollector<ColliderCastHit>
        {
            float3 aabbExtents;
            Ray aabbRay;
            {
                Aabb aabb = input.Collider->CalculateAabb(new RigidTransform { pos = input.Start, rot = input.Orientation },
                    input.QueryContext.InvTargetScale * input.QueryColliderScale);
                aabbExtents = aabb.Extents;
                aabbRay = input.Ray;
                aabbRay.Origin = aabb.Min;
            }

            bool hadHit = false;

            int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
            *top++ = 1;
            do
            {
                Node* node = m_Nodes + *(--top);
                FourTransposedAabbs bounds = node->Bounds;
                bounds.Lx -= aabbExtents.x;
                bounds.Ly -= aabbExtents.y;
                bounds.Lz -= aabbExtents.z;
                bool4 hitMask = bounds.Raycast(aabbRay, collector.MaxFraction, out float4 hitFractions);
                int4 hitData;
                int hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

                if (node->IsLeaf)
                {
                    for (int i = 0; i < hitCount; i++)
                    {
                        hadHit |= leafProcessor.ColliderCastLeaf(input, hitData[i], ref collector);
                        if (collector.EarlyOutOnFirstHit && hadHit)
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    *((int4*)top) = hitData;
                    top += hitCount;
                }
            }
            while (top > stack);

            return hadHit;
        }

        #endregion

        #region Point distance query

        public interface IPointDistanceLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool DistanceLeaf<T>(PointDistanceInput input, int leafData, ref T collector) where T : struct, ICollector<DistanceHit>;
        }

        public unsafe bool Distance<TProcessor, TCollector>(PointDistanceInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IPointDistanceLeafProcessor
            where TCollector : struct, ICollector<DistanceHit>
        {
            UnityEngine.Assertions.Assert.IsTrue(collector.MaxFraction <= input.MaxDistance);

            bool hadHit = false;

            int* binaryStack = stackalloc int[Constants.BinaryStackSize];
            int* stack = binaryStack;
            *stack++ = 1;

            var pointT = new Math.FourTransposedPoints(input.Position);
            var invScaleSq = input.QueryContext.InvTargetScale;
            invScaleSq *= invScaleSq;
            float4 maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction * invScaleSq);

            do
            {
                int nodeIndex = *(--stack);
                Node* node = m_Nodes + nodeIndex;
                float4 distanceToNodesSquared = node->Bounds.DistanceFromPointSquared(ref pointT);
                bool4 overlap = (node->Bounds.Lx <= node->Bounds.Hx) & (distanceToNodesSquared <= maxDistanceSquared);
                int4 hitData;
                int hitCount = math.compress((int*)(&hitData), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (int i = 0; i < hitCount; i++)
                    {
                        hadHit |= leafProcessor.DistanceLeaf(input, hitData[i], ref collector);

                        if (collector.EarlyOutOnFirstHit && hadHit)
                        {
                            return true;
                        }
                    }

                    maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction * invScaleSq);
                }
                else
                {
                    *((int4*)stack) = hitData;
                    stack += hitCount;
                }
            }
            while (stack > binaryStack);

            return hadHit;
        }

        #endregion

        #region Collider distance query

        public interface IColliderDistanceLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool DistanceLeaf<T>(ColliderDistanceInput input, int leafData, ref T collector) where T : struct, ICollector<DistanceHit>;
        }

        public unsafe bool Distance<TProcessor, TCollector>(ColliderDistanceInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IColliderDistanceLeafProcessor
            where TCollector : struct, ICollector<DistanceHit>
        {
            UnityEngine.Assertions.Assert.IsTrue(collector.MaxFraction <= input.MaxDistance);

            bool hadHit = false;

            int* binaryStack = stackalloc int[Constants.BinaryStackSize];
            int* stack = binaryStack;
            *stack++ = 1;

            var invScaleSq = input.QueryContext.InvTargetScale;
            invScaleSq *= invScaleSq;

            Aabb aabb = input.Collider->CalculateAabb(input.Transform,
                input.QueryContext.InvTargetScale * input.Scale);

            FourTransposedAabbs aabbT;
            (&aabbT)->SetAllAabbs(aabb);
            float4 maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction * invScaleSq);

            do
            {
                int nodeIndex = *(--stack);
                Node* node = m_Nodes + nodeIndex;
                float4 distanceToNodesSquared = node->Bounds.DistanceFromAabbSquared(ref aabbT);
                bool4 overlap = (node->Bounds.Lx <= node->Bounds.Hx) & (distanceToNodesSquared <= maxDistanceSquared);
                int4 hitData;
                int hitCount = math.compress((int*)(&hitData), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (int i = 0; i < hitCount; i++)
                    {
                        hadHit |= leafProcessor.DistanceLeaf(input, hitData[i], ref collector);
                        if (collector.EarlyOutOnFirstHit && hadHit)
                        {
                            return true;
                        }

                        maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction * invScaleSq);
                    }
                }
                else
                {
                    *((int4*)stack) = hitData;
                    stack += hitCount;
                }
            }
            while (stack > binaryStack);

            return hadHit;
        }

        #endregion
    }
}
