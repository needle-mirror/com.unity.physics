using System.Diagnostics;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics
{
    // A 4-way bounding volume hierarchy
    public partial struct BoundingVolumeHierarchy
    {
        private readonly unsafe Node* m_Nodes;
        private readonly unsafe CollisionFilter* m_NodeFilters;

        public unsafe Aabb Domain => m_Nodes[1].Bounds.GetCompoundAabb();

        public unsafe BoundingVolumeHierarchy(Node* nodes, CollisionFilter* nodeFilters)
        {
            m_Nodes = nodes;
            m_NodeFilters = nodeFilters;
        }

        public unsafe BoundingVolumeHierarchy(NativeArray<Node> nodes, NativeArray<CollisionFilter> nodeFilters)
        {
            m_Nodes = (Node*)nodes.GetUnsafeReadOnlyPtr();
            m_NodeFilters = (CollisionFilter*)nodeFilters.GetUnsafeReadOnlyPtr();
        }

        public unsafe BoundingVolumeHierarchy(NativeArray<Node> nodes)
        {
            m_Nodes = (Node*)nodes.GetUnsafeReadOnlyPtr();
            m_NodeFilters = null;
        }

        // A node in the hierarchy
        [DebuggerDisplay("{IsLeaf?\"Leaf\":\"Internal\"}, [ {Data[0]}, {Data[1]}, {Data[2]}, {Data[3]} ]")]
        [StructLayout(LayoutKind.Sequential, Size = 128)]
        public struct Node
        {
            public FourTransposedAabbs Bounds;
            public int4 Data;
            public int Flags;

            public static Node Empty => new Node
            {
                Bounds = FourTransposedAabbs.Empty,
                Data = int4.zero,
                IsLeaf = false
            };

            public bool IsInternal { get => Flags == 0; set => Flags = value ? 0 : 1; }
            public bool IsLeaf { get => Flags != 0; set => Flags = value ? 1 : 0; }

            private static readonly int4 minusOne4i = new int4(-1);

            public bool4 AreLeavesValid => (Data != minusOne4i);
            public bool4 AreInternalsValid => (Data != int4.zero);

            public bool IsChildValid(int index)
            {
                if (IsLeaf && Data[index] == -1) return false;
                if (!IsLeaf && Data[index] == 0) return false;
                return true;
            }

            public int NumValidChildren()
            {
                int cnt = 0;
                for (int i = 0; i < 4; i++)
                {
                    cnt += IsChildValid(i) ? 1 : 0;
                }

                return cnt;
            }

            public bool IsLeafValid(int index) => Data[index] != -1;
            public bool IsInternalValid(int index) => Data[index] != 0;

            public void ClearLeafData(int index) => Data[index] = -1;
            public void ClearInternalData(int index) => Data[index] = 0;
        }

        // Utility function
        private static void Swap<T>(ref T a, ref T b) where T : struct { T t = a; a = b; b = t; }

        #region Self overlap query

        public interface ITreeOverlapCollector
        {
            void AddPairs(int l, int4 r, int countR);
            void AddPairs(int4 l, int4 r, int count);
            void FlushIfNeeded();
        }

        public unsafe void BvhOverlap<T>(ref T pairWriter, BoundingVolumeHierarchy other, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            TreeOverlap(ref pairWriter, m_Nodes, other.m_Nodes, m_NodeFilters, other.m_NodeFilters, rootA, rootB);
        }

        public unsafe void SelfBvhOverlap<T>(ref T pairWriter, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            TreeOverlap(ref pairWriter, m_Nodes, m_Nodes, m_NodeFilters,  m_NodeFilters, rootA, rootB);
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
                        ProcessAA(ref treeA[nodeIndex], ref stack, ref stackA, ref stackB, ref pairWriter);
                    }
                    pairWriter.FlushIfNeeded();

                    while (stackA > binaryStackA)
                    {
                        int nodeIndexA = *(--stackA);
                        int nodeIndexB = *(--stackB);

                        if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
                        {
                            ProcessAB(ref treeA[nodeIndexA], ref treeA[nodeIndexB], treeA, treeA, ref stackA, ref stackB, ref pairWriter);
                        }
                        pairWriter.FlushIfNeeded();
                    }
                } while (stack > unaryStack);
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
                        ProcessAB(ref treeA[nodeIndexA], ref treeB[nodeIndexB], treeA, treeB, ref stackA, ref stackB, ref pairWriter);
                    }

                    pairWriter.FlushIfNeeded();
                } while (stackA > binaryStackA);
            }
        }

        private static unsafe void ProcessAA<T>(ref Node node, ref int* stack, ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        {
            int4 nodeData = node.Data;
            FourTransposedAabbs nodeBounds = node.Bounds;

            FourTransposedAabbs* aabbT = stackalloc FourTransposedAabbs[3];
            aabbT[0] = nodeBounds.GetAabbT(0);
            aabbT[1] = nodeBounds.GetAabbT(1);
            aabbT[2] = nodeBounds.GetAabbT(2);

            bool4x3 masks = new bool4x3(
                new bool4(false, true, true, true),
                new bool4(false, false, true, true),
                new bool4(false, false, false, true));

            int4* compressedValues = stackalloc int4[3];
            int* compressedCounts = stackalloc int[3];

            for (int i = 0; i < 3; i++)
            {
                bool4 overlap = aabbT[i].Overlap1Vs4(ref nodeBounds) & masks[i];
                compressedCounts[i] = math.compress((int*)(compressedValues + i), 0, nodeData, overlap);
            }

            if (node.IsLeaf)
            {
                for (int i = 0; i < 3; i++)
                {
                    pairWriter.AddPairs(nodeData[i], compressedValues[i], compressedCounts[i]);
                }
            }
            else
            {
                int4* internalNodes = stackalloc int4[1];
                int numInternals = math.compress((int*)internalNodes, 0, nodeData, node.AreInternalsValid);
                *((int4*)stack) = *internalNodes;
                stack += numInternals;

                for (int i = 0; i < 3; i++)
                {
                    *((int4*)stackA) = new int4(nodeData[i]);
                    *((int4*)stackB) = compressedValues[i];
                    stackA += compressedCounts[i];
                    stackB += compressedCounts[i];
                }
            }
        }

        private static unsafe void ProcessAB<T>(
            ref Node nodeA, ref Node nodeB,
            Node* treeA, Node* treeB,
            ref int* stackA, ref int* stackB,
            ref T pairWriter) where T : struct, ITreeOverlapCollector
        {
            int4 nodeAdata = nodeA.Data;
            int4 nodeBdata = nodeB.Data;

            FourTransposedAabbs nodeABounds = nodeA.Bounds;
            FourTransposedAabbs nodeBBounds = nodeB.Bounds;

            bool4x4 overlapMask = new bool4x4(
                nodeBBounds.Overlap1Vs4(ref nodeABounds, 0),
                nodeBBounds.Overlap1Vs4(ref nodeABounds, 1),
                nodeBBounds.Overlap1Vs4(ref nodeABounds, 2),
                nodeBBounds.Overlap1Vs4(ref nodeABounds, 3));

            int4 compressedData;

            if (nodeA.IsLeaf && nodeB.IsLeaf)
            {
                for (int i = 0; i < 4; i++)
                {
                    int count = math.compress((int*)&compressedData, 0, nodeBdata, overlapMask[i]);
                    pairWriter.AddPairs(nodeAdata[i], compressedData, count);
                }
            }
            else if (nodeA.IsInternal && nodeB.IsInternal)
            {
                for (int i = 0; i < 4; i++)
                {
                    int count = math.compress(stackB, 0, nodeBdata, overlapMask[i]);
                    *((int4*)stackA) = new int4(nodeAdata[i]);
                    stackA += count;
                    stackB += count;
                }
            }
            else
            {
                int* stack = stackB;
                if (nodeA.IsInternal)
                {
                    overlapMask = math.transpose(overlapMask);
                    Swap(ref nodeAdata, ref nodeBdata);
                    Swap(ref nodeABounds, ref nodeBBounds);
                    stack = stackA;
                    treeB = treeA;
                }

                for (int i = 0; i < 4; i++)
                {
                    if (math.any(overlapMask[i]))
                    {
                        int internalsCount = math.compress(stack, 0, nodeBdata, overlapMask[i]);
                        int* internalStack = stack + internalsCount;

                        FourTransposedAabbs aabbT = nodeABounds.GetAabbT(i);
                        int4 leafA = new int4(nodeAdata[i]);

                        do
                        {
                            Node* internalNode = treeB + *(--internalStack);
                            int compressedCount = math.compress((int*)&compressedData, 0, internalNode->Data, aabbT.Overlap1Vs4(ref internalNode->Bounds));

                            if (internalNode->IsLeaf)
                            {
                                pairWriter.AddPairs(leafA, compressedData, compressedCount);
                                pairWriter.FlushIfNeeded();
                            }
                            else
                            {
                                *((int4*)internalStack) = compressedData;
                                internalStack += compressedCount;
                            }
                        } while (internalStack != stack);
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
            } while (stack > binaryStack);
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
            } while (top > stack);

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
                Aabb aabb = input.Collider->CalculateAabb(new RigidTransform(input.Orientation, input.Position));
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
            } while (top > stack);

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
            float4 maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);

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

                    maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);
                }
                else
                {
                    *((int4*)stack) = hitData;
                    stack += hitCount;
                }
            } while (stack > binaryStack);

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

            Aabb aabb = input.Collider->CalculateAabb(input.Transform);
            FourTransposedAabbs aabbT;
            (&aabbT)->SetAllAabbs(aabb);
            float4 maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);

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

                        maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);
                    }
                }
                else
                {
                    *((int4*)stack) = hitData;
                    stack += hitCount;
                }
            } while (stack > binaryStack);

            return hadHit;
        }

        #endregion
    }
}
