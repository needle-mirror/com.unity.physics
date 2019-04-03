using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;

namespace Unity.Physics
{
    // Utilities for building physics meshes
    internal static class MeshBuilder
    {
        internal class TempSection
        {
            public List<Mesh.PrimitiveFlags> PrimitivesFlags;
            public List<Mesh.PrimitiveVertexIndices> Primitives;
            public List<float3> Vertices;
        }

        internal static unsafe List<TempSection> BuildSections(BoundingVolumeHierarchy.Node* nodes, int nodeCount, List<MeshConnectivityBuilder.Primitive> primitives)
        {
            var tempSections = new List<TempSection>();

            // Traverse the tree and break out geometry into sections
            int* nodesIndexStack = stackalloc int[BoundingVolumeHierarchy.Constants.UnaryStackSize];
            int stackSize = 1;
            nodesIndexStack[0] = 1;

            const float uniqueVerticesPerPrimitiveFactor = 1.5f;

            int[] primitivesCountInSubTree = ProducedPrimitivesCountPerSubTree(nodes, nodeCount);

            do
            {
                int nodeIndex = nodesIndexStack[--stackSize];
                int subTreeVertexCountEstimate = (int)(uniqueVerticesPerPrimitiveFactor * primitivesCountInSubTree[nodeIndex]);

                var subTreeIndices = new List<int>();

                if (subTreeVertexCountEstimate < Mesh.Section.MaxNumVertices)
                {
                    subTreeIndices.Add(nodeIndex);
                }
                else
                {
                    // Sub tree is too big, break it up.
                    BoundingVolumeHierarchy.Node node = nodes[nodeIndex];

                    for (int i = 0; i < 4; i++)
                    {
                        if (node.IsChildValid(i))
                        {
                            int childNodeIndex = node.Data[i];
                            int nodeSubTreeVertexCount = (int)(uniqueVerticesPerPrimitiveFactor * primitivesCountInSubTree[childNodeIndex]);

                            if (nodeSubTreeVertexCount < Mesh.Section.MaxNumVertices)
                            {
                                subTreeIndices.Add(childNodeIndex);
                            }
                            else
                            {
                                nodesIndexStack[stackSize++] = childNodeIndex;
                            }
                        }
                    }
                }

                float tempUniqueVertexPrimitiveFactor = 1.0f;
                const float factorStepIncrement = 0.25f;

                while (subTreeIndices.Any())
                {
                    // Try to combine sub trees if multiple sub trees can fit into one section.
                    var nodeIndices = new List<int>();
                    int vertexCountEstimate = 0;

                    foreach (int subTreeNodeIndex in subTreeIndices)
                    {
                        int nodeIndexCount = (int)(tempUniqueVertexPrimitiveFactor * primitivesCountInSubTree[subTreeNodeIndex]);
                        if (vertexCountEstimate + nodeIndexCount < Mesh.Section.MaxNumVertices)
                        {
                            vertexCountEstimate += nodeIndexCount;
                            nodeIndices.Add(subTreeNodeIndex);
                        }
                    }

                    if (!nodeIndices.Any())
                    {
                        // We failed to fit any sub tree into sections.
                        // Split up nodes and push them to stack.
                        foreach (int subTreeNodeIndex in subTreeIndices)
                        {
                            BoundingVolumeHierarchy.Node nodeToSplit = nodes[subTreeNodeIndex];

                            for (int i = 0; i < 4; i++)
                            {
                                if (nodeToSplit.IsChildValid(i))
                                {
                                    nodesIndexStack[stackSize++] = nodeToSplit.Data[i];
                                }
                            }
                        }

                        subTreeIndices.Clear();
                        continue;
                    }

                    // Collect vertices from all sub trees.
                    var tmpVertices = new List<float3>();
                    foreach (int subTreeNodeIndex in nodeIndices)
                    {
                        tmpVertices.AddRange(CollectAllVerticesFromSubTree(nodes, subTreeNodeIndex, primitives));
                    }

                    float3[] allVertices = tmpVertices.ToArray();

                    int[] vertexIndices = new int[allVertices.Length];
                    for (int i = 0; i < vertexIndices.Length; i++)
                    {
                        vertexIndices[i] = i;
                    }

                    MeshConnectivityBuilder.WeldVertices(vertexIndices, ref allVertices);

                    if (allVertices.Length < Mesh.Section.MaxNumVertices)
                    {
                        tempSections.Add(BuildSectionGeometry(tempSections.Count, primitives, nodeIndices, nodes, allVertices));

                        // Remove used indices
                        foreach (int nodeTreeIndex in nodeIndices)
                        {
                            subTreeIndices.Remove(nodeTreeIndex);
                        }
                    }
                    else
                    {
                        // Estimate of num vertices per primitives was wrong.
                        // Increase the tempUniqueVertexPrimitiveFactor.
                        tempUniqueVertexPrimitiveFactor += factorStepIncrement;
                    }
                }
            }
            while (stackSize > 0);

            return tempSections;
        }

        private static unsafe int[] ProducedPrimitivesCountPerSubTree(BoundingVolumeHierarchy.Node* nodes, int nodeCount)
        {
            int[] primitivesPerNode = Enumerable.Repeat(0, nodeCount).ToArray();

            for (int nodeIndex = nodeCount - 1; nodeIndex >= 0; nodeIndex--)
            {
                BoundingVolumeHierarchy.Node node = nodes[nodeIndex];

                if (node.IsLeaf)
                {
                    primitivesPerNode[nodeIndex] = node.NumValidChildren();
                }
                else
                {
                    primitivesPerNode[nodeIndex] =
                        primitivesPerNode[node.Data[0]] + primitivesPerNode[node.Data[1]] +
                        primitivesPerNode[node.Data[2]] + primitivesPerNode[node.Data[3]];
                }
            }

            return primitivesPerNode;
        }

        private static unsafe List<float3> CollectAllVerticesFromSubTree(BoundingVolumeHierarchy.Node* nodes, int subTreeNodeIndex, List<MeshConnectivityBuilder.Primitive> primitives)
        {
            var vertices = new List<float3>();

            int* nodesIndexStack = stackalloc int[BoundingVolumeHierarchy.Constants.UnaryStackSize];
            int stackSize = 1;
            nodesIndexStack[0] = subTreeNodeIndex;

            do
            {
                BoundingVolumeHierarchy.Node node = nodes[nodesIndexStack[--stackSize]];

                if (node.IsLeaf)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (node.IsChildValid(i))
                        {
                            MeshConnectivityBuilder.Primitive p = primitives[node.Data[i]];
                            vertices.Add(p.Vertices[0]);
                            vertices.Add(p.Vertices[1]);
                            vertices.Add(p.Vertices[2]);

                            if (p.Flags.HasFlag(MeshConnectivityBuilder.PrimitiveFlags.DefaultTrianglePairFlags))
                            {
                                vertices.Add(p.Vertices[3]);
                            }
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (node.IsChildValid(i))
                        {
                            nodesIndexStack[stackSize++] = node.Data[i];
                        }
                    }
                }
            } while (stackSize > 0);

            return vertices;
        }

        private static Mesh.PrimitiveFlags ConvertPrimitiveFlags(MeshConnectivityBuilder.PrimitiveFlags flags)
        {
            Mesh.PrimitiveFlags newFlags = 0;
            newFlags |= flags.HasFlag(MeshConnectivityBuilder.PrimitiveFlags.IsTrianglePair) ? Mesh.PrimitiveFlags.IsTrianglePair : Mesh.PrimitiveFlags.IsTriangle;

            if (flags.HasFlag(MeshConnectivityBuilder.PrimitiveFlags.IsFlatConvexQuad))
            {
                newFlags |= Mesh.PrimitiveFlags.IsQuad;
            }

            return newFlags;
        }

        private static unsafe TempSection BuildSectionGeometry(
            int sectionIndex, List<MeshConnectivityBuilder.Primitive> primitives, List<int> subTreeNodeIndices, BoundingVolumeHierarchy.Node* nodes, float3[] vertices)
        {
            var section = new TempSection();
            section.Vertices = vertices.ToList();
            section.Primitives = new List<Mesh.PrimitiveVertexIndices>();
            section.PrimitivesFlags = new List<Mesh.PrimitiveFlags>();

            foreach (int root in subTreeNodeIndices)
            {
                int* nodesIndexStack = stackalloc int[BoundingVolumeHierarchy.Constants.UnaryStackSize];
                int stackSize = 1;
                nodesIndexStack[0] = root;

                do
                {
                    int nodeIndex = nodesIndexStack[--stackSize];
                    ref BoundingVolumeHierarchy.Node node = ref nodes[nodeIndex];

                    if (node.IsLeaf)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            if (node.IsChildValid(i))
                            {
                                MeshConnectivityBuilder.Primitive p = primitives[node.Data[i]];
                                section.PrimitivesFlags.Add(ConvertPrimitiveFlags(p.Flags));

                                int vertexCount = p.Flags.HasFlag(MeshConnectivityBuilder.PrimitiveFlags.IsTrianglePair) ? 4 : 3;

                                Mesh.PrimitiveVertexIndices sectionPrimitive = new Mesh.PrimitiveVertexIndices();
                                byte* vertexIndices = &sectionPrimitive.A;

                                for (int v = 0; v < vertexCount; v++)
                                {
                                    vertexIndices[v] = (byte)Array.IndexOf(vertices, p.Vertices[v]);
                                }

                                if (vertexCount == 3)
                                {
                                    sectionPrimitive.D = sectionPrimitive.C;
                                }

                                section.Primitives.Add(sectionPrimitive);

                                int primitiveSectionIndex = section.Primitives.Count - 1;

                                // Update primitive index in the BVH.
                                node.Data[i] = (sectionIndex << 8) | primitiveSectionIndex;
                            }
                        }
                    }
                    else
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            if (node.IsChildValid(i))
                            {
                                nodesIndexStack[stackSize++] = node.Data[i];
                            }
                        }
                    }
                } while (stackSize > 0);
            }

            return section;
        }
    }

    internal class MeshConnectivityBuilder
    {
        const float k_MergeCoplanarTrianglesTolerance = 1e-4f;

        internal Vertex[] Vertices;
        internal Triangle[] Triangles;
        internal Edge[] Edges;

        /// Vertex.
        internal struct Vertex
        {
            /// Number of triangles referencing this vertex, or, equivalently, number of edge starting from this vertex.
            internal int Cardinality;

            /// true if the vertex is on the boundary, false otherwise.
            /// Note: if true the first edge of the ring is naked.
            /// Conditions: number of naked edges in the 1-ring is greater than 0.
            internal bool Boundary;

            /// true if the vertex is on the border, false otherwise.
            /// Note: if true the first edge of the ring is naked.
            /// Conditions: number of naked edges in the 1-ring is equal to 1.
            internal bool Border;

            /// true is the vertex 1-ring is manifold.
            /// Conditions: number of naked edges in the 1-ring is less than 2 and cardinality is greater than 0.
            internal bool Manifold;

            /// Index of the first edge.
            internal int FirstEdge;
        }

        /// (Half) Edge.
        internal struct Edge
        {
            internal static Edge Invalid() => new Edge { IsValid = false };

            // Triangle index
            internal int Triangle;

            // Starting vertex index
            internal int Start;

            internal bool IsValid;
        }

        internal class Triangle
        {
            public void Clear()
            {
                IsValid = false;
                Links[0].IsValid = false;
                Links[1].IsValid = false;
                Links[2].IsValid = false;
            }

            internal Edge[] Links = new Edge[3];
            internal bool IsValid;
        }

        [Flags]
        internal enum PrimitiveFlags
        {
            IsTrianglePair = 1 << 0,
            IsFlat = 1 << 1,
            IsConvex = 1 << 2,

            DisableInternalEdge = 1 << 3,
            DisableAllEdges = 1 << 4,

            IsFlatConvexQuad = IsTrianglePair | IsFlat | IsConvex,

            DefaultTriangleFlags = IsFlat | IsConvex,
            DefaultTrianglePairFlags = IsTrianglePair
        }

        internal struct Primitive
        {
            internal float3x4 Vertices;
            internal PrimitiveFlags Flags;
        }

        /// Get the opposite edge.
        internal Edge GetLink(Edge e) => e.IsValid ? Triangles[e.Triangle].Links[e.Start] : e;
        internal bool IsBound(Edge e) => GetLink(e).IsValid;
        internal bool IsNaked(Edge e) => !IsBound(e);
        internal Edge GetNext(Edge e) => e.IsValid ? new Edge { Triangle = e.Triangle, Start = (e.Start + 1) % 3, IsValid = true } : e;
        internal Edge GetPrev(Edge e) => e.IsValid ? new Edge { Triangle = e.Triangle, Start = (e.Start + 2) % 3, IsValid = true } : e;

        internal int GetStartVertexIndex(Edge e) => Triangles[e.Triangle].Links[e.Start].Start;

        internal int GetEndVertexIndex(Edge e) => Triangles[e.Triangle].Links[(e.Start + 1) % 3].Start;

        internal bool IsEdgeConcaveOrFlat(Edge edge, int[] indices, float3[] vertices, float4[] planes)
        {
            if (IsNaked(edge))
            {
                return false;
            }

            float3 apex = vertices[GetApexVertexIndex(indices, edge)];
            if (Math.Dotxyz1(planes[edge.Triangle], apex) < -k_MergeCoplanarTrianglesTolerance)
            {
                return false;
            }

            return true;
        }

        internal bool IsTriangleConcaveOrFlat(Edge edge, int[] indices, float3[] vertices, float4[] planes)
        {
            for (int i = 0; i < 3; i++)
            {
                Edge e = GetNext(edge);
                if (!IsEdgeConcaveOrFlat(e, indices, vertices, planes))
                {
                    return false;
                }
            }

            return true;
        }

        internal bool IsFlat(Edge edge, int[] indices, float3[] vertices, float4[] planes)
        {
            Edge link = GetLink(edge);
            if (!link.IsValid)
            {
                return false;
            }

            float3 apex = vertices[GetApexVertexIndex(indices, link)];
            bool flat = math.abs(Math.Dotxyz1(planes[edge.Triangle], apex)) < k_MergeCoplanarTrianglesTolerance;

            apex = vertices[GetApexVertexIndex(indices, edge)];
            flat |= math.abs(Math.Dotxyz1(planes[link.Triangle], apex)) < k_MergeCoplanarTrianglesTolerance;

            return flat;
        }

        internal bool IsConvexQuad(Primitive quad, Edge edge, float4[] planes)
        {
            float4x2 quadPlanes;
            quadPlanes.c0 = planes[edge.Triangle];
            quadPlanes.c1 = planes[GetLink(edge).Triangle];
            if (Math.Dotxyz1(quadPlanes[0], quad.Vertices[3]) < k_MergeCoplanarTrianglesTolerance)
            {
                if (Math.Dotxyz1(quadPlanes[1], quad.Vertices[1]) < k_MergeCoplanarTrianglesTolerance)
                {
                    bool convex = true;
                    for (int i = 0; convex && i < 4; i++)
                    {
                        float3 delta = quad.Vertices[(i + 1) % 4] - quad.Vertices[i];
                        float3 normal = math.normalize(math.cross(delta, quadPlanes[i >> 1].xyz));
                        float4 edgePlane = new float4(normal, math.dot(-normal, quad.Vertices[i]));
                        for (int j = 0; j < 2; j++)
                        {
                            if (Math.Dotxyz1(edgePlane, quad.Vertices[(i + j + 1) % 4]) > k_MergeCoplanarTrianglesTolerance)
                            {
                                convex = false;
                                break;
                            }
                        }
                    }
                    return convex;
                }
            }

            return false;
        }

        internal bool CanEdgeBeDisabled(Edge e, PrimitiveFlags[] flags, int[] indices, float3[] vertices, float4[] planes)
        {
            if (!e.IsValid || IsEdgeConcaveOrFlat(e, indices, vertices, planes) || flags[e.Triangle].HasFlag(PrimitiveFlags.DisableAllEdges))
            {
                return false;
            }

            return true;
        }

        internal bool CanAllEdgesBeDisabled(Edge[] edges, PrimitiveFlags[] flags, int[] indices, float3[] vertices, float4[] planes)
        {
            bool allDisabled = true;
            foreach (Edge e in edges)
            {
                allDisabled &= CanEdgeBeDisabled(e, flags, indices, vertices, planes);
            }

            return allDisabled;
        }

        // Utility function
        private static void Swap<T>(ref T a, ref T b) where T : struct
        {
            T t = a;
            a = b;
            b = t;
        }

        private struct VertexWithHash
        {
            internal float3 Vertex;
            internal ulong Hash;
            internal int Index;
        }

        private static ulong SpatialHash(float3 vertex)
        {
            ulong x, y, z;
            unsafe
            {
                float* tmp = &vertex.x;
                x = *((ulong*)tmp);

                tmp = &vertex.y;
                y = *((ulong*)tmp);

                tmp = &vertex.z;
                z = *((ulong*)tmp);
            }

            const ulong p1 = 73856093;
            const ulong p2 = 19349663;
            const ulong p3 = 83492791;

            return (x * p1) ^ (y * p2) ^ (z * p3);
        }

        public static void WeldVertices(int[] indices, ref float3[] vertices)
        {
            int numVertices = vertices.Length;
            var verticesAndHashes = new VertexWithHash[numVertices];
            for (int i = 0; i < numVertices; i++)
            {
                verticesAndHashes[i].Index = i;
                verticesAndHashes[i].Vertex = vertices[i];
                verticesAndHashes[i].Hash = SpatialHash(verticesAndHashes[i].Vertex);
            }

            var uniqueVertices = new List<float3>();
            var remap = new int[numVertices];
            verticesAndHashes = verticesAndHashes.OrderBy(v => v.Hash).ToArray();

            for (int i = 0; i < numVertices; i++)
            {
                if (verticesAndHashes[i].Index == int.MaxValue)
                {
                    continue;
                }

                uniqueVertices.Add(vertices[verticesAndHashes[i].Index]);
                remap[verticesAndHashes[i].Index] = uniqueVertices.Count - 1;

                for (int j = i + 1; j < numVertices; j++)
                {
                    if (verticesAndHashes[j].Index == int.MaxValue)
                    {
                        continue;
                    }

                    if (verticesAndHashes[i].Hash == verticesAndHashes[j].Hash)
                    {
                        if (verticesAndHashes[i].Vertex.x == verticesAndHashes[j].Vertex.x &&
                            verticesAndHashes[i].Vertex.y == verticesAndHashes[j].Vertex.y &&
                            verticesAndHashes[i].Vertex.z == verticesAndHashes[j].Vertex.z)
                        {
                            remap[verticesAndHashes[j].Index] = remap[verticesAndHashes[i].Index];
                            verticesAndHashes[j].Index = int.MaxValue;
                        }
                    }
                }
            }

            vertices = uniqueVertices.ToArray();

            for (int i = 0; i < indices.Length; i++)
            {
                indices[i] = remap[indices[i]];
            }
        }

        public static bool IsTriangleDegenerate(float3 a, float3 b, float3 c)
        {
            const float defaultTriangleDegeneracyTolerance = 1e-7f;

            // Small area check
            {
                float3 edge1 = a - b;
                float3 edge2 = a - c;
                float3 cross = math.cross(edge1, edge2);

                float3 edge1B = b - a;
                float3 edge2B = b - c;
                float3 crossB = math.cross(edge1B, edge2B);

                bool cmp0 = defaultTriangleDegeneracyTolerance > math.lengthsq(cross);
                bool cmp1 = defaultTriangleDegeneracyTolerance > math.lengthsq(crossB);
                if (cmp0 || cmp1)
                {
                    return true;
                }
            }

            // Point triangle distance check
            {
                float3 q = a - b;
                float3 r = c - b;

                float qq = math.dot(q, q);
                float rr = math.dot(r, r);
                float qr = math.dot(q, r);

                float qqrr = qq * rr;
                float qrqr = qr * qr;
                float det = (qqrr - qrqr);

                return det == 0.0f;
            }
        }

        internal MeshConnectivityBuilder(int[] indices, float3[] vertices)
        {
            int numIndices = indices.Length;
            int numTriangles = numIndices / 3;
            int numVertices = vertices.Length;

            Vertices = new Vertex[numVertices];
            Triangles = new Triangle[numTriangles];
            for (int i = 0; i < numTriangles; i++)
            {
                Triangles[i] = new Triangle();
                Triangles[i].Clear();
            }

            int numEdges = 0;

            // Compute cardinality and triangle flags.
            for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
            {
                var triangleIndices = new ArraySegment<int>(indices, triangleIndex * 3, 3);
                Triangles[triangleIndex].IsValid =
                    triangleIndices.Array[triangleIndices.Offset + 0] != triangleIndices.Array[triangleIndices.Offset + 1] &&
                    triangleIndices.Array[triangleIndices.Offset + 1] != triangleIndices.Array[triangleIndices.Offset + 2] &&
                    triangleIndices.Array[triangleIndices.Offset + 0] != triangleIndices.Array[triangleIndices.Offset + 2];

                if (Triangles[triangleIndex].IsValid)
                {
                    Vertices[triangleIndices.Array[triangleIndices.Offset + 0]].Cardinality++;
                    Vertices[triangleIndices.Array[triangleIndices.Offset + 1]].Cardinality++;
                    Vertices[triangleIndices.Array[triangleIndices.Offset + 2]].Cardinality++;
                }
            }

            // Compute vertex first edge index.
            for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
            {
                int cardinality = Vertices[vertexIndex].Cardinality;
                Vertices[vertexIndex].FirstEdge = cardinality > 0 ? numEdges : 0;
                numEdges += cardinality;
            }

            // Compute edges and triangles links.
            int[] counters = Enumerable.Repeat(0, numVertices).ToArray();
            Edges = new Edge[numEdges];

            for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
            {
                if (!Triangles[triangleIndex].IsValid)
                {
                    continue;
                }

                int indexInIndices = triangleIndex * 3;
                for (int i = 2, j = 0; j < 3; i = j++)
                {
                    int vertexI = indices[indexInIndices + i];
                    int thisEdgeIndex = Vertices[vertexI].FirstEdge + counters[vertexI]++;
                    Edges[thisEdgeIndex] = new Edge { Triangle = triangleIndex, Start = i, IsValid = true };

                    int vertexJ = indices[indexInIndices + j];
                    Vertex other = Vertices[vertexJ];
                    int count = counters[vertexJ];

                    for (int k = 0; k < count; k++)
                    {
                        Edge edge = Edges[other.FirstEdge + k];

                        int endVertexOffset = (edge.Start + 1) % 3;
                        int endVertex = indices[edge.Triangle * 3 + endVertexOffset];
                        if (endVertex == vertexI)
                        {
                            Triangles[triangleIndex].Links[i] = edge;
                            Triangles[edge.Triangle].Links[edge.Start] = Edges[thisEdgeIndex];
                            break;
                        }
                    }
                }
            }

            // Compute vertices attributes.
            for (int vertexIndex = 0; vertexIndex < numVertices; vertexIndex++)
            {
                int nakedEdgeIndex = -1;
                int numNakedEdge = 0;
                {
                    int firstEdgeIndex = Vertices[vertexIndex].FirstEdge;
                    int numVertexEdges = Vertices[vertexIndex].Cardinality;
                    for (int i = 0; i < numVertexEdges; i++)
                    {
                        int edgeIndex = firstEdgeIndex + i;
                        if (IsNaked(Edges[edgeIndex]))
                        {
                            nakedEdgeIndex = i;
                            numNakedEdge++;
                        }
                    }
                }

                ref Vertex vertex = ref Vertices[vertexIndex];
                vertex.Manifold = numNakedEdge < 2 && vertex.Cardinality > 0;
                vertex.Boundary = numNakedEdge > 0;
                vertex.Border = numNakedEdge == 1 && vertex.Manifold;

                // Make sure that naked edge appears first.
                if (nakedEdgeIndex > 0)
                {
                    Swap(ref Edges[vertex.FirstEdge], ref Edges[vertex.FirstEdge + nakedEdgeIndex]);
                }

                // Order ring as fan.
                if (vertex.Manifold)
                {
                    int firstEdge = vertex.FirstEdge;
                    int count = vertex.Cardinality;
                    for (int i = 0; i < count - 1; i++)
                    {
                        Edge prevEdge = GetPrev(Edges[firstEdge + i]);
                        if (IsBound(prevEdge))
                        {
                            int triangle = GetLink(prevEdge).Triangle;
                            if (Edges[firstEdge + i + 1].Triangle != triangle)
                            {
                                bool found = false;
                                for (int j = i + 2; j < count; ++j)
                                {
                                    if (Edges[firstEdge + j].Triangle == triangle)
                                    {
                                        Swap(ref Edges[firstEdge + i + 1], ref Edges[firstEdge + j]);
                                        found = true;
                                        break;
                                    }
                                }

                                if (!found)
                                {
                                    vertex.Manifold = false;
                                    vertex.Border = false;
                                    break;
                                }
                            }
                        }
                    }

                    if (vertex.Manifold)
                    {
                        Edge lastEdge = GetPrev(Edges[firstEdge + count - 1]);
                        if (vertex.Border)
                        {
                            if (IsBound(lastEdge))
                            {
                                vertex.Manifold = false;
                                vertex.Border = false;
                            }
                        }
                        else
                        {
                            if (IsNaked(lastEdge) || GetLink(lastEdge).Triangle != Edges[firstEdge].Triangle)
                            {
                                vertex.Manifold = false;
                            }
                        }
                    }
                }
            }
        }

        private struct EdgeData
        {
            internal Edge Edge;
            internal float Value;
        }

        private static int4 GetVertexIndices(int[] indices, Edge edge)
        {
            int4 vertexIndices;
            int triangleIndicesIndex = edge.Triangle * 3;
            vertexIndices.x = indices[triangleIndicesIndex + edge.Start];
            vertexIndices.y = indices[triangleIndicesIndex + ((edge.Start + 1) % 3)];
            vertexIndices.z = indices[triangleIndicesIndex + ((edge.Start + 2) % 3)];
            vertexIndices.w = 0;
            return vertexIndices;
        }

        private static int GetApexVertexIndex(int[] indices, Edge edge)
        {
            int triangleIndicesIndex = edge.Triangle * 3;
            return indices[triangleIndicesIndex + ((edge.Start + 2) % 3)];
        }

        private static float CalcTwiceSurfaceArea(float3 a, float3 b, float3 c)
        {
            float3 d0 = b - a;
            float3 d1 = c - a;
            return math.length(math.cross(d0, d1));
        }

        internal List<Primitive> EnumerateQuadDominantGeometry(int[] indices, float3[] vertices)
        {
            int numTriangles = indices.Length / 3;
            PrimitiveFlags[] flags = new PrimitiveFlags[numTriangles];
            var quadRoots = new List<Edge>();
            var triangleRoots = new List<Edge>();

            // Generate triangle planes
            var planes = new float4[indices.Length / 3];
            for (int i = 0; i < numTriangles; i++)
            {
                int triangleIndex = i * 3;
                float3 v0 = vertices[indices[triangleIndex]];
                float3 v1 = vertices[indices[triangleIndex + 1]];
                float3 v2 = vertices[indices[triangleIndex + 2]];

                float3 normal = math.normalize(math.cross(v0 - v1, v0 - v2));
                planes[i] = new float4(normal, -math.dot(normal, v0));
            }

            var edges = new EdgeData[Edges.Length];

            for (int i = 0; i < edges.Length; i++)
            {
                Edge e = Edges[i];

                ref EdgeData edgeData = ref edges[i];
                edgeData.Edge = Edge.Invalid();
                edgeData.Value = float.MaxValue;

                if (IsBound(e))
                {
                    Edge linkEdge = GetLink(e);
                    int4 vis = GetVertexIndices(indices, e);
                    vis[3] = GetApexVertexIndex(indices, linkEdge);

                    float3x4 quadVertices = new float3x4(vertices[vis[0]], vertices[vis[1]], vertices[vis[2]], vertices[vis[3]]);
                    Aabb quadAabb = Aabb.CreateFromPoints(quadVertices);

                    float aabbSurfaceArea = quadAabb.SurfaceArea;

                    if (aabbSurfaceArea > Math.Constants.Eps)
                    {
                        float quadSurfaceArea = CalcTwiceSurfaceArea(quadVertices[0], quadVertices[1], quadVertices[2]) + CalcTwiceSurfaceArea(quadVertices[0], quadVertices[1], quadVertices[3]);
                        edgeData.Value = (aabbSurfaceArea - quadSurfaceArea) / aabbSurfaceArea;
                        edgeData.Edge = vis[0] < vis[1] ? e : linkEdge;
                    }
                }
            }

            edges = edges.OrderBy(e => e.Value).ToArray();

            bool[] freeTriangles = Enumerable.Repeat(true, numTriangles).ToArray();

            var primitives = new List<Primitive>();

            // Generate quads
            foreach (EdgeData edgeData in edges)
            {
                if (!edgeData.Edge.IsValid)
                {
                    break;
                }

                int t0 = edgeData.Edge.Triangle;
                Edge linkEdge = GetLink(edgeData.Edge);
                int t1 = linkEdge.Triangle;

                if (freeTriangles[t0] && freeTriangles[t1])
                {
                    Edge nextEdge = GetNext(edgeData.Edge);
                    int4 vis = GetVertexIndices(indices, nextEdge);
                    vis[3] = GetApexVertexIndex(indices, linkEdge);

                    var primitive = new Primitive
                    {
                        Vertices = new float3x4(vertices[vis[0]], vertices[vis[1]], vertices[vis[2]], vertices[vis[3]]),
                        Flags = PrimitiveFlags.DefaultTrianglePairFlags
                    };

                    if (IsTriangleDegenerate(primitive.Vertices[0], primitive.Vertices[1], primitive.Vertices[2]) ||
                        IsTriangleDegenerate(primitive.Vertices[0], primitive.Vertices[2], primitive.Vertices[3]))
                    {
                        continue;
                    }

                    if (IsEdgeConcaveOrFlat(edgeData.Edge, indices, vertices, planes))
                    {
                        primitive.Flags |= PrimitiveFlags.DisableInternalEdge;

                        if (IsTriangleConcaveOrFlat(edgeData.Edge, indices, vertices, planes) &&
                            IsTriangleConcaveOrFlat(linkEdge, indices, vertices, planes))
                        {
                            primitive.Flags |= PrimitiveFlags.DisableAllEdges;
                        }
                    }

                    if (IsFlat(edgeData.Edge, indices, vertices, planes))
                    {
                        primitive.Flags |= PrimitiveFlags.IsFlat;
                    }

                    if (IsConvexQuad(primitive, edgeData.Edge, planes))
                    {
                        primitive.Flags |= PrimitiveFlags.IsConvex;
                    }

                    primitives.Add(primitive);

                    freeTriangles[t0] = false;
                    freeTriangles[t1] = false;

                    flags[t0] = primitive.Flags;
                    flags[t1] = primitive.Flags;

                    quadRoots.Add(edgeData.Edge);
                }
            }

            // Generate triangles
            for (int i = 0; i < numTriangles; i++)
            {
                if (!freeTriangles[i])
                {
                    continue;
                }

                Edge edge = new Edge { Triangle = i, Start = 0, IsValid = true };
                while (edge.IsValid && GetStartVertexIndex(edge) < GetEndVertexIndex(edge))
                {
                    edge = GetNext(edge);
                }

                int4 vis = GetVertexIndices(indices, edge);

                var primitive = new Primitive
                {
                    Vertices = new float3x4(vertices[vis[0]], vertices[vis[1]], vertices[vis[2]], vertices[vis[2]]),
                    Flags = PrimitiveFlags.DefaultTriangleFlags
                };

                if (IsTriangleConcaveOrFlat(edge, indices, vertices, planes))
                {
                    primitive.Flags |= PrimitiveFlags.DisableAllEdges;
                }

                primitives.Add(primitive);
                triangleRoots.Add(edge);
                flags[edge.Triangle] = primitives.Last().Flags;
            }

            DisableEdgesOfAdjacentPrimitives(primitives, indices, vertices, planes, flags, quadRoots, triangleRoots);

            return primitives;
        }

        private void DisableEdgesOfAdjacentPrimitives(
            List<Primitive> primitives, int[] indices, float3[] vertices, float4[] planes, PrimitiveFlags[] flags,
            List<Edge> quadRoots, List<Edge> triangleRoots)
        {
            for (int quadIndex = 0; quadIndex < quadRoots.Count; quadIndex++)
            {
                Edge root = quadRoots[quadIndex];
                Edge link = GetLink(root);
                PrimitiveFlags quadFlags = flags[root.Triangle];
                if (quadFlags.HasFlag(PrimitiveFlags.IsFlatConvexQuad) && !quadFlags.HasFlag(PrimitiveFlags.DisableAllEdges))
                {
                    Edge[] outerBoundary = new Edge[4];
                    outerBoundary[0] = GetLink(GetNext(root));
                    outerBoundary[1] = GetLink(GetPrev(root));
                    outerBoundary[2] = GetLink(GetNext(link));
                    outerBoundary[3] = GetLink(GetPrev(link));

                    if (CanAllEdgesBeDisabled(outerBoundary, flags, indices, vertices, planes))
                    {
                        quadFlags |= PrimitiveFlags.DisableAllEdges;
                    }
                }

                // Sync triangle flags.
                flags[root.Triangle] = quadFlags;
                flags[link.Triangle] = quadFlags;

                // Write primitive flags.
                primitives[quadIndex] = new Primitive
                {
                    Vertices = primitives[quadIndex].Vertices,
                    Flags = quadFlags
                };
            }

            for (int triangleIndex = 0; triangleIndex < triangleRoots.Count; triangleIndex++)
            {
                Edge root = triangleRoots[triangleIndex];
                PrimitiveFlags triangleFlags = flags[root.Triangle];
                if (!triangleFlags.HasFlag(PrimitiveFlags.DisableAllEdges))
                {
                    Edge[] outerBoundary = new Edge[3];
                    outerBoundary[0] = GetLink(root);
                    outerBoundary[1] = GetLink(GetNext(root));
                    outerBoundary[2] = GetLink(GetPrev(root));

                    if (CanAllEdgesBeDisabled(outerBoundary, flags, indices, vertices, planes))
                    {
                        triangleFlags |= PrimitiveFlags.DisableAllEdges;
                    }
                }

                // Sync triangle flags.
                flags[root.Triangle] = triangleFlags;

                // Write primitive flags.
                int primitiveIndex = quadRoots.Count + triangleIndex;
                primitives[primitiveIndex] = new Primitive
                {
                    Vertices = primitives[primitiveIndex].Vertices,
                    Flags = triangleFlags
                };
            }
        }
    }
}
