using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    /// <summary>
    /// Convex hull builder.
    /// </summary>
    public struct ConvexHullBuilder : IDisposable
    {
        public ElementPool<Vertex> Vertices;
        public ElementPool<Triangle> Triangles;

        public int Dimension { get; private set; }
        public int NumFaces { get; private set; }
        public int NumFaceVertices { get; private set; }
        public Plane ProjectionPlane { get; private set; }

        private IntegerSpace m_IntegerSpace;
        private Aabb m_IntegerSpaceAabb;
        private uint m_NextUid;

        public Aabb IntegerSpaceAabb
        {
            get => m_IntegerSpaceAabb;
            set { m_IntegerSpaceAabb = value; UpdateIntSpace(); }
        }

        /// <summary>
        /// Convex hull vertex.
        /// </summary>
        [DebuggerDisplay("{Cardinality}:{Position}")]
        public struct Vertex : IPoolElement
        {
            public float3 Position;
            public int3 IntPosition;
            public int Cardinality;
            public uint UserData;
            public readonly uint Uid;
            public int NextFree { get; private set; }

            public bool IsAllocated => Cardinality != -1;

            public Vertex(float3 position, uint userData, uint uid)
            {
                Position = position;
                UserData = userData;
                Uid = uid;
                Cardinality = 0;
                IntPosition = new int3(0);
                NextFree = -1;
            }

            void IPoolElement.MarkFree(int nextFree)
            {
                Cardinality = -1;
                NextFree = nextFree;
            }
        }

        /// <summary>
        /// Convex hull triangle.
        /// </summary>
        [DebuggerDisplay("#{FaceIndex}[{Vertex0}, {Vertex1}, {Vertex2}]")]
        public struct Triangle : IPoolElement
        {
            public int Vertex0, Vertex1, Vertex2;
            public Edge Link0, Link1, Link2;
            public int FaceIndex;
            public readonly uint Uid;
            public int NextFree { get; private set; }

            public bool IsAllocated => FaceIndex != -2;

            public Triangle(int vertex0, int vertex1, int vertex2, uint uid)
            {
                FaceIndex = 0;
                Vertex0 = vertex0;
                Vertex1 = vertex1;
                Vertex2 = vertex2;
                Link0 = Edge.Invalid;
                Link1 = Edge.Invalid;
                Link2 = Edge.Invalid;
                Uid = uid;
                NextFree = -1;
            }

            public unsafe int GetVertex(int index) { fixed (int* p = &Vertex0) { return p[index]; } }
            public unsafe void SetVertex(int index, int value) { fixed (int* p = &Vertex0) { p[index] = value; } }

            public unsafe Edge GetLink(int index) { fixed (Edge* p = &Link0) { return p[index]; } }
            public unsafe void SetLink(int index, Edge handle) { fixed (Edge* p = &Link0) { p[index] = handle; } }

            void IPoolElement.MarkFree(int nextFree)
            {
                FaceIndex = -2;
                NextFree = nextFree;
            }
        }

        /// <summary>
        /// An edge of a triangle, used internally to traverse hull topology.
        /// </summary>
        [DebuggerDisplay("[{value>>2}:{value&3}]")]
        public struct Edge : IEquatable<Edge>
        {
            public readonly int Value;

            public bool IsValid => Value != Invalid.Value;
            public int TriangleIndex => Value >> 2;
            public int EdgeIndex => Value & 3;

            public static readonly Edge Invalid = new Edge(0x7fffffff);

            public Edge(int value) { Value = value; }
            public Edge(int triangleIndex, int edgeIndex) { Value = triangleIndex << 2 | edgeIndex; }

            public Edge Next => IsValid ? new Edge(TriangleIndex, (EdgeIndex + 1) % 3) : Invalid;
            public Edge Prev => IsValid ? new Edge(TriangleIndex, (EdgeIndex + 2) % 3) : Invalid;

            public bool Equals(Edge other) => Value == other.Value;
        }

        /// <summary>
        /// An edge of a face (possibly made from multiple triangles).
        /// </summary>
        public struct FaceEdge
        {
            public Edge Start;      // the first edge of the face
            public Edge Current;    // the current edge of the face

            public bool IsValid => Current.IsValid;

            public static readonly FaceEdge Invalid = new FaceEdge { Start = Edge.Invalid, Current = Edge.Invalid };

            public static implicit operator Edge(FaceEdge fe) => fe.Current;
        }

        /// <summary>
        /// Convex hull mass properties.
        /// </summary>
        public struct MassProperties
        {
            public float3 CenterOfMass;
            public float3x3 InertiaTensor;
            public float SurfaceArea;
            public float Volume;
        }

        /// <summary>
        /// Edge collapse information, used internally to simplify convex hull.
        /// </summary>
        public struct EdgeCollapse
        {
            public int StartVertex;
            public int EndVertex;
            public float4 Data;
        }

        /// <summary>
        /// A quantized integer space.
        /// </summary>
        private struct IntegerSpace
        {
            public readonly float3 Offset;
            public readonly float3 Scale;
            public readonly float3 InvScale;

            public IntegerSpace(Aabb aabb, int resolution, bool uniform, float minExtent)
            {
                if (uniform)
                {
                    float3 c = aabb.Center;
                    var m = new float3(math.cmax(aabb.Max - c));
                    aabb.Min = c - m;
                    aabb.Max = c + m;
                }
                float3 extents = math.max(minExtent, (aabb.Max - aabb.Min));
                Offset = aabb.Min;
                Scale = extents / resolution;
                InvScale = math.select(resolution / extents, new float3(0), Scale <= 0);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int3 ToIntegerSpace(float3 x) => new int3((x - Offset) * InvScale + 0.5f);

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public float3 ToFloatSpace(int3 x) => x * Scale + Offset;
        }

        /// <summary>
        /// Create using internal storage.
        /// </summary>
        public ConvexHullBuilder(int verticesCapacity, int trianglesCapacity, Allocator allocator = Allocator.Persistent)
        {
            Vertices = new ElementPool<Vertex>(verticesCapacity, allocator);
            Triangles = new ElementPool<Triangle>(trianglesCapacity, allocator);
            Dimension = -1;
            NumFaces = 0;
            NumFaceVertices = 0;
            m_NextUid = 1;
            m_IntegerSpaceAabb = Aabb.Empty;
            m_IntegerSpace = new IntegerSpace();
            ProjectionPlane = new Plane(new float3(0), 0);
        }

        /// <summary>
        /// Create using external storage.
        /// </summary>
        public unsafe ConvexHullBuilder(Vertex* vertices, int verticesCapacity, Triangle* triangles, int triangleCapacity)
        {
            Vertices = new ElementPool<Vertex>(vertices, verticesCapacity);
            Triangles = new ElementPool<Triangle>(triangles, triangleCapacity);
            Dimension = -1;
            NumFaces = 0;
            NumFaceVertices = 0;
            ProjectionPlane = new Plane(new float3(0), 0);
            m_NextUid = 1;
            m_IntegerSpaceAabb = Aabb.Empty;
            m_IntegerSpace = new IntegerSpace();
        }

        /// <summary>
        /// Copy the content of another convex hull into this one.
        /// </summary>
        public void CopyFrom(ConvexHullBuilder other)
        {
            Vertices.CopyFrom(other.Vertices);
            Triangles.CopyFrom(other.Triangles);
            Dimension = other.Dimension;
            NumFaces = other.NumFaces;
            NumFaceVertices = other.NumFaceVertices;
            ProjectionPlane = other.ProjectionPlane;
            m_IntegerSpaceAabb = other.m_IntegerSpaceAabb;
            m_IntegerSpace = other.m_IntegerSpace;
            m_NextUid = other.m_NextUid;
        }

        /// <summary>
        /// Dispose internal convex hull storages if any.
        /// </summary>
        [BurstDiscard]
        public void Dispose()
        {
            Vertices.Dispose();
            Triangles.Dispose();
        }

        #region Construction

        /// <summary>
        /// Reset the convex hull.
        /// </summary>
        public void Reset(bool keepIntegerSpaceAabb = false)
        {
            Vertices.Clear();
            Triangles.Clear();
            Dimension = -1;
            NumFaces = 0;
            NumFaceVertices = 0;
            ProjectionPlane = new Plane(new float3(0), 0);
            if (!keepIntegerSpaceAabb)
            {
                m_IntegerSpaceAabb = Aabb.Empty;
                m_IntegerSpace = new IntegerSpace();
            }
        }

        /// <summary>
        /// Add a point the the convex hull.
        /// </summary>
        /// <param name="point">Point to insert.</param>
        /// <param name="userData">User data attached to the new vertex if insertion succeeds.</param>        
        /// <returns>true if the insertion succeeded, false otherwise.</returns>
        public unsafe bool AddPoint(float3 point, uint userData = 0)
        {
            // Acceptance tolerances
            const float minDistanceFromPoint = 1e-5f;   // for dimension = 1

            // Reset faces.
            NumFaces = 0;
            NumFaceVertices = 0;

            // Update integer space.
            bool intSpaceUpdated = false;
            if (!m_IntegerSpaceAabb.Contains(point))
            {
                m_IntegerSpaceAabb.Include(point);
                UpdateIntSpace();
                intSpaceUpdated = true;
            }

            int3 intPoint = m_IntegerSpace.ToIntegerSpace(point);

            // Return false if there is not enough room to allocate a vertex.
            if (Vertices.PeakCount >= Vertices.Capacity)
            {
                return false;
            }

            // Insert vertex.
            switch (Dimension)
            {
                // Empty hull, just add a vertex.
                case -1:
                {
                    AllocateVertex(point, userData);
                    Dimension = 0;
                }
                break;

                // 0 dimensional hull, make a line.
                case 0:
                {
                    if (math.lengthsq(Vertices[0].Position - point) <= minDistanceFromPoint) return false;
                    AllocateVertex(point, userData);
                    Dimension = 1;
                }
                break;

                // 1 dimensional hull, make a triangle.
                case 1:
                {
                    if (IsTriangleAreaZero(Vertices[0].IntPosition, Vertices[1].IntPosition, intPoint))
                    {
                        // Extend the line.
                        var delta = Vertices[1].Position - Vertices[0].Position;
                        float3 diff = point - Vertices[0].Position;
                        float dot = math.dot(diff, delta);
                        float solution = dot * math.rcp(math.lengthsq(delta));
                        float3 projectedPosition = Vertices[0].Position + solution * delta;
                        if (solution < 0)
                        {
                            Vertex projection = Vertices[0];
                            projection.Position = projectedPosition;
                            projection.IntPosition = m_IntegerSpace.ToIntegerSpace(projectedPosition);
                            projection.UserData = userData;
                            Vertices[0] = projection;
                        }
                        else if (solution > 1)
                        {
                            Vertex projection = Vertices[1];
                            projection.Position = projectedPosition;
                            projection.IntPosition = m_IntegerSpace.ToIntegerSpace(projectedPosition);
                            projection.UserData = userData;
                            Vertices[1] = projection;
                        }
                    }
                    else
                    {
                        // Extend dimension.
                        AllocateVertex(point, userData);
                        Dimension = 2;
                        ProjectionPlane = ComputePlane(0, 1, 2, true);
                    }
                }
                break;

                // 2 dimensional hull, make a volume or expand face.
                case 2:
                {
                    long det = Det64(0, 1, 2, intPoint);
                    if (det == 0)
                    {
                        bool* isOutside = stackalloc bool[Vertices.PeakCount];
                        bool isOutsideAny = false;
                        for (int i = Vertices.PeakCount - 1, j = 0; j < Vertices.PeakCount; i = j++)
                        {
                            float sign = math.dot(ProjectionPlane.Normal, math.cross(Vertices[j].Position - point, Vertices[i].Position - point));
                            isOutsideAny |= isOutside[i] = sign > 0;
                        }
                        if (isOutsideAny)
                        {
                            Vertex* newVertices = stackalloc Vertex[Vertices.PeakCount + 1];
                            int numNewVertices = 1;
                            newVertices[0] = new Vertex(point, userData, m_NextUid++);
                            newVertices[0].IntPosition = m_IntegerSpace.ToIntegerSpace(point);
                            for (int i = Vertices.PeakCount - 1, j = 0; j < Vertices.PeakCount; i = j++)
                            {
                                if (isOutside[i] && isOutside[i] != isOutside[j])
                                {
                                    newVertices[numNewVertices++] = Vertices[j];
                                    for (; ; )
                                    {
                                        if (isOutside[j]) break;
                                        j = (j + 1) % Vertices.PeakCount;
                                        newVertices[numNewVertices++] = Vertices[j];
                                    }
                                    break;
                                }
                            }

                            Vertices.CopyFrom(newVertices, numNewVertices);
                        }
                    }
                    else
                    {
                        // Extend dimension.
                        ProjectionPlane = new Plane(new float3(0), 0);

                        // Orient tetrahedron.
                        if (det > 0)
                        {
                            Vertex t = Vertices[2];
                            Vertices[2] = Vertices[1];
                            Vertices[1] = t;
                        }

                        // Allocate vertex.
                        int nv = Vertices.PeakCount;
                        int vertexIndex = AllocateVertex(point, userData);

                        // Build tetrahedron.
                        Dimension = 3;
                        Edge nt0 = AllocateTriangle(0, 1, 2);
                        Edge nt1 = AllocateTriangle(1, 0, vertexIndex);
                        Edge nt2 = AllocateTriangle(2, 1, vertexIndex);
                        Edge nt3 = AllocateTriangle(0, 2, vertexIndex);
                        BindEdges(nt0, nt1); BindEdges(nt0.Next, nt2); BindEdges(nt0.Prev, nt3);
                        BindEdges(nt1.Prev, nt2.Next); BindEdges(nt2.Prev, nt3.Next); BindEdges(nt3.Prev, nt1.Next);

                        // Re-insert other vertices.
                        bool success = true;
                        for (int i = 3; i < nv; ++i)
                        {
                            Vertex vertex = Vertices[i];
                            Vertices.Release(i);
                            success = success & AddPoint(vertex.Position, vertex.UserData);
                        }
                        return success;
                    }
                }
                break;

                // 3 dimensional hull, add vertex.
                case 3:
                {
                    if (intSpaceUpdated)
                    {
                        // Check if convexity still holds.
                        bool isFlat = false;
                        for (Edge edge = GetFirstPrimaryEdge(); edge.IsValid; edge = GetNextPrimaryEdge(edge))
                        {
                            int i = StartVertex(edge), j = EndVertex(edge), k = ApexVertex(edge), l = ApexVertex(GetLinkedEdge(edge));
                            long det = Det64(i, j, k, l);
                            if (det > 0)
                            {
                                // Found a concave edge, release triangles, create tetrahedron and reinsert vertices.
                                for (int t = Triangles.GetFirstIndex(); t != -1;)
                                {
                                    int n = Triangles.GetNextIndex(t);
                                    ReleaseTriangle(t, false);
                                    t = n;
                                }

                                Edge t0 = AllocateTriangle(j, i, k);
                                Edge t1 = AllocateTriangle(i, j, l);
                                Edge t2 = AllocateTriangle(l, k, i);
                                Edge t3 = AllocateTriangle(k, l, j);
                                BindEdges(t0, t1); BindEdges(t2, t3);
                                BindEdges(t0.Next, t2.Next); BindEdges(t0.Prev, t3.Prev);
                                BindEdges(t1.Next, t3.Next); BindEdges(t1.Prev, t2.Prev);

                                for (int v = Vertices.GetFirstIndex(); v != -1;)
                                {
                                    int n = Vertices.GetNextIndex(v);
                                    if (Vertices[v].Cardinality == 0)
                                    {
                                        Vertex cv = Vertices[v];
                                        Vertices.Release(v);
                                        AddPoint(cv.Position, cv.UserData);
                                    }
                                    v = n;
                                }
                                break;
                            }

                            isFlat &= det == 0;
                        }

                        // Hull became flat following int space update.
                        Assert.IsFalse(isFlat);
                    }

                    int* nextTriangles = stackalloc int[Triangles.PeakCount];
                    for (int i = 0; i < Triangles.PeakCount; i++)
                    {
                        nextTriangles[i] = -1;
                    }

                    Edge* newEdges = stackalloc Edge[Vertices.PeakCount];
                    for (int i = 0; i < Vertices.PeakCount; i++)
                    {
                        newEdges[i] = Edge.Invalid;
                    }

                    // Classify all triangles as either front(faceIndex = 1) or back(faceIndex = -1).
                    int firstFrontTriangleIndex = -1, numFrontTriangles = 0, numBackTriangles = 0;
                    float3 floatPoint = m_IntegerSpace.ToFloatSpace(intPoint);
                    float maxDistance = 0.0f;
                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        Triangle triangle = Triangles[triangleIndex];
                        long det = Det64(triangle.Vertex0, triangle.Vertex1, triangle.Vertex2, intPoint);
                        if (det == 0)
                        {
                            // Check for duplicated vertex.
                            if (math.all(Vertices[triangle.Vertex0].IntPosition == intPoint)) return false;
                            if (math.all(Vertices[triangle.Vertex1].IntPosition == intPoint)) return false;
                            if (math.all(Vertices[triangle.Vertex2].IntPosition == intPoint)) return false;
                        }
                        if (det > 0)
                        {
                            newEdges[triangle.Vertex0] = Edge.Invalid;
                            newEdges[triangle.Vertex1] = Edge.Invalid;
                            newEdges[triangle.Vertex2] = Edge.Invalid;

                            nextTriangles[triangleIndex] = firstFrontTriangleIndex;
                            firstFrontTriangleIndex = triangleIndex;

                            triangle.FaceIndex = 1;
                            numFrontTriangles++;

                            Plane plane = ComputePlane(triangleIndex, true);
                            float distance = math.dot(plane.Normal, floatPoint) + plane.Distance;
                            maxDistance = math.max(distance, maxDistance);
                        }
                        else
                        {
                            triangle.FaceIndex = -1;
                            numBackTriangles++;
                        }
                        Triangles[triangleIndex] = triangle;
                    }

                    // Return false if the vertex is inside the hull
                    if (numFrontTriangles == 0 || numBackTriangles == 0)
                    {
                        return false;
                    }

                    // Return false is the vertex is too close to the surface to be inserted
                    if (maxDistance <= minDistanceFromPoint)
                    {
                        return false;
                    }

                    // Link boundary loop.
                    Edge loopEdge = Edge.Invalid;
                    int loopCount = 0;
                    for (int frontTriangle = firstFrontTriangleIndex; frontTriangle != -1; frontTriangle = nextTriangles[frontTriangle])
                    {
                        for (int j = 0; j < 3; ++j)
                        {
                            var edge = new Edge(frontTriangle, j);
                            Edge linkEdge = GetLinkedEdge(edge);
                            if (Triangles[linkEdge.TriangleIndex].FaceIndex == -1)
                            {
                                int vertexIndex = StartVertex(linkEdge);

                                // Vertex already bound.
                                Assert.IsTrue(newEdges[vertexIndex].Equals(Edge.Invalid));

                                // Link.
                                newEdges[vertexIndex] = linkEdge;
                                loopEdge = linkEdge;
                                loopCount++;
                            }
                        }
                    }

                    // Return false if there is not enough room to allocate new triangles.
                    if ((Triangles.PeakCount + loopCount - numFrontTriangles) > Triangles.Capacity)
                    {
                        return false;
                    }

                    // Release front triangles.
                    do
                    {
                        int next = nextTriangles[firstFrontTriangleIndex];
                        ReleaseTriangle(firstFrontTriangleIndex);
                        firstFrontTriangleIndex = next;
                    } while (firstFrontTriangleIndex != -1);

                    // Add vertex.
                    int newVertex = AllocateVertex(point, userData);

                    // Add fan of triangles.
                    Edge firstFanEdge = Edge.Invalid, lastFanEdge = Edge.Invalid;
                    for (int i = 0; i < loopCount; ++i)
                    {
                        int v0 = StartVertex(loopEdge), v1 = EndVertex(loopEdge);
                        Edge t = AllocateTriangle(v1, v0, newVertex);
                        BindEdges(loopEdge, t);
                        if (lastFanEdge.IsValid)
                            BindEdges(t.Next, lastFanEdge.Prev);
                        else
                            firstFanEdge = t;

                        lastFanEdge = t;
                        loopEdge = newEdges[v1];
                    }
                    BindEdges(lastFanEdge.Prev, firstFanEdge.Next);
                }
                break;
            }
            return true;
        }

        /// <summary>
        /// Set the face index for each triangle.
        /// Triangles lying in the same plane will have the same face index.
        /// </summary>
        [BurstDiscard]
        public void BuildFaceIndices(float maxAngle = (float)(1 * System.Math.PI / 180))
        {
            float maxCosAngle = math.cos(maxAngle);
            const float convexEps = 1e-5f;

            NumFaces = 0;
            NumFaceVertices = 0;

            switch (Dimension)
            {
                case 2:
                    NumFaces = 1;
                    NumFaceVertices = Vertices.PeakCount;
                    break;

                case 3:
                {
                    var sortedTriangles = new List<int>();
                    var triangleAreas = new float[Triangles.PeakCount];
                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        Triangle t = Triangles[triangleIndex];
                        float3 o = Vertices[t.Vertex0].Position;
                        float3 a = Vertices[t.Vertex1].Position - o;
                        float3 b = Vertices[t.Vertex2].Position - o;
                        triangleAreas[triangleIndex] = math.lengthsq(math.cross(a, b));
                        t.FaceIndex = -1;
                        Triangles[triangleIndex] = t;
                        sortedTriangles.Add(triangleIndex);
                    }

                    sortedTriangles.Sort((a, b) => triangleAreas[b].CompareTo(triangleAreas[a]));

                    var boundaryEdges = new List<Edge>();
                    foreach (int triangleIndex in sortedTriangles)
                    {
                        if (Triangles[triangleIndex].FaceIndex != -1)
                        {
                            continue;
                        }
                        int newFaceIndex = NumFaces++;
                        float3 normal = ComputePlane(triangleIndex).Normal;
                        Triangle t = Triangles[triangleIndex]; t.FaceIndex = newFaceIndex; Triangles[triangleIndex] = t;

                        boundaryEdges.Clear();
                        boundaryEdges.Add(new Edge(triangleIndex, 0));
                        boundaryEdges.Add(new Edge(triangleIndex, 1));
                        boundaryEdges.Add(new Edge(triangleIndex, 2));

                        while (true)
                        {
                            int openBoundaryEdgeIndex = -1;
                            float maxArea = -1;

                            for (int i = 0; i < boundaryEdges.Count; ++i)
                            {
                                Edge edge = boundaryEdges[i];
                                Edge linkedEdge = GetLinkedEdge(edge);

                                int linkedTriangleIndex = linkedEdge.TriangleIndex;

                                if (Triangles[linkedTriangleIndex].FaceIndex != -1) continue;
                                if (triangleAreas[linkedTriangleIndex] <= maxArea) continue;
                                if (math.dot(normal, ComputePlane(linkedTriangleIndex).Normal) < maxCosAngle) continue;

                                int apex = ApexVertex(linkedEdge);
                                Plane p0 = PlaneFromTwoEdges(Vertices[apex].Position, Vertices[apex].Position - Vertices[StartVertex(edge)].Position, normal);
                                Plane p1 = PlaneFromTwoEdges(Vertices[apex].Position, Vertices[EndVertex(edge)].Position - Vertices[apex].Position, normal);

                                var accept = true;
                                for (int j = 1; accept && j < (boundaryEdges.Count - 1); ++j)
                                {
                                    float3 x = Vertices[EndVertex(boundaryEdges[(i + j) % boundaryEdges.Count])].Position;
                                    float d = math.max(Dotxyz1(p0, x), Dotxyz1(p1, x));
                                    accept &= d < convexEps;
                                }

                                if (accept)
                                {
                                    openBoundaryEdgeIndex = i;
                                    maxArea = triangleAreas[linkedTriangleIndex];
                                }
                            }

                            if (openBoundaryEdgeIndex != -1)
                            {
                                Edge linkedEdge = GetLinkedEdge(boundaryEdges[openBoundaryEdgeIndex]);
                                boundaryEdges[openBoundaryEdgeIndex] = linkedEdge.Prev;
                                boundaryEdges.Insert(openBoundaryEdgeIndex, linkedEdge.Next);

                                Triangle tri = Triangles[linkedEdge.TriangleIndex];
                                tri.FaceIndex = newFaceIndex;
                                Triangles[linkedEdge.TriangleIndex] = tri;
                            }
                            else
                            {
                                break;
                            }
                        }
                        NumFaceVertices += boundaryEdges.Count;
                    }
                }
                break;
            }
        }

        /// <summary>
        /// Remove redundant (colinear, coplanar) vertices from a 3D convex hull.        
        /// </summary>
        /// <param name="minCosAngle">Assume that edges that form a angle smaller than this value are flat</param>
        [BurstDiscard]
        public unsafe void RemoveRedundantVertices(float minCosAngle = 0.999f)
        {
            var newVertices = new List<Vertex>();
            for (bool remove = true; remove;)
            {
                remove = false;
                if (Dimension != 3) return;

                for (var primaryEdge = GetFirstPrimaryEdge(); primaryEdge.IsValid; primaryEdge = GetNextPrimaryEdge(primaryEdge))
                {
                    var edge = primaryEdge;
                    int cardinality = 0;
                    do
                    {
                        // Increment cardinality if planes on both side of the edge form an angle greater than acos(minCosAngle).
                        var plane0 = ComputePlane(edge.TriangleIndex);
                        var plane1 = ComputePlane(GetLinkedEdge(edge).TriangleIndex);
                        if (math.dot(plane0.Normal, plane1.Normal) < minCosAngle) cardinality++;

                        // Turn clockwise around the primary edge start vertex.
                        edge = GetLinkedEdge(edge.Prev);
                    } while (edge.Value != primaryEdge.Value);

                    // Only add vertices with cardinality greater or equal to 3.
                    if (cardinality >= 3)
                        newVertices.Add(Vertices[StartVertex(primaryEdge)]);
                    else
                        remove = true;
                }

                Reset(true);
                foreach (var vertex in newVertices) AddPoint(vertex.Position, vertex.UserData);
                newVertices.Clear();
            }
        }

        /// <summary>
        /// Simplify convex hull.
        /// </summary>
        /// <param name="maxError"></param>
        /// <param name="minVertices"></param>
        /// <param name="volumeConservation"></param>
        [BurstDiscard]
        public unsafe void SimplifyVertices(float maxError, int minVertices = 0, float volumeConservation = 1)
        {
            // Ensure that parameters are valid.
            maxError = math.max(0, maxError);
            minVertices = math.max(Dimension + 1, minVertices);
            volumeConservation = math.clamp(volumeConservation, 0, 1);

            // Remove colinear and coplanar vertices first.
            RemoveRedundantVertices();

            // Keep collapsing edges until no changes are possible.            
            int numVertices = Vertices.PeakCount;
            var newPoints = new List<EdgeCollapse>();
            while (true)
            {
                // 2D hull.
                if (Dimension == 2 && Vertices.PeakCount > 3)
                {
                    Plane projectionPlane = ProjectionPlane;

                    Plane* edgePlanes = stackalloc Plane[Vertices.PeakCount];
                    for (int n = Vertices.PeakCount, i = n - 1, j = 0; j < n; i = j++)
                    {
                        edgePlanes[i] = PlaneFromTwoEdges(Vertices[i].Position, Vertices[j].Position - Vertices[i].Position, projectionPlane.Normal);
                    }

                    float4x4 m;
                    m.c2 = projectionPlane;
                    m.c3 = new float4(0, 0, 0, 1);

                    for (int n = Vertices.PeakCount, j = 0; j < n; ++j)
                    {
                        int i = (j + n - 1) % n, k = (j + 1) % n;
                        m.c0 = edgePlanes[i];
                        m.c1 = edgePlanes[k];
                        float3 position = math.inverse(math.transpose(m)).c3.xyz;
                        var vertex = new float4(position, Dotxyz1(edgePlanes[j], position));
                        newPoints.Add(new EdgeCollapse { StartVertex = j, EndVertex = k, Data = vertex });
                    }
                }
                // 3D hull.
                else if (Dimension == 3)
                {
                    var perVertexPlanes = new float4[Vertices.PeakCount];

                    // Compute angle-weighted normals per vertex and store it in 'perVertexPlanes'.
                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        Triangle t = Triangles[triangleIndex];
                        var p = (float4)ComputePlane(triangleIndex);
                        float3 v0 = Vertices[t.Vertex0].Position;
                        float3 v1 = Vertices[t.Vertex1].Position;
                        float3 v2 = Vertices[t.Vertex2].Position;
                        perVertexPlanes[t.Vertex0] += p * math.acos(math.dot(math.normalize(v1 - v0), math.normalize(v2 - v0)));
                        perVertexPlanes[t.Vertex1] += p * math.acos(math.dot(math.normalize(v0 - v1), math.normalize(v2 - v1)));
                        perVertexPlanes[t.Vertex2] += p * math.acos(math.dot(math.normalize(v0 - v2), math.normalize(v1 - v2)));
                    }

                    for (int vertexIndex = 0; vertexIndex < Vertices.PeakCount; ++vertexIndex)
                    {
                        if (!Vertices[vertexIndex].IsAllocated)
                        {
                            numVertices--;
                            perVertexPlanes[vertexIndex] = new Plane();
                        }
                        else
                        {
                            perVertexPlanes[vertexIndex] = PlaneFromDirection(Vertices[vertexIndex].Position, perVertexPlanes[vertexIndex].xyz);
                        }
                    }

                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        for (int sideIndex = 0; sideIndex < 3; ++sideIndex)
                        {
                            var edge = new Edge(triangleIndex, sideIndex);
                            if (!IsPrimaryEdge(edge)) continue;

                            float3 midPoint = (Vertices[StartVertex(edge)].Position + Vertices[EndVertex(edge)].Position) / 2;
                            Plane midPlane = PlaneFromDirection(midPoint, perVertexPlanes[StartVertex(edge)].xyz + perVertexPlanes[EndVertex(edge)].xyz);

                            float4x4 m;
                            m.c0 = perVertexPlanes[StartVertex(edge)];
                            m.c1 = perVertexPlanes[EndVertex(edge)];
                            m.c2 = PlaneFromTwoEdges(midPoint, perVertexPlanes[StartVertex(edge)].xyz, perVertexPlanes[EndVertex(edge)].xyz);
                            m.c3 = new float4(0, 0, 0, 1);

                            float det = Det(m.c0.xyz, m.c1.xyz, m.c2.xyz);
                            if (det > 1e-6f)
                            {
                                float4 vertex = math.inverse(math.transpose(m)).c3;
                                vertex.w = Dotxyz1(midPlane, vertex.xyz);
                                newPoints.Add(new EdgeCollapse { StartVertex = StartVertex(edge), EndVertex = EndVertex(edge), Data = vertex });
                            }
                        }
                    }
                }

                int verticesRemoved = 0;

                if (newPoints.Count > 0 || numVertices > minVertices)
                {
                    // Sort new points.
                    newPoints.Sort((a, b) => a.Data.w < b.Data.w ? -1 : (a.Data.w > b.Data.w ? 1 : 0));

                    // Filter out new points.
                    bool* exclude = stackalloc bool[Vertices.PeakCount];
                    for (int i = 0; i < Vertices.PeakCount; ++i)
                    {
                        exclude[i] = false;
                    }

                    for (int i = 0; i < newPoints.Count; ++i)
                    {
                        EdgeCollapse p = newPoints[i];
                        if (numVertices <= minVertices || exclude[p.StartVertex] || exclude[p.EndVertex] || p.Data.w <= 0 || p.Data.w > maxError)
                        {
                            p.Data.w = -1;
                            newPoints[i] = p;
                        }
                        else
                        {
                            if (volumeConservation < 1)
                            {
                                float3 midPoint = (Vertices[p.StartVertex].Position + Vertices[p.EndVertex].Position) / 2;
                                p.Data.xyz = midPoint + (midPoint - p.Data.xyz) * volumeConservation;
                                newPoints[i] = p;
                            }
                            exclude[p.StartVertex] = exclude[p.EndVertex] = true;
                            numVertices--;
                            verticesRemoved++;
                        }
                    }

                    // Add existing vertices not referenced by edge collapse.
                    for (int i = 0; i < Vertices.PeakCount; i++)
                    {
                        if (!exclude[i])
                        {
                            newPoints.Add(new EdgeCollapse { Data = new float4(Vertices[i].Position, 1) });
                        }
                    }

                    // Rebuild convex hull.
                    Reset(keepIntegerSpaceAabb: true);
                    for (int i = 0; i < newPoints.Count; ++i)
                    {
                        if (newPoints[i].Data.w > 0)
                        {
                            AddPoint(newPoints[i].Data.xyz, (uint)i);
                        }
                    }
                    RemoveRedundantVertices();
                }

                if (verticesRemoved > 0)
                {
                    newPoints.Clear();
                    numVertices = Vertices.PeakCount;
                }
                else
                {
                    break;
                }
            }
        }

        /// <summary>
        /// Simplify faces.
        /// </summary>
        /// <param name="minError">Stop refining when the error is below this number (unit: distance)</param>
        /// <param name="maxFaces">Maximum number of faces allowed.</param>
        /// <param name="maxVertices">Maximum number of vertices allowed</param>
        /// <param name="minAngle">Do not generate face that form an angle smaller than this value with its neighbors (unit: angle in radian)</param>
        [BurstDiscard]
        public void SimplifyFaces(float minError, int maxFaces = int.MaxValue, int maxVertices = int.MaxValue, float minAngle = 0)
        {
            // This method is only working on 3D convex hulls.
            Assert.IsTrue(Dimension == 3);

            RemoveRedundantVertices();
            BuildFaceIndices();

            maxFaces = math.clamp(maxFaces, 6, NumFaces);
            minError = math.max(0, minError);

            float cosAngle = math.clamp(math.cos(minAngle), 0, 1);
            var planes = new List<Plane>();

            foreach (int triangleIndex in Triangles.Indices)
            {
                planes.Add(ComputePlane(triangleIndex));
            }

            Aabb actualAabb = ComputeAabb();

            Reset(true);
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(false, false, false)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(true, false, false)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(true, true, false)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(false, true, false)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(false, false, true)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(true, false, true)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(true, true, true)));
            AddPoint(math.select(actualAabb.Min, actualAabb.Max, new bool3(false, true, true)));

            BuildFaceIndices();

            while (NumFaces <= maxFaces && Vertices.PeakCount <= maxVertices && planes.Count > 0)
            {
                float maxd = 0;
                int bestPlane = -1;

                float3[] normals = null;
                if (cosAngle < 1)
                {
                    normals = new float3[Triangles.PeakCount];
                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        normals[triangleIndex] = ComputePlane(triangleIndex).Normal;
                    }
                }

                for (int i = 0; i < planes.Count; ++i)
                {
                    float d = Dotxyz1(planes[i], Vertices[ComputeSupportingVertex(planes[i].Normal)].Position);
                    if (d > maxd)
                    {
                        if (normals != null)
                        {
                            bool isAngleOk = true;
                            foreach (int triangleIndex in Triangles.Indices)
                            {
                                if (math.dot(normals[triangleIndex], planes[i].Normal) > cosAngle)
                                {
                                    isAngleOk = false;
                                    break;
                                }
                            }
                            if (!isAngleOk)
                            {
                                continue;
                            }
                        }
                        bestPlane = i;
                        maxd = d;
                    }
                }

                if (bestPlane == -1) break;
                if (maxd <= minError) break;

                ConvexHullBuilder negHull, posHull;
                SplitByPlane(planes[bestPlane], out negHull, out posHull);
                planes.RemoveAt(bestPlane);
                if (negHull.Dimension == 3)
                {
                    negHull.RemoveRedundantVertices();
                    negHull.BuildFaceIndices();
                    if (negHull.NumFaces <= maxFaces && negHull.Vertices.PeakCount <= maxVertices)
                    {
                        CopyFrom(negHull);
                    }
                }
                posHull.Dispose();
                negHull.Dispose();
            }
        }

        /// <summary>
        /// Offset vertices with respect to the surface by a given amount while minimizing distortions.
        /// </summary>
        /// <param name="surfaceOffset">a positive value will move vertices outward while a negative value will push them toward the inside</param>
        /// <param name="minRadius">Prevent vertices to be shrunk too much</param>
        [BurstDiscard]
        public void OffsetVertices(float surfaceOffset, float minRadius = 0)
        {
            minRadius = math.abs(minRadius);

            // This method is only working on 3D convex hulls.
            Assert.IsTrue(Dimension == 3);

            float3 com = ComputeMassProperties().CenterOfMass;
            Vertex[] newVertices = new Vertex[Vertices.PeakCount];
            int numVertices = 0;
            foreach (int vertexIndex in Vertices.Indices)
            {
                Edge edge = GetVertexEdge(vertexIndex);
                float3 normal = ComputeVertexNormal(edge);
                float offset = surfaceOffset;
                float3 position = Vertices[vertexIndex].Position;
                if (surfaceOffset < 0)
                {
                    // Clip offset to prevent flipping.
                    var plane = new float4(normal, -math.dot(com, normal) - minRadius);
                    offset = math.max(-Dotxyz1(plane, position), offset);
                }
                newVertices[numVertices++] = new Vertex
                {
                    Position = position + normal * offset,
                    UserData = Vertices[vertexIndex].UserData
                };
            }

            Reset(true);
            for (int i = 0; i < numVertices; ++i)
            {
                AddPoint(newVertices[i].Position, newVertices[i].UserData);
            }
        }

        /// <summary>
        /// Split this convex hull by a plane.
        /// </summary>
        [BurstDiscard]
        public void SplitByPlane(Plane plane, out ConvexHullBuilder negHull, out ConvexHullBuilder posHull, uint isecUserData = 0)
        {
            negHull = new ConvexHullBuilder(Vertices.PeakCount * 2, Vertices.PeakCount * 4);
            posHull = new ConvexHullBuilder(Vertices.PeakCount * 2, Vertices.PeakCount * 4);
            negHull.IntegerSpaceAabb = IntegerSpaceAabb;
            posHull.IntegerSpaceAabb = IntegerSpaceAabb;

            // Compute vertices signed distance to plane and add them to the hull they belong too.
            const float minD2P = 1e-6f;
            var perVertexD2P = new float[Vertices.PeakCount];
            foreach (int i in Vertices.Indices)
            {
                float3 position = Vertices[i].Position;
                perVertexD2P[i] = Dotxyz1(plane, position);
                if (math.abs(perVertexD2P[i]) <= minD2P) perVertexD2P[i] = 0;
                if (perVertexD2P[i] <= 0) negHull.AddPoint(position, Vertices[i].UserData);
                if (perVertexD2P[i] >= 0) posHull.AddPoint(position, Vertices[i].UserData);
            }

            // Add intersecting vertices.
            switch (Dimension)
            {
                case 1:
                {
                    if ((perVertexD2P[0] * perVertexD2P[1]) < 0)
                    {
                        float sol = perVertexD2P[0] / (perVertexD2P[0] - perVertexD2P[1]);
                        float3 isec = Vertices[0].Position + (Vertices[1].Position - Vertices[0].Position) * sol;
                        negHull.AddPoint(isec, isecUserData);
                        posHull.AddPoint(isec, isecUserData);
                    }
                }
                break;
                case 2:
                {
                    for (int n = Vertices.PeakCount, i = n - 1, j = 0; j < n; i = j++)
                    {
                        if ((perVertexD2P[i] * perVertexD2P[j]) < 0)
                        {
                            float sol = perVertexD2P[i] / (perVertexD2P[i] - perVertexD2P[j]);
                            float3 isec = Vertices[i].Position + (Vertices[j].Position - Vertices[i].Position) * sol;
                            negHull.AddPoint(isec, isecUserData);
                            posHull.AddPoint(isec, isecUserData);
                        }
                    }
                }
                break;
                case 3:
                {
                    var edges = new List<EdgeCollapse>();
                    for (var edge = GetFirstPrimaryEdge(); edge.IsValid; edge = GetNextPrimaryEdge(edge))
                    {
                        float d = math.dot(ComputePlane(edge.TriangleIndex).Normal, ComputePlane(GetLinkedEdge(edge).TriangleIndex).Normal);
                        edges.Add(new EdgeCollapse { StartVertex = StartVertex(edge), EndVertex = EndVertex(edge), Data = new float4(d) });
                    }

                    // Insert sharpest edge intersection first to improve quality.
                    edges.Sort((a, b) => a.Data.x.CompareTo(b.Data.x));

                    foreach (var edge in edges)
                    {
                        int i = edge.StartVertex, j = edge.EndVertex;
                        if ((perVertexD2P[i] * perVertexD2P[j]) < 0)
                        {
                            float sol = perVertexD2P[i] / (perVertexD2P[i] - perVertexD2P[j]);
                            float3 isec = Vertices[i].Position + (Vertices[j].Position - Vertices[i].Position) * sol;
                            negHull.AddPoint(isec, isecUserData);
                            posHull.AddPoint(isec, isecUserData);
                        }
                    }
                }
                break;
            }
        }

        private int AllocateVertex(float3 point, uint userData)
        {
            Assert.IsTrue(m_IntegerSpaceAabb.Contains(point));
            var vertex = new Vertex(point, userData, m_NextUid++) { IntPosition = m_IntegerSpace.ToIntegerSpace(point) };
            return Vertices.Allocate(vertex);
        }

        private Edge AllocateTriangle(int vertex0, int vertex1, int vertex2)
        {
            Triangle triangle = new Triangle(vertex0, vertex1, vertex2, m_NextUid++);
            int triangleIndex = Triangles.Allocate(triangle);

            Vertex v;
            v = Vertices[vertex0]; v.Cardinality++; Vertices[vertex0] = v;
            v = Vertices[vertex1]; v.Cardinality++; Vertices[vertex1] = v;
            v = Vertices[vertex2]; v.Cardinality++; Vertices[vertex2] = v;

            return new Edge(triangleIndex, 0);
        }

        private void ReleaseTriangle(int triangle, bool releaseOrphanVertices = true)
        {
            for (int i = 0; i < 3; ++i)
            {
                int j = Triangles[triangle].GetVertex(i);
                Vertex v = Vertices[j];
                v.Cardinality--;
                Vertices[j] = v;
                if (v.Cardinality == 0 && releaseOrphanVertices)
                {
                    Vertices.Release(j);
                }
            }

            Triangles.Release(triangle);
        }

        private void BindEdges(Edge lhs, Edge rhs)
        {
            // Incompatible edges.
            Assert.IsTrue(EndVertex(lhs) == StartVertex(rhs) && StartVertex(lhs) == EndVertex(rhs));

            Triangle lf = Triangles[lhs.TriangleIndex];
            Triangle rf = Triangles[rhs.TriangleIndex];
            lf.SetLink(lhs.EdgeIndex, rhs);
            rf.SetLink(rhs.EdgeIndex, lhs);
            Triangles[lhs.TriangleIndex] = lf;
            Triangles[rhs.TriangleIndex] = rf;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void UpdateIntSpace()
        {
            const int quantizationBits = 16;
            const float minExtent = 1e-5f;
            m_IntegerSpace = new IntegerSpace(m_IntegerSpaceAabb, (1 << quantizationBits) - 1, true, minExtent);
            foreach (int i in Vertices.Indices)
            {
                Vertex v = Vertices[i];
                v.IntPosition = m_IntegerSpace.ToIntegerSpace(Vertices[i].Position);
                Vertices[i] = v;
            }
        }

        #endregion

        #region Edge methods

        /// <summary>
        /// Returns one of the triangle edges starting from a given vertex.
        /// Note: May be one of the inner edges of a face.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Edge GetVertexEdge(int vertexIndex)
        {
            Assert.IsTrue(Dimension == 3, "This method is only working on 3D convex hulls.");
            foreach (int triangleIndex in Triangles.Indices)
            {
                Triangle triangle = Triangles[triangleIndex];
                if (triangle.Vertex0 == vertexIndex) return new Edge(triangleIndex, 0);
                if (triangle.Vertex1 == vertexIndex) return new Edge(triangleIndex, 1);
                if (triangle.Vertex2 == vertexIndex) return new Edge(triangleIndex, 2);
            }
            return Edge.Invalid;
        }

        /// <summary>
        /// Returns an edge's linked edge on the neighboring triangle.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Edge GetLinkedEdge(Edge edge) => edge.IsValid ? Triangles[edge.TriangleIndex].GetLink(edge.EdgeIndex) : edge;

        /// <summary>
        /// Returns true if the edge's triangle has a higher index than its linked triangle.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IsPrimaryEdge(Edge edge) => edge.TriangleIndex > GetLinkedEdge(edge).TriangleIndex;

        public Edge GetFirstPrimaryEdge()
        {
            foreach (int triangleIndex in Triangles.Indices)
            {
                for (int vi = 0; vi < 3; ++vi)
                {
                    var edge = new Edge(triangleIndex, vi);
                    if (IsPrimaryEdge(edge))
                    {
                        return edge;
                    }
                }
            }
            return Edge.Invalid;
        }

        public Edge GetNextPrimaryEdge(Edge edge)
        {
            while (edge.EdgeIndex < 2)
            {
                edge = edge.Next;
                if (IsPrimaryEdge(edge))
                {
                    return edge;
                }
            }
            for (int triangleIndex = Triangles.GetNextIndex(edge.TriangleIndex); triangleIndex != -1; triangleIndex = Triangles.GetNextIndex(triangleIndex))
            {
                for (int vi = 0; vi < 3; ++vi)
                {
                    edge = new Edge(triangleIndex, vi);
                    if (IsPrimaryEdge(edge))
                    {
                        return edge;
                    }
                }
            }
            return Edge.Invalid;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int StartVertex(Edge edge) => Triangles[edge.TriangleIndex].GetVertex(edge.EdgeIndex);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int EndVertex(Edge edge) => StartVertex(edge.Next);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int ApexVertex(Edge edge) => StartVertex(edge.Prev);


        /// <summary>
        /// Returns (the first edge of) the first face.
        /// </summary>
        public FaceEdge GetFirstFace()
        {
            return NumFaces > 0 ? GetFirstFace(0) : FaceEdge.Invalid;
        }

        /// <summary>
        /// Returns the first face edge from a given face index.
        /// </summary>
        public FaceEdge GetFirstFace(int faceIndex)
        {
            foreach (int triangleIndex in Triangles.Indices)
            {
                if (Triangles[triangleIndex].FaceIndex != faceIndex)
                {
                    continue;
                }
                for (int i = 0; i < 3; ++i)
                {
                    var edge = new Edge(triangleIndex, i);
                    if (Triangles[GetLinkedEdge(edge).TriangleIndex].FaceIndex != faceIndex)
                    {
                        return new FaceEdge { Start = edge, Current = edge };
                    }
                }
            }
            return FaceEdge.Invalid;
        }

        /// <summary>
        /// Returns (the first edge of) the next face.
        /// </summary>
        public FaceEdge GetNextFace(FaceEdge fe)
        {
            int faceIndex = fe.IsValid ? Triangles[fe.Start.TriangleIndex].FaceIndex + 1 : 0;
            if (faceIndex < NumFaces)
                return GetFirstFace(faceIndex);
            return FaceEdge.Invalid;
        }

        /// <summary>
        /// Returns the next edge within a face.
        /// </summary>
        public FaceEdge GetNextFaceEdge(FaceEdge fe)
        {
            int faceIndex = Triangles[fe.Start.TriangleIndex].FaceIndex;
            bool found = false;
            fe.Current = fe.Current.Next;
            for (int n = Vertices[StartVertex(fe.Current)].Cardinality; n > 0; --n)
            {
                if (Triangles[GetLinkedEdge(fe.Current).TriangleIndex].FaceIndex == faceIndex)
                {
                    fe.Current = GetLinkedEdge(fe.Current).Next;
                }
                else
                {
                    found = true;
                    break;
                }
            }

            if (!found || fe.Current.Equals(fe.Start))
                return FaceEdge.Invalid;
            return fe;
        }

        #endregion

        #region Queries

        /// <summary>
        /// Compute vertex normal.
        /// </summary>
        [BurstDiscard]
        public float3 ComputeVertexNormal(int vertex)
        {
            return ComputeVertexNormal(GetVertexEdge(vertex));
        }

        /// <summary>
        /// Compute vertex normal.
        /// </summary>
        [BurstDiscard]
        public float3 ComputeVertexNormal(Edge edge)
        {
            float3 n = new float3(0);
            switch (Dimension)
            {
                case 2:
                    n = ProjectionPlane.Normal;
                    break;

                case 3:
                    float3 origin = Vertices[StartVertex(edge)].Position;
                    float3 dir0 = math.normalize(Vertices[EndVertex(edge)].Position - origin);
                    for (int c = Vertices[StartVertex(edge)].Cardinality, i = 0; i < c; ++i)
                    {
                        edge = GetLinkedEdge(edge).Next;
                        float3 dir1 = math.normalize(Vertices[EndVertex(edge)].Position - origin);
                        n += math.acos(math.dot(dir0, dir1)) * ComputePlane(edge.TriangleIndex).Normal;
                        dir0 = dir1;
                    }
                    n = math.normalize(n);
                    break;
            }
            return n;
        }

        /// <summary>
        /// Returns the AABB of the convex hull.
        /// </summary>
        public Aabb ComputeAabb()
        {
            Aabb aabb = Aabb.Empty;
            foreach (Vertex vertex in Vertices.Elements)
            {
                aabb.Include(vertex.Position);
            }
            return aabb;
        }

        /// <summary>
        /// Returns the supporting vertex index in a given direction.
        /// </summary>
        public int ComputeSupportingVertex(float3 direction)
        {
            int bi = 0;
            float maxDot = float.MinValue;
            foreach (int i in Vertices.Indices)
            {
                float d = math.dot(direction, Vertices[i].Position);
                if (d > maxDot)
                {
                    bi = i;
                    maxDot = d;
                }
            }
            return bi;
        }

        /// <summary>
        /// Returns the centroid of the convex hull.
        /// </summary>
        public float3 ComputeCentroid()
        {
            float4 sum = new float4(0);
            foreach (Vertex vertex in Vertices.Elements)
            {
                sum += new float4(vertex.Position, 1);
            }

            if (sum.w > 0)
                return sum.xyz / sum.w;
            return new float3(0);
        }

        /// <summary>
        /// Compute the mass properties of the convex hull.
        /// Note: Inertia computation adapted from S. Melax, http://www.melax.com/volint.
        /// </summary>        
        public unsafe MassProperties ComputeMassProperties()
        {
            var mp = new MassProperties();
            switch (Dimension)
            {
                case 0:
                    mp.CenterOfMass = Vertices[0].Position;
                    break;
                case 1:
                    mp.CenterOfMass = (Vertices[0].Position + Vertices[1].Position) * 0.5f;
                    break;
                case 2:
                {
                    float3 offset = ComputeCentroid();
                    for (int n = Vertices.PeakCount, i = n - 1, j = 0; j < n; i = j++)
                    {
                        float w = math.length(math.cross(Vertices[i].Position - offset, Vertices[j].Position - offset));
                        mp.CenterOfMass += (Vertices[i].Position + Vertices[j].Position + offset) * w;
                        mp.SurfaceArea += w;
                    }
                    mp.CenterOfMass /= mp.SurfaceArea * 3;
                    mp.InertiaTensor = float3x3.identity; // <todo>
                    mp.SurfaceArea *= 0.5f;
                }
                break;
                case 3:
                {
                    float3 offset = ComputeCentroid();
                    int numTriangles = 0;
                    float* dets = stackalloc float[Triangles.Capacity];
                    foreach (int i in Triangles.Indices)
                    {
                        float3 v0 = Vertices[Triangles[i].Vertex0].Position - offset;
                        float3 v1 = Vertices[Triangles[i].Vertex1].Position - offset;
                        float3 v2 = Vertices[Triangles[i].Vertex2].Position - offset;
                        float w = Det(v0, v1, v2);
                        mp.CenterOfMass += (v0 + v1 + v2) * w;
                        mp.Volume += w;
                        mp.SurfaceArea += math.length(math.cross(v1 - v0, v2 - v0));
                        dets[i] = w;
                        numTriangles++;
                    }

                    mp.CenterOfMass = mp.CenterOfMass / (mp.Volume * 4) + offset;

                    var diag = new float3(0);
                    var offd = new float3(0);

                    foreach (int i in Triangles.Indices)
                    {
                        float3 v0 = Vertices[Triangles[i].Vertex0].Position - mp.CenterOfMass;
                        float3 v1 = Vertices[Triangles[i].Vertex1].Position - mp.CenterOfMass;
                        float3 v2 = Vertices[Triangles[i].Vertex2].Position - mp.CenterOfMass;
                        diag += (v0 * v1 + v1 * v2 + v2 * v0 + v0 * v0 + v1 * v1 + v2 * v2) * dets[i];
                        offd += (v0.yzx * v1.zxy + v1.yzx * v2.zxy + v2.yzx * v0.zxy +
                                v0.yzx * v2.zxy + v1.yzx * v0.zxy + v2.yzx * v1.zxy +
                                (v0.yzx * v0.zxy + v1.yzx * v1.zxy + v2.yzx * v2.zxy) * 2) * dets[i];
                        numTriangles++;
                    }

                    diag /= mp.Volume * (60 / 6);
                    offd /= mp.Volume * (120 / 6);

                    mp.InertiaTensor.c0 = new float3(diag.y + diag.z, -offd.z, -offd.y);
                    mp.InertiaTensor.c1 = new float3(-offd.z, diag.x + diag.z, -offd.x);
                    mp.InertiaTensor.c2 = new float3(-offd.y, -offd.x, diag.x + diag.y);

                    mp.SurfaceArea /= 2;
                    mp.Volume /= 6;
                }
                break;
            }

            return mp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Plane ComputePlane(int vertex0, int vertex1, int vertex2, bool fromIntCoordinates)
        {
            float3 o, a, b;
            if (fromIntCoordinates)
            {
                o = m_IntegerSpace.ToFloatSpace(Vertices[vertex0].IntPosition);
                a = m_IntegerSpace.ToFloatSpace(Vertices[vertex1].IntPosition) - o;
                b = m_IntegerSpace.ToFloatSpace(Vertices[vertex2].IntPosition) - o;
            }
            else
            {
                o = Vertices[vertex0].Position;
                a = Vertices[vertex1].Position - o;
                b = Vertices[vertex2].Position - o;
            }
            float3 n = math.normalize(math.cross(a, b));
            return new Plane(n, -math.dot(n, o));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane ComputePlane(int triangleIndex, bool fromIntCoordinates = true)
        {
            return ComputePlane(Triangles[triangleIndex].Vertex0, Triangles[triangleIndex].Vertex1, Triangles[triangleIndex].Vertex2, fromIntCoordinates);
        }

        #endregion

        #region Helpers

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long Det64(int3 a, int3 b, int3 c, int3 d)
        {
            long ax = b.x - a.x, ay = b.y - a.y, az = b.z - a.z;
            long bx = c.x - a.x, by = c.y - a.y, bz = c.z - a.z;
            long cx = d.x - a.x, cy = d.y - a.y, cz = d.z - a.z;
            long kx = ay * bz - az * by, ky = az * bx - ax * bz, kz = ax * by - ay * bx;
            return kx * cx + ky * cy + kz * cz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long Det64(int a, int b, int c, int d)
        {
            return Det64(Vertices[a].IntPosition, Vertices[b].IntPosition, Vertices[c].IntPosition, Vertices[d].IntPosition);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long Det64(int a, int b, int c, int3 d)
        {
            return Det64(Vertices[a].IntPosition, Vertices[b].IntPosition, Vertices[c].IntPosition, d);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IsTriangleAreaZero(int3 a, int3 b, int3 c)
        {
            long x0 = a.x - b.x, y0 = a.y - b.y, z0 = a.z - b.z;
            long x1 = a.x - c.x, y1 = a.y - c.y, z1 = a.z - c.z;
            long cx = x1 * y0 - x0 * y1, cy = x1 * z0 - x0 * z1, cz = y1 * z0 - y0 * z1;
            return cx == 0 && cy == 0 && cz == 0;
        }

        #endregion
    }


    // Extensions, not part of the core functionality (yet)
    public static class ConvexHullBuilderExtensions
    {
        /// <summary>
        /// For 3D convex hulls, vertex indices are not always continuous, this methods compact them.
        /// </summary>
        [BurstDiscard]
        public static unsafe void CompactVertices(this ConvexHullBuilder builder)
        {
            if (builder.Dimension == 3)
            {
                int* newIndices = stackalloc int[builder.Vertices.PeakCount];
                ConvexHullBuilder.Vertex* newVertices = stackalloc ConvexHullBuilder.Vertex[builder.Vertices.PeakCount];
                int numNewVertices = 0;
                bool needsCompacting = false;
                foreach (int vertexIndex in builder.Vertices.Indices)
                {
                    needsCompacting |= vertexIndex != numNewVertices;
                    newIndices[vertexIndex] = numNewVertices;
                    newVertices[numNewVertices++] = builder.Vertices[vertexIndex];
                }

                if (needsCompacting)
                {
                    builder.Vertices.CopyFrom(newVertices, numNewVertices);

                    foreach (int triangleIndex in builder.Triangles.Indices)
                    {
                        ConvexHullBuilder.Triangle t = builder.Triangles[triangleIndex];
                        t.Vertex0 = newIndices[t.Vertex0];
                        t.Vertex1 = newIndices[t.Vertex1];
                        t.Vertex2 = newIndices[t.Vertex2];
                        builder.Triangles[triangleIndex] = t;
                    }
                }
            }
        }
    }
}
