using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using static Unity.Physics.Math;
using Debug = UnityEngine.Debug;

namespace Unity.Physics.Tests.Collision.Queries
{
    public unsafe class ConvexConvexDistanceTest : MonoBehaviour
    {
        public enum DistributionType
        {
            Box,
            Sphere
        }

        public struct Face
        {
            public Plane Plane;
            public int FirstVertex;
            public int NumVertices;
        }

        public struct VertexFaces
        {
            public int FirstFace;
            public int NumFaces;
        }

        public struct HullFaceData
        {
            public Face[] Faces;
            public int[] FaceVertices;

            public VertexFaces[] VertexFaces;
            public int[] FaceIndices;
        }

        /// <summary>
        /// KISS Based PRNG (http://www.cs.ucl.ac.uk/staff/d.jones/GoodPracticeRNG.pdf)
        /// </summary>
        public struct PseudoRandomNumberGenerator
        {
            public PseudoRandomNumberGenerator(UInt32 seed)
            {
                m_x = 123456789;
                m_y = seed == 0 ? 234567891 : seed;
                m_z = 345678912;
                m_w = 456789123;
                m_c = 0;
            }

            public UInt32 NextUInt32()
            {
                m_y ^= (m_y << 5);
                m_y ^= (m_y >> 7);
                m_y ^= (m_y << 22);
                Int32 t = (Int32)(m_z + m_w + m_c);
                m_z = m_w;
                m_c = (UInt32)(t < 0 ? 1 : 0);
                m_w = (UInt32)(t & 2147483647);
                m_x += 1411392427;
                return m_x + m_y + m_w;
            }

            public float NextFloat()
            {
                return NextUInt32() * (1.0f / 4294967296.0f);
            }

            public float NextFloat(float center, float halfExtent)
            {
                return (NextFloat() * 2 - 1) * halfExtent + center;
            }

            public float2 NextFloat2(float center = 0, float halfExtent = 1)
            {
                return new float2(NextFloat(center, halfExtent), NextFloat(center, halfExtent));
            }

            public float3 NextFloat3(float center = 0, float halfExtent = 1)
            {
                return new float3(NextFloat2(center, halfExtent), NextFloat(center, halfExtent));
            }

            public float4 NextFloat4(float center = 0, float halfExtent = 1)
            {
                return new float4(NextFloat3(center, halfExtent), NextFloat(center, halfExtent));
            }

            UInt32 m_x, m_y, m_z, m_w, m_c;
        }

        public ConvexHullBuilder Hull = new ConvexHullBuilder(16384, 16384 * 2);
        public HullFaceData HullData;

        public ConvexHullBuilder.MassProperties MassProperties;
        public UnityEngine.Mesh SourceMesh = null;
        public bool UpdateMesh;
        public bool CollideOthers = false;
        internal ConvexConvexDistanceQueries.PenetrationHandling PenetrationHandling = ConvexConvexDistanceQueries.PenetrationHandling.Exact3D;
        public bool ShowCso = false;
        public bool ShowFaces = false;
        public bool ShowTriangles = false;
        public bool ShowVertexNormals = false;
        public bool ShowProjection = true;
        public bool ShowManifold = false;
        public bool ShowLabels = false;
        public bool Experimental = false;
        public bool TraceQueryResults = false;
        public float FaceSimplificationError = 1;
        public float FaceMinAngle = 0;
        public float VertexSimplificationError = 1;
        public float MaxFaceAngle = 0.1f;
        public int MinVertices = 0;
        public int MaxFaces = 16;
        public float VolumeConservation = 1;
        public float Offset = 0;
        public DistributionType Distribution = DistributionType.Box;
        public PseudoRandomNumberGenerator Prng = new PseudoRandomNumberGenerator(0);

        // Use this for initialization
        void Start()
        {
            GetComponent<MeshFilter>().sharedMesh = new UnityEngine.Mesh();
            UpdateMesh = true;
        }

        // Update is called once per frame
        void Update()
        {
#if UNITY_EDITOR
            HandleUtility.Repaint();
#endif
        }

        public void Reset()
        {
            Prng = new PseudoRandomNumberGenerator(0);
            UpdateMesh = false;
            Hull.Reset();
            HullData.Faces = null;
            HullData.VertexFaces = null;
            GetComponent<MeshFilter>().sharedMesh = new UnityEngine.Mesh();
        }

        public float3[] GetVertexArray()
        {
            var vtx = new List<float3>();
            foreach (var vertex in Hull.Vertices.Elements)
            {
                vtx.Add(vertex.Position);
            }
            if (vtx.Count == 0)
            {
                vtx.Add(float3.zero);
            }
            return vtx.ToArray();
        }

        public void AddRandomPoints(int count)
        {
            for (int i = 0; i < count; ++i)
            {
                var point = Prng.NextFloat3(0, 1);
                if (Distribution == DistributionType.Sphere)
                {
                    point = math.normalize(point);
                }
                Hull.AddPoint(point);
            }
            UpdateMesh = true;
        }

        public void Scale(float3 factors)
        {
            var points = new List<float3>();
            foreach (var vertex in Hull.Vertices.Elements)
            {
                points.Add(vertex.Position * factors);
            }
            Hull.Reset();
            foreach (var p in points)
            {
                Hull.AddPoint(p);
            }
            UpdateMesh = true;
        }

        public void SplitByPlane(Plane plane)
        {
            plane.Distance = -math.dot(plane.Normal, MassProperties.CenterOfMass);

            ConvexHullBuilder neg, pos;
            Hull.SplitByPlane(plane, out neg, out pos, 0);
            Hull.CopyFrom(pos);
            neg.Dispose();
            pos.Dispose();
            UpdateMesh = true;
        }

        private void UpdateMeshNow()
        {
            MassProperties = Hull.ComputeMassProperties();

            Hull.CompactVertices();
            Hull.BuildFaceIndices((float)(MaxFaceAngle * System.Math.PI / 180));

            {
                HullData.Faces = new Face[Hull.NumFaces];
                HullData.FaceVertices = new int[Hull.NumFaceVertices];
                int nextVertex = 0;
                for (var faceEdge = Hull.GetFirstFace(); faceEdge.IsValid; faceEdge = Hull.GetNextFace(faceEdge))
                {
                    var triangleIndex = ((ConvexHullBuilder.Edge)faceEdge).TriangleIndex;
                    Face newFace = new Face
                    {
                        Plane = Hull.ComputePlane(triangleIndex),
                        FirstVertex = nextVertex,
                        NumVertices = 0
                    };
                    for (var edge = faceEdge; edge.IsValid; edge = Hull.GetNextFaceEdge(edge))
                    {
                        HullData.FaceVertices[nextVertex++] = Hull.StartVertex(edge);
                    }
                    newFace.NumVertices = nextVertex - newFace.FirstVertex;
                    HullData.Faces[Hull.Triangles[triangleIndex].FaceIndex] = newFace;
                }

                var indices = new List<int>();
                var set = new List<int>();
                HullData.VertexFaces = new VertexFaces[Hull.Vertices.PeakCount];
                for (int i = 0; i < Hull.Vertices.PeakCount; ++i)
                {
                    var cardinality = Hull.Vertices[i].Cardinality;
                    var edge = Hull.GetVertexEdge(i);
                    for (int j = Hull.Vertices[i].Cardinality; j > 0; --j)
                    {
                        int faceIndex = Hull.Triangles[edge.TriangleIndex].FaceIndex;
                        if (set.IndexOf(faceIndex) == -1) set.Add(faceIndex);
                        edge = Hull.GetLinkedEdge(edge).Next;
                    }
                    set.Sort();
                    HullData.VertexFaces[i] = new VertexFaces { FirstFace = indices.Count, NumFaces = set.Count };
                    indices.AddRange(set);
                    set.Clear();
                }
                HullData.FaceIndices = indices.ToArray();
            }

            Vector3[] vertices = null;
            int[] triangles = null;
            switch (Hull.Dimension)
            {
                case 2:
                    vertices = new Vector3[Hull.Vertices.PeakCount];
                    triangles = new int[(Hull.Vertices.PeakCount - 2) * 2 * 3];
                    for (int i = 0; i < Hull.Vertices.PeakCount; ++i)
                    {
                        vertices[i] = Hull.Vertices[i].Position;
                    }
                    for (int i = 2; i < Hull.Vertices.PeakCount; ++i)
                    {
                        int j = (i - 2) * 6;
                        triangles[j + 0] = 0; triangles[j + 1] = i - 1; triangles[j + 2] = i;
                        triangles[j + 3] = 0; triangles[j + 4] = i; triangles[j + 5] = i - 1;
                    }
                    break;
                case 3:
                    vertices = new Vector3[Hull.Triangles.PeakCount * 3];
                    triangles = new int[Hull.Triangles.PeakCount * 3];
                    for (int i = 0; i < Hull.Triangles.PeakCount; ++i)
                    {
                        if (Hull.Triangles[i].IsAllocated)
                        {
                            vertices[i * 3 + 0] = Hull.Vertices[Hull.Triangles[i].Vertex0].Position;
                            vertices[i * 3 + 1] = Hull.Vertices[Hull.Triangles[i].Vertex1].Position;
                            vertices[i * 3 + 2] = Hull.Vertices[Hull.Triangles[i].Vertex2].Position;
                            triangles[i * 3 + 0] = i * 3 + 0;
                            triangles[i * 3 + 1] = i * 3 + 1;
                            triangles[i * 3 + 2] = i * 3 + 2;
                        }
                        else
                        {
                            triangles[i * 3 + 0] = 0;
                            triangles[i * 3 + 1] = 0;
                            triangles[i * 3 + 2] = 0;
                        }
                    }
                    break;
            }

            var mesh = GetComponent<MeshFilter>().sharedMesh;

            mesh.Clear();
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
        }

        static Face SelectBestFace(Face[] faces, float3 direction)
        {
            var bestDot = math.dot(faces[0].Plane.Normal, direction);
            var bestFace = 0;
            for (int i = 1; i < faces.Length; ++i)
            {
                var d = math.dot(faces[i].Plane.Normal, direction);
                if (d > bestDot)
                {
                    bestDot = d;
                    bestFace = i;
                }
            }
            return faces[bestFace];
        }

        static Face SelectBestFace(List<int> subset, Face[] faces, float3 direction)
        {
            var bestDot = math.dot(faces[subset[0]].Plane.Normal, direction);
            var bestFace = subset[0];
            for (int i = 1; i < subset.Count; ++i)
            {
                var d = math.dot(faces[subset[i]].Plane.Normal, direction);
                if (d > bestDot)
                {
                    bestDot = d;
                    bestFace = subset[i];
                }
            }
            return faces[bestFace];
        }

        static List<int> GetPerVertexFaces(int vertex, ref HullFaceData data)
        {
            var set = new List<int>();
            VertexFaces vf = data.VertexFaces[vertex];
            for (int j = 0; j < vf.NumFaces; ++j) set.Add(data.FaceIndices[vf.FirstFace + j]);
            return set;
        }

        static void DrawFace(MTransform trs, ref ConvexHullBuilder hull, Face face, int[] vertices, float offset, Color color)
        {
            Gizmos.color = color;
            var translation = face.Plane.Normal * offset;
            for (int i = face.NumVertices - 1, j = 0; j < face.NumVertices; i = j++)
            {
                var a = hull.Vertices[vertices[face.FirstVertex + i]].Position;
                var b = hull.Vertices[vertices[face.FirstVertex + j]].Position;
                a = Mul(trs, a + translation);
                b = Mul(trs, b + translation);
                Gizmos.DrawLine(a, b);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float DistanceToPlaneEps(float4 plane, float3 point, float eps)
        {
            var d2P = math.dot(plane, new float4(point, 1));
            if ((d2P * d2P) <= eps)
                return 0;
            return d2P;
        }

        static void DrawManifold(bool useExerimentalMethod, ConvexConvexDistanceQueries.Result distance,
                          MTransform trsA, ref ConvexHullBuilder hullA, ref HullFaceData dataA,
                            MTransform trsB, ref ConvexHullBuilder hullB, ref HullFaceData dataB)
        {
            MTransform btoA = Mul(Inverse(trsA), trsB);
            MTransform atoB = Mul(Inverse(trsB), trsA);

            Face faceA = SelectBestFace(dataA.Faces, distance.ClosestPoints.NormalInA);
            Face faceB = SelectBestFace(dataB.Faces, math.mul(atoB.Rotation, -distance.ClosestPoints.NormalInA));
            if (useExerimentalMethod)
            {
                var legacyFaceA = faceA;
                var legacyFaceB = faceB;

                // Experimental method.

                // Extract faces sub-set of A.
                {
                    float bestDot = -2;
                    var normal = distance.ClosestPoints.NormalInA;
                    for (int i = 0; i < distance.SimplexDimension; ++i)
                    {
                        int vi = distance.SimplexVertexA(i);
                        for (int j = 0; j < dataA.VertexFaces[vi].NumFaces; ++j)
                        {
                            var k = dataA.FaceIndices[dataA.VertexFaces[vi].FirstFace + j];
                            var face = dataA.Faces[k];
                            var d = math.dot(face.Plane.Normal, normal);
                            if (d > bestDot)
                            {
                                faceA = face;
                                bestDot = d;
                            }
                        }
                    }
                }

                // Extract faces sub-set of B.
                {
                    float bestDot = -2;
                    var normal = math.mul(atoB.Rotation, -distance.ClosestPoints.NormalInA);
                    for (int i = 0; i < distance.SimplexDimension; ++i)
                    {
                        int vi = distance.SimplexVertexB(i);
                        for (int j = 0; j < dataB.VertexFaces[vi].NumFaces; ++j)
                        {
                            var k = dataB.FaceIndices[dataB.VertexFaces[vi].FirstFace + j];
                            var face = dataB.Faces[k];
                            var d = math.dot(face.Plane.Normal, normal);
                            if (d > bestDot)
                            {
                                faceB = face;
                                bestDot = d;
                            }
                        }
                    }
                }

                if (legacyFaceA.FirstVertex != faceA.FirstVertex || legacyFaceB.FirstVertex != faceB.FirstVertex)
                {
                    Debug.LogError("Different face set found.");
                }
            }

            var facePlaneAinA = faceA.Plane;
            var facePlaneBinA = TransformPlane(btoA, faceB.Plane);

            /*drawFace(trsA, ref hullA, faceA, dataA.faceVertices, 0.01f, Color.yellow);
            drawFace(trsB, ref hullB, faceB, dataB.faceVertices, 0.01f, Color.yellow);*/

            const float eps = 1e-6f;
            var va = new List<float3>();
            var vb = new List<float3>();
            for (int i = 0; i < faceA.NumVertices; ++i)
            {
                va.Add(hullA.Vertices[dataA.FaceVertices[faceA.FirstVertex + i]].Position);
            }
            for (int i = 0; i < faceB.NumVertices; ++i)
            {
                vb.Add(Mul(btoA, hullB.Vertices[dataB.FaceVertices[faceB.FirstVertex + i]].Position));
            }

            var vt = new List<float3>();
            for (int ai = va.Count - 1, aj = 0; aj < va.Count; ai = aj++)
            {
                var plane = new float4(math.normalize(math.cross(distance.ClosestPoints.NormalInA, va[ai] - va[aj])), 0);
                plane.w = -math.dot(plane.xyz, va[ai]);

                if (vb.Count > 1)
                {
                    int bi = vb.Count - 1;
                    float d2Pi = DistanceToPlaneEps(plane, vb[bi], eps);
                    for (int bj = 0; bj < vb.Count; bi = bj++)
                    {
                        var d2Pj = DistanceToPlaneEps(plane, vb[bj], eps);
                        if ((d2Pi * d2Pj) < 0)
                        {
                            float isec = d2Pi / (d2Pi - d2Pj);
                            vt.Add(vb[bi] + (vb[bj] - vb[bi]) * isec);
                        }

                        if (d2Pj <= 0) vt.Add(vb[bj]);

                        d2Pi = d2Pj;
                    }
                }

                var temp = vb;
                vb = vt;
                vt = temp;
                vt.Clear();
            }

            if (vb.Count == 0)
            {
                vb.Add(distance.ClosestPoints.PositionOnBinA);
            }

            {
                GUIStyle labelStyle = new GUIStyle();
                labelStyle.fontSize = 16;

                var projectionInvDen = 1 / math.dot(distance.ClosestPoints.NormalInA, facePlaneAinA.Normal);

                int i = vb.Count - 1;
                var piOnB = vb[i];
                var piD = math.dot(facePlaneAinA, new float4(piOnB, 1)) * projectionInvDen;
                var piOnA = piOnB - distance.ClosestPoints.NormalInA * piD;
                for (int j = 0; j < vb.Count; i = j++)
                {
#if UNITY_EDITOR
                    var center = Mul(trsA, (piOnA + piOnB) / 2);
                    Handles.Label(center, $"{piD}", labelStyle);
#endif
                    var pjOnB = vb[j];
                    var pjD = math.dot(facePlaneAinA, new float4(pjOnB, 1)) * projectionInvDen;
                    var pjOnA = pjOnB - distance.ClosestPoints.NormalInA * pjD;

                    Gizmos.DrawLine(Mul(trsA, piOnB), Mul(trsA, pjOnB));
                    Gizmos.DrawLine(Mul(trsA, piOnA), Mul(trsA, pjOnA));

                    Gizmos.DrawLine(Mul(trsA, pjOnB), Mul(trsA, pjOnA));

                    piOnB = pjOnB;
                    piOnA = pjOnA;
                    piD = pjD;
                }
            }
        }

        void OnDrawGizmos()
        {
            {
                var s = 0.25f;
                var com = MassProperties.CenterOfMass;
                Gizmos.color = Color.white;
                Gizmos.DrawLine(transform.TransformPoint(com + new float3(-s, 0, 0)), transform.TransformPoint(com + new float3(+s, 0, 0)));
                Gizmos.DrawLine(transform.TransformPoint(com + new float3(0, -s, 0)), transform.TransformPoint(com + new float3(0, +s, 0)));
                Gizmos.DrawLine(transform.TransformPoint(com + new float3(0, 0, -s)), transform.TransformPoint(com + new float3(0, 0, +s)));
            }

            if (UpdateMesh)
            {
                UpdateMesh = false;
                UpdateMeshNow();
            }

            // Display faces.
            if (ShowFaces && HullData.Faces != null)
            {
                MTransform trs = new MTransform(transform.rotation, transform.position);

                Gizmos.color = Color.white;
                foreach (Face face in HullData.Faces)
                {
                    var offset = face.Plane.Normal * 0.0001f;
                    for (int i = face.NumVertices - 1, j = 0; j < face.NumVertices; i = j++)
                    {
                        var a = Hull.Vertices[HullData.FaceVertices[face.FirstVertex + i]].Position;
                        var b = Hull.Vertices[HullData.FaceVertices[face.FirstVertex + j]].Position;
                        a = Mul(trs, a + offset);
                        b = Mul(trs, b + offset);
                        Gizmos.DrawLine(a, b);
                    }
                }
            }

            // Display triangles.
            if (ShowTriangles)
            {
                MTransform trs = new MTransform(transform.rotation, transform.position);
                Gizmos.color = Color.white;
                for (int i = Hull.Triangles.GetFirstIndex(); i != -1; i = Hull.Triangles.GetNextIndex(i))
                {
                    var a = Mul(trs, Hull.Vertices[Hull.Triangles[i].Vertex0].Position);
                    var b = Mul(trs, Hull.Vertices[Hull.Triangles[i].Vertex1].Position);
                    var c = Mul(trs, Hull.Vertices[Hull.Triangles[i].Vertex2].Position);
                    Gizmos.DrawLine(a, b);
                    Gizmos.DrawLine(b, c);
                    Gizmos.DrawLine(c, a);
                }
            }

            // Display vertex normals.
            if (ShowVertexNormals)
            {
                MTransform trs = new MTransform(transform.rotation, transform.position);
                Gizmos.color = Color.white;
                for (var vertex = Hull.Triangles.GetFirstIndex(); vertex != -1; vertex = Hull.Triangles.GetNextIndex(vertex))
                {
                    var normal = math.mul(trs.Rotation, Hull.ComputeVertexNormal(vertex)) * 0.25f;
                    var start = Mul(trs, Hull.Vertices[vertex].Position);
                    Gizmos.DrawRay(start, normal);
                }
            }

            // Display labels.
            if (ShowLabels)
            {
            }

            // Compute distance to every other convex hulls.
            if (CollideOthers)
            {
                var thisId = GetInstanceID();
                var cvxs = FindObjectsOfType<ConvexConvexDistanceTest>();

                MTransform transformA = new MTransform(transform.rotation, transform.position);
                float3[] verticesA = GetVertexArray();
                foreach (var cvx in cvxs)
                {
                    if (cvx.GetInstanceID() == thisId) continue;

                    MTransform transformB = new MTransform(cvx.transform.rotation, cvx.transform.position);
                    float3[] verticesB = cvx.GetVertexArray();

                    MTransform btoA = Mul(Inverse(transformA), transformB);

                    ConvexConvexDistanceQueries.Result result;
                    fixed (float3* va = verticesA)
                    {
                        fixed (float3* vb = verticesB)
                        {
                            result = ConvexConvexDistanceQueries.ConvexConvex(va, verticesA.Length, vb, verticesB.Length, btoA, PenetrationHandling);
                        }
                    }

                    var from = Mul(transformA, result.ClosestPoints.PositionOnAinA);
                    var to = Mul(transformA, result.ClosestPoints.PositionOnBinA);

                    if (TraceQueryResults)
                    {
                        Debug.Log($"Iterations={result.Iterations}, plane={result.ClosestPoints.NormalInA}, distance={result.ClosestPoints.Distance}");
                        Debug.Log($"Features A = [{result.Simplex[0] >> 16}, {result.Simplex[1] >> 16}, {result.Simplex[2] >> 16}]");
                        Debug.Log($"Features B = [{result.Simplex[0] & 0xffff}, {result.Simplex[1] & 0xffff}, {result.Simplex[2] & 0xffff}]");
                    }

                    if (ShowManifold && Hull.Dimension == 3 && cvx.Hull.Dimension == 3)
                    {
                        DrawManifold(Experimental, result,
                            transformA, ref Hull, ref HullData,
                            transformB, ref cvx.Hull, ref cvx.HullData);
                    }

                    if (ShowProjection)
                    {
                        Gizmos.color = Color.white;
                        var trs = Mul(transformA, new MTransform(float3x3.identity, result.ClosestPoints.NormalInA * result.ClosestPoints.Distance));
                        {
                            var tv = stackalloc float3[Hull.Vertices.PeakCount];
                            for (var vertex = Hull.Vertices.GetFirstIndex(); vertex != -1; vertex = Hull.Vertices.GetNextIndex(vertex))
                            {
                                tv[vertex] = Mul(trs, Hull.Vertices[vertex].Position);
                            }

                            if (Hull.Dimension == 3)
                            {
                                for (var edge = Hull.GetFirstPrimaryEdge(); edge.IsValid; edge = Hull.GetNextPrimaryEdge(edge))
                                {
                                    Gizmos.DrawLine(tv[Hull.StartVertex(edge)], tv[Hull.EndVertex(edge)]);
                                }
                            }
                            else if (Hull.Dimension >= 1)
                            {
                                for (int i = Hull.Vertices.PeakCount - 1, j = 0; j < Hull.Vertices.PeakCount; i = j++)
                                {
                                    Gizmos.DrawLine(tv[i], tv[j]);
                                }
                            }
                        }
                    }

                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(from, 0.05f);

                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(to, 0.05f);

                    Gizmos.color = Color.white;
                    Gizmos.DrawLine(from, to);

                    if (ShowCso)
                    {
                        Gizmos.color = Color.yellow;

                        using (var cso = new ConvexHullBuilder(8192, 8192 * 2))
                        {
                            for (int i = 0; i < verticesA.Length; ++i)
                            {
                                for (int j = 0; j < verticesB.Length; ++j)
                                {
                                    cso.AddPoint(verticesA[i] - Mul(btoA, verticesB[j]));
                                }
                            }
                            if (cso.Dimension == 2)
                            {
                                for (int n = cso.Vertices.PeakCount, i = n - 1, j = 0; j < n; i = j++)
                                {
                                    Gizmos.DrawLine(cso.Vertices[i].Position, cso.Vertices[j].Position);
                                }
                            }
                            else if (cso.Dimension == 3)
                            {
                                foreach (var triangle in cso.Triangles.Elements)
                                {
                                    Gizmos.DrawLine(cso.Vertices[triangle.Vertex0].Position, cso.Vertices[triangle.Vertex1].Position);
                                    Gizmos.DrawLine(cso.Vertices[triangle.Vertex1].Position, cso.Vertices[triangle.Vertex2].Position);
                                    Gizmos.DrawLine(cso.Vertices[triangle.Vertex2].Position, cso.Vertices[triangle.Vertex0].Position);
                                }
                            }
                            Gizmos.DrawLine(new float3(-0.1f, 0, 0), new float3(+0.1f, 0, 0));
                            Gizmos.DrawLine(new float3(0, -0.1f, 0), new float3(0, +0.1f, 0));
                            Gizmos.DrawLine(new float3(0, 0, -0.1f), new float3(0, 0, +0.1f));
                        }
                    }
                }
            }

            // Draw vertices.
#if UNITY_EDITOR
            GUIStyle labelStyle = new GUIStyle();
            labelStyle.fontSize = 24;
#endif

            Gizmos.color = Color.yellow;
            for (int i = Hull.Vertices.GetFirstIndex(); i != -1; i = Hull.Vertices.GetNextIndex(i))
            {
                var w = transform.TransformPoint(Hull.Vertices[i].Position);
#if UNITY_EDITOR
                if (ShowLabels)
                {
                    Handles.color = Color.white;
                    Handles.Label(w, $"{i}:{Hull.Vertices[i].Cardinality}", labelStyle);
                }
                else
#endif
                {
                    Gizmos.DrawSphere(w, 0.01f);
                }
            }
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(ConvexConvexDistanceTest))]
    public class ConvexTestEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            ConvexConvexDistanceTest cvx = (ConvexConvexDistanceTest)target;


            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Reset")) cvx.Reset();
            if (GUILayout.Button("Rebuild faces")) cvx.UpdateMesh = true;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Simplify vertices"))
            {
                cvx.Hull.SimplifyVertices(cvx.VertexSimplificationError, cvx.MinVertices, cvx.VolumeConservation);
                cvx.UpdateMesh = true;
            }
            if (GUILayout.Button("Simplify faces"))
            {
                cvx.Hull.SimplifyFaces(cvx.FaceSimplificationError, cvx.MaxFaces, int.MaxValue, cvx.FaceMinAngle);
                cvx.UpdateMesh = true;
            }
            if (GUILayout.Button("Offset vertices"))
            {
                cvx.Hull.OffsetVertices(cvx.Offset);
                cvx.UpdateMesh = true;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("X*2")) cvx.Scale(new float3(2, 1, 1));
            if (GUILayout.Button("Y*2")) cvx.Scale(new float3(1, 2, 1));
            if (GUILayout.Button("Z*2")) cvx.Scale(new float3(1, 1, 2));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("X/2")) cvx.Scale(new float3(0.5f, 1, 1));
            if (GUILayout.Button("Y/2")) cvx.Scale(new float3(1, 0.5f, 1));
            if (GUILayout.Button("Z/2")) cvx.Scale(new float3(1, 1, 0.5f));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Cut X-")) cvx.SplitByPlane(new Plane(new float3(1, 0, 0), 0));
            if (GUILayout.Button("Cut Y-")) cvx.SplitByPlane(new Plane(new float3(0, 1, 0), 0));
            if (GUILayout.Button("Cut Z-")) cvx.SplitByPlane(new Plane(new float3(0, 0, 1), 0));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Cut X+")) cvx.SplitByPlane(new Plane(new float3(-1, 0, 0), 0));
            if (GUILayout.Button("Cut Y+")) cvx.SplitByPlane(new Plane(new float3(0, -1, 0), 0));
            if (GUILayout.Button("Cut Z+")) cvx.SplitByPlane(new Plane(new float3(0, 0, -1), 0));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("+1 point")) cvx.AddRandomPoints(1);
            if (GUILayout.Button("+10 point")) cvx.AddRandomPoints(10);
            if (GUILayout.Button("+100 point")) cvx.AddRandomPoints(100);
            if (GUILayout.Button("+1000 point")) cvx.AddRandomPoints(1000);
            if (GUILayout.Button("Debug insert"))
            {
                cvx.Hull.AddPoint(new float3(0, 1, 0), 16384);
                cvx.UpdateMesh = true;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (cvx.SourceMesh != null && GUILayout.Button("Mesh"))
            {
                cvx.Reset();
                var vertices = cvx.SourceMesh.vertices;
                var sw = new Stopwatch();
                sw.Start();

                Aabb aabb = Aabb.Empty;
                for (int i = 0; i < vertices.Length; ++i)
                {
                    aabb.Include(vertices[i]);
                }
                cvx.Hull.IntegerSpaceAabb = aabb;

                for (int i = 0; i < vertices.Length; ++i)
                {
                    cvx.Hull.AddPoint(vertices[i]);
                }
                Debug.Log($"Build time {sw.ElapsedMilliseconds} ms");
                cvx.UpdateMesh = true;
            }
            if (GUILayout.Button("Box"))
            {
                cvx.Reset();
                cvx.Hull.AddPoint(new float3(-1, -1, -1));
                cvx.Hull.AddPoint(new float3(+1, -1, -1));
                cvx.Hull.AddPoint(new float3(+1, +1, -1));
                cvx.Hull.AddPoint(new float3(-1, +1, -1));
                cvx.Hull.AddPoint(new float3(-1, -1, +1));
                cvx.Hull.AddPoint(new float3(+1, -1, +1));
                cvx.Hull.AddPoint(new float3(+1, +1, +1));
                cvx.Hull.AddPoint(new float3(-1, +1, +1));
                cvx.UpdateMesh = true;
            }
            if (GUILayout.Button("Cylinder"))
            {
                cvx.Reset();
                var pi2 = math.acos(0) * 4;
                for (int i = 0, n = 32; i < n; ++i)
                {
                    var angle = pi2 * i / n;
                    var xy = new float2(math.sin(angle), math.cos(angle));
                    cvx.Hull.AddPoint(new float3(xy, -1));
                    cvx.Hull.AddPoint(new float3(xy, +1));
                }
                cvx.UpdateMesh = true;
            }

            if (GUILayout.Button("Cone"))
            {
                cvx.Reset();
                var pi2 = math.acos(0) * 4;
                for (int i = 0, n = 32; i < n; ++i)
                {
                    var angle = pi2 * i / n;
                    var xy = new float2(math.sin(angle), math.cos(angle));
                    cvx.Hull.AddPoint(new float3(xy, 0));
                }
                cvx.Hull.AddPoint(new float3(0, 0, 1));
                cvx.UpdateMesh = true;
            }

            if (GUILayout.Button("Circle"))
            {
                cvx.Reset();
                var pi2 = math.acos(0) * 4;
                for (int i = 0, n = 32; i < n; ++i)
                {
                    var angle = pi2 * i / n;
                    var xy = new float2(math.sin(angle), math.cos(angle));
                    cvx.Hull.AddPoint(new float3(xy, 0));
                }
                cvx.UpdateMesh = true;
            }
            GUILayout.EndHorizontal();

            SceneView.RepaintAll();
        }
    }
#endif
}
