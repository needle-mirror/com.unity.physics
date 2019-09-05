using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    static class DrawingUtility
    {
        // generate pairs of points for each edge
        public static void GetConvexHullEdges(ref ConvexHull hull, List<Vector3> edges)
        {
            edges.Clear();

            foreach (var face in hull.Faces)
            {
                for (int edge = 0, numFaceVertices = face.NumVertices; edge < numFaceVertices; ++edge)
                {
                    var fromIndex = hull.FaceVertexIndices[face.FirstIndex + edge];
                    var toIndex = hull.FaceVertexIndices[face.FirstIndex + (edge + 1) % face.NumVertices];

                    edges.Add(hull.Vertices[fromIndex]);
                    edges.Add(hull.Vertices[toIndex]);
                }
            }
        }

        // generate pairs of points for each edge
        public static void GetMeshEdges(ref Mesh mesh, List<Vector3> edges)
        {
            edges.Clear();

            for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
            {
                ref var section = ref mesh.Sections[sectionIndex];
                for (int primitive = 0, count = section.PrimitiveVertexIndices.Length; primitive < count; ++primitive)
                {
                    var vi = section.PrimitiveVertexIndices[primitive];
                    var v = new float3x4(
                        section.Vertices[vi.A],
                        section.Vertices[vi.B],
                        section.Vertices[vi.C],
                        section.Vertices[vi.D]
                    );

                    var flags = section.PrimitiveFlags[primitive];
                    if (flags.HasFlag(Mesh.PrimitiveFlags.IsQuad))
                    {
                        edges.Add(v[0]);
                        edges.Add(v[1]);
                        edges.Add(v[1]);
                        edges.Add(v[2]);
                        edges.Add(v[2]);
                        edges.Add(v[3]);
                        edges.Add(v[3]);
                        edges.Add(v[0]);
                    }
                    else if (flags.HasFlag(Mesh.PrimitiveFlags.IsTrianglePair))
                    {
                        edges.Add(v[0]);
                        edges.Add(v[1]);
                        edges.Add(v[1]);
                        edges.Add(v[2]);
                        edges.Add(v[2]);
                        edges.Add(v[3]);
                        edges.Add(v[3]);
                        edges.Add(v[0]);
                        // shared edge
                        edges.Add(v[0]);
                        edges.Add(v[2]);
                    }
                    else
                    {
                        edges.Add(v[0]);
                        edges.Add(v[1]);
                        edges.Add(v[1]);
                        edges.Add(v[2]);
                        edges.Add(v[2]);
                        edges.Add(v[0]);
                    }
                }
            }
        }
    }
}
