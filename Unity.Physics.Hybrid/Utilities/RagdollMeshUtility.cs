using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using static Unity.Physics.Math;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// The purpose of this Utility is to help gather the meshes needed to build a Ragdoll, or
    /// compound shape. It is called by the SceneCreationSystem.cs in the PhysicsSamples when calling
    /// CreateBody. Each mesh returned by these methods may be appended to a List of meshes to create
    /// a larger body.
    /// It is also used by debug display methods
    /// </summary>
    internal class AppendMeshColliders
    {
        public unsafe class GetMeshes : MonoBehaviour
        {
            public static UnityEngine.Mesh ReferenceSphere =>
                GetReferenceMesh(ref _cachedReferenceSphere, PrimitiveType.Sphere, false);
            public static UnityEngine.Mesh ReferenceSimpleSphere =>
                GetReferenceMesh(ref _cachedReferenceSimpleSphere, PrimitiveType.Sphere, true);
            public static UnityEngine.Mesh ReferenceCylinder =>
                GetReferenceMesh(ref _cachedReferenceCylinder, PrimitiveType.Cylinder, false);
            public static UnityEngine.Mesh ReferenceCapsule =>
                GetReferenceMesh(ref _cachedReferenceCapsule, PrimitiveType.Capsule, false);
            public static UnityEngine.Mesh ReferenceCube =>
                GetReferenceMesh(ref _cachedReferenceCube, PrimitiveType.Cube, false);

            private static UnityEngine.Mesh _cachedReferenceSphere;
            private static UnityEngine.Mesh _cachedReferenceSimpleSphere;
            private static UnityEngine.Mesh _cachedReferenceCylinder;
            private static UnityEngine.Mesh _cachedReferenceCapsule;
            private static UnityEngine.Mesh _cachedReferenceCube;

            private static UnityEngine.Mesh GetReferenceMesh(ref UnityEngine.Mesh cache, PrimitiveType type, bool isSimple)
            {
                if (cache == null)
                {
                    cache = CreateReferenceMesh(type, isSimple);
                }

                return cache;
            }

            internal static void ClearReferenceMeshes()
            {
                _cachedReferenceSphere = null;
                _cachedReferenceSimpleSphere = null;
                _cachedReferenceCylinder = null;
                _cachedReferenceCapsule = null;
                _cachedReferenceCube = null;
            }

            private static UnityEngine.Mesh CreateReferenceMesh(PrimitiveType type, bool isSimple)
            {
                UnityEngine.Mesh refMesh;
                switch (type)
                {
                    case PrimitiveType.Cylinder:
                        refMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");
                        break;

                    case PrimitiveType.Capsule:
                        refMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Capsule.fbx");
                        break;

                    case PrimitiveType.Cube:
                        refMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("Cube.fbx");
                        break;

                    case PrimitiveType.Sphere:
                        refMesh = (isSimple) ?
                            Resources.GetBuiltinResource<UnityEngine.Mesh>("icosahedron.fbx") :
                            Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Sphere.fbx");

                        //refMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("icosphere.fbx"); //intermediate
                        break;
                    default:
                        throw new NotImplementedException($"Unable to create reference mesh of type {type}");
                }

                if (refMesh == null)
                    throw new NotImplementedException($"Unable to create reference mesh of type {type}");

                return refMesh;
            }

            // Combination mesh+scale, to enable sharing spheres
            public class DisplayResult
            {
                public UnityEngine.Mesh Mesh;
                public Vector3 Scale;
                public Vector3 Position;
                public Quaternion Orientation;

                [Preserve] public float4x4 Transform => float4x4.TRS(Position, Orientation, Scale);
            }

            internal static float3 ComputeHullCentroid(ref ConvexHull hull)
            {
                float3 centroid = float3.zero;

                for (int i = 0; i < hull.NumVertices; i++)
                {
                    centroid += hull.Vertices[i];
                }

                centroid /= hull.NumVertices;

                return centroid;
            }

            private static void AppendConvex(ref List<DisplayResult> results, ref ConvexHull hull, RigidTransform worldFromCollider, float uniformScale = 1)
            {
                int totalNumVertices = 0;
                for (int f = 0; f < hull.NumFaces; f++)
                {
                    totalNumVertices += hull.Faces[f].NumVertices + 1;
                }

                Vector3[] vertices = new Vector3[totalNumVertices];
                Vector3[] normals = new Vector3[totalNumVertices];
                int[] triangles = new int[(totalNumVertices - hull.NumFaces) * 3];

                // Calculate centroid to approximately render convex radius effect
                Vector3 hullCentroid = ComputeHullCentroid(ref hull);

                int startVertexIndex = 0;
                int curTri = 0;

                for (int f = 0; f < hull.NumFaces; f++)
                {
                    Vector3 avgFace = Vector3.zero;
                    Vector3 faceNormal = hull.Planes[f].Normal;

                    for (int fv = 0; fv < hull.Faces[f].NumVertices; fv++)
                    {
                        int origVIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
                        Vector3 origV = hull.Vertices[origVIndex];

                        // find the direction from centroid to the vertex
                        Vector3 dir = origV - hullCentroid;
                        Vector3 dirNormalized = math.normalize(dir);

                        Vector3 vertexWithConvexRadius = origV + dirNormalized * hull.ConvexRadius;

                        vertices[startVertexIndex + fv] = vertexWithConvexRadius;
                        normals[startVertexIndex + fv] = faceNormal;

                        avgFace += vertexWithConvexRadius;

                        triangles[curTri * 3 + 0] = startVertexIndex + fv;
                        triangles[curTri * 3 + 1] = startVertexIndex + (fv + 1) % hull.Faces[f].NumVertices;
                        triangles[curTri * 3 + 2] = startVertexIndex + hull.Faces[f].NumVertices;
                        curTri++;
                    }

                    avgFace *= 1.0f / hull.Faces[f].NumVertices;
                    vertices[startVertexIndex + hull.Faces[f].NumVertices] = avgFace;
                    normals[startVertexIndex + hull.Faces[f].NumVertices] = faceNormal;

                    startVertexIndex += hull.Faces[f].NumVertices + 1;
                }

                var mesh = new UnityEngine.Mesh
                {
                    hideFlags = HideFlags.HideAndDontSave,
                    vertices = vertices,
                    normals = normals,
                    triangles = triangles
                };

                results.Add(new DisplayResult
                {
                    Mesh = mesh,
                    Scale = new float3(uniformScale),
                    Position = worldFromCollider.pos,
                    Orientation = worldFromCollider.rot
                });
            }

            public static void AppendSphere(ref List<DisplayResult> results, SphereCollider* sphere, RigidTransform worldFromCollider, float uniformScale = 1)
            {
                float r = sphere->Radius * uniformScale;
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceSphere,
                    Scale = new Vector3(r * 2.0f, r * 2.0f, r * 2.0f),
                    Position = math.transform(worldFromCollider, sphere->Center * uniformScale),
                    Orientation = worldFromCollider.rot,
                });
            }

            public static void AppendCapsule(ref List<DisplayResult> results, CapsuleCollider* capsule, RigidTransform worldFromCollider, float uniformScale = 1)
            {
                float r = capsule->Radius * uniformScale;
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceSphere,
                    Scale = new Vector4(r * 2.0f, r * 2.0f, r * 2.0f),
                    Position = math.transform(worldFromCollider, capsule->Vertex0 * uniformScale),
                    Orientation = worldFromCollider.rot
                });
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceSphere,
                    Scale = new Vector4(r * 2.0f, r * 2.0f, r * 2.0f),
                    Position = math.transform(worldFromCollider, capsule->Vertex1 * uniformScale),
                    Orientation = worldFromCollider.rot
                });
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceCylinder,
                    Scale = new Vector4(r * 2.0f, math.length(capsule->Vertex1 - capsule->Vertex0) * 0.5f * uniformScale, r * 2.0f),
                    Position = math.transform(worldFromCollider, (capsule->Vertex0 + capsule->Vertex1) * 0.5f * uniformScale),
                    Orientation = math.mul(worldFromCollider.rot, Quaternion.FromToRotation(new float3(0, 1, 0), math.normalizesafe(capsule->Vertex1 - capsule->Vertex0)))
                });
            }

            public static void AppendMesh(ref List<DisplayResult> results, MeshCollider* meshCollider, RigidTransform worldFromCollider, float uniformScale = 1)
            {
                var vertices = new List<Vector3>();
                var normals = new List<Vector3>();
                var triangles = new List<int>();
                int vertexIndex = 0;

                ref Mesh mesh = ref meshCollider->Mesh;

                for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
                {
                    ref Mesh.Section section = ref mesh.Sections[sectionIndex];
                    for (int primitiveIndex = 0; primitiveIndex < section.PrimitiveVertexIndices.Length; primitiveIndex++)
                    {
                        Mesh.PrimitiveVertexIndices vertexIndices = section.PrimitiveVertexIndices[primitiveIndex];
                        Mesh.PrimitiveFlags flags = section.PrimitiveFlags[primitiveIndex];
                        int numTriangles = flags.HasFlag(Mesh.PrimitiveFlags.IsTrianglePair) ? 2 : 1;

                        float3x4 v = new float3x4(
                            section.Vertices[vertexIndices.A],
                            section.Vertices[vertexIndices.B],
                            section.Vertices[vertexIndices.C],
                            section.Vertices[vertexIndices.D]);

                        for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
                        {
                            float3 a = v[0];
                            float3 b = v[1 + triangleIndex];
                            float3 c = v[2 + triangleIndex];
                            vertices.Add(a);
                            vertices.Add(b);
                            vertices.Add(c);

                            triangles.Add(vertexIndex++);
                            triangles.Add(vertexIndex++);
                            triangles.Add(vertexIndex++);

                            float3 n = math.normalize(math.cross((b - a), (c - a)));
                            normals.Add(n);
                            normals.Add(n);
                            normals.Add(n);
                        }
                    }
                }

                var displayMesh = new UnityEngine.Mesh
                {
                    hideFlags = HideFlags.HideAndDontSave,
                    indexFormat = vertices.Count > UInt16.MaxValue
                        ? UnityEngine.Rendering.IndexFormat.UInt32
                        : UnityEngine.Rendering.IndexFormat.UInt16
                };
                displayMesh.SetVertices(vertices);
                displayMesh.SetNormals(normals);
                displayMesh.SetTriangles(triangles, 0);

                results.Add(new DisplayResult
                {
                    Mesh = displayMesh,
                    Scale = new Vector3(uniformScale, uniformScale, uniformScale),
                    Position = worldFromCollider.pos,
                    Orientation = worldFromCollider.rot,
                });
            }

            public static void AppendCompound(ref List<DisplayResult> results, CompoundCollider* compoundCollider, RigidTransform worldFromCollider, float uniformScale = 1.0f)
            {
                for (int i = 0; i < compoundCollider->Children.Length; i++)
                {
                    ref CompoundCollider.Child child = ref compoundCollider->Children[i];
                    ScaledMTransform mWorldFromCollider = new ScaledMTransform(worldFromCollider, uniformScale);
                    ScaledMTransform mWorldFromChild = ScaledMTransform.Mul(mWorldFromCollider, new MTransform(child.CompoundFromChild));

                    var worldFromChild = new RigidTransform
                    {
                        pos = mWorldFromChild.Translation,
                        rot = new quaternion(mWorldFromChild.Rotation)
                    };

                    AppendCollider(ref results, child.Collider, worldFromChild, uniformScale);
                }
            }

            public static void AppendTerrain(ref List<DisplayResult> results, TerrainCollider* terrainCollider, RigidTransform worldFromCollider, float uniformScale = 1.0f)
            {
                ref var terrain = ref terrainCollider->Terrain;

                var numVertices = (terrain.Size.x - 1) * (terrain.Size.y - 1) * 6;
                var vertices = new List<Vector3>(numVertices);
                var normals = new List<Vector3>(numVertices);
                var triangles = new List<int>(numVertices);

                int vertexIndex = 0;
                for (int i = 0; i < terrain.Size.x - 1; i++)
                {
                    for (int j = 0; j < terrain.Size.y - 1; j++)
                    {
                        int i0 = i;
                        int i1 = i + 1;
                        int j0 = j;
                        int j1 = j + 1;
                        float3 v0 = new float3(i0, terrain.Heights[i0 + terrain.Size.x * j0], j0) * terrain.Scale;
                        float3 v1 = new float3(i1, terrain.Heights[i1 + terrain.Size.x * j0], j0) * terrain.Scale;
                        float3 v2 = new float3(i0, terrain.Heights[i0 + terrain.Size.x * j1], j1) * terrain.Scale;
                        float3 v3 = new float3(i1, terrain.Heights[i1 + terrain.Size.x * j1], j1) * terrain.Scale;
                        float3 n0 = math.normalize(new float3(v0.y - v1.y, 1.0f, v0.y - v2.y));
                        float3 n1 = math.normalize(new float3(v2.y - v3.y, 1.0f, v1.y - v3.y));

                        vertices.Add(v1);
                        vertices.Add(v0);
                        vertices.Add(v2);
                        vertices.Add(v1);
                        vertices.Add(v2);
                        vertices.Add(v3);

                        normals.Add(n0);
                        normals.Add(n0);
                        normals.Add(n0);
                        normals.Add(n1);
                        normals.Add(n1);
                        normals.Add(n1);

                        triangles.Add(vertexIndex++);
                        triangles.Add(vertexIndex++);
                        triangles.Add(vertexIndex++);
                        triangles.Add(vertexIndex++);
                        triangles.Add(vertexIndex++);
                        triangles.Add(vertexIndex++);
                    }
                }

                var displayMesh = new UnityEngine.Mesh
                {
                    hideFlags = HideFlags.HideAndDontSave,
                    indexFormat = vertices.Count > UInt16.MaxValue
                        ? UnityEngine.Rendering.IndexFormat.UInt32
                        : UnityEngine.Rendering.IndexFormat.UInt16
                };
                displayMesh.SetVertices(vertices);
                displayMesh.SetNormals(normals);
                displayMesh.SetTriangles(triangles, 0);

                results.Add(new DisplayResult
                {
                    Mesh = displayMesh,
                    Scale = new Vector3(uniformScale, uniformScale, uniformScale),
                    Position = worldFromCollider.pos,
                    Orientation = worldFromCollider.rot
                });
            }

            public static void AppendCollider(ref List<DisplayResult> results, Collider* collider, RigidTransform worldFromCollider, float uniformScale = 1.0f)
            {
                switch (collider->Type)
                {
                    case ColliderType.Box:
                    case ColliderType.Triangle:
                    case ColliderType.Quad:
                    case ColliderType.Cylinder:
                    case ColliderType.Convex:
                        AppendConvex(ref results, ref ((ConvexCollider*)collider)->ConvexHull, worldFromCollider, uniformScale);
                        break;
                    case ColliderType.Sphere:
                        AppendSphere(ref results, (SphereCollider*)collider, worldFromCollider, uniformScale);
                        break;
                    case ColliderType.Capsule:
                        AppendCapsule(ref results, (CapsuleCollider*)collider, worldFromCollider, uniformScale);
                        break;
                    case ColliderType.Mesh:
                        AppendMesh(ref results, (MeshCollider*)collider, worldFromCollider, uniformScale);
                        break;
                    case ColliderType.Compound:
                        AppendCompound(ref results, (CompoundCollider*)collider, worldFromCollider, uniformScale);
                        break;
                    case ColliderType.Terrain:
                        AppendTerrain(ref results, (TerrainCollider*)collider, worldFromCollider, uniformScale);
                        break;
                }
            }

            public static List<DisplayResult> BuildDebugDisplayMesh(BlobAssetReference<Collider> collider, float uniformScale = 1.0f) =>
                BuildDebugDisplayMesh((Collider*)collider.GetUnsafePtr(), uniformScale);

            // Called often (via reflection) from CreateMeshFromCollider(BlobAssetReference<Collider> collider) in SceneCreationSystem.cs
            static List<DisplayResult> BuildDebugDisplayMesh(Collider* collider, float uniformScale = 1.0f)
            {
                List<DisplayResult> results = new List<DisplayResult>();
                AppendCollider(ref results, collider, RigidTransform.identity, uniformScale);
                return results;
            }
        }
    }
}
