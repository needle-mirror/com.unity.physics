using System;
using System.Collections.Generic;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// A system to display debug geometry for all body colliders
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class DisplayBodyColliders : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;

        //<todo.eoin.udebug This is not great; we weren't able to reuse any of the DebugStream
        //< system or jobify this system, since we couldn't guarantee the lifetime of the debug
        //< display objects. Caching those objects across frames should allow for improving this
        //< and some reuse of the DebugDraw code.
        unsafe class DrawComponent : MonoBehaviour
        {
            public NativeArray<RigidBody> Bodies;
            public int NumDynamicBodies;
            public int EnableColliders;

            protected static UnityEngine.Mesh ReferenceSphere => GetReferenceMesh(ref CachedReferenceSphere, PrimitiveType.Sphere);
            protected static UnityEngine.Mesh ReferenceCylinder => GetReferenceMesh(ref CachedReferenceCylinder, PrimitiveType.Cylinder);

            protected static UnityEngine.Mesh CachedReferenceSphere;
            protected static UnityEngine.Mesh CachedReferenceCylinder;

            static UnityEngine.Mesh GetReferenceMesh(ref UnityEngine.Mesh cache, PrimitiveType type)
            {
                if (cache == null)
                {
                    cache = CreateReferenceMesh(type);
                }
                return cache;
            }

            static UnityEngine.Mesh CreateReferenceMesh(PrimitiveType type)
            {
                switch (type)
                {
                    case PrimitiveType.Cylinder:
                        return Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");
                    case PrimitiveType.Sphere:
                        return Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Sphere.fbx");
                    default:
                        throw new NotImplementedException($"No reference mesh specified for {type}");
                }
            }

            // Combination mesh+scale, to enable sharing spheres
            public class DisplayResult
            {
                public UnityEngine.Mesh Mesh;
                public Vector3 Scale;
                public Vector3 Position;
                public Quaternion Orientation;

                [Preserve]
                public float4x4 Transform => float4x4.TRS(Position, Orientation, Scale);
            }

            private static void AppendConvex(ref ConvexHull hull, RigidTransform worldFromCollider, ref List<DisplayResult> results)
            {
                int totalNumVertices = 0;
                for (int f = 0; f < hull.NumFaces; f++)
                {
                    totalNumVertices += hull.Faces[f].NumVertices + 1;
                }

                Vector3[] vertices = new Vector3[totalNumVertices];
                Vector3[] normals = new Vector3[totalNumVertices];
                int[] triangles = new int[(totalNumVertices - hull.NumFaces) * 3];

                int startVertexIndex = 0;
                int curTri = 0;
                for (int f = 0; f < hull.NumFaces; f++)
                {
                    Vector3 avgFace = Vector3.zero;
                    Vector3 faceNormal = hull.Planes[f].Normal;

                    for (int fv = 0; fv < hull.Faces[f].NumVertices; fv++)
                    {
                        int origV = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
                        vertices[startVertexIndex + fv] = hull.Vertices[origV];
                        normals[startVertexIndex + fv] = faceNormal;

                        Vector3 v = hull.Vertices[origV];
                        avgFace += v;

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
                    Scale = Vector3.one,
                    Position = worldFromCollider.pos,
                    Orientation = worldFromCollider.rot
                });
            }

            public static void AppendSphere(SphereCollider* sphere, RigidTransform worldFromCollider, ref List<DisplayResult> results)
            {
                float r = sphere->Radius;
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceSphere,
                    Scale = new Vector4(r * 2.0f, r * 2.0f, r * 2.0f),
                    Position = math.transform(worldFromCollider, sphere->Center),
                    Orientation = worldFromCollider.rot,
                });
            }

            public static void AppendCapsule(CapsuleCollider* capsule, RigidTransform worldFromCollider, ref List<DisplayResult> results)
            {
                float r = capsule->Radius;
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceSphere,
                    Scale = new Vector4(r * 2.0f, r * 2.0f, r * 2.0f),
                    Position = math.transform(worldFromCollider, capsule->Vertex0),
                    Orientation = worldFromCollider.rot
                });
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceSphere,
                    Scale = new Vector4(r * 2.0f, r * 2.0f, r * 2.0f),
                    Position = math.transform(worldFromCollider, capsule->Vertex1),
                    Orientation = worldFromCollider.rot
                });
                results.Add(new DisplayResult
                {
                    Mesh = ReferenceCylinder,
                    Scale = new Vector4(r * 2.0f, math.length(capsule->Vertex1 - capsule->Vertex0) * 0.5f, r * 2.0f),
                    Position = math.transform(worldFromCollider, (capsule->Vertex0 + capsule->Vertex1) * 0.5f),
                    Orientation = math.mul(worldFromCollider.rot, Quaternion.FromToRotation(new float3(0, 1, 0), math.normalizesafe(capsule->Vertex1 - capsule->Vertex0)))
                });
            }

            public static void AppendMesh(MeshCollider* meshCollider, RigidTransform worldFromCollider, ref List<DisplayResult> results)
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
                    Scale = Vector3.one,
                    Position = worldFromCollider.pos,
                    Orientation = worldFromCollider.rot
                });
            }

            public static void AppendCompound(CompoundCollider* compoundCollider, RigidTransform worldFromCollider, ref List<DisplayResult> results)
            {
                for (int i = 0; i < compoundCollider->Children.Length; i++)
                {
                    ref CompoundCollider.Child child = ref compoundCollider->Children[i];
                    RigidTransform worldFromChild = math.mul(worldFromCollider, child.CompoundFromChild);
                    AppendCollider(child.Collider, worldFromChild, ref results);
                }
            }

            public static void AppendTerrain(TerrainCollider* terrainCollider, RigidTransform worldFromCollider, ref List<DisplayResult> results)
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
                    Scale = Vector3.one,
                    Position = worldFromCollider.pos,
                    Orientation = worldFromCollider.rot
                });
            }

            public static void AppendCollider(Collider* collider, RigidTransform worldFromCollider, ref List<DisplayResult> results)
            {
                switch (collider->Type)
                {
                    case ColliderType.Box:
                    case ColliderType.Triangle:
                    case ColliderType.Quad:
                    case ColliderType.Cylinder:
                    case ColliderType.Convex:
                        AppendConvex(ref ((ConvexCollider*)collider)->ConvexHull, worldFromCollider, ref results);
                        break;
                    case ColliderType.Sphere:
                        AppendSphere((SphereCollider*)collider, worldFromCollider, ref results);
                        break;
                    case ColliderType.Capsule:
                        AppendCapsule((CapsuleCollider*)collider, worldFromCollider, ref results);
                        break;
                    case ColliderType.Mesh:
                        AppendMesh((MeshCollider*)collider, worldFromCollider, ref results);
                        break;
                    case ColliderType.Compound:
                        AppendCompound((CompoundCollider*)collider, worldFromCollider, ref results);
                        break;
                    case ColliderType.Terrain:
                        AppendTerrain((TerrainCollider*)collider, worldFromCollider, ref results);
                        break;
                }
            }

            static List<DisplayResult> BuildDebugDisplayMesh(BlobAssetReference<Collider> collider) =>
                BuildDebugDisplayMesh((Collider*)collider.GetUnsafePtr());

            static List<DisplayResult> BuildDebugDisplayMesh(Collider* collider)
            {
                List<DisplayResult> results = new List<DisplayResult>();
                AppendCollider(collider, RigidTransform.identity, ref results);
                return results;
            }

            public void OnDrawGizmos()
            {
                if (EnableColliders == 0)
                    return;

                if (!Bodies.IsCreated)
                    return;

                for (int b = 0; b < Bodies.Length; b++)
                {
                    var body = Bodies[b];
                    if (!body.Collider.IsCreated)
                        continue;

                    // Draw collider
                    {
                        List<DisplayResult> displayResults = BuildDebugDisplayMesh(body.Collider);
                        if (displayResults.Count == 0)
                            continue;

                        if (b < NumDynamicBodies)
                        {
                            Gizmos.color = new UnityEngine.Color(1.0f, 0.7f, 0.0f);
                        }
                        else
                        {
                            Gizmos.color = new UnityEngine.Color(0.7f, 0.7f, 0.7f);
                        }

                        foreach (DisplayResult dr in displayResults)
                        {
                            if (EnableColliders != 0)
                            {
                                Vector3 position = math.transform(body.WorldFromBody, dr.Position);
                                Quaternion orientation = math.mul(body.WorldFromBody.rot, dr.Orientation);
                                Gizmos.DrawMesh(dr.Mesh, position, orientation, dr.Scale);
                            }
                            if (dr.Mesh != CachedReferenceCylinder && dr.Mesh != CachedReferenceSphere)
                            {
                                // Cleanup any meshes that are not our cached ones
                                Destroy(dr.Mesh);
                            }
                        }
                    }
                }
            }
        }

#pragma warning disable 618
        DrawComponent m_DrawComponent;
#pragma warning restore 618

        protected override void OnCreate()
        {
            base.OnCreate();
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
        }

        protected override void OnDestroy()
        {
            if (m_DrawComponent != null)
            {
                m_DrawComponent.Bodies = default;
            }

            m_BuildPhysicsWorldSystem = null;
            base.OnDestroy();
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadOnly();
        }

        protected override void OnUpdate()
        {
            if (!HasSingleton<PhysicsDebugDisplayData>())
                return;

            if (m_BuildPhysicsWorldSystem.PhysicsWorld.NumBodies == 0)
                return;

            int drawColliders = GetSingleton<PhysicsDebugDisplayData>().DrawColliders;

            if (m_DrawComponent == null)
            {
                // Need to make a GO and attach our DrawComponent MonoBehaviour
                // so that the rendering can happen on the main thread.
                GameObject drawObject = new GameObject();
#pragma warning disable 618
                m_DrawComponent = drawObject.AddComponent<DrawComponent>();
#pragma warning restore 618
                drawObject.name = "DebugColliderDisplay";
            }

            m_DrawComponent.Bodies = m_BuildPhysicsWorldSystem.PhysicsWorld.Bodies;
            m_DrawComponent.NumDynamicBodies = m_BuildPhysicsWorldSystem.PhysicsWorld.NumDynamicBodies;
            m_DrawComponent.EnableColliders = drawColliders;
        }
    }
}
