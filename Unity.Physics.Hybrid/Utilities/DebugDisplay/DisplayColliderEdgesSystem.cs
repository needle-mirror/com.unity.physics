using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.DebugDisplay;
using Unity.Jobs;
using static Unity.Physics.Math;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [BurstCompile(FloatPrecision.Low, FloatMode.Fast)]
    internal struct DisplayColliderEdgesJob : IJobParallelFor
    {
        [ReadOnly] internal NativeArray<RigidBody> Bodies;

        public void Execute(int i)
        {
            var body = Bodies[i];
            var collider = body.Collider;

            if (collider.IsCreated)
            {
                DrawColliderEdges(collider, body.WorldFromBody, body.Scale);
            }
        }

        internal unsafe static void DrawColliderEdges(BlobAssetReference<Collider> collider, RigidTransform worldFromCollider, float uniformScale,
            bool drawVertices = false)
        {
            DrawColliderEdges((Collider*)collider.GetUnsafePtr(), worldFromCollider, uniformScale, drawVertices);
        }

        static unsafe void DrawColliderEdges(Collider* collider, RigidTransform worldFromCollider, float uniformScale, bool drawVertices = false)
        {
            switch (collider->CollisionType)
            {
                case CollisionType.Convex:
                    DrawConvexColliderEdges((ConvexCollider*)collider, worldFromCollider, uniformScale, drawVertices);
                    break;
                case CollisionType.Composite:
                    switch (collider->Type)
                    {
                        case ColliderType.Compound:
                            DrawCompoundColliderEdges((CompoundCollider*)collider, worldFromCollider, uniformScale, drawVertices);
                            break;
                        case ColliderType.Mesh:
                            DrawMeshColliderEdges((MeshCollider*)collider, worldFromCollider, uniformScale);
                            break;
                    }
                    break;
            }
        }

        static void GetDebugDrawEdge(ref ConvexHull hullIn, ConvexHull.Face faceIn, int edgeIndex, out float3 from,
            out float3 to)
        {
            byte fromIndex = hullIn.FaceVertexIndices[faceIn.FirstIndex + edgeIndex];
            byte toIndex = hullIn.FaceVertexIndices[faceIn.FirstIndex + (edgeIndex + 1) % faceIn.NumVertices];
            from = hullIn.Vertices[fromIndex];
            to = hullIn.Vertices[toIndex];
        }

        static void WorldLine(float3 a, float3 b, ColorIndex ci, float4x4 worldMatrix)
        {
            a = math.transform(worldMatrix, a);
            b = math.transform(worldMatrix, b);
            PhysicsDebugDisplaySystem.Line(a, b, ci);
        }

        static unsafe void DrawConvexColliderEdges(ConvexCollider* collider, RigidTransform worldFromConvex, float uniformScale,
            bool drawVertices = false)
        {
            float4x4 worldMatrix = math.float4x4(worldFromConvex);

            // Apply sale to matrix
            worldMatrix.c0 *= uniformScale;
            worldMatrix.c1 *= uniformScale;
            worldMatrix.c2 *= uniformScale;

            ref ConvexHull hull = ref collider->ConvexHull;
            float3 centroid = float3.zero;

            void ExpandHullVertices(float3* vertexPtr, int numVertices, ref ConvexHull convexHull)
            {
                for (int i = 0; i < numVertices; i++)
                {
                    float3 direction = vertexPtr[i] - centroid;
                    float3 directionNormalized = math.normalize(direction);

                    vertexPtr[i] += directionNormalized * convexHull.ConvexRadius;
                }
            }

            // centroid is only needed in those cases
            if (hull.FaceLinks.Length > 0 || (drawVertices && hull.VertexEdges.Length > 0))
            {
                centroid = AppendMeshColliders.GetMeshes.ComputeHullCentroid(ref hull);
            }

            if (hull.FaceLinks.Length > 0)
            {
                foreach (ConvexHull.Face face in hull.Faces)
                {
                    for (int edgeIndex = 0; edgeIndex < face.NumVertices; edgeIndex++)
                    {
                        float3* verts = stackalloc float3[2];
                        GetDebugDrawEdge(ref hull, face, edgeIndex, out verts[0], out verts[1]);
                        ExpandHullVertices(verts, 2, ref hull);

                        WorldLine(verts[0], verts[1], ColorIndex.Green, worldMatrix);
                    }
                }
            }
            else
            {
                float radius;
                float3 center;
                int method;
                switch (collider->Type)
                {
                    case ColliderType.Capsule:
                        radius = uniformScale * ((CapsuleCollider*)collider)->Radius;
                        var vertex0 = uniformScale * ((CapsuleCollider*)collider)->Vertex0;
                        var vertex1 = uniformScale * ((CapsuleCollider*)collider)->Vertex1;
                        method = 2;
                        switch (method)
                        {
                            case 1:  // Original method (recommended for ragdolls)
                                WorldLine(hull.Vertices[0], hull.Vertices[1], ColorIndex.Green, worldMatrix);
                                break;
                            case 2: // Wireframe generated capsule
                                center = -0.5f * (vertex1 - vertex0) + vertex1;
                                vertex0 = math.transform(worldFromConvex, vertex0);
                                vertex1 = math.transform(worldFromConvex, vertex1);
                                var axis = vertex1 - vertex0; //axis in wfc-space
                                var colliderOrientation = new RigidTransform(Quaternion.FromToRotation(Vector3.up, -axis), worldFromConvex.pos);
                                var height = 0.5f * math.length(axis) + radius;
                                DrawColliderUtility.DrawPrimitiveCapsuleEdges(radius, height, center, colliderOrientation);
                                break;
                        }
                        break;

                    case ColliderType.Cylinder:
                        // keep using DebugDraw since the lines are simpler than the primitive cylinder mesh
                        for (var f = 0; f < hull.Faces.Length; f++)
                        {
                            var face = hull.Faces[f];
                            for (var v = 0; v < face.NumVertices - 1; v++)
                            {
                                byte i = hull.FaceVertexIndices[face.FirstIndex + v];
                                byte j = hull.FaceVertexIndices[face.FirstIndex + v + 1];
                                WorldLine(hull.Vertices[i], hull.Vertices[j], ColorIndex.Green, worldMatrix);
                            }

                            // Draw final line between first and last vertices
                            {
                                byte i = hull.FaceVertexIndices[face.FirstIndex + face.NumVertices - 1];
                                byte j = hull.FaceVertexIndices[face.FirstIndex];
                                WorldLine(hull.Vertices[i], hull.Vertices[j], ColorIndex.Green, worldMatrix);
                            }
                        }
                        break;

                    case ColliderType.Sphere:
                        radius = uniformScale * ((SphereCollider*)collider)->Radius;
                        center = uniformScale * ((SphereCollider*)collider)->Center;
                        method = 1;
                        switch (method)
                        {
                            case 1: // Use a cached mesh that can be either a complex mesh, or simple a icosahedron
                                //TODO: go through the render pipe for debug draw and strealine vertex process so we only upload 1 mesh and just schedule batches
                                DrawColliderUtility.DrawPrimitiveSphereEdges(radius, center, worldFromConvex);
                                break;
                            case 2: // Draw a small triple axis at the centre
                                var offset = new float3(radius * 0.5f);
                                for (var i = 0; i < 3; i++)
                                {
                                    offset[i] = -offset[i];
                                    WorldLine(hull.Vertices[0] - offset, hull.Vertices[0] + offset, ColorIndex.Green, worldMatrix);
                                }
                                break;
                        }
                        break;
                }
            }

            // This section is used to highlight the edges of the corners in red DebugDraw lines. These are drawn on top
            // of the green DebugDraw edges. It can be a useful little tool to highlight where your corners are.
            // drawVertices=false everywhere as default.
            if (drawVertices && hull.VertexEdges.Length > 0)
            {
                foreach (ConvexHull.Edge vertexEdge in hull.VertexEdges)
                {
                    ConvexHull.Face face = hull.Faces[vertexEdge.FaceIndex];
                    float3* verts = stackalloc float3[2];
                    GetDebugDrawEdge(ref hull, face, vertexEdge.EdgeIndex, out verts[0], out verts[1]);
                    ExpandHullVertices(verts, 2, ref hull);

                    float3 r3 = new float3(0.01f, 0f, 0f);
                    WorldLine(verts[0] - r3, verts[0] + r3, ColorIndex.Red, worldMatrix);
                    WorldLine(verts[0] - r3.yzx, verts[0] + r3.yzx, ColorIndex.Red, worldMatrix);
                    WorldLine(verts[0] - r3.zxy, verts[0] + r3.zxy, ColorIndex.Red, worldMatrix);

                    float3 direction = (verts[1] - verts[0]) * 0.25f;
                    WorldLine(verts[0], verts[0] + direction, ColorIndex.Red, worldMatrix);
                }
            }
        }

        static unsafe void DrawMeshColliderEdges(MeshCollider* meshCollider, RigidTransform worldFromCollider, float uniformScale
        )
        {
            ref Mesh mesh = ref meshCollider->Mesh;

            float4x4 worldMatrix = new float4x4(worldFromCollider);
            worldMatrix.c0 *= uniformScale;
            worldMatrix.c1 *= uniformScale;
            worldMatrix.c2 *= uniformScale;

            for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
            {
                ref Mesh.Section section = ref mesh.Sections[sectionIndex];
                for (int primitiveIndex = 0; primitiveIndex < section.PrimitiveVertexIndices.Length; primitiveIndex++)
                {
                    Mesh.PrimitiveVertexIndices vertexIndices = section.PrimitiveVertexIndices[primitiveIndex];
                    Mesh.PrimitiveFlags flags = section.PrimitiveFlags[primitiveIndex];
                    bool isTrianglePair = (flags & Mesh.PrimitiveFlags.IsTrianglePair) != 0;
                    bool isQuad = (flags & Mesh.PrimitiveFlags.IsQuad) != 0;

                    var v0 = math.transform(worldMatrix, section.Vertices[vertexIndices.A]);
                    var v1 = math.transform(worldMatrix, section.Vertices[vertexIndices.B]);
                    var v2 = math.transform(worldMatrix, section.Vertices[vertexIndices.C]);
                    var v3 = math.transform(worldMatrix, section.Vertices[vertexIndices.D]);

                    if (isQuad)
                    {
                        PhysicsDebugDisplaySystem.Line(v0, v1, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v1, v2, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v2, v3, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v3, v0, ColorIndex.Green);
                    }
                    else if (isTrianglePair)
                    {
                        PhysicsDebugDisplaySystem.Line(v0, v1, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v1, v2, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v2, v3, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v3, v0, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v0, v2, ColorIndex.Green);
                    }
                    else
                    {
                        PhysicsDebugDisplaySystem.Line(v0, v1, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v1, v2, ColorIndex.Green);
                        PhysicsDebugDisplaySystem.Line(v2, v0, ColorIndex.Green);
                    }
                }
            }
        }

        static unsafe void DrawCompoundColliderEdges(CompoundCollider* compoundCollider, RigidTransform worldFromCompound, float uniformScale,
            bool drawVertices = false)
        {
            for (int i = 0; i < compoundCollider->NumChildren; i++)
            {
                ref CompoundCollider.Child child = ref compoundCollider->Children[i];

                ScaledMTransform mWorldFromCompound = new ScaledMTransform(worldFromCompound, uniformScale);
                ScaledMTransform mWorldFromChild = ScaledMTransform.Mul(mWorldFromCompound, new MTransform(child.CompoundFromChild));
                RigidTransform worldFromChild = new RigidTransform(mWorldFromChild.Rotation, mWorldFromChild.Translation);

                var childCollider = child.Collider;
                DrawColliderEdges(childCollider, worldFromChild, uniformScale, drawVertices);
            }
        }

        static unsafe void DrawConnectivity(RigidBody body, bool drawVertices = false)
        {
            if (body.Collider.Value.Type == ColliderType.Convex)
                DrawConvexColliderEdges((ConvexCollider*)body.Collider.GetUnsafePtr(), body.WorldFromBody, body.Scale, drawVertices);
        }

        static unsafe void DrawMeshEdges(RigidBody body) =>
            DrawMeshColliderEdges((MeshCollider*)body.Collider.GetUnsafePtr(), body.WorldFromBody, body.Scale);
    }

    /// A system to display debug geometry for all body collider edges
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    [CreateAfter(typeof(PhysicsDebugDisplaySystem))]
    internal partial class DisplayBodyColliderEdges : SystemBase
    {
        public static readonly List<int> SphereList = new List<int>();
        public static readonly List<Vector3> SphereVertexList = new List<Vector3>();

        public static readonly List<int> CapsuleList = new List<int>();
        public static readonly List<Vector3> CapsuleVertexList = new List<Vector3>();
        public static List<float3> CapsuleEdgeList = new List<float3>();

        protected override void OnCreate()
        {
            DebugMeshCache.GetMesh(PrimitiveType.Capsule).GetTriangles(CapsuleList, 0);
            DebugMeshCache.GetMesh(PrimitiveType.Capsule).GetVertices(CapsuleVertexList);
            CapsuleList.TrimExcess();
            CapsuleVertexList.TrimExcess();

            DebugMeshCache.GetMesh(MeshType.Icosahedron).GetTriangles(SphereList, 0);
            DebugMeshCache.GetMesh(MeshType.Icosahedron).GetVertices(SphereVertexList);
            SphereList.TrimExcess();
            SphereVertexList.TrimExcess();

            CapsuleEdgeList = DrawColliderUtility.DrawCapsuleWireFrame().ToList();
        }

        protected override void OnDestroy()
        {
            SphereList.Clear();
            SphereVertexList.Clear();

            CapsuleList.Clear();
            CapsuleVertexList.Clear();
            CapsuleEdgeList.Clear();
        }

        protected override void OnUpdate()
        {
#if UNITY_EDITOR
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliderEdges == 0)
                return;

            var world = GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

            if (world.NumBodies == 0)
            {
                return;
            }

            Dependency = new DisplayColliderEdgesJob()
            {
                Bodies = world.Bodies,
            }.Schedule(world.NumBodies, 16, Dependency);
#endif
        }
    }
}
