using Unity.Burst;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.DebugDisplay;
using Unity.Jobs;

namespace Unity.Physics.Authoring
{
    [BurstCompile(FloatPrecision.Low, FloatMode.Fast)]
    public struct DisplayColliderEdgesJob : IJob
    {
        [ReadOnly] internal NativeArray<RigidBody> Bodies;
        public DebugStream.Context OutputStream;

        public void Execute()
        {
            OutputStream.Begin(0);

            for (var i = 0; i < Bodies.Length; ++i)
            {
                var body = Bodies[i];
                var collider = body.Collider;

                if (!collider.IsCreated)
                    continue;

                DrawColliderEdges(collider, body.WorldFromBody, OutputStream);
            }

            OutputStream.End();
        }

        static void GetDebugDrawEdge(ref ConvexHull hullIn, ConvexHull.Face faceIn, int edgeIndex, out float3 from,
            out float3 to)
        {
            byte fromIndex = hullIn.FaceVertexIndices[faceIn.FirstIndex + edgeIndex];
            byte toIndex = hullIn.FaceVertexIndices[faceIn.FirstIndex + (edgeIndex + 1) % faceIn.NumVertices];
            from = hullIn.Vertices[fromIndex];
            to = hullIn.Vertices[toIndex];
        }

        static unsafe void DrawColliderEdges(ConvexCollider* collider, RigidTransform worldFromConvex,
            DebugStream.Context outputStream, bool drawVertices = false)
        {
            void WorldLine(float3 a, float3 b, ColorIndex ci)
            {
                a = math.transform(worldFromConvex, a);
                b = math.transform(worldFromConvex, b);
                outputStream.Line(a, b, ci);
            }

            ref ConvexHull hull = ref collider->ConvexHull;

            if (hull.FaceLinks.Length > 0)
            {
                foreach (ConvexHull.Face face in hull.Faces)
                {
                    for (int edgeIndex = 0; edgeIndex < face.NumVertices; edgeIndex++)
                    {
                        GetDebugDrawEdge(ref hull, face, edgeIndex, out float3 from, out float3 to);
                        WorldLine(from, to, ColorIndex.Green);
                    }
                }
            }
            else
            {
                // TODO: Remove this code when connectivity to the other collider types #386
                switch (collider->Type)
                {
                    case ColliderType.Capsule:
                        WorldLine(hull.Vertices[0], hull.Vertices[1], ColorIndex.Green);
                        break;
                    case ColliderType.Cylinder:
                        for (int f = 0; f < hull.Faces.Length; f++)
                        {
                            var face = hull.Faces[f];
                            for (int v = 0; v < face.NumVertices - 1; v++)
                            {
                                byte i = hull.FaceVertexIndices[face.FirstIndex + v];
                                byte j = hull.FaceVertexIndices[face.FirstIndex + v + 1];
                                WorldLine(hull.Vertices[i], hull.Vertices[j], ColorIndex.Green);
                            }

                            // Draw final line between first and last vertices
                            {
                                byte i = hull.FaceVertexIndices[face.FirstIndex + face.NumVertices - 1];
                                byte j = hull.FaceVertexIndices[face.FirstIndex];
                                WorldLine(hull.Vertices[i], hull.Vertices[j], ColorIndex.Green);
                            }
                        }
                        break;
                    case ColliderType.Sphere:
                        // No edges on sphere but nice to see center
                        float3 offset = new float3(hull.ConvexRadius * 0.5f);
                        for (int i = 0; i < 3; i++)
                        {
                            offset[i] = -offset[i];
                            WorldLine(hull.Vertices[0] - offset, hull.Vertices[0] + offset, ColorIndex.Green);
                        }
                        break;
                }
            }

            if (drawVertices && hull.VertexEdges.Length > 0)
            {
                foreach (ConvexHull.Edge vertexEdge in hull.VertexEdges)
                {
                    ConvexHull.Face face = hull.Faces[vertexEdge.FaceIndex];
                    GetDebugDrawEdge(ref hull, face, vertexEdge.EdgeIndex, out float3 from, out float3 to);

                    float3 r3 = new float3(0.01f, 0f, 0f);
                    WorldLine(from - r3, from + r3, ColorIndex.Red);
                    WorldLine(from - r3.yzx, from + r3.yzx, ColorIndex.Red);
                    WorldLine(from - r3.zxy, from + r3.zxy, ColorIndex.Red);

                    float3 direction = (to - from) * 0.25f;
                    WorldLine(from, from + direction, ColorIndex.Red);
                }
            }
        }

        static unsafe void DrawColliderEdges(MeshCollider* meshCollider, RigidTransform worldFromCollider,
            DebugStream.Context outputStream)
        {
            ref Mesh mesh = ref meshCollider->Mesh;

            for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
            {
                ref Mesh.Section section = ref mesh.Sections[sectionIndex];
                for (int primitiveIndex = 0; primitiveIndex < section.PrimitiveVertexIndices.Length; primitiveIndex++)
                {
                    Mesh.PrimitiveVertexIndices vertexIndices = section.PrimitiveVertexIndices[primitiveIndex];
                    Mesh.PrimitiveFlags flags = section.PrimitiveFlags[primitiveIndex];
                    bool isTrianglePair = (flags & Mesh.PrimitiveFlags.IsTrianglePair) != 0;
                    bool isQuad = (flags & Mesh.PrimitiveFlags.IsQuad) != 0;

                    var v0 = math.transform(worldFromCollider, section.Vertices[vertexIndices.A]);
                    var v1 = math.transform(worldFromCollider, section.Vertices[vertexIndices.B]);
                    var v2 = math.transform(worldFromCollider, section.Vertices[vertexIndices.C]);
                    var v3 = math.transform(worldFromCollider, section.Vertices[vertexIndices.D]);

                    if (isQuad)
                    {
                        outputStream.Line(v0, v1, ColorIndex.Green);
                        outputStream.Line(v1, v2, ColorIndex.Green);
                        outputStream.Line(v2, v3, ColorIndex.Green);
                        outputStream.Line(v3, v0, ColorIndex.Green);
                    }
                    else if (isTrianglePair)
                    {
                        outputStream.Line(v0, v1, ColorIndex.Green);
                        outputStream.Line(v1, v2, ColorIndex.Green);
                        outputStream.Line(v2, v3, ColorIndex.Green);
                        outputStream.Line(v3, v0, ColorIndex.Green);
                        outputStream.Line(v0, v2, ColorIndex.Green);
                    }
                    else
                    {
                        outputStream.Line(v0, v1, ColorIndex.Green);
                        outputStream.Line(v1, v2, ColorIndex.Green);
                        outputStream.Line(v2, v0, ColorIndex.Green);
                    }
                }
            }
        }

        static unsafe void DrawColliderEdges(CompoundCollider* compoundCollider, RigidTransform worldFromCompound,
            DebugStream.Context outputStream, bool drawVertices = false)
        {
            for (int i = 0; i < compoundCollider->NumChildren; i++)
            {
                ref CompoundCollider.Child child = ref compoundCollider->Children[i];
                var childCollider = child.Collider;
                var worldFromChild = math.mul(worldFromCompound, child.CompoundFromChild);
                DrawColliderEdges(childCollider, worldFromChild, outputStream, drawVertices);
            }
        }

        static unsafe void DrawColliderEdges(Collider* collider, RigidTransform worldFromCollider, DebugStream.Context outputStream,
            bool drawVertices = false)
        {
            switch (collider->CollisionType)
            {
                case CollisionType.Convex:
                    DrawColliderEdges((ConvexCollider*)collider, worldFromCollider, outputStream, drawVertices);
                    break;
                case CollisionType.Composite:
                    switch (collider->Type)
                    {
                        case ColliderType.Compound:
                            DrawColliderEdges((CompoundCollider*)collider, worldFromCollider, outputStream, drawVertices);
                            break;
                        case ColliderType.Mesh:
                            DrawColliderEdges((MeshCollider*)collider, worldFromCollider, outputStream);
                            break;
                    }
                    break;
            }
        }

        internal unsafe static void DrawColliderEdges(BlobAssetReference<Collider> collider, RigidTransform worldFromCollider,
            DebugStream.Context outputStream, bool drawVertices = false)
        {
            DrawColliderEdges((Collider*)collider.GetUnsafePtr(), worldFromCollider, outputStream, drawVertices);
        }

        static unsafe void DrawConnectivity(RigidBody body, DebugStream.Context outputStream, bool drawVertices = false)
        {
            if (body.Collider.Value.Type == ColliderType.Convex)
                DrawColliderEdges((ConvexCollider*)body.Collider.GetUnsafePtr(), body.WorldFromBody, outputStream, drawVertices);
        }

        static unsafe void DrawMeshEdges(RigidBody body, DebugStream.Context outputStream) =>
            DrawColliderEdges((MeshCollider*)body.Collider.GetUnsafePtr(), body.WorldFromBody, outputStream);
    }

    /// A system to display debug geometry for all body collider edges
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class DisplayBodyColliderEdges : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnDestroy()
        {
            m_BuildPhysicsWorldSystem = null;
            m_DebugStreamSystem = null;
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadOnly();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawColliderEdges != 0))
            {
                return;
            }

            if (m_BuildPhysicsWorldSystem.PhysicsWorld.NumBodies == 0)
            {
                return;
            }

            Dependency = new DisplayColliderEdgesJob()
            {
                Bodies = m_BuildPhysicsWorldSystem.PhysicsWorld.Bodies,
                OutputStream = m_DebugStreamSystem.GetContext(1)
            }.Schedule(Dependency);
        }
    }
}
