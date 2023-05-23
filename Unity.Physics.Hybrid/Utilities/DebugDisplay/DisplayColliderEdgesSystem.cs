using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.DebugDisplay;
using Unity.Jobs;
using Unity.Physics.Systems;
using Unity.Transforms;
using static Unity.Physics.Math;
using UnityEngine;

namespace Unity.Physics.Authoring
{
#if UNITY_EDITOR
    [BurstCompile(FloatPrecision.Low, FloatMode.Fast)]
    public struct DisplayColliderEdgesJob : IJobParallelFor
    {
        [ReadOnly] private NativeArray<RigidBody> RigidBodies;
        [ReadOnly] private PrimitiveColliderGeometries Geometries;
        [ReadOnly] private float CollidersEdgesScale;

        internal static JobHandle ScheduleJob(in NativeArray<RigidBody> rigidBodies, float collidersEdgesScales, in PrimitiveColliderGeometries geometries, JobHandle inputDeps)
        {
            return new DisplayColliderEdgesJob
            {
                RigidBodies = rigidBodies,
                Geometries = geometries,
                CollidersEdgesScale = collidersEdgesScales
            }.Schedule(rigidBodies.Length, 16, inputDeps);
        }

        public void Execute(int i)
        {
            var rigidBody = RigidBodies[i];
            if (RigidBodies[i].Collider.IsCreated)
            {
                DrawColliderEdges(rigidBody.Collider, rigidBody.WorldFromBody, rigidBody.Scale * CollidersEdgesScale, ref Geometries);
            }
        }

        private static unsafe void DrawColliderEdges(BlobAssetReference<Collider> collider, RigidTransform worldFromCollider, float uniformScale, ref PrimitiveColliderGeometries geometries,
            bool drawVertices = false)
        {
            DrawColliderEdges((Collider*)collider.GetUnsafePtr(), worldFromCollider, uniformScale, ref geometries, drawVertices);
        }

        static unsafe void DrawColliderEdges(Collider* collider, RigidTransform worldFromCollider, float uniformScale, ref PrimitiveColliderGeometries geometries, bool drawVertices = false)
        {
            switch (collider->CollisionType)
            {
                case CollisionType.Convex:
                    DrawConvexColliderEdges((ConvexCollider*)collider, worldFromCollider, uniformScale, ref geometries, drawVertices);
                    break;
                case CollisionType.Composite:
                    switch (collider->Type)
                    {
                        case ColliderType.Compound:
                            DrawCompoundColliderEdges((CompoundCollider*)collider, worldFromCollider, uniformScale, ref geometries, drawVertices);
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

        static unsafe void DrawConvexColliderEdges(ConvexCollider* collider, RigidTransform worldFromConvex, float uniformScale, ref PrimitiveColliderGeometries geometries, bool drawVertices = false)
        {
            float4x4 worldMatrix = math.float4x4(worldFromConvex);

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
                float height;
                switch (collider->Type)
                {
                    case ColliderType.Capsule:
                        radius = ((CapsuleCollider*)collider)->Radius;
                        var vertex0 = ((CapsuleCollider*)collider)->Vertex0;
                        var vertex1 = ((CapsuleCollider*)collider)->Vertex1;
                        center = -0.5f * (vertex1 - vertex0) + vertex1;
                        var axis = vertex1 - vertex0; //axis in wfc-space
                        var colliderOrientation = Quaternion.FromToRotation(Vector3.up, -axis);
                        height = 0.5f * math.length(axis) + radius;
                        DrawColliderUtility.DrawPrimitiveCapsuleEdges(radius, height, center, colliderOrientation, worldFromConvex, ref geometries.CapsuleGeometry, uniformScale);
                        break;

                    case ColliderType.Cylinder:
                        radius = ((CylinderCollider*)collider)->Radius;
                        height = ((CylinderCollider*)collider)->Height;
                        var colliderPosition = ((CylinderCollider*)collider)->Center;
                        colliderOrientation = ((CylinderCollider*)collider)->Orientation * Quaternion.FromToRotation(Vector3.up, Vector3.back);
                        DrawColliderUtility.DrawPrimitiveCylinderEdges(radius, height, colliderPosition, colliderOrientation, worldFromConvex, ref geometries.CylinderGeometry, uniformScale);
                        break;

                    case ColliderType.Sphere:
                        radius = ((SphereCollider*)collider)->Radius;
                        center = ((SphereCollider*)collider)->Center;
                        //TODO: go through the render pipe for debug draw and strealine vertex process so we only upload 1 mesh and just schedule batches
                        DrawColliderUtility.DrawPrimitiveSphereEdges(radius, center, worldFromConvex, ref geometries.SphereGeometry, uniformScale);
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

        static unsafe void DrawMeshColliderEdges(MeshCollider* meshCollider, RigidTransform worldFromCollider, float uniformScale)
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

        static unsafe void DrawCompoundColliderEdges(CompoundCollider* compoundCollider, RigidTransform worldFromCompound, float uniformScale, ref PrimitiveColliderGeometries geometries,
            bool drawVertices = false)
        {
            for (int i = 0; i < compoundCollider->NumChildren; i++)
            {
                ref CompoundCollider.Child child = ref compoundCollider->Children[i];

                ScaledMTransform mWorldFromCompound = new ScaledMTransform(worldFromCompound, uniformScale);
                ScaledMTransform mWorldFromChild = ScaledMTransform.Mul(mWorldFromCompound, new MTransform(child.CompoundFromChild));
                RigidTransform worldFromChild = new RigidTransform(mWorldFromChild.Rotation, mWorldFromChild.Translation);

                var childCollider = child.Collider;
                DrawColliderEdges(childCollider, worldFromChild, uniformScale, ref geometries, drawVertices);
            }
        }
    }

    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    internal partial struct DisplayBodyColliderEdges_Default : ISystem
    {
        private PrimitiveColliderGeometries DefaultGeometries;

        void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsWorldSingleton>();
            state.RequireForUpdate<PhysicsCollider>();
            state.RequireForUpdate<PhysicsDebugDisplayData>();

            DrawColliderUtility.CreateGeometries(out DefaultGeometries);
        }

        void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliderEdges == 0)
                return;

            if (SystemAPI.TryGetSingleton(out PhysicsWorldSingleton physicsWorldSingleton))
            {
                state.Dependency = DisplayColliderEdgesJob.ScheduleJob(physicsWorldSingleton.PhysicsWorld.Bodies, 1.0f, DefaultGeometries, state.Dependency);
            }
        }

        void OnDestroy(ref SystemState state)
        {
            DefaultGeometries.Dispose();
        }
    }

    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateInGroup(typeof(PhysicsDisplayDebugGroup))]
    [UpdateAfter(typeof(CleanPhysicsDebugDataSystem_Editor))]
    [UpdateBefore(typeof(PhysicsDebugDisplaySystem_Editor))]
    internal partial struct DisplayBodyColliderEdges_Editor : ISystem
    {
        private PrimitiveColliderGeometries DefaultGeometries;

        void OnCreate(ref SystemState state)
        {
            var colliderQuery = state.GetEntityQuery(ComponentType.ReadOnly<PhysicsCollider>(), ComponentType.ReadOnly<LocalToWorld>());
            state.RequireForUpdate(colliderQuery);
            state.RequireForUpdate<PhysicsDebugDisplayData>();

            DrawColliderUtility.CreateGeometries(out DefaultGeometries);
        }

        void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliderEdges == 0)
                return;

            var rigidBodiesList = new NativeList<RigidBody>(Allocator.TempJob);
            foreach (var(collider, localToWorld, localTransform) in
                     SystemAPI.Query<RefRO<PhysicsCollider>, RefRO<LocalToWorld>, RefRO<LocalTransform>>())
            {
                var rigidTransform = DecomposeRigidBodyTransform(localToWorld.ValueRO.Value);
                rigidBodiesList.Add(new RigidBody()
                {
                    Collider = collider.ValueRO.Value,
                    WorldFromBody = rigidTransform,
                    Scale = localTransform.ValueRO.Scale
                });
            }

            if (rigidBodiesList.IsEmpty)
            {
                rigidBodiesList.Dispose();
                return;
            }

            var displayHandle = DisplayColliderEdgesJob.ScheduleJob(rigidBodiesList.AsArray(), 1.0f, DefaultGeometries, state.Dependency);
            var disposeHandle = rigidBodiesList.Dispose(displayHandle);

            state.Dependency = disposeHandle;
        }

        void OnDestroy(ref SystemState state)
        {
            DefaultGeometries.Dispose();
        }
    }
#endif
}
