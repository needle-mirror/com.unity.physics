using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.DebugDisplay;
using Unity.Transforms;
using static Unity.Physics.Math;

namespace Unity.Physics.Authoring
{
#if UNITY_EDITOR

    [BurstCompile(FloatPrecision.Low, FloatMode.Fast)]
    public struct DisplayColliderFacesJob : IJobParallelFor
    {
        [ReadOnly] private NativeArray<RigidBody> RigidBodies;
        [ReadOnly] private NativeArray<BodyMotionType> BodiesMotionTypes;
        [ReadOnly] private PrimitiveColliderGeometries Geometries;
        [ReadOnly] private float CollidersFacesScale;

        internal static JobHandle ScheduleJob(in NativeArray<RigidBody> rigidBodies, in NativeArray<BodyMotionType> bodiesMotionTypes, float collidersFacesScale, in PrimitiveColliderGeometries geometries, JobHandle inputDeps)
        {
            return new DisplayColliderFacesJob
            {
                RigidBodies = rigidBodies,
                BodiesMotionTypes = bodiesMotionTypes,
                Geometries = geometries,
                CollidersFacesScale = collidersFacesScale
            }.Schedule(rigidBodies.Length, 16, inputDeps);
        }

        public void Execute(int i)
        {
            var rigidBody = RigidBodies[i];
            var bodyMotionType = BodiesMotionTypes[i];
            var collider = rigidBody.Collider;

            if (collider.IsCreated)
            {
                DrawColliderFaces(collider, rigidBody.WorldFromBody, bodyMotionType, rigidBody.Scale * CollidersFacesScale);
            }
        }

        private unsafe void DrawColliderFaces(BlobAssetReference<Collider> collider, RigidTransform worldFromCollider,
            BodyMotionType bodyMotionType, float uniformScale = 1.0f)
        {
            DrawColliderFaces((Collider*)collider.GetUnsafePtr(), worldFromCollider, bodyMotionType, uniformScale);
        }

        private unsafe void DrawColliderFaces(Collider* collider, RigidTransform worldFromCollider,
            BodyMotionType bodyMotionType, float uniformScale = 1.0f)
        {
            Quaternion colliderOrientation;
            float3 colliderPosition;
            float radius;
            ColorIndex color = DrawColliderUtility.GetColorIndex(bodyMotionType);

            switch (collider->Type)
            {
                case ColliderType.Cylinder:
                    radius = ((CylinderCollider*)collider)->Radius;
                    var height = ((CylinderCollider*)collider)->Height;
                    colliderPosition = ((CylinderCollider*)collider)->Center;
                    colliderOrientation = ((CylinderCollider*)collider)->Orientation * Quaternion.FromToRotation(Vector3.up, Vector3.back);

                    DrawColliderUtility.DrawPrimitiveCylinderFaces(radius, height, colliderPosition, colliderOrientation, worldFromCollider, ref Geometries.CylinderGeometry, color, uniformScale);
                    break;

                case ColliderType.Box:
                    colliderPosition = ((BoxCollider*)collider)->Center;
                    var size = ((BoxCollider*)collider)->Size;
                    colliderOrientation = ((BoxCollider*)collider)->Orientation;

                    DrawColliderUtility.DrawPrimitiveBoxFaces(size, colliderPosition, colliderOrientation, worldFromCollider, ref Geometries.BoxGeometry, color, uniformScale);
                    break;

                case ColliderType.Triangle:
                case ColliderType.Quad:
                case ColliderType.Convex:
                    DrawConvexFaces(ref ((ConvexCollider*)collider)->ConvexHull, worldFromCollider, color, uniformScale);
                    break;

                case ColliderType.Sphere:
                    radius = ((SphereCollider*)collider)->Radius;
                    colliderPosition = ((SphereCollider*)collider)->Center;

                    DrawColliderUtility.DrawPrimitiveSphereFaces(radius, colliderPosition, worldFromCollider, ref Geometries.SphereGeometry, color, uniformScale);
                    break;

                case ColliderType.Capsule:
                    radius = ((CapsuleCollider*)collider)->Radius;

                    var vertex0 = ((CapsuleCollider*)collider)->Vertex0;
                    var vertex1 = ((CapsuleCollider*)collider)->Vertex1;
                    var axis = vertex1 - vertex0; //axis in wfc-space

                    height = 0.5f * math.length(axis) + radius;

                    colliderPosition = (vertex1 + vertex0) / 2.0f; //axis in wfc-space
                    colliderOrientation = Quaternion.FromToRotation(Vector3.up, -axis);

                    DrawColliderUtility.DrawPrimitiveCapsuleFaces(radius, height, colliderPosition, colliderOrientation, worldFromCollider, ref Geometries.CapsuleGeometry, color, uniformScale);

                    break;

                case ColliderType.Mesh:
                    DrawMeshColliderFaces((MeshCollider*)collider, worldFromCollider, color, uniformScale);
                    break;

                case ColliderType.Compound:
                    DrawCompoundColliderFaces((CompoundCollider*)collider, worldFromCollider, bodyMotionType, uniformScale);
                    break;

                case ColliderType.Terrain:
                    //TODO [#3792]: Terrain should use DebugDraw rather than Gizmos and add uniform scale support.
                    //AppendMeshColliders.GetMeshes.AppendTerrain((TerrainCollider*)collider, worldFromCollider, ref results);
                    break;
            }
        }

        // Covers: collider->Type = Box, Triangle, Quad, Convex, Cylinder(before started using primitives)
        private static void DrawConvexFaces(ref ConvexHull hull, RigidTransform worldFromCollider,
            ColorIndex ci, float uniformScale = 1.0f)
        {
            for (var f = 0; f < hull.NumFaces; f++)
            {
                var countVert = hull.Faces[f].NumVertices;

                if (countVert == 3) // A triangle
                {
                    var vertices = new NativeArray<float3>(3, Allocator.Temp);
                    for (var fv = 0; fv < countVert; fv++)
                    {
                        var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
                        vertices[fv] = uniformScale * hull.Vertices[origVertexIndex];
                    }
                    DrawColliderUtility.DrawTriangle(vertices[0], vertices[1], vertices[2], worldFromCollider, ci);
                    vertices.Dispose();
                }
                else if (countVert == 4) // A quad: break into two triangles
                {
                    var vertices = new NativeArray<float3>(4, Allocator.Temp);
                    for (var fv = 0; fv < countVert; fv++)
                    {
                        var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
                        vertices[fv] = uniformScale * hull.Vertices[origVertexIndex];
                    }
                    DrawColliderUtility.DrawTriangle(vertices[0], vertices[1], vertices[2], worldFromCollider, ci);
                    DrawColliderUtility.DrawTriangle(vertices[2], vertices[3], vertices[0], worldFromCollider, ci);
                    vertices.Dispose();
                }
                else // find the average vertex and then use to break into triangles
                {
                    var faceCentroid = float3.zero;
                    var scaledVertices = new NativeArray<float3>(countVert, Allocator.Temp);
                    for (var i = 0; i < countVert; i++)
                    {
                        var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + i];
                        scaledVertices[i] = uniformScale * hull.Vertices[origVertexIndex];

                        faceCentroid += scaledVertices[i];
                    }
                    faceCentroid /= countVert;

                    for (var j = 0; j < countVert; j++)
                    {
                        var vertices = new NativeArray<float3>(3, Allocator.Temp);
                        if (j < countVert - 1)
                        {
                            vertices[0] = scaledVertices[j];
                            vertices[1] = scaledVertices[j + 1];
                        }
                        else //close the circle of triangles
                        {
                            vertices[0] = scaledVertices[j];
                            vertices[1] = scaledVertices[0];
                        }
                        vertices[2] = faceCentroid;
                        DrawColliderUtility.DrawTriangle(vertices[0], vertices[1], vertices[2], worldFromCollider, ci);
                        vertices.Dispose();
                    }
                    scaledVertices.Dispose();
                }
            }
        }

        private unsafe void DrawCompoundColliderFaces(CompoundCollider* compoundCollider, RigidTransform worldFromCollider,
            BodyMotionType bodyMotionType, float uniformScale = 1.0f)
        {
            for (var i = 0; i < compoundCollider->Children.Length; i++)
            {
                ref CompoundCollider.Child child = ref compoundCollider->Children[i];

                ScaledMTransform mWorldFromCompound = new ScaledMTransform(worldFromCollider, 1.0f);
                ScaledMTransform mWorldFromChild = ScaledMTransform.Mul(mWorldFromCompound, new MTransform(child.CompoundFromChild));
                RigidTransform worldFromChild = new RigidTransform(mWorldFromChild.Rotation, mWorldFromChild.Translation);

                DrawColliderFaces(child.Collider, worldFromChild, bodyMotionType, uniformScale);
            }
        }

        private static unsafe void DrawMeshColliderFaces(MeshCollider* meshCollider, RigidTransform worldFromCollider,
            DebugDisplay.ColorIndex ci, float uniformScale = 1.0f)
        {
            ref Mesh mesh = ref meshCollider->Mesh;

            float4x4 worldMatrix = new float4x4(worldFromCollider);
            worldMatrix.c0 *= uniformScale;
            worldMatrix.c1 *= uniformScale;
            worldMatrix.c2 *= uniformScale;

            var nothing = new RigidTransform();
            for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
            {
                ref Mesh.Section section = ref mesh.Sections[sectionIndex];
                for (int primitiveIndex = 0; primitiveIndex < section.PrimitiveVertexIndices.Length; primitiveIndex++)
                {
                    Mesh.PrimitiveVertexIndices vertexIndices = section.PrimitiveVertexIndices[primitiveIndex];
                    Mesh.PrimitiveFlags flags = section.PrimitiveFlags[primitiveIndex];
                    var numTriangles = 1;
                    if ((flags & Mesh.PrimitiveFlags.IsTrianglePair) != 0)
                    {
                        numTriangles = 2;
                    }

                    float3x4 v = new float3x4(
                        math.transform(worldMatrix, section.Vertices[vertexIndices.A]),
                        math.transform(worldMatrix, section.Vertices[vertexIndices.B]),
                        math.transform(worldMatrix, section.Vertices[vertexIndices.C]),
                        math.transform(worldMatrix, section.Vertices[vertexIndices.D]));

                    for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
                    {
                        DrawColliderUtility.DrawTriangle(v[0], v[1 + triangleIndex], v[2 + triangleIndex], nothing, ci);
                    }
                }
            }
        }
    }

    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    internal partial struct DisplayBodyCollidersSystem_Default : ISystem
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
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliders == 0)
                return;

            state.EntityManager.CompleteDependencyBeforeRO<PhysicsWorldSingleton>();
            if (SystemAPI.TryGetSingleton(out PhysicsWorldSingleton physicsWorldSingleton))
            {
                ref var physicsWorld = ref physicsWorldSingleton.PhysicsWorld;
                var bodyMotionTypes = new NativeArray<BodyMotionType>(physicsWorld.NumBodies, Allocator.TempJob);
                for (int index = 0; index < physicsWorld.NumBodies; ++index)
                {
                    if (index < physicsWorld.NumDynamicBodies)
                    {
                        bodyMotionTypes[index] = physicsWorld.MotionVelocities[index].IsKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic;
                    }
                    else
                    {
                        bodyMotionTypes[index] = BodyMotionType.Static;
                    }
                }

                var displayHandle = DisplayColliderFacesJob.ScheduleJob(physicsWorldSingleton.PhysicsWorld.Bodies, bodyMotionTypes, 1.0f, DefaultGeometries, state.Dependency);
                var disposeHandle = bodyMotionTypes.Dispose(displayHandle);

                state.Dependency = disposeHandle;
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
    internal partial struct DisplayBodyCollidersSystem_Editor : ISystem
    {
        private PrimitiveColliderGeometries DefaultGeometries;

        void OnCreate(ref SystemState state)
        {
            var colliderQuery = state.GetEntityQuery(ComponentType.ReadOnly<PhysicsCollider>(), ComponentType.ReadOnly<LocalToWorld>());
            state.RequireForUpdate(colliderQuery);
            state.RequireForUpdate<PhysicsDebugDisplayData>();

            DrawColliderUtility.CreateGeometries(out DefaultGeometries);
        }

        private void CreateRigidBody(in float4x4 localToWorld, float scale, BlobAssetReference<Collider> collider, out RigidBody rigidBody)
        {
            var rigidTransform = DecomposeRigidBodyTransform(localToWorld);
            rigidBody = new RigidBody
            {
                Collider = collider,
                WorldFromBody = rigidTransform,
                Scale = scale
            };
        }

        void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliders == 0)
                return;

            // First add the dynamic bodies
            var rigidBodies = new NativeList<RigidBody>(Allocator.TempJob);
            var bodyMotionTypes = new NativeList<BodyMotionType>(Allocator.TempJob);

            foreach (var(collider, localToWorld, localTransform, physicsMass) in
                     SystemAPI.Query<RefRO<PhysicsCollider>, RefRO<LocalToWorld>, RefRO<LocalTransform>, RefRO<PhysicsMass>>())
            {
                CreateRigidBody(localToWorld.ValueRO.Value, localTransform.ValueRO.Scale, collider.ValueRO.Value, out var rigidBody);
                rigidBodies.Add(rigidBody);
                bodyMotionTypes.Add(physicsMass.ValueRO.IsKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic);
            }

            // Then add the rest
            foreach (var(collider, localToWorld, localTransform) in
                     SystemAPI.Query<RefRO<PhysicsCollider>, RefRO<LocalToWorld>, RefRO<LocalTransform>>().WithNone<PhysicsMass>())
            {
                CreateRigidBody(localToWorld.ValueRO.Value, localTransform.ValueRO.Scale, collider.ValueRO.Value, out var rigidBody);
                rigidBodies.Add(rigidBody);
                bodyMotionTypes.Add(BodyMotionType.Static);
            }

            var displayHandle = DisplayColliderFacesJob.ScheduleJob(rigidBodies.AsArray(), bodyMotionTypes.AsArray(), 1.0f, DefaultGeometries, state.Dependency);
            var disposeHandle = JobHandle.CombineDependencies(rigidBodies.Dispose(displayHandle), bodyMotionTypes.Dispose(displayHandle));

            state.Dependency = disposeHandle;
        }

        void OnDestroy(ref SystemState state)
        {
            DefaultGeometries.Dispose();
        }
    }
#endif
}
