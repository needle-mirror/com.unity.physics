using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using static Unity.Physics.Math;

namespace Unity.Physics.Authoring
{
    [BurstCompile]
    internal struct CalculateNumberOfPrimitiveCollidersJob : IJob
    {
        public NativeReference<int> NumPrimitiveColliders;

        [ReadOnly]
        public NativeArray<RigidBody> Bodies;

        private unsafe int CalcNumPrimitiveCollidersRecursive(Collider* colliderPtr)
        {
            int numPrimitive = 0;

            if (colliderPtr->Type == ColliderType.Box || colliderPtr->Type == ColliderType.Sphere ||
                colliderPtr->Type == ColliderType.Capsule || colliderPtr->Type == ColliderType.Cylinder)
            {
                numPrimitive = 1;
            }
            else if (colliderPtr->Type == ColliderType.Compound)
            {
                CompoundCollider* compound = (CompoundCollider*)colliderPtr;
                for (int i = 0; i < compound->NumChildren; i++)
                {
                    numPrimitive += CalcNumPrimitiveCollidersRecursive(compound->Children[i].Collider);
                }
            }

            return numPrimitive;
        }

        public void Execute()
        {
            unsafe
            {
                for (int i = 0; i < Bodies.Length; i++)
                {
                    if (Bodies[i].Collider.IsCreated)
                    {
                        NumPrimitiveColliders.Value += CalcNumPrimitiveCollidersRecursive((Collider*)Bodies[i].Collider.GetUnsafePtr());
                    }
                }
            }
        }
    }

    /// A system to display debug geometry for all body colliders
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    internal partial class DisplayBodyColliders : SystemBase
    {
        public struct PrimitiveInfo
        {
            public enum PrimitiveFlags : byte
            {
                Sphere = 1 << 0,
                Capsule = 1 << 1,
                Cylinder = 1 << 2,
                Box = 1 << 3,
                Dynamic = 1 << 4
            }

            public float4x4 Trs;
            public PrimitiveFlags Flags;
        }

        [BurstCompile(FloatPrecision.Low, FloatMode.Fast)]
        public unsafe struct DisplayColliderFacesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RigidBody> Bodies;
            [ReadOnly] public int DynamicBodies;
            public NativeList<PrimitiveInfo>.ParallelWriter PrimitiveWriter;

            public void Execute(int i)
            {
                var body = Bodies[i];
                var collider = body.Collider;

                if (collider.IsCreated)
                {
                    DrawColliderFaces(collider, body.WorldFromBody, i < DynamicBodies, body.Scale);
                }
            }

            private unsafe void DrawColliderFaces(BlobAssetReference<Collider> collider, RigidTransform worldFromCollider,
                bool isDynamic, float uniformScale = 1.0f)
            {
                DrawColliderFaces((Collider*)collider.GetUnsafePtr(), worldFromCollider, isDynamic, uniformScale);
            }

            private unsafe void DrawColliderFaces(Collider* collider, RigidTransform worldFromCollider,
                bool isDynamic, float uniformScale = 1.0f)
            {
                quaternion colliderOrientation;
                float3 colliderPosition;
                float radius;
                float adjustScale = 0.01f;  //so that scale of collider face != collider > rendering doesn't flicker
                float3 newScale;
                Matrix4x4 trs;

                switch (collider->Type)
                {
                    case ColliderType.Cylinder:
                        radius = ((CylinderCollider*)collider)->Radius;
                        var height = ((CylinderCollider*)collider)->Height;

                        // Cylinder collider needs an extra rotation because the collider is built in the z-axis, but is
                        // baked along the y-axis. Apply a rotation in the x-axis to transform between them
                        colliderOrientation = Quaternion.Euler(-90, 0, 0);
                        colliderOrientation = math.mul(((CylinderCollider*)collider)->Orientation, colliderOrientation);
                        colliderOrientation = math.mul(worldFromCollider.rot, colliderOrientation);

                        colliderPosition = worldFromCollider.pos + math.mul(worldFromCollider.rot, uniformScale * ((CylinderCollider*)collider)->Center);
                        newScale = (uniformScale * new float3(2.0f * radius, 0.5f * height, 2.0f * radius)) + adjustScale;

                        trs = float4x4.TRS(colliderPosition, colliderOrientation, newScale);
                        PrimitiveWriter.AddNoResize(new PrimitiveInfo
                        {
                            Trs = trs,
                            Flags = PrimitiveInfo.PrimitiveFlags.Cylinder | (isDynamic ? PrimitiveInfo.PrimitiveFlags.Dynamic : 0)
                        });
                        break;

                    case ColliderType.Box:
                        colliderOrientation = math.mul(worldFromCollider.rot, ((BoxCollider*)collider)->Orientation);
                        colliderPosition = worldFromCollider.pos + math.mul(worldFromCollider.rot, uniformScale * ((BoxCollider*)collider)->Center);
                        newScale = (uniformScale * ((BoxCollider*)collider)->Size) + adjustScale;

                        trs = float4x4.TRS(colliderPosition, colliderOrientation, newScale);
                        PrimitiveWriter.AddNoResize(new PrimitiveInfo
                        {
                            Trs = trs,
                            Flags = PrimitiveInfo.PrimitiveFlags.Box | (isDynamic ? PrimitiveInfo.PrimitiveFlags.Dynamic : 0)
                        });
                        break;

                    case ColliderType.Triangle:
                    case ColliderType.Quad:
                    case ColliderType.Convex:
                        DrawConvexFaces(ref ((ConvexCollider*)collider)->ConvexHull, worldFromCollider, DrawColliderUtility.GetColorIndex(isDynamic), uniformScale);
                        break;

                    case ColliderType.Sphere:
                        radius = uniformScale * (2.0f * ((SphereCollider*)collider)->Radius) + adjustScale;
                        newScale = radius * new float3(1.0f, 1.0f, 1.0f);
                        colliderPosition = worldFromCollider.pos + math.mul(worldFromCollider.rot, uniformScale * ((SphereCollider*)collider)->Center);

                        trs = float4x4.TRS(colliderPosition, worldFromCollider.rot, newScale);
                        PrimitiveWriter.AddNoResize(new PrimitiveInfo
                        {
                            Trs = trs,
                            Flags = PrimitiveInfo.PrimitiveFlags.Sphere | (isDynamic ? PrimitiveInfo.PrimitiveFlags.Dynamic : 0)
                        });
                        break;

                    case ColliderType.Capsule:
                        radius = uniformScale * ((CapsuleCollider*)collider)->Radius;
                        var vertex0 = math.transform(worldFromCollider, uniformScale * ((CapsuleCollider*)collider)->Vertex0);
                        var vertex1 = math.transform(worldFromCollider, uniformScale * ((CapsuleCollider*)collider)->Vertex1);

                        var axis = vertex1 - vertex0; //axis in wfc-space
                        colliderOrientation = Quaternion.FromToRotation(Vector3.up, -axis);
                        newScale = new float3(2.0f * radius, 0.5f * math.length(axis) + radius, 2.0f * radius) + adjustScale;

                        trs = float4x4.TRS(-0.5f * axis + vertex1, colliderOrientation, newScale);
                        PrimitiveWriter.AddNoResize(new PrimitiveInfo
                        {
                            Trs = trs,
                            Flags = PrimitiveInfo.PrimitiveFlags.Capsule | (isDynamic ? PrimitiveInfo.PrimitiveFlags.Dynamic : 0)
                        });

                        break;

                    case ColliderType.Mesh:
                        DrawMeshColliderFaces((MeshCollider*)collider, worldFromCollider, DrawColliderUtility.GetColorIndex(isDynamic), uniformScale);
                        break;

                    case ColliderType.Compound:
                        DrawCompoundColliderFaces((CompoundCollider*)collider, worldFromCollider, isDynamic, uniformScale);
                        break;

                    case ColliderType.Terrain:
                        //TODO [#3792]: Terrain should use DebugDraw rather than Gizmos and add uniform scale support.
                        //AppendMeshColliders.GetMeshes.AppendTerrain((TerrainCollider*)collider, worldFromCollider, ref results);
                        break;
                }
            }

            // Covers: collider->Type = Box, Triangle, Quad, Convex, Cylinder(before started using primitives)
            private static void DrawConvexFaces(ref ConvexHull hull, RigidTransform worldFromCollider,
                DebugDisplay.ColorIndex ci, float uniformScale = 1.0f)
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
                        DrawColliderUtility.DrawTriangle(vertices[0], vertices[2], vertices[1], worldFromCollider, ci);
                        DrawColliderUtility.DrawTriangle(vertices[2], vertices[0], vertices[3], worldFromCollider, ci);
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
                            DrawColliderUtility.DrawTriangle(vertices[0], vertices[2], vertices[1], worldFromCollider, ci);
                            vertices.Dispose();
                        }
                        scaledVertices.Dispose();
                    }
                }
            }

            private unsafe void DrawCompoundColliderFaces(CompoundCollider* compoundCollider, RigidTransform worldFromCollider,
                bool isDynamic, float uniformScale = 1.0f)
            {
                for (var i = 0; i < compoundCollider->Children.Length; i++)
                {
                    ref CompoundCollider.Child child = ref compoundCollider->Children[i];

                    ScaledMTransform mWorldFromCompound = new ScaledMTransform(worldFromCollider, uniformScale);
                    ScaledMTransform mWorldFromChild = ScaledMTransform.Mul(mWorldFromCompound, new MTransform(child.CompoundFromChild));
                    RigidTransform worldFromChild = new RigidTransform(mWorldFromChild.Rotation, mWorldFromChild.Translation);

                    DrawColliderFaces(child.Collider, worldFromChild, isDynamic, uniformScale);
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

        protected override void OnDestroy()
        {
            AppendMeshColliders.GetMeshes.ClearReferenceMeshes();
        }

        protected override void OnUpdate()
        {
#if UNITY_EDITOR
            if (!TryGetSingleton<PhysicsDebugDisplayData>(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliders == 0)
                return;

            var world = GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            if (world.NumBodies == 0)
                return;

            CompleteDependency();
            NativeReference<int> NumPrimitiveColliders = new NativeReference<int>(0, Allocator.TempJob);
            new CalculateNumberOfPrimitiveCollidersJob { NumPrimitiveColliders = NumPrimitiveColliders, Bodies = world.Bodies }.Run();
            NativeList<PrimitiveInfo> primitives = new NativeList<PrimitiveInfo>(NumPrimitiveColliders.Value, Allocator.TempJob);
            NumPrimitiveColliders.Dispose();

            // Gather the TRS data. If collider is a cached primitive type, it will write to the NativeList
            var colliderJobHandle = new DisplayColliderFacesJob
            {
                Bodies = world.Bodies,
                DynamicBodies = world.NumDynamicBodies,
                PrimitiveWriter = primitives.AsParallelWriter()
            }.Schedule(world.Bodies.Length, 16, Dependency);

            colliderJobHandle.Complete();

            // Push the TRS NativeList to a C# List for later rendering:
            DrawMeshUtility.SaveTRSList(primitives);
            primitives.Dispose();
#endif
        }
    }
}
