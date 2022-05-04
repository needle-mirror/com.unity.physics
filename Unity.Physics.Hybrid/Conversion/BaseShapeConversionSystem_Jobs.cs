using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    public partial class BaseShapeConversionSystem<T>
    {
#if !(UNITY_ANDROID && !UNITY_64) // !Android32
        // Getting memory alignment errors from HashUtility.Hash128 on Android32
        [BurstCompile]
#endif
        unsafe struct GeneratePhysicsShapeHashesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<ShapeComputationData> ComputationData;

            public NativeArray<Hash128> Hashes;

            static Aabb RotatedBoxAabb(float3 center, float3 size, quaternion orientation)
            {
                var extents = 0.5f * size;
                var aabb = new Aabb { Min = float.MaxValue, Max = float.MinValue };
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(-1f, -1f, -1f)))); // 000
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(-1f, -1f,  1f)))); // 001
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(-1f,  1f, -1f)))); // 010
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(-1f,  1f,  1f)))); // 011
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(1f, -1f, -1f))));  // 100
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(1f, -1f,  1f))));  // 101
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(1f,  1f, -1f))));  // 110
                aabb.Include(center + math.mul(orientation, math.mul(extents, new float3(1f,  1f,  1f))));  // 111
                return aabb;
            }

            static Aabb RotatedCylinderAabb(float3 center, float height, float radius, quaternion orientation)
            {
                var cylinder = new Aabb
                {
                    Min = center + new float3(2f * radius, -height, 2f * radius),
                    Max = center + new float3(2f * radius, height, 2f * radius)
                };
                return RotatedBoxAabb(cylinder.Center, cylinder.Extents, orientation);
            }

            public void Execute(int index)
            {
                var shapeData = ComputationData[index];

                var material = shapeData.Material;
                var collisionFilter = shapeData.CollisionFilter;
                var shapeType = shapeData.ShapeType;

                var bytes = new NativeList<byte>(Allocator.Temp);
                bytes.Append(ref shapeType);
                bytes.Append(ref shapeData.ForceUniqueIdentifier);
                bytes.Append(ref collisionFilter);
                bytes.Append(ref material);

                switch (shapeType)
                {
                    case ShapeType.Box:
                    {
                        var p = shapeData.BoxProperties;
                        bytes.Append(ref p);
                        var aabb = RotatedBoxAabb(p.Center, p.Size, shapeData.BoxProperties.Orientation);
                        HashableShapeInputs.GetQuantizedTransformations(shapeData.BodyFromShape, aabb, out var transformations);
                        bytes.Append(ref transformations);
                        break;
                    }
                    case ShapeType.Capsule:
                    {
                        var p = shapeData.CapsuleProperties;
                        bytes.Append(ref p);
                        var v0 = p.Vertex0;
                        var v1 = p.Vertex1;
                        var r = p.Radius;
                        var aabb = RotatedCylinderAabb(p.GetCenter(), p.GetHeight(), r, quaternion.LookRotationSafe(v0 - v1, math.up()));
                        HashableShapeInputs.GetQuantizedTransformations(shapeData.BodyFromShape, aabb, out var transformations);
                        bytes.Append(ref transformations);
                        break;
                    }
                    case ShapeType.Cylinder:
                    {
                        var p = shapeData.CylinderProperties;
                        bytes.Append(ref p);
                        var aabb = RotatedCylinderAabb(p.Center, p.Height, p.Radius, shapeData.CylinderProperties.Orientation);
                        HashableShapeInputs.GetQuantizedTransformations(shapeData.BodyFromShape, aabb, out var transformations);
                        bytes.Append(ref transformations);
                        break;
                    }
                    case ShapeType.Plane:
                    {
                        var v = shapeData.PlaneVertices;
                        bytes.Append(ref v);
                        var planeCenter = math.lerp(v.c0, v.c2, 0.5f);
                        var planeSize = math.abs(v.c0 - v.c2);
                        var aabb = RotatedBoxAabb(planeCenter, planeSize, quaternion.LookRotationSafe(v.c1 - v.c2, math.cross(v.c1 - v.c0, v.c2 - v.c1)));
                        HashableShapeInputs.GetQuantizedTransformations(shapeData.BodyFromShape, aabb, out var transformations);
                        bytes.Append(ref transformations);
                        break;
                    }
                    case ShapeType.Sphere:
                    {
                        var p = shapeData.SphereProperties;
                        bytes.Append(ref p);
                        var aabb = new Aabb { Min = p.Center - new float3(p.Radius), Max = p.Center + new float3(p.Radius) };
                        HashableShapeInputs.GetQuantizedTransformations(shapeData.BodyFromShape, aabb, out var transformations);
                        bytes.Append(ref transformations);
                        break;
                    }
                    case ShapeType.ConvexHull:
                    {
                        Hashes[index] = shapeData.Instance.Hash; // precomputed when gathering inputs
                        return;
                    }
                    case ShapeType.Mesh:
                    {
                        Hashes[index] = shapeData.Instance.Hash; // precomputed when gathering inputs
                        return;
                    }
                }

                Hashes[index] = HashUtility.Hash128(bytes.GetUnsafeReadOnlyPtr(), bytes.Length);
            }
        }

        [BurstCompile]
        struct CreateBlobAssetsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<ShapeComputationData> ComputeData;
            [ReadOnly] public NativeArray<int> ToComputeTable;

            public NativeArray<BlobAssetReference<Collider>> BlobAssets;

            public void Execute(int i)
            {
                var shapeData = ComputeData[ToComputeTable[i]];
                switch (shapeData.ShapeType)
                {
                    case ShapeType.Box:
                    {
                        BlobAssets[i] = BoxCollider.Create(
                            shapeData.BoxProperties, shapeData.CollisionFilter, shapeData.Material
                        );
                        return;
                    }
                    case ShapeType.Capsule:
                    {
                        BlobAssets[i] = CapsuleCollider.Create(
                            shapeData.CapsuleProperties, shapeData.CollisionFilter, shapeData.Material
                        );
                        return;
                    }
                    case ShapeType.Cylinder:
                    {
                        BlobAssets[i] = CylinderCollider.Create(
                            shapeData.CylinderProperties, shapeData.CollisionFilter, shapeData.Material
                        );
                        return;
                    }
                    case ShapeType.Plane:
                    {
                        var v = shapeData.PlaneVertices;
                        BlobAssets[i] = PolygonCollider.CreateQuad(
                            v.c0, v.c1, v.c2, v.c3, shapeData.CollisionFilter, shapeData.Material
                        );
                        return;
                    }
                    case ShapeType.Sphere:
                    {
                        BlobAssets[i] = SphereCollider.Create(
                            shapeData.SphereProperties, shapeData.CollisionFilter, shapeData.Material
                        );
                        return;
                    }
                    // Note : Mesh and Hull are not computed here as they are in a separated set of jobs
                    default:
                        return;
                }
            }
        }

        struct ConvertToHashMapJob<TKey, TValue> : IJob
            where TKey : struct, IEquatable<TKey>
            where TValue : struct
        {
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<KeyValuePair<TKey, TValue>> Input;

            public NativeParallelHashMap<TKey, TValue> Output;

            public void Execute()
            {
                for (int i = 0, count = Input.Length; i < count; ++i)
                {
                    var input = Input[i];
                    Output.TryAdd(input.Key, input.Value);
                }
            }
        }

        struct DisposeContainerJob<TElement> : IJob where TElement : struct
        {
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<TElement> Container;

            public void Execute() {}
        }
    }
}
