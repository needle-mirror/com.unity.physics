using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("stewart", 6)]
    public sealed class PhysicsShapeConversionSystem : BaseShapeConversionSystem<PhysicsShapeAuthoring>
    {
        Material ProduceMaterial(PhysicsShapeAuthoring shape)
        {
            var materialTemplate = shape.MaterialTemplate;
            if (materialTemplate != null)
                DeclareAssetDependency(shape.gameObject, materialTemplate);
            return shape.GetMaterial();
        }

        static CollisionFilter ProduceCollisionFilter(PhysicsShapeAuthoring shape) => shape.GetFilter();

        protected override bool ShouldConvertShape(PhysicsShapeAuthoring shape) => shape.enabled;

        protected override GameObject GetPrimaryBody(PhysicsShapeAuthoring shape) => shape.GetPrimaryBody();

        internal override ShapeComputationData GenerateComputationData(
            PhysicsShapeAuthoring shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = new ShapeComputationData();
            res.Instance = colliderInstance;
            res.Material = ProduceMaterial(shape);
            res.CollisionFilter = ProduceCollisionFilter(shape);
            res.ForceUniqueIdentifier = shape.ForceUnique ? (uint)shape.GetInstanceID() : 0u;

            var transform = shape.transform;
            var localToWorld = transform.localToWorldMatrix;
            var shapeToWorld = shape.GetShapeToWorldMatrix();
            EulerAngles orientation;

            res.ShapeType = shape.ShapeType;
            switch (shape.ShapeType)
            {
                case ShapeType.Box:
                {
                    res.BoxProperties = shape.GetBoxProperties(out orientation)
                        .BakeToBodySpace(localToWorld, shapeToWorld, orientation);
                    break;
                }
                case ShapeType.Capsule:
                {
                    res.CapsuleProperties = shape.GetCapsuleProperties()
                        .BakeToBodySpace(localToWorld, shapeToWorld)
                        .ToRuntime();
                    break;
                }
                case ShapeType.Sphere:
                {
                    res.SphereProperties = shape.GetSphereProperties(out orientation)
                        .BakeToBodySpace(localToWorld, shapeToWorld, ref orientation);
                    break;
                }
                case ShapeType.Cylinder:
                {
                    res.CylinderProperties = shape.GetCylinderProperties(out orientation)
                        .BakeToBodySpace(localToWorld, shapeToWorld, orientation);
                    break;
                }
                case ShapeType.Plane:
                {
                    shape.GetPlaneProperties(out var center, out var size, out orientation);
                    PhysicsShapeExtensions.BakeToBodySpace(
                        center, size, orientation, localToWorld, shapeToWorld,
                        out res.PlaneVertices.c0, out res.PlaneVertices.c1, out res.PlaneVertices.c2, out res.PlaneVertices.c3
                    );
                    break;
                }
                case ShapeType.ConvexHull:
                {
                    res.ConvexHullProperties.Filter = res.CollisionFilter;
                    res.ConvexHullProperties.Material = res.Material;
                    res.ConvexHullProperties.GenerationParameters = shape.ConvexHullGenerationParameters.ToRunTime();

                    res.Instance.Hash = shape.GetBakedConvexInputs(meshAssets);

                    if (BlobComputationContext.NeedToComputeBlobAsset(res.Instance.Hash))
                    {
                        if (TryGetRegisteredConvexInputs(res.Instance.Hash, out var convexInputs))
                        {
                            res.ConvexHullProperties.PointCount = convexInputs.PointCount;
                            res.ConvexHullProperties.PointsStart = convexInputs.PointsStart;
                        }
                        else
                        {
                            using (var pointCloud = new NativeList<float3>(65535, Allocator.Temp))
                            {
                                shape.GetBakedConvexProperties(pointCloud);
                                if (pointCloud.Length == 0)
                                {
                                    throw new InvalidOperationException(
                                        $"No vertices associated with {shape.name}. Add a {typeof(MeshFilter)} component or assign a readable {nameof(PhysicsShapeAuthoring.CustomMesh)}."
                                    );
                                }

                                res.ConvexHullProperties.PointCount = pointCloud.Length;
                                res.ConvexHullProperties.PointsStart = allConvexHullPoints.Length;
                                allConvexHullPoints.AddRange(pointCloud);
                            }
                        }
                    }

                    break;
                }
                case ShapeType.Mesh:
                {
                    res.MeshProperties.Filter = res.CollisionFilter;
                    res.MeshProperties.Material = res.Material;

                    res.Instance.Hash = shape.GetBakedMeshInputs();

                    if (BlobComputationContext.NeedToComputeBlobAsset(res.Instance.Hash))
                    {
                        if (TryGetRegisteredMeshInputs(res.Instance.Hash, out var meshInputs))
                        {
                            res.MeshProperties.VerticesStart = meshInputs.VerticesStart;
                            res.MeshProperties.VertexCount = meshInputs.VertexCount;
                            res.MeshProperties.TrianglesStart = meshInputs.TrianglesStart;
                            res.MeshProperties.TriangleCount = meshInputs.TriangleCount;
                        }
                        else
                        {
                            const int defaultVertexCount = 2048;
                            using (var vertices = new NativeList<float3>(defaultVertexCount, Allocator.Temp))
                            using (var triangles = new NativeList<int3>(defaultVertexCount - 2, Allocator.Temp))
                            {
                                shape.GetBakedMeshProperties(vertices, triangles, meshAssets);
                                if (vertices.Length == 0 || triangles.Length == 0)
                                {
                                    throw new InvalidOperationException(
                                        $"Invalid mesh data associated with {shape.name}. " +
                                        $"Add a {typeof(MeshFilter)} component or assign a {nameof(PhysicsShapeAuthoring.CustomMesh)}. " +
                                        "Ensure that you have enabled Read/Write on the mesh's import settings."
                                    );
                                }

                                res.MeshProperties.VerticesStart = allMeshVertices.Length;
                                res.MeshProperties.VertexCount = vertices.Length;
                                res.MeshProperties.TrianglesStart = allMeshTriangles.Length;
                                res.MeshProperties.TriangleCount = triangles.Length;
                                allMeshVertices.AddRange(vertices);
                                allMeshTriangles.AddRange(triangles);
                            }
                        }
                    }

                    break;
                }
            }

            return res;
        }
    }
}
