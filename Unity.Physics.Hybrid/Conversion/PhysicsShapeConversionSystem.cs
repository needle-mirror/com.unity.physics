using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 1)]
    public sealed class PhysicsShapeConversionSystem : BaseShapeConversionSystem<PhysicsShapeAuthoring>
    {
        static Material ProduceMaterial(PhysicsShapeAuthoring shape) => shape.GetMaterial();

        static CollisionFilter ProduceCollisionFilter(PhysicsShapeAuthoring shape) => shape.GetFilter();

        protected override bool ShouldConvertShape(PhysicsShapeAuthoring shape) => shape.enabled;

        protected override GameObject GetPrimaryBody(PhysicsShapeAuthoring shape) => shape.GetPrimaryBody();

        internal override ShapeComputationData GenerateComputationData(
            PhysicsShapeAuthoring shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles
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
                    res.CapsuleProperties = shape.GetCapsuleProperties(out orientation)
                        .BakeToBodySpace(localToWorld, shapeToWorld, out _, out _, ref orientation);
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
                    using (var pointCloud = new NativeList<float3>(65535, Allocator.Temp))
                    {
                        shape.GetBakedConvexProperties(pointCloud, out var hullGenerationParameters, out res.Instance.Hash);
                        if (pointCloud.Length == 0)
                        {
                            throw new InvalidOperationException(
                                $"No vertices associated with {shape.name}. Add a {typeof(MeshFilter)} component or assign a readable {nameof(PhysicsShapeAuthoring.CustomMesh)}."
                            );
                        }
                        res.ConvexHullProperties = new ConvexInput
                        {
                            GenerationParameters = hullGenerationParameters.ToRunTime(),
                            PointCount = pointCloud.Length,
                            PointsStart = allConvexHullPoints.Length,
                            Filter = res.CollisionFilter,
                            Material =  res.Material
                        };
                        allConvexHullPoints.AddRange(pointCloud);
                    }
                    break;
                }
                case ShapeType.Mesh:
                {
                    const int defaultVertexCount = 2048;
                    using (var vertices = new NativeList<float3>(defaultVertexCount, Allocator.Temp))
                    using (var triangles = new NativeList<int3>(defaultVertexCount - 2, Allocator.Temp))
                    {
                        shape.GetBakedMeshProperties(vertices, triangles, out res.Instance.Hash);
                        if (vertices.Length == 0 || triangles.Length == 0)
                        {
                            throw new InvalidOperationException(
                                $"Invalid mesh data associated with {shape.name}. " +
                                $"Add a {typeof(MeshFilter)} component or assign a {nameof(PhysicsShapeAuthoring.CustomMesh)}. " +
                                "Ensure that you have enabled Read/Write on the mesh's import settings."
                            );
                        }
                        res.MeshProperties = new MeshInput
                        {
                            VerticesStart = allMeshVertices.Length,
                            VertexCount = vertices.Length,
                            TrianglesStart = allMeshTriangles.Length,
                            TriangleCount = triangles.Length,
                            Filter = res.CollisionFilter,
                            Material =  res.Material
                        };
                        allMeshVertices.AddRange(vertices);
                        allMeshTriangles.AddRange(triangles);
                    }
                    break;
                }
            }

            return res;
        }
    }
}
