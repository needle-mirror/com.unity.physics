using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    public class PhysicsShapeConversionSystem : BaseShapeConversionSystem<PhysicsShape>
    {
        static Material ProduceMaterial(PhysicsShape shape)
        {
            // TODO: TBD how we will author editor content for other shape flags
            var flags = new Material.MaterialFlags();
            if (shape.IsTrigger)
            {
                flags = Material.MaterialFlags.IsTrigger;
            }
            else if (shape.RaisesCollisionEvents)
            {
                flags = Material.MaterialFlags.EnableCollisionEvents;
            }

            return new Material
            {
                Friction = shape.Friction.Value,
                FrictionCombinePolicy = shape.Friction.CombineMode,
                Restitution = shape.Restitution.Value,
                RestitutionCombinePolicy = shape.Restitution.CombineMode,
                Flags = flags
            };
        }

        static CollisionFilter ProduceCollisionFilter(PhysicsShape shape)
        {
            // TODO: determine optimal workflow for specifying group index
            return new CollisionFilter
            {
                BelongsTo = unchecked((uint)shape.BelongsTo),
                CollidesWith = unchecked((uint)shape.CollidesWith),
            };
        }

        protected override bool ShouldConvertShape(PhysicsShape shape) => shape.enabled;

        protected override GameObject GetPrimaryBody(PhysicsShape shape) => shape.GetPrimaryBody();

        protected override BlobAssetReference<Collider> ProduceColliderBlob(PhysicsShape shape)
        {
            var material = ProduceMaterial(shape);
            var collisionFilter = ProduceCollisionFilter(shape);

            var blob = new BlobAssetReference<Collider>();
            switch (shape.ShapeType)
            {
                case ShapeType.Box:
                    shape.GetBakedBoxProperties(out var center, out var size, out var orientation, out var convexRadius);
                    blob = BoxCollider.Create(
                        center,
                        orientation,
                        size,
                        convexRadius,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Capsule:
                    shape.GetBakedCapsuleProperties(out center, out var height, out var radius, out orientation, out var v0, out var v1);
                    blob = CapsuleCollider.Create(
                        v0,
                        v1,
                        radius,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Sphere:
                    shape.GetBakedSphereProperties(out center, out radius, out orientation);
                    blob = SphereCollider.Create(
                        center,
                        radius,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Cylinder:
                    shape.GetBakedCylinderProperties(out center, out height, out radius, out orientation, out convexRadius);
                    blob = CylinderCollider.Create(
                        center,
                        height,
                        radius,
                        orientation,
                        convexRadius,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Plane:
                    shape.GetBakedPlaneProperties(out v0, out v1, out var v2, out var v3);
                    blob = PolygonCollider.CreateQuad(
                        v0,
                        v1,
                        v2,
                        v3,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.ConvexHull:
                    var pointCloud = new NativeList<float3>(65535, Allocator.Temp);
                    shape.GetConvexHullProperties(pointCloud);
                    if (pointCloud.Length == 0)
                    {
                        pointCloud.Dispose();
                        throw new InvalidOperationException(
                            $"No vertices associated with {shape.name}. Add a {typeof(MeshFilter)} component or assign {nameof(PhysicsShape.CustomMesh)}."
                        );
                    }
                    shape.GetBakedConvexProperties(pointCloud, out radius);
                    blob = ConvexCollider.Create(
                        pointCloud,
                        radius,
                        1f,
                        collisionFilter,
                        material);
                    pointCloud.Dispose();
                    break;
                case ShapeType.Mesh:
                    // TODO: no convex radius?
                    var mesh = shape.GetMesh();
                    if (mesh == null)
                    {
                        throw new InvalidOperationException(
                            $"No mesh associated with {shape.name}. Add a {typeof(MeshFilter)} component or assign {nameof(PhysicsShape.CustomMesh)}."
                        );
                    }
                    else
                    {
                        pointCloud = new NativeList<float3>(mesh.vertexCount, Allocator.Temp);
                        var triangles = new NativeList<int>(mesh.vertexCount, Allocator.Temp);
                        shape.GetBakedMeshProperties(pointCloud, triangles);
                        blob = MeshCollider.Create(pointCloud.ToArray(), triangles.ToArray(), collisionFilter, material);
                        pointCloud.Dispose();
                        triangles.Dispose();
                    }
                    break;
                default:
                    throw new UnimplementedShapeException(shape.ShapeType);
            }
            return blob;
        }

        protected override byte GetCustomFlags(PhysicsShape shape) => shape.CustomFlags;
    }
}
