using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(FirstPassPhysicsBodyConversionSystem))]
    [UpdateAfter(typeof(FirstPassLegacyRigidbodyConversionSystem))]
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
                CategoryBits = unchecked((uint)shape.BelongsTo),
                MaskBits = unchecked((uint)shape.CollidesWith),
            };
        }

        protected override bool ShouldConvertShape(PhysicsShape shape) => shape.enabled;

        protected override GameObject GetPrimaryBody(PhysicsShape shape) => shape.GetPrimaryBody();

        protected override BlobAssetReference<Collider> ProduceColliderBlob(PhysicsShape shape)
        {
            var material = ProduceMaterial(shape);
            var collisionFilter = ProduceCollisionFilter(shape);

            var blob = new BlobAssetReference<Collider>();
            shape.GetBakeTransformation(out var linearScalar, out var radiusScalar);
            switch (shape.ShapeType)
            {
                case ShapeType.Box:
                    shape.GetBoxProperties(out var center, out var size, out quaternion orientation);
                    blob = BoxCollider.Create(
                        center * linearScalar,
                        orientation,
                        size * linearScalar,
                        shape.ConvexRadius * radiusScalar,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Capsule:
                    shape.GetCapsuleProperties(out var v0, out var v1, out var radius);
                    blob = CapsuleCollider.Create(
                        v0 * linearScalar,
                        v1 * linearScalar,
                        radius * radiusScalar,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Sphere:
                    shape.GetSphereProperties(out center, out radius, out orientation);
                    blob = SphereCollider.Create(
                        center * linearScalar,
                        radius * radiusScalar,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Cylinder:
                    shape.GetCylinderProperties(out center, out var height, out radius, out orientation);
                    blob = CylinderCollider.Create(
                        center,
                        height,
                        radius,
                        orientation,
                        shape.ConvexRadius * radiusScalar,
                        linearScalar,
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Plane:
                    shape.GetPlaneProperties(out v0, out v1, out var v2, out var v3);
                    blob = PolygonCollider.CreateQuad(
                        v0 * linearScalar,
                        v1 * linearScalar,
                        v2 * linearScalar,
                        v3 * linearScalar,
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
                    blob = ConvexCollider.Create(
                        pointCloud,
                        shape.ConvexRadius * radiusScalar,
                        linearScalar,
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
                        blob = MeshCollider.Create(mesh.GetScaledVertices(linearScalar), mesh.triangles, collisionFilter, material);
                    }
                    break;
                default:
                    break;
            }
            return blob;
        }

        protected override byte GetCustomFlags(PhysicsShape shape) => shape.CustomFlags;
    }
}
