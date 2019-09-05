using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    public sealed class PhysicsShapeConversionSystem : BaseShapeConversionSystem<PhysicsShapeAuthoring>
    {
        static Material ProduceMaterial(PhysicsShapeAuthoring shape)
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
                Flags = flags,
                CustomTags = shape.CustomTags.Value
            };
        }

        static CollisionFilter ProduceCollisionFilter(PhysicsShapeAuthoring shape)
        {
            // TODO: determine optimal workflow for specifying group index
            return new CollisionFilter
            {
                BelongsTo = shape.BelongsTo.Value,
                CollidesWith = shape.CollidesWith.Value
            };
        }

        protected override bool ShouldConvertShape(PhysicsShapeAuthoring shape) => shape.enabled;

        protected override GameObject GetPrimaryBody(PhysicsShapeAuthoring shape) => shape.GetPrimaryBody();

        protected override BlobAssetReference<Collider> ProduceColliderBlob(PhysicsShapeAuthoring shape)
        {
            var material = ProduceMaterial(shape);
            var collisionFilter = ProduceCollisionFilter(shape);

            BlobAssetReference<Collider> blob = default;
            switch (shape.ShapeType)
            {
                case ShapeType.Box:
                    blob = BoxCollider.Create(
                        shape.GetBakedBoxProperties(),
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Capsule:
                    blob = CapsuleCollider.Create(
                        shape.GetBakedCapsuleProperties(out var center, out var height, out var orientation),
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Sphere:
                    blob = SphereCollider.Create(
                        shape.GetBakedSphereProperties(out orientation),
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Cylinder:
                    blob = CylinderCollider.Create(
                        shape.GetBakedCylinderProperties(),
                        collisionFilter,
                        material);
                    break;
                case ShapeType.Plane:
                    shape.GetBakedPlaneProperties(out var v0, out var v1, out var v2, out var v3);
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
                        throw new InvalidOperationException(
                            $"No vertices associated with {shape.name}. Add a {typeof(MeshFilter)} component or assign a readable {nameof(PhysicsShapeAuthoring.CustomMesh)}."
                        );
                    }
                    shape.GetBakedConvexProperties(pointCloud, out var hullGenerationParameters);
                    RegisterConvexColliderDeferred(
                        shape, pointCloud, hullGenerationParameters.ToRunTime(), collisionFilter, material
                    );
                    pointCloud.Dispose();
                    break;
                case ShapeType.Mesh:
                    const int defaultVertexCount = 2048;
                    pointCloud = new NativeList<float3>(defaultVertexCount, Allocator.Temp);
                    var triangles = new NativeList<int>((defaultVertexCount - 2) * 3, Allocator.Temp);
                    shape.GetBakedMeshProperties(pointCloud, triangles);

                    if (pointCloud.Length == 0 || triangles.Length == 0)
                    {
                        triangles.Dispose();
                        pointCloud.Dispose();
                        throw new InvalidOperationException(
                            $"Invalid mesh data associated with {shape.name}. " +
                            $"Add a {typeof(MeshFilter)} component or assign a {nameof(PhysicsShapeAuthoring.CustomMesh)}. " +
                            "Ensure that you have enabled Read/Write on the mesh's import settings."
                        );
                    }

                    RegisterMeshColliderDeferred(shape, pointCloud, triangles, collisionFilter, material);
                    triangles.Dispose();
                    pointCloud.Dispose();
                    break;
                default:
                    throw new UnimplementedShapeException(shape.ShapeType);
            }
            return blob;
        }
    }
}
