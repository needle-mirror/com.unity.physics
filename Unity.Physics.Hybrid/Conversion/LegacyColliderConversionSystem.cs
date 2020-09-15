#if LEGACY_PHYSICS
using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using LegacyPhysics = UnityEngine.Physics;
using LegacyCollider = UnityEngine.Collider;
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;

namespace Unity.Physics.Authoring
{
    public abstract class BaseLegacyColliderConversionSystem<T> : BaseShapeConversionSystem<T> where T : LegacyCollider
    {
        static readonly IReadOnlyDictionary<PhysicMaterialCombine, Material.CombinePolicy> k_MaterialCombineLookup =
            new Dictionary<PhysicMaterialCombine, Material.CombinePolicy>
            {
                { PhysicMaterialCombine.Average, Material.CombinePolicy.ArithmeticMean },
                { PhysicMaterialCombine.Maximum, Material.CombinePolicy.Maximum },
                { PhysicMaterialCombine.Minimum, Material.CombinePolicy.Minimum }
            };

        static PhysicMaterial DefaultMaterial
        {
            get
            {
                if (s_DefaultMaterial == null)
                    s_DefaultMaterial = new PhysicMaterial { hideFlags = HideFlags.DontSave };
                return s_DefaultMaterial;
            }
        }
        static PhysicMaterial s_DefaultMaterial;

        Material ProduceMaterial(LegacyCollider collider)
        {
            // n.b. need to manually opt in to collision events with legacy colliders if desired
            var material = new Material();
            if (collider.isTrigger)
            {
                material.CollisionResponse = CollisionResponsePolicy.RaiseTriggerEvents;
            }

            var legacyMaterial = collider.sharedMaterial;
            if (legacyMaterial == null)
                legacyMaterial = DefaultMaterial;
            else
                DeclareAssetDependency(collider.gameObject, legacyMaterial);

            material.Friction = legacyMaterial.dynamicFriction;
            if (k_MaterialCombineLookup.TryGetValue(legacyMaterial.frictionCombine, out var combine))
                material.FrictionCombinePolicy = combine;
            else
                Debug.LogWarning(
                    $"{collider.name} uses {legacyMaterial.name}, which specifies non-convertible mode {legacyMaterial.frictionCombine} for {nameof(PhysicMaterial.frictionCombine)}.",
                    collider
                );

            material.Restitution = legacyMaterial.bounciness;
            if (k_MaterialCombineLookup.TryGetValue(legacyMaterial.bounceCombine, out combine))
                material.RestitutionCombinePolicy = combine;
            else
                Debug.LogWarning(
                    $"{collider.name} uses {legacyMaterial.name}, which specifies non-convertible mode {legacyMaterial.bounceCombine} for {nameof(PhysicMaterial.bounceCombine)}.",
                    collider
                );

            return material;
        }

        internal override ShapeComputationData GenerateComputationData(
            T shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices,
            NativeList<int3> allMeshTriangles, HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            return new ShapeComputationData
            {
                Instance = colliderInstance,
                Material = ProduceMaterial(shape),
                CollisionFilter = ProduceCollisionFilter(shape)
            };
        }

        protected static CollisionFilter ProduceCollisionFilter(LegacyCollider collider)
        {
            var layer = collider.gameObject.layer;
            var filter = new CollisionFilter { BelongsTo = (uint)(1 << collider.gameObject.layer) };
            for (var i = 0; i < 32; ++i)
                filter.CollidesWith |= (uint)(LegacyPhysics.GetIgnoreLayerCollision(layer, i) ? 0 : 1 << i);
            return filter;
        }

        protected override bool ShouldConvertShape(T shape) => shape.enabled;
        protected override GameObject GetPrimaryBody(T shape) => shape.GetPrimaryBody();
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 3)]
    public sealed class LegacyBoxColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacyBox>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacyBox shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);
            res.ShapeType = ShapeType.Box;

            var shapeLocalToWorld = shape.transform.localToWorldMatrix;
            var worldCenter = math.mul(shapeLocalToWorld, new float4(shape.center, 1f));
            var transformRotation  = shape.transform.rotation;
            var rigidBodyTransform = Math.DecomposeRigidBodyTransform(shapeLocalToWorld);
            var shapeFromWorld = math.inverse(new float4x4(rigidBodyTransform));

            var orientationFixup = math.inverse(math.mul(math.inverse(transformRotation), rigidBodyTransform.rot));

            var geometry = new BoxGeometry
            {
                Center = math.mul(shapeFromWorld, worldCenter).xyz,
                Orientation = orientationFixup
            };

            var linearScale = float4x4.TRS(float3.zero, math.inverse(orientationFixup), shape.transform.lossyScale).DecomposeScale();
            geometry.Size = math.abs(shape.size * linearScale);

            geometry.BevelRadius = math.min(ConvexHullGenerationParameters.Default.BevelRadius, math.cmin(geometry.Size) * 0.5f);

            res.BoxProperties = geometry;

            return res;
        }
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 3)]
    public sealed class LegacyCapsuleColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacyCapsule>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacyCapsule shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);

            res.ShapeType = ShapeType.Capsule;

            var shapeLocalToWorld   = (float4x4)shape.transform.localToWorldMatrix;
            var transformRotation   = shape.transform.rotation;
            var rigidBodyTransform  = Math.DecomposeRigidBodyTransform(shapeLocalToWorld);
            var orientationFixup    = math.inverse(math.mul(math.inverse(transformRotation), rigidBodyTransform.rot));
            var linearScale         = float4x4.TRS(float3.zero, math.inverse(orientationFixup), shape.transform.lossyScale).DecomposeScale();

            // radius is max of the two non-height axes
            var radius = shape.radius * math.cmax(new float3(math.abs(linearScale)) { [shape.direction] = 0f });

            var ax = new float3 { [shape.direction] = 1f };
            var vertex = ax * (0.5f * shape.height);
            var worldCenter = math.mul(shapeLocalToWorld, new float4(shape.center, 0f));
            var offset = math.mul(math.inverse(new float4x4(rigidBodyTransform)), worldCenter).xyz - shape.center * math.abs(linearScale);

            var v0 = math.mul(orientationFixup, offset + ((float3)shape.center + vertex) * math.abs(linearScale) - ax * radius);
            var v1 = math.mul(orientationFixup, offset + ((float3)shape.center - vertex) * math.abs(linearScale) + ax * radius);

            res.CapsuleProperties = new CapsuleGeometry { Vertex0 = v0, Vertex1 = v1, Radius = radius };

            return res;
        }
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 3)]
    public sealed class LegacySphereColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacySphere>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacySphere shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);
            res.ShapeType = ShapeType.Sphere;

            var shapeLocalToWorld = shape.transform.localToWorldMatrix;
            var worldCenter = math.mul(shapeLocalToWorld, new float4(shape.center, 1f));
            var transformRotation  = shape.transform.rotation;
            var rigidBodyTransform  = Math.DecomposeRigidBodyTransform(shapeLocalToWorld);
            var orientationFixup    = math.inverse(math.mul(math.inverse(transformRotation), rigidBodyTransform.rot));

            var shapeFromWorld = math.inverse(new float4x4(rigidBodyTransform));
            var center = math.mul(shapeFromWorld, worldCenter).xyz;

            var linearScale = float4x4.TRS(float3.zero, math.inverse(orientationFixup), shape.transform.lossyScale).DecomposeScale();
            var radius = shape.radius * math.cmax(math.abs(linearScale));

            res.SphereProperties = new SphereGeometry { Center = center, Radius = radius };

            return res;
        }
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 5)]
    public sealed class LegacyMeshColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacyMesh>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacyMesh shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            if (shape.sharedMesh == null)
            {
                throw new InvalidOperationException(
                    $"No {nameof(LegacyMesh.sharedMesh)} assigned to {typeof(MeshCollider)} on {shape.name}."
                );
            }

            if (!shape.sharedMesh.IsValidForConversion(shape.gameObject))
            {
                throw new InvalidOperationException(
                    $"Mesh '{shape.sharedMesh}' assigned to {typeof(MeshCollider)} on {shape.name} is not readable. Ensure that you have enabled Read/Write on its import settings."
                );
            }

            meshAssets.Add(shape.sharedMesh);

            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);

            if (shape.convex)
            {
                res.ShapeType = ShapeType.ConvexHull;
                res.ConvexHullProperties.Material = res.Material;
                res.ConvexHullProperties.Filter = res.CollisionFilter;
                res.ConvexHullProperties.GenerationParameters = ConvexHullGenerationParameters.Default;
            }
            else
            {
                res.ShapeType = ShapeType.Mesh;
                res.MeshProperties.Material = res.Material;
                res.MeshProperties.Filter = res.CollisionFilter;
                res.ConvexHullProperties.GenerationParameters = default;
            }

            var transform = shape.transform;
            var rigidBodyTransform = Math.DecomposeRigidBodyTransform(transform.localToWorldMatrix);
            var bakeFromShape = math.mul(math.inverse(new float4x4(rigidBodyTransform)), transform.localToWorldMatrix);

            res.Instance.Hash = HashableShapeInputs.GetHash128(
                0u,
                res.ConvexHullProperties.GenerationParameters,
                res.Material,
                res.CollisionFilter,
                bakeFromShape,
                new NativeArray<HashableShapeInputs>(1, Allocator.Temp) { [0] = HashableShapeInputs.FromMesh(shape.sharedMesh, float4x4.identity) },
                default,
                default
            );

            if (BlobComputationContext.NeedToComputeBlobAsset(res.Instance.Hash))
            {
                if (shape.convex && TryGetRegisteredConvexInputs(res.Instance.Hash, out var convexInputs))
                {
                    res.ConvexHullProperties.PointsStart = convexInputs.PointsStart;
                    res.ConvexHullProperties.PointCount = convexInputs.PointCount;
                }
                else if (!shape.convex && TryGetRegisteredMeshInputs(res.Instance.Hash, out var meshInputs))
                {
                    res.MeshProperties.VerticesStart = meshInputs.VerticesStart;
                    res.MeshProperties.VertexCount = meshInputs.VertexCount;
                    res.MeshProperties.TrianglesStart = meshInputs.TrianglesStart;
                    res.MeshProperties.TriangleCount = meshInputs.TriangleCount;
                }
                else
                {
                    var pointCloud = new NativeList<float3>(shape.sharedMesh.vertexCount, Allocator.Temp);
                    var triangles = new NativeList<int3>(shape.sharedMesh.triangles.Length / 3, Allocator.Temp);
                    PhysicsShapeAuthoring.AppendMeshPropertiesToNativeBuffers(
                        float4x4.identity, shape.sharedMesh,
                        pointCloud, triangles,
                        default, default
                    );
                    for (int i = 0, count = pointCloud.Length; i < count; ++i)
                        pointCloud[i] = math.mul(bakeFromShape, new float4(pointCloud[i], 1f)).xyz;

                    if (shape.convex)
                    {
                        res.ConvexHullProperties.PointsStart = allConvexHullPoints.Length;
                        res.ConvexHullProperties.PointCount = pointCloud.Length;
                        allConvexHullPoints.AddRange(pointCloud);
                    }
                    else
                    {
                        if (pointCloud.Length == 0 || triangles.Length == 0)
                        {
                            throw new InvalidOperationException(
                                $"Invalid mesh data associated with {shape.name}. " +
                                "Ensure that you have enabled Read/Write on the mesh's import settings."
                            );
                        }

                        res.MeshProperties.VerticesStart = allMeshVertices.Length;
                        res.MeshProperties.VertexCount = pointCloud.Length;
                        res.MeshProperties.TrianglesStart = allMeshTriangles.Length;
                        res.MeshProperties.TriangleCount = triangles.Length;
                        allMeshVertices.AddRange(pointCloud);
                        allMeshTriangles.AddRange(triangles);
                    }
                }
            }

            return res;
        }
    }
}
#endif
