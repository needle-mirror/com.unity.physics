#if LEGACY_PHYSICS
using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using LegacyPhysics = UnityEngine.Physics;
using LegacyCollider = UnityEngine.Collider;
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;
using UnityMesh = UnityEngine.Mesh;

namespace Unity.Physics.Authoring
{
    public abstract class ColliderBaker<T> : BaseColliderBaker<T> where T : UnityEngine.Collider
    {
        public static List<UnityEngine.Collider> colliderComponents = new List<UnityEngine.Collider>();
        public static List<PhysicsShapeAuthoring> physicsShapeComponents = new List<PhysicsShapeAuthoring>();

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

        internal Material ProduceMaterial(LegacyCollider collider)
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
                DependsOn(legacyMaterial);

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

        internal ShapeComputationDataBaking GenerateComputationDataGeneric(LegacyCollider shape, ColliderInstanceBaking colliderInstance)
        {
            return new ShapeComputationDataBaking
            {
                Instance = colliderInstance,
                Material = ProduceMaterial(shape),
                CollisionFilter = ProduceCollisionFilter(shape)
            };
        }

        internal CollisionFilter ProduceCollisionFilter(LegacyCollider collider)
        {
            // Declaring the dependency on the GameObject with GetLayer, so the baker rebakes if the layer changes
            var layer = GetLayer(collider);
            var filter = new CollisionFilter { BelongsTo = (uint)(1 << collider.gameObject.layer) };
            for (var i = 0; i < 32; ++i)
                filter.CollidesWith |= (uint)(LegacyPhysics.GetIgnoreLayerCollision(layer, i) ? 0 : 1 << i);
            return filter;
        }

        bool ShouldConvertShape(T collider)
        {
            return collider.enabled;
        }

        internal abstract ShapeComputationDataBaking GenerateComputationData(T shapeData, ColliderInstanceBaking colliderInstance, Entity colliderEntity);

        ShapeComputationDataBaking GetInputDataFromAuthoringComponent(T shape, Entity colliderEntity)
        {
            GameObject shapeGameObject = shape.gameObject;
            var body = GetPrimaryBody(shapeGameObject, out bool hasBodyComponent);
            var child = shapeGameObject;
            var shapeInstanceID = shape.GetInstanceID();

            // Track dependencies to the transforms
            Transform shapeTransform = GetComponent<Transform>(shape);
            Transform bodyTransform = GetComponent<Transform>(body);
            var instance = new ColliderInstanceBaking
            {
                AuthoringComponentId = shapeInstanceID,
                BodyEntity = GetEntity(body),
                ShapeEntity = GetEntity(shapeGameObject),
                ChildEntity = GetEntity(child),
                BodyFromShape = ColliderInstanceBaking.GetCompoundFromChild(shapeTransform, bodyTransform),
                BodyInHierarchy = hasBodyComponent
            };

            var data = GenerateComputationData(shape, instance, colliderEntity);

            data.Instance.ConvertedAuthoringInstanceID = shapeInstanceID;
            data.Instance.ConvertedBodyInstanceID = bodyTransform.GetInstanceID();

            // The root colliders with no body in the parent hierarchy needs a PhysicsWorldIndex
            if (!hasBodyComponent && body == shapeGameObject)
            {
                GetComponents(colliderComponents);
                GetComponents(physicsShapeComponents);
                // We need to check that there are no other colliders in the same object, if so, only the first one should do this, otherwise there will be 2 bakers adding this to the entity
                // This will be needed to trigger BuildCompoundColliderBakingSystem
                // If they are legacy Colliders and PhysicsShapeAuthoring in the same object, the PhysicsShapeAuthoring will add this
                if (physicsShapeComponents.Count == 0 && colliderComponents.Count > 0 && colliderComponents[0].GetInstanceID() == shapeInstanceID)
                {
                    AddSharedComponent(new PhysicsWorldIndex());

                    AddComponent(new PhysicsCompoundData()
                    {
                        AssociateBlobToBody = false,
                        ConvertedBodyInstanceID = shapeInstanceID,
                        Hash = default,
                    });

                    AddComponent<PhysicsRootBaked>();

                    AddComponent<PhysicsCollider>();
                    AddBuffer<PhysicsColliderKeyEntityPair>();

                    PostProcessTransform(bodyTransform, BodyMotionType.Static);
                }
            }

            return data;
        }

        public override void Bake(T authoring)
        {
            var shapeBakingData = new PhysicsColliderAuthoringData();

            // First pass
            Profiler.BeginSample("Collect Inputs from Authoring Components");

            if (ShouldConvertShape(authoring))
            {
                // We can have multiple Colliders of the same type on the same game object, so instead of adding the components to the baking entity
                // we add the components to an additional entity. These new entities will be processed by the baking system
                var colliderEntity = CreateAdditionalEntity(TransformUsageFlags.None, true);
                shapeBakingData.ShapeComputationalData = GetInputDataFromAuthoringComponent(authoring, colliderEntity);
                AddComponent(colliderEntity, shapeBakingData);

                // The data will be filled in by the BaseShapeBakingSystem, but we add it here so it gets reverted from the entity if the collider component is deleted
                AddComponent(colliderEntity, new PhysicsColliderBakedData()
                {
                    BodyEntity = shapeBakingData.ShapeComputationalData.Instance.BodyEntity,
                    BodyFromShape = shapeBakingData.ShapeComputationalData.Instance.BodyFromShape,
                    ChildEntity = shapeBakingData.ShapeComputationalData.Instance.ChildEntity,
                    // It is a leaf if the Shape Entity equals Body Entity
                    IsLeafEntityBody = (shapeBakingData.ShapeComputationalData.Instance.ShapeEntity.Equals(shapeBakingData.ShapeComputationalData.Instance.BodyEntity))
                });
            }

            Profiler.EndSample();
        }
    }

    class LegacyBoxBaker : ColliderBaker<LegacyBox>
    {
        internal override ShapeComputationDataBaking GenerateComputationData(LegacyBox shape, ColliderInstanceBaking colliderInstance, Entity colliderEntity)
        {
            var res = GenerateComputationDataGeneric(shape, colliderInstance);
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

    class LegacySphereBaker : ColliderBaker<LegacySphere>
    {
        internal override ShapeComputationDataBaking GenerateComputationData(LegacySphere shape, ColliderInstanceBaking colliderInstance, Entity colliderEntity)
        {
            var res = GenerateComputationDataGeneric(shape, colliderInstance);
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

    class LegacyCapsuleBaker : ColliderBaker<LegacyCapsule>
    {
        internal override ShapeComputationDataBaking GenerateComputationData(LegacyCapsule shape, ColliderInstanceBaking colliderInstance, Entity colliderEntity)
        {
            var res = GenerateComputationDataGeneric(shape, colliderInstance);

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

    class LegacyMeshBaker : ColliderBaker<LegacyMesh>
    {
        internal override ShapeComputationDataBaking GenerateComputationData(LegacyMesh shape, ColliderInstanceBaking colliderInstance, Entity colliderEntity)
        {
            UnityMesh mesh = shape.sharedMesh;
            if (mesh == null)
            {
                throw new InvalidOperationException(
                    $"No {nameof(LegacyMesh.sharedMesh)} assigned to {typeof(MeshCollider)} on {shape.name}."
                );
            }

            if (!mesh.IsValidForConversion(shape.gameObject))
            {
                throw new InvalidOperationException(
                    $"Mesh '{mesh}' assigned to {typeof(MeshCollider)} on {shape.name} is not readable. Ensure that you have enabled Read/Write on its import settings."
                );
            }

            // No need to check for null mesh as this has been checked earlier in the function
            DependsOn(mesh);

            var res = GenerateComputationDataGeneric(shape, colliderInstance);

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

            var meshBakingData = new PhysicsMeshAuthoringData()
            {
                Convex = shape.convex,
                Mesh = mesh,
                BakeFromShape = bakeFromShape,
                MeshID = mesh.GetInstanceID(),
                MeshBounds = mesh.bounds,
                ChildToShape = float4x4.identity
            };
            AddComponent(colliderEntity, meshBakingData);

            return res;
        }
    }
}
#endif
