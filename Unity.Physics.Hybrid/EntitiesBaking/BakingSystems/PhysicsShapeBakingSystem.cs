using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using LegacyCollider = UnityEngine.Collider;
using UnityMesh = UnityEngine.Mesh;

namespace Unity.Physics.Authoring
{
    class PhysicsShapeBaker : BaseColliderBaker<PhysicsShapeAuthoring>
    {
        public static List<PhysicsShapeAuthoring> physicsShapeComponents = new List<PhysicsShapeAuthoring>();

        protected bool ShouldConvertShape(PhysicsShapeAuthoring authoring)
        {
            return authoring.enabled;
        }

        ShapeComputationDataBaking GetInputDataFromAuthoringComponent(PhysicsShapeAuthoring shape, Entity colliderEntity)
        {
            GameObject shapeGameObject = shape.gameObject;
            var body = GetPrimaryBody(shapeGameObject, out bool hasBodyComponent, out bool isStaticBody);
            var child = shapeGameObject;
            var shapeInstanceID = shape.GetInstanceID();

            var bodyEntity = GetEntity(body);

            // prepare the static root
            if (isStaticBody)
            {
                var staticRootMarker = CreateAdditionalEntity(TransformUsageFlags.Default, true, "StaticRootBakeMarker");
                AddComponent(staticRootMarker, new BakeStaticRoot() { Body = bodyEntity, ConvertedBodyInstanceID = body.transform.GetInstanceID() });
            }

            // Track dependencies to the transforms
            Transform shapeTransform = GetComponent<Transform>(shape);
            Transform bodyTransform = GetComponent<Transform>(body);
            var instance = new ColliderInstanceBaking
            {
                AuthoringComponentId = shapeInstanceID,
                BodyEntity = bodyEntity,
                ShapeEntity = GetEntity(shapeGameObject),
                ChildEntity = GetEntity(child),
                BodyFromShape = ColliderInstanceBaking.GetCompoundFromChild(shapeTransform, bodyTransform),
            };

            var data = GenerateComputationData(shape, instance, colliderEntity);

            data.Instance.ConvertedAuthoringInstanceID = shapeInstanceID;
            data.Instance.ConvertedBodyInstanceID = bodyTransform.GetInstanceID();

            // The root colliders with no body in the parent hierarchy needs a PhysicsWorldIndex
            if (!hasBodyComponent && body == shapeGameObject)
            {
                GetComponents(physicsShapeComponents);
                // We need to check that there are no other colliders in the same object, if so, only the first one should do this, otherwise there will be 2 bakers adding this to the entity
                // This will be needed to trigger BuildCompoundColliderBakingSystem
                // If they are legacy Colliders and PhysicsShapeAuthoring in the same object, the PhysicsShapeAuthoring will add this
                if (physicsShapeComponents.Count > 0 && physicsShapeComponents[0].GetInstanceID() == shapeInstanceID)
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

        Material ProduceMaterial(PhysicsShapeAuthoring shape)
        {
            var materialTemplate = shape.MaterialTemplate;
            if (materialTemplate != null)
                DependsOn(materialTemplate);
            return shape.GetMaterial();
        }

        CollisionFilter ProduceCollisionFilter(PhysicsShapeAuthoring shape)
        {
            return shape.GetFilter();
        }

        UnityEngine.Mesh GetMesh(PhysicsShapeAuthoring shape, out float4x4 childToShape)
        {
            var mesh = shape.CustomMesh;
            childToShape = float4x4.identity;

            if (mesh == null)
            {
                // Try to get a mesh in the children
                var filter = GetComponentInChildren<MeshFilter>();
                if (filter != null && filter.sharedMesh != null)
                {
                    mesh = filter.sharedMesh;
                    var childTransform = GetComponent<Transform>(filter);
                    childToShape = math.mul(shape.transform.worldToLocalMatrix, childTransform.localToWorldMatrix);;
                }
            }

            if (mesh == null)
            {
                throw new InvalidOperationException(
                    $"No {nameof(PhysicsShapeAuthoring.CustomMesh)} assigned on {shape.name}."
                );
            }
            DependsOn(mesh);
            return mesh;
        }

        internal ShapeComputationDataBaking GenerateComputationData(PhysicsShapeAuthoring shape, ColliderInstanceBaking colliderInstance, Entity colliderEntity
        )
        {
            var res = new ShapeComputationDataBaking
            {
                Instance = colliderInstance,
                Material = ProduceMaterial(shape),
                CollisionFilter = ProduceCollisionFilter(shape),
                ForceUniqueIdentifier = shape.ForceUnique ? (uint)shape.GetInstanceID() : 0u
            };

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

                    // TODO: BAKING - SUPPORT FOR ONLY CUSTOM MESH FOR NOW. Added the following errors to replicate the legacy behaviour. The original call is the line below:
                    // res.Instance.Hash = shape.GetBakedConvexInputs(meshAssets);
                    var mesh = GetMesh(shape, out var childToShape);
                    if (mesh == null)
                    {
                        throw new InvalidOperationException(
                            $"No {nameof(PhysicsShapeAuthoring.CustomMesh)} assigned on {shape.name}."
                        );
                    }
                    if (!mesh.IsValidForConversion(shape.gameObject))
                    {
                        throw new InvalidOperationException(
                            $"Mesh '{mesh}' assigned on {shape.name} is not readable. Ensure that you have enabled Read/Write on its import settings."
                        );
                    }

                    var bakeFromShape = shape.GetLocalToShapeMatrix();

                    var meshBakingData = new PhysicsMeshAuthoringData()
                    {
                        Convex = true,
                        Mesh = mesh,
                        BakeFromShape = bakeFromShape,
                        MeshID = mesh.GetInstanceID(),
                        MeshBounds = mesh.bounds,
                        ChildToShape = childToShape
                    };
                    AddComponent(colliderEntity, meshBakingData);

                    break;
                }
                case ShapeType.Mesh:
                {
                    res.MeshProperties.Filter = res.CollisionFilter;
                    res.MeshProperties.Material = res.Material;

                    // TODO: BAKING - SUPPORT FOR ONLY CUSTOM MESH FOR NOW. Added the following errors to replicate the legacy behaviour. The original call is the line below:
                    //res.Instance.Hash = shape.GetBakedMeshInputs(meshAssets);
                    var mesh = GetMesh(shape, out var childToShape);
                    if (mesh == null)
                    {
                        throw new InvalidOperationException(
                            $"No {nameof(PhysicsShapeAuthoring.CustomMesh)} assigned on {shape.name}."
                        );
                    }
                    if (!mesh.IsValidForConversion(shape.gameObject))
                    {
                        throw new InvalidOperationException(
                            $"Mesh '{mesh}' assigned on {shape.name} is not readable. Ensure that you have enabled Read/Write on its import settings."
                        );
                    }

                    var bakeFromShape = shape.GetLocalToShapeMatrix();

                    var meshBakingData = new PhysicsMeshAuthoringData()
                    {
                        Convex = false,
                        Mesh = mesh,
                        BakeFromShape = bakeFromShape,
                        MeshID = mesh.GetInstanceID(),
                        MeshBounds = mesh.bounds,
                        ChildToShape = childToShape
                    };
                    AddComponent(colliderEntity, meshBakingData);

                    break;
                }
            }

            return res;
        }

        public override void Bake(PhysicsShapeAuthoring authoring)
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
                AddComponent<PhysicsColliderBakedData>(colliderEntity, new PhysicsColliderBakedData()
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
}
