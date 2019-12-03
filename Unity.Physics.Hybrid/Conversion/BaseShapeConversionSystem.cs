using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    public abstract partial class BaseShapeConversionSystem<T> : GameObjectConversionSystem where T : Component
    {
        protected abstract bool ShouldConvertShape(T shape);
        protected abstract GameObject GetPrimaryBody(T shape);

        internal abstract ShapeComputationData GenerateComputationData(
            T shape, ColliderInstance colliderInstance,
            NativeList<float3> allConvexHullPoints, NativeList<float3> allMeshVertices, NativeList<int3> allMeshTriangles
        );

        void CreateNewBlobAssets(
            NativeArray<ShapeComputationData> computeData,
            NativeArray<int> toCompute,
            int count,
            NativeArray<BlobAssetReference<Collider>> blobAssets
        )
        {
            var job = new CreateBlobAssetsJob
            {
                ComputeData = computeData,
                ToComputeTable = toCompute,
                BlobAssets = blobAssets
            };
            job.Schedule(count, 1).Complete();
        }
        
        void GetInputDataFromAuthoringComponent(T shape)
        {
            if (!ShouldConvertShape(shape))
                return;

            var body = GetPrimaryBody(shape);

            var instance = new ColliderInstance
            {
                AuthoringComponentId = shape.GetInstanceID(),
                BodyEntity = GetPrimaryEntity(body),
                ShapeEntity = GetPrimaryEntity(shape),
                BodyFromShape = ColliderInstance.GetCompoundFromChild(shape.transform, body.transform)
            };

            var data = GenerateComputationData(shape, instance, m_ConvexColliderPoints, m_MeshColliderVertices, m_MeshColliderTriangles);
            data.Instance.ConvertedAuthoringComponentIndex = m_EndColliderConversionSystem.PushAuthoringComponent(shape);
            data.Instance.ConvertedBodyTransformIndex = m_EndColliderConversionSystem.PushAuthoringComponent(body.transform);
            m_ShapeComputationData.Add(data);

            if (body == shape.gameObject)
                DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(instance.BodyEntity, body.transform);
        }

        NativeHashMap<Entity, Entity> m_AllBodiesByLeaf;
        NativeList<ShapeComputationData> m_ShapeComputationData;

        BuildCompoundCollidersConversionSystem m_BuildCompoundsSystem;
        BeginColliderConversionSystem m_BeginColliderConversionSystem;
        EndColliderConversionSystem m_EndColliderConversionSystem;

        internal BlobAssetComputationContext<int, Collider> BlobComputationContext =>
            m_BeginColliderConversionSystem.BlobComputationContext;

        EntityQuery m_ShapeQuery;

        protected override void OnCreate()
        {
            base.OnCreate();

            m_ShapeQuery = EntityManager.CreateEntityQuery(ComponentType.ReadOnly<T>());

            m_BeginColliderConversionSystem = World.GetOrCreateSystem<BeginColliderConversionSystem>();
            m_BuildCompoundsSystem = World.GetOrCreateSystem<BuildCompoundCollidersConversionSystem>();
            m_EndColliderConversionSystem = World.GetOrCreateSystem<EndColliderConversionSystem>();
            
            // A map from leaf shape entities to their respective bodies
            m_AllBodiesByLeaf = new NativeHashMap<Entity, Entity>(16, Allocator.Persistent);

            // A list of inputs gathered from authoring components
            const int defaultShapeCount = 128;
            m_ShapeComputationData = new NativeList<ShapeComputationData>(defaultShapeCount, Allocator.Persistent);

            // Lists to store input data for deferred convex and mesh jobs
            const int defaultPointsPerShape = 1024;

            m_ConvexColliderJobs = new NativeHashMap<Hash128, ConvexInput>(defaultShapeCount, Allocator.Persistent);
            m_ConvexColliderPoints = new NativeList<float3>(defaultShapeCount * defaultPointsPerShape, Allocator.Persistent);

            m_MeshColliderJobs = new NativeHashMap<Hash128, MeshInput>(defaultShapeCount, Allocator.Persistent);
            m_MeshColliderVertices = new NativeList<float3>(defaultShapeCount * defaultPointsPerShape, Allocator.Persistent);
            m_MeshColliderTriangles = new NativeList<int3>(defaultShapeCount * defaultPointsPerShape / 2 / 3, Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();

            m_AllBodiesByLeaf.Dispose();

            m_ShapeComputationData.Dispose();

            m_ConvexColliderJobs.Dispose();
            m_ConvexColliderPoints.Dispose();

            m_MeshColliderJobs.Dispose();
            m_MeshColliderVertices.Dispose();
            m_MeshColliderTriangles.Dispose();
        }

        protected override void OnUpdate()
        {
            var shapeCount = m_ShapeQuery.CalculateEntityCount();
            if (shapeCount == 0)
                return;

            // First pass
            Profiler.BeginSample("Collect Inputs from Authoring Components");

            Entities.ForEach<T>(GetInputDataFromAuthoringComponent);

            Profiler.EndSample();

            // Second pass
            Profiler.BeginSample("Generate Hashes for Inputs");

            var hashes = new NativeArray<Hash128>(shapeCount, Allocator.TempJob);
            GenerateShapesHash(hashes);

            Profiler.EndSample();

            // Third pass
            Profiler.BeginSample("Determine New Colliders to Create");

            var shapeIndicesNeedingNewBlobs = new NativeArray<int>(shapeCount, Allocator.TempJob);
            var numNewBlobAssets = 0;

            // Parse all the entries, associate the computed hash with its GameObject, check if we need to compute the BlobAsset
            for (var i = 0; i < shapeCount; i++)
            {
                var hash = hashes[i];
                var shapeData = m_ShapeComputationData[i];
                var instance = shapeData.Instance;
                instance.Hash = hash;
                shapeData.Instance = instance;
                m_ShapeComputationData[i] = shapeData;
                var convertedIndex = shapeData.Instance.ConvertedAuthoringComponentIndex;
                var gameObject =
                    m_EndColliderConversionSystem.GetConvertedAuthoringComponent(convertedIndex).gameObject;
                BlobComputationContext.AssociateBlobAssetWithGameObject(hash, gameObject);

                if (BlobComputationContext.NeedToComputeBlobAsset(hash))
                {
                    BlobComputationContext.AddBlobAssetToCompute(hash, 0);
                    if (shapeData.ShapeType == ShapeType.ConvexHull)
                    {
                        if (!m_ConvexColliderJobs.TryGetValue(hash, out _))
                            m_ConvexColliderJobs.TryAdd(hash, shapeData.ConvexHullProperties);
                    }
                    else if (shapeData.ShapeType == ShapeType.Mesh)
                    {
                        if (!m_MeshColliderJobs.TryGetValue(hash, out _))
                            m_MeshColliderJobs.TryAdd(hash, shapeData.MeshProperties);
                    }
                    else
                        shapeIndicesNeedingNewBlobs[numNewBlobAssets++] = i;
                }

                m_BuildCompoundsSystem.SetLeafDirty(instance);

                // Detect re-parenting of a shape (the shape has a different body than the current one, if any)
                if (m_AllBodiesByLeaf.TryGetValue(instance.ShapeEntity, out var bodyEntity))
                {
                    if (!bodyEntity.Equals(instance.BodyEntity))
                    {
                        // Mark the former body dirty to trigger re-computation of its compound
                        m_BuildCompoundsSystem.SetLeafDirty(
                            new ColliderInstance
                            {
                                BodyEntity = bodyEntity,
                                ShapeEntity = bodyEntity
                            }
                        );
                        m_AllBodiesByLeaf[instance.ShapeEntity] = instance.BodyEntity;
                    }
                }
                else
                {
                    m_AllBodiesByLeaf.Add(instance.ShapeEntity, instance.BodyEntity);
                }
            }

            Profiler.EndSample();

            // Compute the BlobAssets
            Profiler.BeginSample("Create New Colliders");

            using (var blobAssets = new NativeArray<BlobAssetReference<Collider>>(shapeCount, Allocator.TempJob))
            {
                CreateNewBlobAssets(m_ShapeComputationData, shapeIndicesNeedingNewBlobs, numNewBlobAssets, blobAssets);

                for (var i = 0; i < numNewBlobAssets; i++)
                {
                    var index = shapeIndicesNeedingNewBlobs[i];
                    BlobComputationContext.AddComputedBlobAsset(hashes[index], blobAssets[i]);
                }
            }

            shapeIndicesNeedingNewBlobs.Dispose();
            hashes.Dispose();

            Profiler.EndSample();

            // Convert convex hulls and meshes
            Profiler.BeginSample("Convert Hulls and Meshes");
            
            ConvertHullsAndMeshes();
            
            Profiler.EndSample();

            m_AllBodiesByLeaf.Clear();

            m_ShapeComputationData.Clear();

            m_ConvexColliderJobs.Clear();
            m_ConvexColliderPoints.Clear();

            m_MeshColliderJobs.Clear();
            m_MeshColliderVertices.Clear();
            m_MeshColliderTriangles.Clear();
        }

        void GenerateShapesHash(NativeArray<Hash128> hashes)
        {
            var count = m_ShapeComputationData.Length;
            var job = new GeneratePhysicsShapeHashesJob
            {
                ComputationData = m_ShapeComputationData,
                Hashes = hashes
            };

            job.Schedule(count, 1).Complete();
        }

        void ConvertHullsAndMeshes()
        {
            var convexJob = ProduceConvexColliders(
                m_ConvexColliderJobs, m_ConvexColliderPoints, out var convexColliders
            );
            convexJob = new DisposeContainerJob<Hash128>
            {
                Container = m_ConvexColliderJobs.GetKeyArray(Allocator.TempJob)
            }.Schedule(convexJob);

            var meshJob = ProduceMeshColliders(
                m_MeshColliderJobs, m_MeshColliderVertices, m_MeshColliderTriangles, out var meshColliders
            );
            meshJob = new DisposeContainerJob<Hash128>
            {
                Container = m_MeshColliderJobs.GetKeyArray(Allocator.TempJob)
            }.Schedule(meshJob);

            JobHandle.CombineDependencies(convexJob, meshJob).Complete();

            if (convexColliders.Length > 0)
            {
                using (var kvp = convexColliders.GetKeyValueArrays(Allocator.Temp))
                {
                    for (int i = 0; i < kvp.Keys.Length; i++)
                        BlobComputationContext.AddComputedBlobAsset(kvp.Keys[i], kvp.Values[i]);
                }
            }

            if (meshColliders.Length > 0)
            {
                using (var kvp = meshColliders.GetKeyValueArrays(Allocator.Temp))
                {
                    for (int i = 0; i < kvp.Keys.Length; i++)
                        BlobComputationContext.AddComputedBlobAsset(kvp.Keys[i], kvp.Values[i]);
                }
            }

            convexColliders.Dispose();
            meshColliders.Dispose();
        }
    }
}
