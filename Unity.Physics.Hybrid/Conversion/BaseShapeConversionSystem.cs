using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    struct ComparableEntity : IEquatable<ComparableEntity>, IComparable<ComparableEntity>
    {
        public Entity Entity;

        public bool Equals(ComparableEntity other) => Entity.Equals(other.Entity);

        public int CompareTo(ComparableEntity other) => Entity.Index - other.Entity.Index;
    }

    struct LeafShapeData
    {
        public Entity LeafEntity;
        public CompoundCollider.ColliderBlobInstance ColliderBlobInstance;
    }

    public abstract partial class BaseShapeConversionSystem<T> : GameObjectConversionSystem where T : Component
    {
        protected abstract bool ShouldConvertShape(T shape);
        protected abstract GameObject GetPrimaryBody(T shape);
        protected abstract BlobAssetReference<Collider> ProduceColliderBlob(T shape);

        void ConvertShape(T shape)
        {
            if (!ShouldConvertShape(shape))
                return;

            var body = GetPrimaryBody(shape);
            var bodyEntity = GetPrimaryEntity(body);

            BlobAssetReference<Collider> colliderBlob;
            try
            {
                colliderBlob = ProduceColliderBlob(shape);
            }
            catch (Exception e)
            {
                var ex = new InvalidOperationException(
                    $"'{shape.name}' produced an invalid collider during conversion process and has been skipped.", e
                );
                // TODO: use GameObjectConversionSystem.LogWarning() when entities version is upgraded
                Debug.LogException(ex, shape);
                return;
            }

            if (body == shape.gameObject)
                DstEntityManager.RemoveParentAndSetWorldTranslationAndRotation(bodyEntity, body.transform);

            if (!colliderBlob.IsCreated)
                return;

            // store shape and transform relative to the body in case there are several that need to become a compound
            // Note: Not including relative scale, since that is baked into the colliders
            m_ExtraColliders.Add(
                new ComparableEntity { Entity = bodyEntity },
                new LeafShapeData
                {
                    LeafEntity = GetPrimaryEntity(shape),
                    ColliderBlobInstance = new CompoundCollider.ColliderBlobInstance
                    {
                        CompoundFromChild = GetCompoundFromChild(shape),
                        Collider = colliderBlob
                    }
                }
            );
        }

        RigidTransform GetCompoundFromChild(T shape)
        {
            var body = GetPrimaryBody(shape);
            var worldFromBody = new RigidTransform(body.transform.rotation, body.transform.position);
            var worldFromShape = new RigidTransform(shape.transform.rotation, shape.transform.position);
            return math.mul(math.inverse(worldFromBody), worldFromShape);
        }

        NativeMultiHashMap<ComparableEntity, LeafShapeData> m_ExtraColliders;

        NativeList<ConvertConvexColliderInput> m_ConvexColliderJobs;
        List<T> m_ConvexShapes = new List<T>(64);
        NativeList<float3> m_ConvexColliderPoints;

        internal void RegisterConvexColliderDeferred(
            T shape,
            NativeArray<float3> points,
            ConvexHullGenerationParameters generationParameters,
            CollisionFilter filter, Material material
        )
        {
            m_ConvexShapes.Add(shape);
            m_ConvexColliderJobs.Add(new ConvertConvexColliderInput
            {
                BodyEntity = GetPrimaryEntity(GetPrimaryBody(shape)),
                LeafEntity = GetPrimaryEntity(shape),
                CompoundFromChild = GetCompoundFromChild(shape),
                GenerationParameters = generationParameters,
                PointsStart = m_ConvexColliderPoints.Length,
                PointCount = points.Length,
                Filter = filter,
                Material = material
            });
            m_ConvexColliderPoints.AddRange(points);
        }

        NativeList<ConvertMeshColliderInput> m_MeshColliderJobs;
        List<T> m_MeshShapes = new List<T>(64);
        NativeList<float3> m_MeshColliderVertices;
        NativeList<int> m_MeshColliderIndices;

        internal void RegisterMeshColliderDeferred(
            T shape, NativeArray<float3> vertices, NativeArray<int> indices, CollisionFilter filter, Material material
        )
        {
            if (indices.Length == 0)
            {
                throw new InvalidOperationException(
                    $"No triangles associated with {shape.name}. Ensure mesh import settings enables Read/Write"
                );
            }

            m_MeshColliderJobs.Add(new ConvertMeshColliderInput
            {
                BodyEntity = GetPrimaryEntity(GetPrimaryBody(shape)),
                LeafEntity = GetPrimaryEntity(shape),
                VerticesStart = m_MeshColliderVertices.Length,
                VertexCount = vertices.Length,
                IndicesStart = m_MeshColliderIndices.Length,
                IndexCount = indices.Length,
                CompoundFromChild = GetCompoundFromChild(shape),
                Filter = filter,
                Material = material
            });
            m_MeshShapes.Add(shape);
            m_MeshColliderVertices.AddRange(vertices);
            m_MeshColliderIndices.AddRange(indices);
        }

        static void AppendLeafShapeDataToShapeMap(
            NativeArray<KeyValuePair<Entity, LeafShapeData>> leafShapeDataPerBody,
            NativeMultiHashMap<ComparableEntity, LeafShapeData> shapesPerBody,
            IReadOnlyList<T> sourceShapes
        )
        {
            for (var i = 0; i < leafShapeDataPerBody.Length; ++i)
            {
                var leaf = leafShapeDataPerBody[i].Value;
                if (leaf.ColliderBlobInstance.Collider.IsCreated)
                    shapesPerBody.Add(new ComparableEntity { Entity = leafShapeDataPerBody[i].Key }, leaf);
                else
                {
                    var ex = new InvalidOperationException(
                        $"'{sourceShapes[i].name}' produced an invalid convex collider during conversion process and has been skipped."
                    );

                    // TODO: use GameObjectConversionSystem.LogWarning() when entities version is upgraded
                    Debug.LogException(ex, sourceShapes[i]);
                }
            }
        }

        protected override void OnUpdate()
        {
            var shapeQuery = EntityManager.CreateEntityQuery(ComponentType.ReadOnly<T>());
            var shapeCount = shapeQuery.CalculateEntityCount();

            // A map from entities to arrays of colliders
            m_ExtraColliders = new NativeMultiHashMap<ComparableEntity, LeafShapeData>(shapeCount, Allocator.Temp);

            // Lists to store input data for deferred convex and mesh jobs
            const int defaultPointsPerShape = 1024;

            m_ConvexColliderJobs = new NativeList<ConvertConvexColliderInput>(shapeCount, Allocator.TempJob);
            m_ConvexColliderPoints = new NativeList<float3>(shapeCount * defaultPointsPerShape, Allocator.TempJob);

            m_MeshColliderJobs = new NativeList<ConvertMeshColliderInput>(shapeCount, Allocator.TempJob);
            m_MeshColliderVertices = new NativeList<float3>(shapeCount * defaultPointsPerShape, Allocator.TempJob);
            m_MeshColliderIndices = new NativeList<int>(shapeCount * defaultPointsPerShape / 2, Allocator.TempJob);

            // First pass.
            // Convert all shape authoring components into colliders, and collect them for each primary body
            Entities.ForEach<T>(ConvertShape);

            // Second pass.
            // Produce all convex and mesh collider blobs in parallel
            const int arrayLength = 5;
            var convexColliders =
                new NativeArray<KeyValuePair<Entity, LeafShapeData>>(m_ConvexColliderJobs.Length, Allocator.TempJob);
            var convexJob = new ProduceConvexCollidersJob
            {
                InputParameters = m_ConvexColliderJobs,
                AllPoints = m_ConvexColliderPoints,
                Output = convexColliders
            }.Schedule(m_ConvexColliderJobs.Length, arrayLength);

            var meshColliders =
                new NativeArray<KeyValuePair<Entity, LeafShapeData>>(m_MeshColliderJobs.Length, Allocator.TempJob);
            var meshJob = new ProduceMeshCollidersJob
            {
                InputParameters = m_MeshColliderJobs,
                AllVertices = m_MeshColliderVertices,
                AllIndices = m_MeshColliderIndices,
                Output = meshColliders
            }.Schedule(m_MeshColliderJobs.Length, arrayLength);

            JobHandle.CombineDependencies(convexJob, meshJob).Complete();

            AppendLeafShapeDataToShapeMap(convexColliders, m_ExtraColliders, m_ConvexShapes);
            convexColliders.Dispose();

            AppendLeafShapeDataToShapeMap(meshColliders, m_ExtraColliders, m_MeshShapes);
            meshColliders.Dispose();

            // Final pass.
            // Assign PhysicsCollider components to rigid bodies, merging multiples into compounds as needed
            var keys = m_ExtraColliders.GetUniqueKeyArray(Allocator.Temp);
            using (keys.Item1)
            {
                for (var k = 0; k < keys.Item2; ++k)
                {
                    ComparableEntity body = keys.Item1[k];
                    var collider = DstEntityManager.HasComponent<PhysicsCollider>(body.Entity)
                        ? DstEntityManager.GetComponentData<PhysicsCollider>(body.Entity)
                        : new PhysicsCollider();
                    var children = new NativeList<CompoundCollider.ColliderBlobInstance>(16, Allocator.Temp);

                    // collect any existing valid shapes
                    if (collider.IsValid)
                    {
                        ColliderType colliderType;
                        unsafe { colliderType = collider.ColliderPtr->Type; }

                        // if there is already a compound, add its leaves to the list of children
                        if (colliderType == ColliderType.Compound)
                        {
                            unsafe
                            {
                                var existingChildren = ((CompoundCollider*)collider.ColliderPtr)->Children;
                                for (int i = 0, count = existingChildren.Length; i < count; ++i)
                                {
                                    children.Add(new CompoundCollider.ColliderBlobInstance
                                    {
                                        Collider = BlobAssetReference<Collider>.Create(
                                            existingChildren[i].Collider,
                                            existingChildren[i].Collider->MemorySize
                                        ),
                                        CompoundFromChild = existingChildren[i].CompoundFromChild
                                    });
                                }
                            }
                        }
                        // otherwise add the single collider to the list of children
                        else
                        {
                            children.Add(
                                new CompoundCollider.ColliderBlobInstance
                                {
                                    Collider = collider.Value,
                                    CompoundFromChild = RigidTransform.identity
                                }
                            );
                        }
                    }

                    // if collider is already valid, a shape already existed from another system
                    var isSingleShapeOnPrimaryBody = !collider.IsValid;
                    // collect all children found by this system
                    if (m_ExtraColliders.TryGetFirstValue(body, out var child, out var iterator))
                    {
                        do
                        {
                            children.Add(child.ColliderBlobInstance);
                            isSingleShapeOnPrimaryBody &= child.LeafEntity.Equals(body.Entity);
                        } while (m_ExtraColliders.TryGetNextValue(out child, ref iterator));
                    }

                    // if there is a single shape on the primary body, use it as-is, otherwise create a compound
                    // (assume a single leaf should still be a compound so that local offset values in authoring representation are retained)
                    collider.Value = isSingleShapeOnPrimaryBody
                        ? children[0].Collider
                        : CompoundCollider.Create(children);

                    children.Dispose();

                    DstEntityManager.AddOrSetComponent(body.Entity, collider);
                }
            }

            m_ConvexShapes.Clear();
            m_ConvexColliderJobs.Dispose();
            m_ConvexColliderPoints.Dispose();

            m_MeshShapes.Clear();
            m_MeshColliderJobs.Dispose();
            m_MeshColliderVertices.Dispose();
            m_MeshColliderIndices.Dispose();
        }
    }

    struct ConvertConvexColliderInput
    {
        public Entity BodyEntity;
        public Entity LeafEntity;
        public RigidTransform CompoundFromChild;
        public ConvexHullGenerationParameters GenerationParameters;
        public int PointsStart;
        public int PointCount;
        public CollisionFilter Filter;
        public Material Material;
    }

    [BurstCompile(CompileSynchronously = true)]
    unsafe struct ProduceConvexCollidersJob : IJobParallelFor
    {
        [ReadOnly] public  NativeArray<ConvertConvexColliderInput> InputParameters;
        [NativeDisableUnsafePtrRestriction]
        [ReadOnly] public NativeArray<float3> AllPoints;
        [NativeDisableParallelForRestriction]
        public NativeArray<KeyValuePair<Entity, LeafShapeData>> Output;

        public void Execute(int index)
        {
            var inputParameters = InputParameters[index];
            var points = new NativeArray<float3>(
                inputParameters.PointCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory
            );
            UnsafeUtility.MemCpy(
                points.GetUnsafePtr(),
                (float3*)AllPoints.GetUnsafeReadOnlyPtr() + inputParameters.PointsStart,
                UnsafeUtility.SizeOf<float3>() * inputParameters.PointCount
            );
            Output[index] = new KeyValuePair<Entity, LeafShapeData>(
                inputParameters.BodyEntity,
                new LeafShapeData
                {
                    LeafEntity = inputParameters.LeafEntity,
                    ColliderBlobInstance = new CompoundCollider.ColliderBlobInstance
                    {
                        CompoundFromChild = inputParameters.CompoundFromChild,
                        Collider = ConvexCollider.Create(
                            points, inputParameters.GenerationParameters, inputParameters.Filter, inputParameters.Material
                        )
                    }
                }
            );
        }
    }

    struct ConvertMeshColliderInput
    {
        public Entity BodyEntity;
        public Entity LeafEntity;
        public RigidTransform CompoundFromChild;
        public int VerticesStart;
        public int VertexCount;
        public int IndicesStart;
        public int IndexCount;
        public CollisionFilter Filter;
        public Material Material;
    }

    [BurstCompile(CompileSynchronously = true)]
    unsafe struct ProduceMeshCollidersJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<ConvertMeshColliderInput> InputParameters;
        [NativeDisableUnsafePtrRestriction]
        [ReadOnly] public NativeArray<float3> AllVertices;
        [NativeDisableUnsafePtrRestriction]
        [ReadOnly] public NativeArray<int> AllIndices;
        [NativeDisableParallelForRestriction]
        public NativeArray<KeyValuePair<Entity, LeafShapeData>> Output;

        public void Execute(int index)
        {
            var inputParameters = InputParameters[index];
            var vertices = new NativeArray<float3>(
                inputParameters.VertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory
            );
            UnsafeUtility.MemCpy(
                vertices.GetUnsafePtr(),
                (float3*)AllVertices.GetUnsafeReadOnlyPtr() + inputParameters.VerticesStart,
                UnsafeUtility.SizeOf<float3>() * inputParameters.VertexCount
            );
            var indices = new NativeArray<int>(
                inputParameters.IndexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory
            );
            UnsafeUtility.MemCpy(
                indices.GetUnsafePtr(),
                (int*)AllIndices.GetUnsafeReadOnlyPtr() + inputParameters.IndicesStart,
                UnsafeUtility.SizeOf<int>() * inputParameters.IndexCount
            );
            Output[index] = new KeyValuePair<Entity, LeafShapeData>(
                inputParameters.BodyEntity,
                new LeafShapeData
                {
                    LeafEntity = inputParameters.LeafEntity,
                    ColliderBlobInstance = new CompoundCollider.ColliderBlobInstance
                    {
                        CompoundFromChild = inputParameters.CompoundFromChild,
                        Collider = MeshCollider.Create(
                            vertices, indices, inputParameters.Filter, inputParameters.Material
                        )
                    }
                }
            );
        }
    }
}

