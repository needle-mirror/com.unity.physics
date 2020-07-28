using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Physics.Authoring
{
    public partial class BaseShapeConversionSystem<T>
    {
        internal struct MeshInput
        {
            public int VerticesStart;
            public int VertexCount;
            public int TrianglesStart;
            public int TriangleCount;
            public CollisionFilter Filter;
            public Material Material;
        }

        NativeHashMap<Hash128, MeshInput> m_MeshColliderJobs;
        NativeList<float3> m_MeshColliderVertices;
        NativeList<int3> m_MeshColliderTriangles;

        JobHandle ProduceMeshColliders(
            NativeHashMap<Hash128, MeshInput> inputs,
            NativeList<float3> vertices,
            NativeList<int3> indices,
            out NativeHashMap<Hash128, BlobAssetReference<Collider>> meshColliders,
            JobHandle inputDeps = default
        )
        {
            // create collider blob assets
            var meshCollidersArray =
                new NativeArray<KeyValuePair<Hash128, BlobAssetReference<Collider>>>(inputs.Count(), Allocator.TempJob);
            const int arrayLength = 5;
            var jobHandle = new ProduceMeshCollidersJob
            {
                InputKeys = inputs.GetKeyArray(Allocator.TempJob),
                InputValues = inputs.GetValueArray(Allocator.TempJob),
                AllVertices = vertices,
                AllIndices = indices,
                Output = meshCollidersArray
            }.Schedule(inputs.Count(), arrayLength, inputDeps);

            // put blob assets into hash map
            meshColliders = new NativeHashMap<Hash128, BlobAssetReference<Collider>>(inputs.Count(), Allocator.TempJob);
            jobHandle = new ConvertToHashMapJob<Hash128, BlobAssetReference<Collider>>
            {
                Input = meshCollidersArray,
                Output = meshColliders
            }.Schedule(jobHandle);

            return jobHandle;
        }

        [BurstCompile(CompileSynchronously = true)]
        struct ProduceMeshCollidersJob : IJobParallelFor
        {
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<Hash128> InputKeys;
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<MeshInput> InputValues;
            [NativeDisableUnsafePtrRestriction]
            [ReadOnly] public NativeArray<float3> AllVertices;
            [NativeDisableUnsafePtrRestriction]
            [ReadOnly] public NativeArray<int3> AllIndices;

            [NativeDisableParallelForRestriction]
            public NativeArray<KeyValuePair<Hash128, BlobAssetReference<Collider>>> Output;

            public void Execute(int index)
            {
                var inputParameters = InputValues[index];

                Output[index] = new KeyValuePair<Hash128, BlobAssetReference<Collider>>(
                    InputKeys[index],
                    MeshCollider.Create(
                        AllVertices.GetSubArray(inputParameters.VerticesStart, inputParameters.VertexCount),
                        AllIndices.GetSubArray(inputParameters.TrianglesStart, inputParameters.TriangleCount),
                        inputParameters.Filter, inputParameters.Material
                    )
                );
            }
        }
    }
}
