using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    public partial class BaseShapeConversionSystem<T>
    {
        internal struct ConvexInput
        {
            public ConvexHullGenerationParameters GenerationParameters;
            public int PointsStart;
            public int PointCount;
            public CollisionFilter Filter;
            public Material Material;
        }

        NativeHashMap<Hash128, ConvexInput> m_ConvexColliderJobs;
        NativeList<float3> m_ConvexColliderPoints;

        static JobHandle ProduceConvexColliders(
            NativeHashMap<Hash128, ConvexInput> inputs, NativeArray<float3> points,
            out NativeHashMap<Hash128, BlobAssetReference<Collider>> convexColliders,
            JobHandle inputDeps = default
        )
        {
            // create collider blob assets
            var convexCollidersArray =
                new NativeArray<KeyValuePair<Hash128, BlobAssetReference<Collider>>>(inputs.Length, Allocator.TempJob);
            const int arrayLength = 5;
            var jobHandle = new ProduceConvexCollidersJob
            {
                InputKeys = inputs.GetKeyArray(Allocator.TempJob),
                InputValues = inputs.GetValueArray(Allocator.TempJob),
                AllPoints = points,
                Output = convexCollidersArray
            }.Schedule(inputs.Length, arrayLength, inputDeps);

            // put blob assets into hash map
            convexColliders = new NativeHashMap<Hash128, BlobAssetReference<Collider>>(inputs.Length, Allocator.TempJob);
            jobHandle = new ConvertToHashMapJob<Hash128, BlobAssetReference<Collider>>
            {
                Input = convexCollidersArray,
                Output = convexColliders
            }.Schedule(jobHandle);

            return jobHandle;
        }

        [BurstCompile(CompileSynchronously = true)]
        unsafe struct ProduceConvexCollidersJob : IJobParallelFor
        {
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<Hash128> InputKeys;
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<ConvexInput> InputValues;
            [NativeDisableUnsafePtrRestriction]
            [ReadOnly] public NativeArray<float3> AllPoints;

            [NativeDisableParallelForRestriction]
            public NativeArray<KeyValuePair<Hash128, BlobAssetReference<Collider>>> Output;

            public void Execute(int index)
            {
                var inputParameters = InputValues[index];
                var points = new NativeArray<float3>(
                    inputParameters.PointCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory
                );
                UnsafeUtility.MemCpy(
                    points.GetUnsafePtr(),
                    (float3*)AllPoints.GetUnsafeReadOnlyPtr() + inputParameters.PointsStart,
                    UnsafeUtility.SizeOf<float3>() * inputParameters.PointCount
                );

                Output[index] = new KeyValuePair<Hash128, BlobAssetReference<Collider>>(
                    InputKeys[index],
                    ConvexCollider.Create(points, inputParameters.GenerationParameters, inputParameters.Filter, inputParameters.Material)
                );
            }
        }
    }
}
