using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Structure for hashing input data used for convex and mesh colliders.
    /// An instance represents relevant inputs from a single mesh renderer or skinned mesh renderer contributing to the collider's geometry.
    /// </summary>
    struct HashableShapeInputs
    {
        // solver generally not expected to be precise under 0.01 units
        internal const float k_DefaultLinearPrecision = 0.001f;

        readonly Aabb Bounds;

        public readonly int MeshKey; // TODO: currently assumes each mesh asset will have consistent data between hashes; should instead hash vertices/triangles
        public readonly float4x4 BodyFromShape;
        public readonly int IncludedIndicesStartIndex;
        public readonly int IncludedIndicesCount;
        public readonly int BlendShapeWeightsStartIndex;
        public readonly int BlendShapeWeightsCount;

        public static HashableShapeInputs FromMesh(UnityEngine.Mesh mesh, float4x4 leafToBody) => mesh == null
            ? new HashableShapeInputs(default, default, leafToBody)
            : new HashableShapeInputs(mesh.GetInstanceID(), mesh.bounds, leafToBody);

        public static HashableShapeInputs FromSkinnedMesh(
            UnityEngine.Mesh mesh, float4x4 leafToBody,
            NativeArray<int> includedIndices, NativeList<int> allIncludedIndices,
            NativeArray<float> blendShapeWeights, NativeList<float> allBlendShapeWeights
        )
        {
            return mesh == null
                ? new HashableShapeInputs(default, default, leafToBody)
                : new HashableShapeInputs(mesh.GetInstanceID(), mesh.bounds, leafToBody, includedIndices, allIncludedIndices, blendShapeWeights, allBlendShapeWeights);
        }

        HashableShapeInputs(
            int meshKey, Bounds bounds, float4x4 leafToBody,
            float linearPrecision = k_DefaultLinearPrecision
        )
        {
            Bounds = new Aabb { Min = bounds.min, Max = bounds.max };

            GetQuantizedTransformations(
                leafToBody, Bounds,
                out var translation, out var orientation, out var scale, out var shear,
                linearPrecision
            );

            MeshKey = meshKey;
            BodyFromShape = math.mul(
                new float4x4(new RigidTransform(orientation, translation)),
                math.mul(new float4x4(shear, 0f), float4x4.Scale(scale))
            );
            IncludedIndicesStartIndex = BlendShapeWeightsStartIndex = -1;
            IncludedIndicesCount = BlendShapeWeightsCount = 0;
        }

        HashableShapeInputs(
            int meshKey, Bounds bounds, float4x4 leafToBody,
            NativeArray<int> includedIndices, NativeList<int> allIncludedIndices,
            NativeArray<float> blendShapeWeights, NativeList<float> allBlendShapeWeights,
            float linearPrecision = k_DefaultLinearPrecision
        )
        {
            this = new HashableShapeInputs(meshKey, bounds, leafToBody, linearPrecision);

            if (allIncludedIndices.IsCreated)
            {
                IncludedIndicesStartIndex = allIncludedIndices.Length;
                IncludedIndicesCount = includedIndices.Length;
                allIncludedIndices.AddRange(includedIndices);
            }

            if (allBlendShapeWeights.IsCreated)
            {
                BlendShapeWeightsStartIndex = allBlendShapeWeights.Length;
                BlendShapeWeightsCount = blendShapeWeights.Length;
                allBlendShapeWeights.AddRange(blendShapeWeights);
            }
        }

        internal static void GetQuantizedTransformations(
            float4x4 leafToBody, Aabb bounds, out float4x4 transformations,
            float linearPrecision = k_DefaultLinearPrecision
        )
        {
            GetQuantizedTransformations(
                leafToBody, bounds, out var t, out var r, out var s, out var sh, linearPrecision
            );
            transformations = math.mul(new float4x4(sh, 0f), float4x4.TRS(t, r, s));
        }

        static void GetQuantizedTransformations(
            float4x4 leafToBody, Aabb bounds,
            out float3 translation, out quaternion orientationQ, out float3 scale, out float3x3 shear,
            float linearPrecision = k_DefaultLinearPrecision
        )
        {
            translation = RoundToNearest(leafToBody.c3.xyz, linearPrecision);

            var farthestPoint =
                math.abs(math.lengthsq(bounds.Max) > math.lengthsq(bounds.Min) ? bounds.Max : bounds.Min);

            // round scale using precision inversely proportional to mesh size along largest axis
            // (i.e. amount to scale farthest point one unit of linear precision)
            var scalePrecision = linearPrecision / math.max(math.cmax(farthestPoint), math.FLT_MIN_NORMAL);
            scale = RoundToNearest(
                new float3(
                    math.length(leafToBody.c0.xyz),
                    math.length(leafToBody.c1.xyz),
                    math.length(leafToBody.c2.xyz)
                ),
                scalePrecision
            );
            if (math.determinant(leafToBody) < 0f)
                scale.x *= -1f;

            shear = new float3x3(
                leafToBody.c0.xyz / math.max(math.abs(scale.x), math.FLT_MIN_NORMAL) * math.sign(scale.x),
                leafToBody.c1.xyz / math.max(math.abs(scale.y), math.FLT_MIN_NORMAL) * math.sign(scale.y),
                leafToBody.c2.xyz / math.max(math.abs(scale.z), math.FLT_MIN_NORMAL) * math.sign(scale.z)
            );
            var orientation = float3x3.LookRotationSafe(shear.c2, shear.c1);

            // if shear is very nearly identity, hash it as identity
            // TODO: quantize shear
            shear = math.mul(shear, math.inverse(orientation));
            if (!PhysicsShapeExtensions.HasShear(new float4x4(shear, 0f)))
                shear = float3x3.identity;

            // round orientation using precision inversely proportional to scaled mesh size
            // (i.e. radians to rotate farthest scaled point one unit of linear precision)
            var angularPrecision = math.min(linearPrecision / math.length(farthestPoint * scale), math.PI);
            var axisPrecision = math.min(math.cos(angularPrecision), math.sin(angularPrecision));
            orientation.c0 = math.normalize(RoundToNearest(orientation.c0, axisPrecision));
            orientation.c1 = math.normalize(RoundToNearest(orientation.c1, axisPrecision));
            orientation.c2 = math.normalize(RoundToNearest(orientation.c2, axisPrecision));

            translation = Sanitize(translation);
            orientationQ = new quaternion(Sanitize(orientation));
            scale = Sanitize(scale);
            shear = Sanitize(shear);
        }

        static float3 RoundToNearest(float3 value, float3 intervalPerAxis) =>
            math.round(value / intervalPerAxis) * intervalPerAxis;

        // prevents hashing negative zero
        static float3 Sanitize(float3 value)
        {
            for (var i = 0; i < 3; ++i)
            {
                if (value[i] == 0f)
                    value[i] = 0f;
            }
            return value;
        }

        static float3x3 Sanitize(float3x3 value)
        {
            for (var i = 0; i < 3; ++i)
                value[i] = Sanitize(value[i]);
            return value;
        }

        public static Hash128 GetHash128(
            uint uniqueIdentifier,
            ConvexHullGenerationParameters hullGenerationParameters,
            Material material,
            CollisionFilter filter,
            float4x4 bakeFromShape,
            NativeArray<HashableShapeInputs> inputs,
            NativeArray<int> allIncludedIndices,
            NativeArray<float> allBlendShapeWeights,
            float linearPrecision = k_DefaultLinearPrecision
        )
        {
            var numInputs = inputs.IsCreated ? inputs.Length : 0;

            // quantize shape-level transforms
            var bounds = new Aabb { Min = float.MaxValue, Max = float.MinValue };
            for (int i = 0; i < numInputs; i++)
            {
                var d = inputs[i];
                bounds.Include(math.mul(d.BodyFromShape, new float4(d.Bounds.Min, 1f)).xyz);
                bounds.Include(math.mul(d.BodyFromShape, new float4(d.Bounds.Max, 1f)).xyz);
            }
            GetQuantizedTransformations(bakeFromShape, bounds, out var bakeMatrix, linearPrecision);

            // bakeFromShape only contains scale/shear information, so only the inner 3x3 needs to contribute to the hash
            var scaleShear = new float3x3(bakeMatrix.c0.xyz, bakeMatrix.c1.xyz, bakeMatrix.c2.xyz);

            var builder = new SpookyHashBuilder(Allocator.Temp);
            builder.Append(ref uniqueIdentifier);
            builder.Append(ref hullGenerationParameters);
            builder.Append(ref material);
            builder.Append(ref filter);
            builder.Append(ref scaleShear);
            builder.Append(inputs);
            builder.Append(allIncludedIndices);
            builder.Append(allBlendShapeWeights);
            return builder.Finish();
        }
    }
}
