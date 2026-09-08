using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using UnityEngine;
using FloatRange = Unity.Physics.Math.FloatRange;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Stores the keys of a managed AnimationCurve inside an unmanaged BlobAsset that can serialized inside a SubScene.
    /// At runtime, the corresponding AnimationCurve can be recreated and evaluated.
    /// </summary>
    public struct AnimationCurveBlob
    {
        public BlobArray<Keyframe> Keys;
        public WrapMode PreWrapMode, PostWrapMode;
    }

    public static class AnimationCurveExtensions
    {
        public static BlobAssetReference<AnimationCurveBlob> ToBlobAssetReference(this AnimationCurve curve,
            Allocator allocator)
        {
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<AnimationCurveBlob>();
            root.PreWrapMode = curve.preWrapMode;
            root.PostWrapMode = curve.postWrapMode;
            
            var keys = curve.keys;
            var keysBuilder = builder.Allocate(ref root.Keys, keys.Length);
            for (var i = 0; i < keys.Length; i++)
            {
                keysBuilder[i] = keys[i];
            }
            
            var blobAssetReference =  builder.CreateBlobAssetReference<AnimationCurveBlob>(allocator);
            builder.Dispose();
            return blobAssetReference;
        }
    }
    
    public static class AnimationCurveEvaluator
    {
        static Dictionary<BlobAssetReference<AnimationCurveBlob>, AnimationCurve> s_Curves = new();

        /// <summary>
        /// Internally recreates a managed AnimationCurve from an AnimationCurveBlob and evaluate the curve (the
        /// AnimationCurve is cached for later reuses). We could technically avoid recreating an AnimationCurve and
        /// directly evaluate the curve from the AnimationCurveBlob data, but replicating the exact same behavior from
        /// AnimationCurve.Evaluate() is not trivial, the risk of getting a different result is significant.
        /// </summary>
        [BurstDiscard]
        public static float Evaluate(BlobAssetReference<AnimationCurveBlob> blobAssetReference, float time)
        {
            var curve = GetCurve(blobAssetReference);
            return curve?.Evaluate(time) ?? 0.0f;
        }
        
        static AnimationCurve GetCurve(BlobAssetReference<AnimationCurveBlob> blobAssetReference)
        {
            if (!blobAssetReference.IsCreated)
                return null;

            if (s_Curves.TryGetValue(blobAssetReference, out var curve))
                return curve;
            
            var keys = new Keyframe[blobAssetReference.Value.Keys.Length];
            for (var i = 0; i < keys.Length; i++)
            {
                keys[i] = blobAssetReference.Value.Keys[i];
            }
            
            curve = new AnimationCurve();
            curve.keys = keys;
            curve.preWrapMode = blobAssetReference.Value.PreWrapMode;
            curve.postWrapMode = blobAssetReference.Value.PostWrapMode;
            s_Curves.Add(blobAssetReference, curve);

            return curve;
        }
    }
    
    // Stores the initial value of the joint. This data should not be shared
    struct ModifyJointLimitsInitialValue : IComponentData
    {
        public PhysicsJoint Joint;
    }
    
    // Stores a pair of scalar curves to apply to relevant constraints on the joint.
    // The same curves are applied to each joint and therefore can be shared
    struct ModifyJointLimits : ISharedComponentData, IEquatable<ModifyJointLimits>
    {
        public MinMaxCurve AngularRangeScalar;
        public MinMaxCurve LinearRangeScalar;

        public bool Equals(ModifyJointLimits other) =>
            AngularRangeScalar.Equals(other.AngularRangeScalar) && LinearRangeScalar.Equals(other.LinearRangeScalar);

        public override bool Equals(object obj) => obj is ModifyJointLimits other && Equals(other);

        public override int GetHashCode() =>
            unchecked((AngularRangeScalar.GetHashCode() * 397) ^ LinearRangeScalar.GetHashCode());
    }

    // an authoring component to add to a GameObject with one or more Joint
    public class ModifyJointLimitsAuthoring : MonoBehaviour
    {
        public ParticleSystem.MinMaxCurve AngularRangeScalar = new ParticleSystem.MinMaxCurve(
            1f,
            min: new AnimationCurve(
                new Keyframe(0f, 0f, 0f, 0f),
                new Keyframe(2f, -2f, 0f, 0f),
                new Keyframe(4f, 0f, 0f, 0f)
            )
            {
                preWrapMode = WrapMode.Loop,
                postWrapMode = WrapMode.Loop
            },
            max: new AnimationCurve(
                new Keyframe(0f, 1f, 0f, 0f),
                new Keyframe(2f, -1f, 0f, 0f),
                new Keyframe(4f, 1f, 0f, 0f)
            )
            {
                preWrapMode = WrapMode.Loop,
                postWrapMode = WrapMode.Loop
            }
        );

        public ParticleSystem.MinMaxCurve LinearRangeScalar = new ParticleSystem.MinMaxCurve(
            1f,
            min: new AnimationCurve(
                new Keyframe(0f, 1f, 0f, 0f),
                new Keyframe(2f, 0.5f, 0f, 0f),
                new Keyframe(4f, 1f, 0f, 0f)
            )
            {
                preWrapMode = WrapMode.Loop,
                postWrapMode = WrapMode.Loop
            },
            max: new AnimationCurve(
                new Keyframe(0f, 0.5f, 0f, 0f),
                new Keyframe(2f, 0f, 0f, 0f),
                new Keyframe(4f, 0.5f, 0f, 0f)
            )
            {
                preWrapMode = WrapMode.Loop,
                postWrapMode = WrapMode.Loop
            }
        );
    }

    public struct MinMaxCurve
    {
        public MinMaxCurve(ParticleSystem.MinMaxCurve minMaxCurve, Allocator allocator)
        {
            CurveMin = minMaxCurve.curveMin.ToBlobAssetReference(allocator);
            CurveMax = minMaxCurve.curveMax.ToBlobAssetReference(allocator);
        }
        
        public BlobAssetReference<AnimationCurveBlob> CurveMin;
        public BlobAssetReference<AnimationCurveBlob> CurveMax;
    }

    [BakingType]
    public struct ModifyJointLimitsBakingData : IComponentData
    {
        public MinMaxCurve AngularRangeScalar;
        public MinMaxCurve LinearRangeScalar;
    }

    class ModifyJointLimitsBaker : Baker<ModifyJointLimitsAuthoring>
    {
        public override void Bake(ModifyJointLimitsAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new ModifyJointLimitsBakingData
            {
                AngularRangeScalar = new (authoring.AngularRangeScalar, Allocator.Persistent),
                LinearRangeScalar = new (authoring.LinearRangeScalar, Allocator.Persistent)
            });
        }
    }

    // after joints have been converted, find the entities they produced and add ModifyJointLimits to them
    [UpdateAfter(typeof(EndJointBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    partial struct ModifyJointLimitsBakingSystem : ISystem
    {
        private EntityQuery _ModifyJointLimitsBakingDataQuery;
        private EntityQuery _JointEntityBakingQuery;

        public void OnCreate(ref SystemState state)
        {
            _ModifyJointLimitsBakingDataQuery = state.GetEntityQuery(new EntityQueryDesc
            {
                All = new[] {ComponentType.ReadOnly<ModifyJointLimitsBakingData>()},
                Options = EntityQueryOptions.IncludeDisabledEntities | EntityQueryOptions.IncludePrefab
            });

            _JointEntityBakingQuery = state.GetEntityQuery(new EntityQueryDesc
            {
                All = new[] {ComponentType.ReadOnly<JointEntityBaking>()}
            });

            _ModifyJointLimitsBakingDataQuery.AddChangedVersionFilter(typeof(ModifyJointLimitsBakingData));
            _JointEntityBakingQuery.AddChangedVersionFilter(typeof(JointEntityBaking));
        }

        public void OnUpdate(ref SystemState state)
        {
            if (_ModifyJointLimitsBakingDataQuery.IsEmpty && _JointEntityBakingQuery.IsEmpty)
            {
                return;
            }

            // Collect all the joints
            NativeParallelMultiHashMap<Entity, (Entity, PhysicsJoint)> jointsLookUp =
                new NativeParallelMultiHashMap<Entity, (Entity, PhysicsJoint)>(10, Allocator.TempJob);

            foreach (var(jointEntity, physicsJoint, entity) in SystemAPI
                     .Query<RefRO<JointEntityBaking>, RefRO<PhysicsJoint>>().WithEntityAccess()
                     .WithOptions(EntityQueryOptions.IncludeDisabledEntities | EntityQueryOptions.IncludePrefab))
            {
                jointsLookUp.Add(jointEntity.ValueRO.Entity, (entity, physicsJoint.ValueRO));
            }

            var ecb = new EntityCommandBuffer(Allocator.Temp);
            
            foreach (var(modifyJointLimits, entity) in SystemAPI.Query<ModifyJointLimitsBakingData>()
                     .WithEntityAccess().WithOptions(EntityQueryOptions.IncludeDisabledEntities |
                         EntityQueryOptions.IncludePrefab))
            {
                foreach (var joint in jointsLookUp.GetValuesForKey(entity))
                {
                    ecb.AddComponent(joint.Item1, new ModifyJointLimitsInitialValue
                    {
                        Joint = joint.Item2
                    });
                    ecb.SetSharedComponent(joint.Item1, new ModifyJointLimits
                    {
                        AngularRangeScalar = modifyJointLimits.AngularRangeScalar,
                        LinearRangeScalar = modifyJointLimits.LinearRangeScalar
                    });
                }
            }
            
            ecb.Playback(state.EntityManager);

            jointsLookUp.Dispose();
        }
    }

    // apply an animated effect to the limits on supported types of joints
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(PhysicsSystemGroup), OrderLast = true)]
    partial struct ModifyJointLimitsSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            var time = (float)SystemAPI.Time.ElapsedTime;

            foreach (var(joint, initialJointValue, modification) in SystemAPI.Query<RefRW<PhysicsJoint>, RefRO<ModifyJointLimitsInitialValue>, ModifyJointLimits>())
            {
                var animatedAngularScalar = new FloatRange(
                    AnimationCurveEvaluator.Evaluate(modification.AngularRangeScalar.CurveMin, time),
                    AnimationCurveEvaluator.Evaluate(modification.AngularRangeScalar.CurveMax, time)
                );
                var animatedLinearScalar = new FloatRange(
                    AnimationCurveEvaluator.Evaluate(modification.LinearRangeScalar.CurveMin, time),
                    AnimationCurveEvaluator.Evaluate(modification.LinearRangeScalar.CurveMax, time)
                );

                var initial = initialJointValue.ValueRO.Joint;

                // in each case, get relevant properties from the initial value based on joint type, and apply scalar
                switch (joint.ValueRW.JointType)
                {
                    // Custom type could be anything, so this demo just applies changes to all constraints
                    case JointType.Custom:
                        var constraints = initial.GetConstraints();
                        for (var i = 0; i < constraints.Length; i++)
                        {
                            var constraint = constraints[i];
                            var isAngular = constraint.Type == ConstraintType.Angular;
                            var scalar = math.select(animatedLinearScalar, animatedAngularScalar, isAngular);
                            var constraintRange = (FloatRange)(new float2(constraint.Min, constraint.Max) * scalar);
                            constraint.Min = constraintRange.Min;
                            constraint.Max = constraintRange.Max;
                            constraints[i] = constraint;
                        }

                        joint.ValueRW.SetConstraints(constraints);
                        break;
                    // other types have corresponding getters/setters to retrieve more meaningful data
                    case JointType.LimitedDistance:
                        var distanceRange = initial.GetLimitedDistanceRange();
                        joint.ValueRW.SetLimitedDistanceRange(distanceRange * (float2)animatedLinearScalar);
                        break;
                    case JointType.LimitedHinge:
                        var angularRange = initial.GetLimitedHingeRange();
                        joint.ValueRW.SetLimitedHingeRange(angularRange * (float2)animatedAngularScalar);
                        break;
                    case JointType.Prismatic:
                        var distanceOnAxis = initial.GetPrismaticRange();
                        joint.ValueRW.SetPrismaticRange(distanceOnAxis * (float2)animatedLinearScalar);
                        break;
                    // ragdoll joints are composed of two separate joints with different meanings
                    case JointType.RagdollPrimaryCone:
                        initial.GetRagdollPrimaryConeAndTwistRange(
                            out var maxConeAngle,
                            out var angularTwistRange
                        );
                        joint.ValueRW.SetRagdollPrimaryConeAndTwistRange(
                            maxConeAngle * animatedAngularScalar.Max,
                            angularTwistRange * (float2)animatedAngularScalar
                        );
                        break;
                    case JointType.RagdollPerpendicularCone:
                        var angularPlaneRange = initial.GetRagdollPerpendicularConeRange();
                        joint.ValueRW.SetRagdollPerpendicularConeRange(angularPlaneRange *
                            (float2)animatedAngularScalar);
                        break;
                    // remaining types have no limits on their Constraint atoms to meaningfully modify
                    case JointType.BallAndSocket:
                    case JointType.Fixed:
                    case JointType.Hinge:
                        break;
                }
            }
        }
    }
}
