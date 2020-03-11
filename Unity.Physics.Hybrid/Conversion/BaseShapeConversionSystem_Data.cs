using System;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    // structure with minimal data needed to incrementally convert a shape that is possibly part of a compound collider
    struct ColliderInstance : IEquatable<ColliderInstance>
    {
        public int AuthoringComponentId;
        public int ConvertedAuthoringComponentIndex; // index into EndColliderConversionSystem buffer
        public int ConvertedBodyTransformIndex;
        public Entity BodyEntity;
        public Entity ShapeEntity;
        public RigidTransform BodyFromShape;
        public Hash128 Hash;

        public static RigidTransform GetCompoundFromChild(Transform shape, Transform body)
        {
            var worldFromBody = Math.DecomposeRigidBodyTransform(body.transform.localToWorldMatrix);
            var worldFromShape = Math.DecomposeRigidBodyTransform(shape.transform.localToWorldMatrix);
            return math.mul(math.inverse(worldFromBody), worldFromShape);
        }

        public bool Equals(ColliderInstance other)
        {
            return AuthoringComponentId == other.AuthoringComponentId
                && ConvertedAuthoringComponentIndex == other.ConvertedAuthoringComponentIndex
                && BodyEntity.Equals(other.BodyEntity)
                && ShapeEntity.Equals(other.ShapeEntity)
                && BodyFromShape.Equals(other.BodyFromShape)
                && Hash.Equals(other.Hash);
        }

        public override bool Equals(object obj) => obj is ColliderInstance other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = AuthoringComponentId;
                hashCode = (hashCode * 397) ^ ConvertedAuthoringComponentIndex;
                hashCode = (hashCode * 397) ^ BodyEntity.GetHashCode();
                hashCode = (hashCode * 397) ^ ShapeEntity.GetHashCode();
                hashCode = (hashCode * 397) ^ BodyFromShape.GetHashCode();
                hashCode = (hashCode * 397) ^ Hash.GetHashCode();
                return hashCode;
            }
        }

        public ColliderInstanceId ToColliderInstanceId() => new ColliderInstanceId(Hash, AuthoringComponentId);
    }

    struct ColliderInstanceId : IEquatable<ColliderInstanceId>
    {
        public ColliderInstanceId(Hash128 blobDataHash, int authoringComponentId)
        {
            BlobDataHash = blobDataHash;
            AuthoringComponentId = authoringComponentId;
        }

        readonly Hash128 BlobDataHash;
        readonly int AuthoringComponentId;

        public bool Equals(ColliderInstanceId other) =>
            BlobDataHash.Equals(other.BlobDataHash) && AuthoringComponentId == other.AuthoringComponentId;

        public override bool Equals(object obj) => obj is ColliderInstanceId other && Equals(other);

        public override int GetHashCode() =>
            (int)math.hash(new uint2((uint)BlobDataHash.GetHashCode(), (uint)AuthoringComponentId));

        public static bool operator ==(ColliderInstanceId left, ColliderInstanceId right) => left.Equals(right);

        public static bool operator !=(ColliderInstanceId left, ColliderInstanceId right) => !left.Equals(right);
    }

    public partial class BaseShapeConversionSystem<T>
    {
        internal struct ShapeComputationData
        {
            public ColliderInstance Instance;

            public uint ForceUniqueIdentifier;
            public Material Material;
            public CollisionFilter CollisionFilter;

            // TODO: use union to share the same memory zone for each different type
            public ShapeType ShapeType;
            public BoxGeometry BoxProperties;
            public CapsuleGeometry CapsuleProperties;
            public CylinderGeometry CylinderProperties;
            public SphereGeometry SphereProperties;
            public float3x4 PlaneVertices;
            public ConvexInput ConvexHullProperties;
            public MeshInput MeshProperties;

            public float4x4 BodyFromShape => new float4x4(Instance.BodyFromShape);
        }
    }
}