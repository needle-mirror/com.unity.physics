using System;
using Unity.Mathematics;
using static Unity.Physics.BoundingVolumeHierarchy;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // The input to collider cast queries consists of a Collider and its initial orientation,
    // and the Start & End positions of a line segment the Collider is to be swept along.
    public unsafe struct ColliderCastInput
    {
        public Collider* Collider;
        public quaternion Orientation { get; set; }

        public float3 Start
        {
            get => Ray.Origin;
            set
            {
                float3 end = Ray.Origin + Ray.Displacement;
                Ray.Origin = value;
                Ray.Displacement = end - value;
            }
        }
        public float3 End
        {
            get => Ray.Origin + Ray.Displacement;
            set => Ray.Displacement = value - Ray.Origin;
        }

        internal Ray Ray;

        #region Obsolete members
        [Obsolete("Position has been deprecated. Use Start Instead (RemovedAfter 2019-07-29) (UnityUpgradable) -> Start")]
        public float3 Position { get => Ray.Origin; set => Ray.Origin = value; }
        [Obsolete("Direction has been deprecated. Use End Instead (RemovedAfter 2019-07-29)")]
        public float3 Direction { get => Ray.Direction; set => Ray.Direction = value; }
        #endregion
    }

    // A hit from a collider cast query
    public struct ColliderCastHit : IQueryResult
    {
        public float Fraction { get; set; }

        public float3 Position;
        public float3 SurfaceNormal;
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;

        public void Transform(MTransform transform, uint numSubKeyBits, uint subKey)
        {
            Position = Mul(transform, Position);
            SurfaceNormal = math.mul(transform.Rotation, SurfaceNormal);
            ColliderKey.PushSubKey(numSubKeyBits, subKey);
        }

        public void Transform(MTransform transform, int rigidBodyIndex)
        {
            Position = Mul(transform, Position);
            SurfaceNormal = math.mul(transform.Rotation, SurfaceNormal);
            RigidBodyIndex = rigidBodyIndex;
        }
    }

    // Collider cast query implementations
    public static class ColliderCastQueries
    {
        public static unsafe bool ColliderCollider<T>(ColliderCastInput input, Collider* target, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, target->Filter))
            {
                return false;
            }

            switch (input.Collider->CollisionType)
            {
                case CollisionType.Convex:
                    switch (target->Type)
                    {
                        case ColliderType.Sphere:
                        case ColliderType.Capsule:
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Box:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            return ConvexConvex(input, target, ref collector);
                        case ColliderType.Mesh:
                            return ConvexMesh(input, (MeshCollider*)target, ref collector);
                        case ColliderType.Compound:
                            return ConvexCompound(input, (CompoundCollider*)target, ref collector);
                        default:
                            throw new NotImplementedException();
                    }
                case CollisionType.Composite:
                    // no support for casting composite shapes
                    throw new NotImplementedException();
                default:
                    throw new NotImplementedException();
            }
        }

        private static unsafe bool ConvexConvex<T>(ColliderCastInput input, Collider* target, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            //Assert.IsTrue(target->CollisionType == CollisionType.Convex && input.Collider->CollisionType == CollisionType.Convex, "ColliderCast.ConvexConvex can only process convex colliders");

            // Get the current transform
            MTransform targetFromQuery = new MTransform(input.Orientation, input.Start);

            // Conservative advancement
            const float tolerance = 1e-3f;      // return if this close to a hit
            const float keepDistance = 1e-4f;   // avoid bad cases for GJK (penetration / exact hit)
            int iterations = 10;                // return after this many advances, regardless of accuracy
            float fraction = 0.0f;
            while (true)
            {
                if (fraction >= collector.MaxFraction)
                {
                    // Exceeded the maximum fraction without a hit
                    return false;
                }

                // Find the current distance
                DistanceQueries.Result distanceResult = DistanceQueries.ConvexConvex(target, input.Collider, targetFromQuery);

                // Check for a hit
                if (distanceResult.Distance < tolerance || --iterations == 0)
                {
                    targetFromQuery.Translation = input.Start;
                    return collector.AddHit(new ColliderCastHit
                    {
                        Position = distanceResult.PositionOnBinA,
                        SurfaceNormal = -distanceResult.NormalInA,
                        Fraction = fraction,
                        ColliderKey = ColliderKey.Empty,
                        RigidBodyIndex = -1
                    });
                }

                // Check for a miss
                float dot = math.dot(distanceResult.NormalInA, input.Ray.Displacement);
                if (dot <= 0.0f)
                {
                    // Collider is moving away from the target, it will never hit
                    return false;
                }

                // Advance
                fraction += (distanceResult.Distance - keepDistance) / dot;
                if (fraction >= collector.MaxFraction)
                {
                    // Exceeded the maximum fraction without a hit
                    return false;
                }

                targetFromQuery.Translation = math.lerp(input.Start, input.End, fraction);
            }
        }

        private unsafe struct ConvexMeshLeafProcessor : IColliderCastLeafProcessor
        {
            private readonly Mesh* m_Mesh;
            private readonly uint m_NumColliderKeyBits;

            public ConvexMeshLeafProcessor(MeshCollider* meshCollider)
            {
                m_Mesh = &meshCollider->Mesh;
                m_NumColliderKeyBits = meshCollider->NumColliderKeyBits;
            }

            public bool ColliderCastLeaf<T>(ColliderCastInput input, int primitiveKey, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                m_Mesh->GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter);

                if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, filter)) // TODO: could do this check within GetPrimitive()
                {
                    return false;
                }

                int numPolygons = Mesh.GetNumPolygonsInPrimitive(flags);
                bool isQuad = Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad);

                bool acceptHit = false;
                int numHits = collector.NumHits;

                var polygon = new PolygonCollider();
                polygon.InitEmpty();
                for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++)
                {
                    float fraction = collector.MaxFraction;

                    if (isQuad)
                    {
                        polygon.SetAsQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
                    }
                    else
                    {
                        polygon.SetAsTriangle(vertices[0], vertices[1 + polygonIndex], vertices[2 + polygonIndex]);
                    }

                    if (ConvexConvex(input, (Collider*)&polygon, ref collector))
                    {
                        acceptHit = true;
                        // TODO.ma make a version that doesn't transform, just updates collider key
                        collector.TransformNewHits(numHits++, fraction, MTransform.Identity, m_NumColliderKeyBits, (uint)(primitiveKey << 1 | polygonIndex));
                    }
                }

                return acceptHit;
            }
        }

        private static unsafe bool ConvexMesh<T>(ColliderCastInput input, MeshCollider* meshCollider, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            var leafProcessor = new ConvexMeshLeafProcessor(meshCollider);
            return meshCollider->Mesh.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
        }

        private unsafe struct ConvexCompoundLeafProcessor : IColliderCastLeafProcessor
        {
            private readonly CompoundCollider* m_CompoundCollider;

            public ConvexCompoundLeafProcessor(CompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool ColliderCastLeaf<T>(ColliderCastInput input, int leafData, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, child.Collider->Filter))
                {
                    return false;
                }

                // Transform the cast into child space
                ColliderCastInput inputLs = input;
                RigidTransform childFromCompound = math.inverse(child.CompoundFromChild);
                inputLs.Ray.Origin = math.transform(childFromCompound, input.Ray.Origin);
                inputLs.Ray.Displacement = math.mul(childFromCompound.rot, input.Ray.Displacement);
                inputLs.Orientation = math.mul(childFromCompound.rot, input.Orientation);

                int numHits = collector.NumHits;
                float fraction = collector.MaxFraction;
                if (child.Collider->CastCollider(inputLs, ref collector))
                {
                    // Transform results back to compound space
                    collector.TransformNewHits(numHits, fraction, new MTransform(child.CompoundFromChild), m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                    return true;
                }
                return false;
            }
        }

        private static unsafe bool ConvexCompound<T>(ColliderCastInput input, CompoundCollider* compoundCollider, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            var leafProcessor = new ConvexCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
        }
    }
}
