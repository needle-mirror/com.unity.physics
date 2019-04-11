using System;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // A header preceding a number of contact points in a stream.
    public struct ContactHeader
    {
        public BodyIndexPair BodyPair;
        public CustomDataPair BodyCustomDatas;
        public JacobianFlags JacobianFlags;
        public int NumContacts;
        public float3 Normal;
        public float CoefficientOfFriction;
        public float CoefficientOfRestitution;
        public ColliderKeyPair ColliderKeys;

        // followed by NumContacts * ContactPoint
    }

    // A contact point in a manifold. All contacts share the same normal.
    public struct ContactPoint
    {
        public float3 Position; // world space position on object A
        public float Distance;  // separating distance along the manifold normal
    }


    // Contact manifold stream generation functions
    public static class ManifoldQueries
    {
        // A context passed through the manifold generation functions
        private struct Context
        {
            public BodyIndexPair BodyIndices;
            public CustomDataPair BodyCustomDatas;
        }

        // Write a set of contact manifolds for a pair of bodies to the given stream.
        public static unsafe void BodyBody(ref PhysicsWorld world, BodyIndexPair pair, float timeStep, ref BlockStream.Writer contactWriter)
        {
            RigidBody rigidBodyA = world.Bodies[pair.BodyAIndex];
            RigidBody rigidBodyB = world.Bodies[pair.BodyBIndex];

            Collider* colliderA = rigidBodyA.Collider;
            Collider* colliderB = rigidBodyB.Collider;

            if (colliderA == null || colliderB == null || !CollisionFilter.IsCollisionEnabled(colliderA->Filter, colliderB->Filter))
            {
                return;
            }

            // Build combined motion expansion
            MotionExpansion expansion;
            {
                MotionExpansion GetBodyExpansion(int bodyIndex, NativeSlice<MotionVelocity> mvs)
                {
                    return bodyIndex < mvs.Length ? mvs[bodyIndex].CalculateExpansion(timeStep) : MotionExpansion.Zero;
                }
                MotionExpansion expansionA = GetBodyExpansion(pair.BodyAIndex, world.MotionVelocities);
                MotionExpansion expansionB = GetBodyExpansion(pair.BodyBIndex, world.MotionVelocities);
                expansion = new MotionExpansion
                {
                    Linear = expansionA.Linear - expansionB.Linear,
                    Uniform = expansionA.Uniform + expansionB.Uniform + world.CollisionTolerance
                };
            }

            var context = new Context
            {
                BodyIndices = pair,
                BodyCustomDatas = new CustomDataPair { CustomDataA = rigidBodyA.CustomData, CustomDataB = rigidBodyB.CustomData }
            };

            var worldFromA = new MTransform(rigidBodyA.WorldFromBody);
            var worldFromB = new MTransform(rigidBodyB.WorldFromBody);

            // Dispatch to appropriate manifold generator
            switch (colliderA->CollisionType)
            {
                case CollisionType.Convex:
                    switch (colliderB->CollisionType)
                    {
                        case CollisionType.Convex:
                            ConvexConvex(context, ColliderKeyPair.Empty, colliderA, colliderB, worldFromA, worldFromB, expansion.MaxDistance, false, ref contactWriter);
                            break;
                        case CollisionType.Composite:
                            ConvexComposite(context, ColliderKey.Empty, colliderA, colliderB, worldFromA, worldFromB, expansion, false, ref contactWriter);
                            break;
                    }
                    break;
                case CollisionType.Composite:
                    switch (colliderB->CollisionType)
                    {
                        case CollisionType.Convex:
                            CompositeConvex(context, colliderA, colliderB, worldFromA, worldFromB, expansion, false, ref contactWriter);
                            break;
                        case CollisionType.Composite:
                            CompositeComposite(context, colliderA, colliderB, worldFromA, worldFromB, expansion, false, ref contactWriter);
                            break;
                    }
                    break;
            }
        }

        private static unsafe void ConvexConvex(
            Context context, ColliderKeyPair colliderKeys,
            Collider* convexColliderA, Collider* convexColliderB, MTransform worldFromA, MTransform worldFromB,
            float maxDistance, bool flipped, ref BlockStream.Writer contactWriter)
        {
            MTransform aFromB = Mul(Inverse(worldFromA), worldFromB);

            ConvexConvexManifoldQueries.Manifold contactManifold;
            switch (convexColliderA->Type)
            {
                case ColliderType.Sphere:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.SphereSphere(
                                (SphereCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleSphere(
                                (CapsuleCollider*)convexColliderB, (SphereCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Triangle:
                            ConvexConvexManifoldQueries.TriangleSphere(
                                (PolygonCollider*)convexColliderB, (SphereCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxSphere(
                                (BoxCollider*)convexColliderB, (SphereCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((SphereCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Box:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.BoxSphere(
                                (BoxCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Triangle:
                            ConvexConvexManifoldQueries.BoxTriangle(
                                (BoxCollider*)convexColliderA, (PolygonCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxBox(
                                (BoxCollider*)convexColliderA, (BoxCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((BoxCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Capsule:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.CapsuleSphere(
                                (CapsuleCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleCapsule(
                                (CapsuleCollider*)convexColliderA, (CapsuleCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Triangle:
                            ConvexConvexManifoldQueries.CapsuleTriangle(
                                (CapsuleCollider*)convexColliderA, (PolygonCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Quad:
                        case ColliderType.Box:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((CapsuleCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Triangle:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.TriangleSphere(
                                (PolygonCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleTriangle(
                                (CapsuleCollider*)convexColliderB, (PolygonCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxTriangle(
                                (BoxCollider*)convexColliderB, (PolygonCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((PolygonCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Quad:
                case ColliderType.Cylinder:
                case ColliderType.Convex:
                    ConvexConvexManifoldQueries.ConvexConvex(
                        ref ((ConvexCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                        worldFromA, aFromB, maxDistance, out contactManifold);
                    break;
                default:
                    throw new NotImplementedException();
            }

            // Write results to stream
            if (contactManifold.NumContacts > 0)
            {
                if (flipped)
                {
                    contactManifold.Flip();
                }

                var header = new ContactHeader
                {
                    BodyPair = context.BodyIndices,
                    BodyCustomDatas = context.BodyCustomDatas,
                    NumContacts = contactManifold.NumContacts,
                    Normal = contactManifold.Normal,
                    ColliderKeys = colliderKeys
                };

                // Apply materials
                {
                    Material materialA = ((ConvexColliderHeader*)convexColliderA)->Material;
                    Material materialB = ((ConvexColliderHeader*)convexColliderB)->Material;
                    Material.MaterialFlags combinedFlags = materialA.Flags | materialB.Flags;
                    if ((combinedFlags & Material.MaterialFlags.IsTrigger) != 0)
                    {
                        header.JacobianFlags |= JacobianFlags.IsTrigger;
                    }
                    else
                    {
                        if ((combinedFlags & Material.MaterialFlags.EnableCollisionEvents) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableCollisionEvents;
                        }
                        if ((combinedFlags & Material.MaterialFlags.EnableMassFactors) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableMassFactors;
                        }
                        if ((combinedFlags & Material.MaterialFlags.EnableSurfaceVelocity) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableSurfaceVelocity;
                        }
                        if ((combinedFlags & Material.MaterialFlags.EnableMaxImpulse) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableMaxImpulse;
                        }

                        header.CoefficientOfFriction = Material.GetCombinedFriction(materialA, materialB);
                        header.CoefficientOfRestitution = Material.GetCombinedRestitution(materialA, materialB);
                    }
                }

                contactWriter.Write(header);

                for (int contactIndex = 0; contactIndex < header.NumContacts; contactIndex++)
                {
                    contactWriter.Write(contactManifold[contactIndex]);
                }
            }
        }

        private static unsafe void ConvexComposite(
            Context context, ColliderKey convexKeyA,
            Collider* convexColliderA, Collider* compositeColliderB, MTransform worldFromA, MTransform worldFromB,
            MotionExpansion expansion, bool flipped, ref BlockStream.Writer contactWriter)
        {
            // Calculate AABB of A in B
            MTransform bFromWorld = Inverse(worldFromB);
            MTransform bFromA = Mul(bFromWorld, worldFromA);
            var transform = new RigidTransform(new quaternion(bFromA.Rotation), bFromA.Translation); // TODO: avoid this conversion to and back from float3x3
            Aabb aabbAinB = expansion.ExpandAabb(convexColliderA->CalculateAabb(transform));

            // Do the midphase query and build manifolds for any overlapping leaf colliders
            var input = new OverlapAabbInput { Aabb = aabbAinB, Filter = convexColliderA->Filter };
            var collector = new ConvexCompositeOverlapCollector(
                context,
                convexColliderA, convexKeyA, compositeColliderB,
                worldFromA, worldFromB, expansion.MaxDistance, flipped,
                contactWriter);
            OverlapQueries.AabbCollider(input, compositeColliderB, ref collector);

            // Keep updated writer state
            contactWriter = collector.m_ContactWriter;
        }

        private static unsafe void CompositeConvex(
            Context context,
            Collider* compositeColliderA, Collider* convexColliderB, MTransform worldFromA, MTransform worldFromB,
            MotionExpansion expansion, bool flipped, ref BlockStream.Writer contactWriter)
        {
            // Flip the relevant inputs and call convex-vs-composite
            expansion.Linear *= -1.0f;
            ConvexComposite(context, ColliderKey.Empty,
                convexColliderB, compositeColliderA, worldFromB, worldFromA, expansion, !flipped,
                ref contactWriter);
        }

        private unsafe struct ConvexCompositeOverlapCollector : IOverlapCollector
        {
            // Inputs
            readonly Context m_Context;
            readonly Collider* m_ConvexColliderA;
            readonly ColliderKey m_ConvexColliderKey;
            readonly Collider* m_CompositeColliderB;
            readonly MTransform m_WorldFromA;
            readonly MTransform m_WorldFromB;
            readonly float m_CollisionTolerance;
            readonly bool m_Flipped;

            ColliderKeyPath m_CompositeColliderKeyPath;

            // Output
            internal BlockStream.Writer m_ContactWriter;

            public ConvexCompositeOverlapCollector(
                Context context,
                Collider* convexCollider, ColliderKey convexColliderKey, Collider* compositeCollider,
                MTransform worldFromA, MTransform worldFromB, float collisionTolerance, bool flipped,
                BlockStream.Writer contactWriter)
            {
                m_Context = context;
                m_ConvexColliderA = convexCollider;
                m_ConvexColliderKey = convexColliderKey;
                m_CompositeColliderB = compositeCollider;
                m_CompositeColliderKeyPath = ColliderKeyPath.Empty;
                m_WorldFromA = worldFromA;
                m_WorldFromB = worldFromB;
                m_CollisionTolerance = collisionTolerance;
                m_Flipped = flipped;
                m_ContactWriter = contactWriter;
            }

            public void AddRigidBodyIndices(int* indices, int count)
            {
                throw new NotSupportedException();
            }

            public void AddColliderKeys(ColliderKey* keys, int count)
            {
                ColliderKeyPair colliderKeys = new ColliderKeyPair { ColliderKeyA = m_ConvexColliderKey, ColliderKeyB = m_ConvexColliderKey };
                CollisionFilter filter = m_ConvexColliderA->Filter;

                // Collide the convex A with all overlapping leaves of B
                switch (m_CompositeColliderB->Type)
                {
                    // Special case meshes (since we know all polygons will be built on the fly)
                    case ColliderType.Mesh:
                    {
                        Mesh* mesh = &((MeshCollider*)m_CompositeColliderB)->Mesh;
                        uint numMeshKeyBits = mesh->NumColliderKeyBits;
                        var polygon = new PolygonCollider();
                        polygon.InitEmpty();
                        for (int i = 0; i < count; i++)
                        {
                            ColliderKey compositeKey = m_CompositeColliderKeyPath.GetLeafKey(keys[i]);
                            uint meshKey = compositeKey.Value >> (32 - (int)numMeshKeyBits);
                            if (mesh->GetPolygon(meshKey, filter, ref polygon))
                            {
                                if (m_Flipped)
                                {
                                    colliderKeys.ColliderKeyA = compositeKey;
                                }
                                else
                                {
                                    colliderKeys.ColliderKeyB = compositeKey;
                                }

                                ConvexConvex(
                                    m_Context, colliderKeys, m_ConvexColliderA, (Collider*)&polygon,
                                    m_WorldFromA, m_WorldFromB, m_CollisionTolerance, m_Flipped, ref m_ContactWriter);
                            }
                        }
                    }
                    break;

                    // General case for all other composites (compounds, compounds of meshes, etc)
                    default:
                    {
                        for (int i = 0; i < count; i++)
                        {
                            ColliderKey compositeKey = m_CompositeColliderKeyPath.GetLeafKey(keys[i]);
                            m_CompositeColliderB->GetLeaf(compositeKey, out ChildCollider leaf);
                            if (CollisionFilter.IsCollisionEnabled(filter, leaf.Collider->Filter))  // TODO: shouldn't be needed if/when filtering is done fully by the BVH query
                            {
                                if (m_Flipped)
                                {
                                    colliderKeys.ColliderKeyA = compositeKey;
                                }
                                else
                                {
                                    colliderKeys.ColliderKeyB = compositeKey;
                                }

                                MTransform worldFromLeafB = Mul(m_WorldFromB, new MTransform(leaf.TransformFromChild));
                                ConvexConvex(
                                    m_Context, colliderKeys, m_ConvexColliderA, leaf.Collider,
                                    m_WorldFromA, worldFromLeafB, m_CollisionTolerance, m_Flipped, ref m_ContactWriter);
                            }
                        }
                    }
                    break;
                }
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey)
            {
                m_CompositeColliderKeyPath.PushChildKey(compositeKey);
            }

            public void PopCompositeCollider(uint numCompositeKeyBits)
            {
                m_CompositeColliderKeyPath.PopChildKey(numCompositeKeyBits);
            }
        }

        private static unsafe void CompositeComposite(
            Context context,
            Collider* compositeColliderA, Collider* compositeColliderB, MTransform worldFromA, MTransform worldFromB,
            MotionExpansion expansion, bool flipped, ref BlockStream.Writer contactWriter)
        {
            // Flip the order if necessary, so that A has fewer leaves than B
            if (compositeColliderA->NumColliderKeyBits > compositeColliderB->NumColliderKeyBits)
            {
                Collider* c = compositeColliderA;
                compositeColliderA = compositeColliderB;
                compositeColliderB = c;

                MTransform t = worldFromA;
                worldFromA = worldFromB;
                worldFromB = t;

                expansion.Linear *= -1.0f;
                flipped = !flipped;
            }

            var collector = new CompositeCompositeLeafCollector(
                context,
                compositeColliderA, compositeColliderB,
                worldFromA, worldFromB, expansion, flipped,
                contactWriter);
            compositeColliderA->GetLeaves(ref collector);

            // Keep updated writer state
            contactWriter = collector.m_ContactWriter;
        }

        private unsafe struct CompositeCompositeLeafCollector : ILeafColliderCollector
        {
            // Inputs
            readonly Context m_Context;
            readonly Collider* m_CompositeColliderA;
            readonly Collider* m_CompositeColliderB;
            MTransform m_WorldFromA;
            readonly MTransform m_WorldFromB;
            readonly MotionExpansion m_Expansion;
            readonly bool m_Flipped;

            ColliderKeyPath m_KeyPath;

            // Output
            internal BlockStream.Writer m_ContactWriter;

            public CompositeCompositeLeafCollector(
                Context context,
                Collider* compositeColliderA, Collider* compositeColliderB,
                MTransform worldFromA, MTransform worldFromB, MotionExpansion expansion, bool flipped,
                BlockStream.Writer contactWriter)
            {
                m_Context = context;
                m_CompositeColliderA = compositeColliderA;
                m_CompositeColliderB = compositeColliderB;
                m_WorldFromA = worldFromA;
                m_WorldFromB = worldFromB;
                m_Expansion = expansion;
                m_Flipped = flipped;
                m_ContactWriter = contactWriter;
                m_KeyPath = ColliderKeyPath.Empty;
            }

            public void AddLeaf(ColliderKey key, ref ChildCollider leaf)
            {
                MTransform worldFromLeafA = Mul(m_WorldFromA, new MTransform(leaf.TransformFromChild));
                ConvexComposite(
                    m_Context, m_KeyPath.GetLeafKey(key), leaf.Collider, m_CompositeColliderB,
                    worldFromLeafA, m_WorldFromB, m_Expansion, m_Flipped, ref m_ContactWriter);
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey, MTransform parentFromComposite, out MTransform worldFromParent)
            {
                m_KeyPath.PushChildKey(compositeKey);
                worldFromParent = m_WorldFromA;
                m_WorldFromA = Math.Mul(worldFromParent, parentFromComposite);
            }

            public void PopCompositeCollider(uint numCompositeKeyBits, MTransform worldFromParent)
            {
                m_WorldFromA = worldFromParent;
                m_KeyPath.PopChildKey(numCompositeKeyBits);
            }
        }
    }
}
