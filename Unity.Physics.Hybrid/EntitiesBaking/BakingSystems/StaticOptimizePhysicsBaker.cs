using System;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using LegacyPhysics = UnityEngine.Physics;
using LegacyCollider = UnityEngine.Collider;
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;
using UnityMesh = UnityEngine.Mesh;

namespace Unity.Physics.Authoring
{
    [TemporaryBakingType]
    struct StaticOptimizePhysicsBaking : IComponentData { }

    class StaticOptimizePhysicsBaker : BasePhysicsBaker<StaticOptimizeEntity>
    {
        private static List<PhysicsShapeAuthoring> physicsShapeAuthoringList = new List<PhysicsShapeAuthoring>();
        private static List<PhysicsBodyAuthoring> physicsBodyAuthoringList = new List<PhysicsBodyAuthoring>();
#if LEGACY_PHYSICS
        private static List<Rigidbody> rigidBodyList = new List<Rigidbody>();
        private static List<LegacyCollider> legacyColliderList = new List<LegacyCollider>();
#endif

        private bool HasPhysicsBodyInParent()
        {
            bool found = GetComponentInParent<PhysicsBodyAuthoring>();
#if LEGACY_PHYSICS
            found = found || GetComponentInParent<Rigidbody>();
#endif
            return found;
        }

        private void DependencyOnPhysicsBodyInChildren()
        {
            GetComponentsInChildren<PhysicsBodyAuthoring>(physicsBodyAuthoringList);
#if LEGACY_PHYSICS
            GetComponentsInChildren<Rigidbody>(rigidBodyList);
#endif
        }

        private bool HasStaticOptimizeInAncestors(GameObject authoringObj)
        {
            var transform = GetComponent<Transform>();
            if (transform.parent != null)
            {
                return GetComponentInParent<StaticOptimizeEntity>(transform.parent);
            }
            return false;
        }

        private bool IsPotentialColliderRoot(GameObject authoringObj)
        {
            // We only consider potential roots the top StaticOptimizeEntity in the hierarchy
            if (HasStaticOptimizeInAncestors(authoringObj))
                return false;

            // There can't be any physics body up the hierarchy or in the object itself to be a potential root
            if (HasPhysicsBodyInParent())
                return false;

            // There must be at least one PhysicsShape or LegacyCollider in the hierarchy below,
            // but it can't be on the object itself
            GetComponentsInChildren<PhysicsShapeAuthoring>(physicsShapeAuthoringList);
            if (physicsShapeAuthoringList != null && physicsShapeAuthoringList.Count > 0)
            {
                return (physicsShapeAuthoringList[0].gameObject != authoringObj);
            }

#if LEGACY_PHYSICS
            GetComponentsInChildren<LegacyCollider>(legacyColliderList);
            if (legacyColliderList != null && legacyColliderList.Count > 0)
            {
                return (legacyColliderList[0].gameObject != authoringObj);
            }
#endif

            return false;
        }

        public override void Bake(StaticOptimizeEntity authoring)
        {
            if (IsPotentialColliderRoot(authoring.gameObject))
            {
                AddComponent<StaticOptimizePhysicsBaking>();

                AddSharedComponent(new PhysicsWorldIndex());

                AddComponent(new PhysicsCompoundData()
                {
                    AssociateBlobToBody = false,
                    ConvertedBodyInstanceID = authoring.GetInstanceID(),
                    Hash = default,
                });

                AddComponent<PhysicsCollider>();
                AddBuffer<PhysicsColliderKeyEntityPair>();

                DependencyOnPhysicsBodyInChildren();

                Transform bodyTransform = GetComponent<Transform>();
                PostProcessTransform(bodyTransform, BodyMotionType.Static);
            }
        }
    }
}
