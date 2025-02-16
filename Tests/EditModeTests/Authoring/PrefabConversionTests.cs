using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class PrefabConversionTests : PrefabConversionTestsBase
    {

        #if false
        //Temporarily disabling this test due to failures exclusive to Packageworks
        [Test]
        public void PrefabConversion_ChildCollider_ForceUnique([Values] bool forceUniqueCollider)
        {
            var rigidBody = new GameObject("Body", typeof(Rigidbody), typeof(UnityEngine.BoxCollider));

            ValidateChildCollidersInBakedEntityPrefabAreUnique(rigidBody, forceUniqueCollider,
                (gameObject, mass) => { gameObject.GetComponent<Rigidbody>().mass = mass; },
                _ =>
                {
                    if (forceUniqueCollider) rigidBody.AddComponent<ForceUniqueColliderAuthoring>();
                });
        }
        #endif
        
        [Test]
        public void PrefabConversion_GameObjectPrefabInstances_ForceUnique([Values] bool forceUniqueCollider)
        {
            var rigidBody = new GameObject("Body", typeof(Rigidbody), typeof(UnityEngine.BoxCollider));

            ValidateCollidersBakedFromGameObjectPrefabInstancesAreUnique(rigidBody, forceUniqueCollider,
                _ =>
                {
                    if (forceUniqueCollider) rigidBody.AddComponent<ForceUniqueColliderAuthoring>();
                });
        }

        [Test]
        public void PrefabConversion_StableForceUniqueIDInPrefabs()
        {
            var rigidBody = new GameObject("Body", typeof(Rigidbody), typeof(UnityEngine.BoxCollider), typeof(ForceUniqueColliderAuthoring));

            uint GetForceUniqueID(GameObject gameObject)
            {
                var forceUniqueColliderAuthoring = gameObject.GetComponent<ForceUniqueColliderAuthoring>();
                return forceUniqueColliderAuthoring.ForceUniqueID;
            }

            ValidateStableForceUniqueIDForCollidersInPrefabs(rigidBody, GetForceUniqueID);
        }
    }

    class ConversionTestPrefabReference : MonoBehaviour
    {
        public GameObject Prefab;
    }

    struct ConversionTestPrefabComponent : IComponentData
    {
        public Entity Prefab;
    }
    class ConversionTestPrefabReferenceBaker : Baker<ConversionTestPrefabReference>
    {
        public override void Bake(ConversionTestPrefabReference authoring)
        {
            var entity = GetEntity(authoring, TransformUsageFlags.None);
            AddComponent(entity, new ConversionTestPrefabComponent { Prefab = GetEntity(authoring.Prefab, TransformUsageFlags.Dynamic) });
        }
    }

    internal class PrefabConversionTestsBase : BaseHierarchyConversionTest
    {
        string TempPrefabAssetPath => "Assets/Temp.prefab";

        GameObject CreatePrefab(GameObject gameObject)
        {
            return PrefabUtility.SaveAsPrefabAsset(gameObject, TempPrefabAssetPath);
        }

        [TearDown]
        public override void TearDown()
        {
            AssetDatabase.DeleteAsset(TempPrefabAssetPath);

            base.TearDown();
        }

        /// <summary>
        ///     Ensures that a prefab containing two copies of the provided dynamic rigid body with collider game object
        ///     are baked as expected.
        ///
        /// </summary>
        /// <param name="dynamicRigidBodyWithCollider">The game object containing the dynamic rigid body with collider.</param>
        /// <param name="forceUniqueCollider">A flag indicating whether the collider on the provided game object should be forced to be unique.</param>
        /// <param name="setMassFunction">A function required for the validation which sets the mass on the dynamic rigid body to the specified value.</param>
        /// <param name="forceUniqueFunction">A function which ensures that the provided game object's collider is forced to be unique.</param>
        protected void ValidateChildCollidersInBakedEntityPrefabAreUnique(GameObject dynamicRigidBodyWithCollider, bool forceUniqueCollider, Action<GameObject, float> setMassFunction, Action<GameObject> forceUniqueFunction)
        {
            var child1 = dynamicRigidBodyWithCollider;
            var child2 = UnityEngine.Object.Instantiate(child1);

            // create a prefab with a few colliders that use the same collider, to induce collider sharing
            var prefabRoot = new GameObject("Root");
            child1.transform.parent = prefabRoot.transform;
            child2.transform.parent = prefabRoot.transform;

            // move the second child away a little to prevent any sort of simulation issues, since we will be updating the world
            child2.transform.localPosition = new Vector3(0, 10, 0);

            // use mass to uniquely identify the two rigid bodies
            var child1Mass = 1.0f;
            var child2Mass = 2.0f;
            setMassFunction(child1, child1Mass);
            setMassFunction(child2, child2Mass);

            // Force child 1 to be unique if requested (see forceUniqueCollider flag).
            // Child 2 is never unique, i.e., shared, as a baseline.
            if (forceUniqueCollider)
            {
                forceUniqueFunction(child1);
            }

            // create a prefab from the prefab root game object
            var prefab = CreatePrefab(prefabRoot);

            // create a game object that references the prefab, in order to trigger the prefab baking
            var prefabRef = new GameObject("prefab_ref", typeof(ConversionTestPrefabReference));
            var prefabRefComponent = prefabRef.GetComponent<ConversionTestPrefabReference>();
            Assert.That(prefabRefComponent, Is.Not.Null);
            prefabRefComponent.Prefab = prefab;

            // Note: make sure there is no leftover world from other tests, since this test relies on proper cleanup behavior
            // and is thus sensitive to leftovers from previous tests.
            World.DisposeAllWorlds();

            var world = DefaultWorldInitialization.Initialize("Test world");
            try
            {
                using var blobAssetStore = new BlobAssetStore(128);
                ConvertBakeGameObject(prefabRef, world, blobAssetStore);

                using var prefabRootQueryBuilder = new EntityQueryBuilder(Allocator.Temp)
                        .WithOptions(EntityQueryOptions.IncludePrefab)
                        .WithAll<Prefab>()
                        .WithAll<LinkedEntityGroup>();

                using var prefabInstanceRootQueryBuilder = new EntityQueryBuilder(Allocator.Temp)
                        .WithAll<LinkedEntityGroup>();

                using var colliderQueryBuilder = new EntityQueryBuilder(Allocator.Temp)
                        .WithAll<PhysicsCollider>();

                // at this point we expect exactly one prefab roots and no prefab instances or colliders
                using (var prefabRoots = world.EntityManager.CreateEntityQuery(prefabRootQueryBuilder))
                {
                    Assert.That(prefabRoots.CalculateEntityCount(), Is.EqualTo(1));
                }

                using (var prefabInstanceRoots = world.EntityManager.CreateEntityQuery(prefabInstanceRootQueryBuilder))
                {
                    Assert.That(prefabInstanceRoots.CalculateEntityCount(), Is.EqualTo(0));
                }

                using var colliders = world.EntityManager.CreateEntityQuery(colliderQueryBuilder);
                Assert.That(colliders.CalculateEntityCount(), Is.EqualTo(0));

                // instantiate a few instances of the prefab
                using var prefabRefComponentsQuery =
                        world.EntityManager.CreateEntityQuery(typeof(ConversionTestPrefabComponent));
                using var prefabRefComponents =
                        prefabRefComponentsQuery.ToComponentDataArray<ConversionTestPrefabComponent>(Allocator.Temp);
                Assert.That(prefabRefComponents.Length, Is.EqualTo(1));

                var entityPrefab = prefabRefComponents[0].Prefab;
                const int kInstanceCount = 10;
                using var prefabInstances = new NativeArray<Entity>(kInstanceCount, Allocator.Temp);
                world.EntityManager.Instantiate(entityPrefab, prefabInstances);

                // for each entity prefab instance make sure that the contained colliders have the correct collider state
                foreach (var rootEntity in prefabInstances)
                {
                    int prefabColliderCount = 0;

                    // regardless of the forceUniqueCollider flag, we expect the colliders to be shared initially after
                    // prefab instantiation
                    Assert.That(world.EntityManager.HasComponent<LinkedEntityGroup>(rootEntity), Is.True);
                    var linkedEntityGroup = world.EntityManager.GetBuffer<LinkedEntityGroup>(rootEntity);
                    foreach (var element in linkedEntityGroup)
                    {
                        var entity = element.Value;

                        // Note: the LinkedEntityGroup buffer contains all entities in the entity prefab instance, including the root.
                        // We need to filter it out.
                        if (entity != rootEntity)
                        {
                            Assert.That(world.EntityManager.HasComponent<PhysicsCollider>(entity));
                            var collider = world.EntityManager.GetComponentData<PhysicsCollider>(entity);
                            ++prefabColliderCount;
                            Assert.That(collider.IsUnique, Is.False);
                        }
                    }

                    Assert.That(prefabColliderCount, Is.EqualTo(2));
                }

                // update world and expect the colliders to now be unique if requested
                world.Update();

                // validate world content:

                using (var prefabInstanceRoots = world.EntityManager.CreateEntityQuery(prefabInstanceRootQueryBuilder))
                {
                    Assert.That(prefabInstanceRoots.CalculateEntityCount(), Is.EqualTo(kInstanceCount));

                    using var rootEntities = prefabInstanceRoots.ToEntityArray(Allocator.Temp);
                    // ensure the colliders have the correct IsUnique state and share their colliders only if requested.
                    foreach (var rootEntity in rootEntities)
                    {
                        BlobAssetReference<Collider> child1Collider = default, child2Collider = default;

                        // Depending on the forceUniqueCollider flag, we expect the collider in child1 to now be unique.
                        // Child2 is still going to be indicated as not unique.
                        Assert.That(world.EntityManager.HasComponent<LinkedEntityGroup>(rootEntity), Is.True);
                        var linkedEntityGroup = world.EntityManager.GetBuffer<LinkedEntityGroup>(rootEntity);
                        foreach (var element in linkedEntityGroup)
                        {
                            var entity = element.Value;
                            if (entity != rootEntity)
                            {
                                Assert.That(world.EntityManager.HasComponent<PhysicsCollider>(entity));
                                var collider = world.EntityManager.GetComponentData<PhysicsCollider>(element.Value);
                                var mass = world.EntityManager.GetComponentData<PhysicsMass>(element.Value);
                                bool isChild1 = math.abs(mass.InverseMass - 1 / child1Mass) < 1e-4;

                                if (!isChild1)
                                {
                                    // ensure that the mass is as expected
                                    Assert.That(math.abs(mass.InverseMass - 1 / child2Mass) < 1e-4, Is.True);
                                    child2Collider = collider.Value;
                                }
                                else
                                {
                                    child1Collider = collider.Value;
                                }

                                bool expectUnique = isChild1 && forceUniqueCollider;
                                Assert.That(collider.IsUnique, Is.EqualTo(expectUnique));
                            }
                        }

                        Assert.That(child1Collider.IsCreated && child2Collider.IsCreated, Is.True);
                        // ensure that the colliders are not the same if they should not be shared (force unique requested on child 1)
                        unsafe
                        {
                            bool collidersShared = child1Collider.GetUnsafePtr() == child2Collider.GetUnsafePtr();
                            Assert.That(collidersShared, Is.Not.EqualTo(forceUniqueCollider));
                        }
                    }
                }

                world.Update();

                // If we are forcing the colliders to be unique, we are expecting some entities with the ColliderBlobCleanupData component.
                // This component is required as part of the PhysicsCollider.MakeUnique function. Otherwise we don't expect any such components.
                using (var colliderBlobCleanupQuery = world.EntityManager.CreateEntityQuery(new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<ColliderBlobCleanupData>()))
                {
                    Assert.That(colliderBlobCleanupQuery.IsEmpty, Is.Not.EqualTo(forceUniqueCollider));
                }

                // destroy instantiated entities and prefab and expect them to be cleaned up properly
                using (var allEntitiesQuery = world.EntityManager.CreateEntityQuery(new EntityQueryBuilder(Allocator.Temp)
                    .WithAny<LinkedEntityGroup, PhysicsCollider>()
                    .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)))
                {
                    Assert.DoesNotThrow(() => world.EntityManager.DestroyEntity(allEntitiesQuery));
                }

                // Step the world a few times to allow for the collider blob cleanup systems to run and for the entities that previously contained
                // the ColliderBlobCleanupData to be fully destroyed. This will occur once the ColliderBlobCleanupSystem has completed
                // its run and issued the removal of the ColliderBlobCleanupData components, and in addition once any other unrelated
                // ICleanupComponentData components that may have been on the entities have also been removed.
                for (int i = 0; i < 10; ++i)
                {
                    world.Update();
                }

                // make sure there are no more colliders left
                using (var colliderQuery = world.EntityManager.CreateEntityQuery(new EntityQueryBuilder(Allocator.Temp)
                           .WithAny<PhysicsCollider>()
                           .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)))
                {
                    Assert.That(colliderQuery.IsEmpty, Is.True);
                }

                // After we have destroyed the entities we created and we have updated the world, we don't expect there to be
                // any ColliderBlobCleanupData components present. If there are still some, however, the collider should have
                // been disposed, or the PhysicsCollider component is still present.
                using (var colliderBlobCleanupQuery = world.EntityManager.CreateEntityQuery(
                    new EntityQueryBuilder(Allocator.Temp)
                        .WithAll<ColliderBlobCleanupData>()
                        .WithNone<PhysicsCollider>()))
                {
                    if (!colliderBlobCleanupQuery.IsEmpty)
                    {
                        var entities = colliderBlobCleanupQuery.ToEntityArray(Allocator.Temp);
                        var cleanupComponents = colliderBlobCleanupQuery.ToComponentDataArray<ColliderBlobCleanupData>(Allocator.Temp);
                        for (int i = 0; i < entities.Length; ++i)
                        {
                            var cleanupComponent = cleanupComponents[i];
                            Assert.That(cleanupComponent.Value.IsCreated, Is.False);
                        }
                    }
                }
            }
            finally
            {
                World.DisposeAllWorlds();
            }
        }

        /// <summary>
        /// Ensures that when a prefab containing a rigid body with a collider is instantiated multiple times in a scene as
        /// game object prefab instances, the resultant colliders are baked as unique or shared depending on whether
        /// the collider authoring is forced to be unique.
        /// </summary>
        /// <param name="rigidBodyWithCollider">The game object containing the rigid body with collider.</param>
        /// <param name="forceUniqueCollider">A flag indicating whether the collider on the provided game object should be forced to be unique.</param>
        /// <param name="forceUniqueFunction">A function which ensures that the provided game object's collider is forced to be unique.</param>
        protected void ValidateCollidersBakedFromGameObjectPrefabInstancesAreUnique(GameObject rigidBodyWithCollider, bool forceUniqueCollider, Action<GameObject> forceUniqueFunction)
        {
#if UNITY_6000_0_50F1_OR_NEWER
            Assert.Ignore("With 6.0 and higher, this test does not correctly reproduce the Unity Editor behavior of instantiating prefab instances and thus fails.");
#endif

            // create a prefab with a single rigid body with collider
            var prefabRoot = new GameObject("PrefabRoot");
            rigidBodyWithCollider.transform.parent = prefabRoot.transform;

            // force the collider to be unique if requested
            if (forceUniqueCollider)
            {
                forceUniqueFunction(rigidBodyWithCollider);
            }

            // create a prefab from the prefab root game object.
            var prefab = CreatePrefab(prefabRoot);

            // create a root scene game object
            var sceneRoot = new GameObject("SceneRoot");

            // now create a few prefab instances, and, once baked, expect the colliders to be unique or shared depending
            // on the forceUniqueCollider flag

            var instances = new List<GameObject>();
            instances.Add(PrefabUtility.InstantiatePrefab(prefab) as GameObject);
            instances.Add(PrefabUtility.InstantiatePrefab(prefab) as GameObject);

            // to mimic copy-paste, duplicate the prefab instance game objects
#if UNITY_6000_0_50F1_OR_NEWER
            instances.Add(GameObjectUtility.DuplicateGameObject(instances[0]));
            instances.Add(GameObjectUtility.DuplicateGameObject(instances[1]));
#endif

            foreach (var instance in instances)
            {
                Assert.That(instance, Is.Not.Null);
            }

            // attach instances to scene root
            foreach (var instance in instances)
            {
                instance.transform.parent = sceneRoot.transform;
            }

            // now bake the scene root containing the two prefab instances and expect the colliders to be unique or shared depending
            // on the forceUniqueCollider flag
            using var world = DefaultWorldInitialization.Initialize("Test world");
            using var blobAssetStore = new BlobAssetStore(128);
            ConvertBakeGameObject(sceneRoot, world, blobAssetStore);

            using var colliderQueryBuilder = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<PhysicsCollider>();

            // expect two colliders to have been baked
            using var colliderQuery = world.EntityManager.CreateEntityQuery(colliderQueryBuilder);
            Assert.That(colliderQuery.CalculateEntityCount(), Is.EqualTo(instances.Count));

            var colliders = colliderQuery.ToComponentDataArray<PhysicsCollider>(Allocator.Temp);
            foreach (var collider in colliders)
            {
                Assert.That(collider.IsUnique, Is.EqualTo(forceUniqueCollider));
            }

            // Make sure that, unless they are forced to be unique, all colliders are shared, i.e., they point to the
            // same blob asset. Otherwise, they should all be unique.
            var sharedColliderBlobs = !forceUniqueCollider;
            for (int i = 0; i < colliders.Length; ++i)
            {
                for (int j = i + 1; j < colliders.Length; ++j)
                {
                    var blobAsset1 = colliders[i].Value;
                    var blobAsset2 = colliders[j].Value;

                    Assert.That(blobAsset1 == blobAsset2, Is.EqualTo(sharedColliderBlobs));
                }
            }
        }

        /// <summary>
        /// Ensures that the ForceUniqueID assigned to a collider in a prefab remains stable across prefab loads.
        /// </summary>
        /// <param name="rigidBodyWithUniqueCollider">A game object containing a rigid body with unique collider.</param>
        /// <param name="getForceUniqueIDFunction">Function to obtain the force unique ID from the collider authoring in a game object.</param>
        protected void ValidateStableForceUniqueIDForCollidersInPrefabs(GameObject rigidBodyWithUniqueCollider, Func<GameObject, uint> getForceUniqueIDFunction)
        {
            // create a prefab with a single rigid body with collider
            var prefabRoot = new GameObject("PrefabRoot");
            rigidBodyWithUniqueCollider.transform.parent = prefabRoot.transform;

            // create a prefab from the prefab root game object
            CreatePrefab(prefabRoot);

            // load the prefab into an isolated scene
            var prefab = PrefabUtility.LoadPrefabContents(TempPrefabAssetPath);

            // get the rigid body game object from the loaded prefab
            var rigidBodyInPrefab = prefab.gameObject.transform.GetChild(0).gameObject;

            // get the force unique ID from the collider in the created prefab
            var forceUniqueID = getForceUniqueIDFunction(rigidBodyInPrefab);

            // close the prefab asset
            PrefabUtility.UnloadPrefabContents(prefab);

            // load the prefab again
            var reloadedPrefab = PrefabUtility.LoadPrefabContents(TempPrefabAssetPath);

            // get the force unique ID of the collider in the reloaded prefab
            var rigidBodyInReloadedPrefab = reloadedPrefab.gameObject.transform.GetChild(0).gameObject;

            // get the force unique ID from the collider in the reloaded prefab
            var forceUniqueIDReloaded = getForceUniqueIDFunction(rigidBodyInReloadedPrefab);

            // make sure the force unique IDs are the same
            Assert.That(forceUniqueID, Is.EqualTo(forceUniqueIDReloaded));
        }
    }
}
