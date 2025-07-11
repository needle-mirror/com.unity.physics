using System;
using System.Diagnostics;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.TestTools;

namespace Unity.Physics.Tests.Systems
{
    class BuildBroadphase_UnitTests
    {
        static void BuildPhysicsWorld(SystemHandle buildPhysicsWorld, World world)
        {
            buildPhysicsWorld.Update(world.Unmanaged);
            var jobHandle = world.Unmanaged.ResolveSystemStateRef(buildPhysicsWorld).Dependency;
            jobHandle.Complete();
            Assert.IsTrue(jobHandle.IsCompleted);
        }

        static NativeArray<Entity> CreateRigidBodies(World world, int numGroups, BlobAssetReference<Collider> collider,
            bool injectTemporalCoherenceData)
        {
            var entities = new NativeArray<Entity>(numGroups * 3, Allocator.Temp);
            // create distinct groups of three overlapping rigid bodies, one static and two dynamic.
            for (int i = 0; i < numGroups; ++i)
            {
                // create 3 entities with colliders, 1 static and two dynamic, all located at the same place,
                // causing the following overlaps:
                // - two static-dynamic overlaps
                // - one dynamic-dynamic overlap
                for (int j = 0; j < 3; ++j)
                {
                    var entity = j > 0
                        ? world.EntityManager.CreateEntity(typeof(PhysicsCollider), typeof(LocalTransform),
                        typeof(PhysicsWorldIndex), typeof(PhysicsVelocity))
                        : world.EntityManager.CreateEntity(typeof(PhysicsCollider), typeof(LocalTransform),
                        typeof(PhysicsWorldIndex));

                    if (injectTemporalCoherenceData)
                    {
                        world.EntityManager.AddComponentData(entity, PhysicsTemporalCoherenceInfo.Default);
                        world.EntityManager.AddComponentData(entity, new PhysicsTemporalCoherenceTag());
                    }

                    world.EntityManager.SetComponentData(entity, new PhysicsCollider {Value = collider});
                    // place all entities at location (i, 0, 0)
                    world.EntityManager.SetComponentData(entity,
                        new LocalTransform {Position = new float3(i, 0, 0), Scale = 1.0f});

                    entities[i * 3 + j] = entity;
                }
            }

            return entities;
        }

        static unsafe void ValidateTree(Broadphase.Tree tree, int numExpectedElements)
        {
            tree.BoundingVolumeHierarchy.CheckIntegrity(numExpectedElements, tree.BodyFilters.GetUnsafePtr());
        }

        static void ValidateBroadphase(Broadphase broadphase, int numGroups, bool incrementalStatic)
        {
            int numDynamicBodies = numGroups * 2;
            ValidateTree(broadphase.DynamicTree, numDynamicBodies);

            // +1 for the static body only if we are not in incremental mode.
            // The default static body should not be included in the tree but it is part of the fully built tree.
            int numStaticBodies = incrementalStatic ? numGroups : numGroups + 1;
            ValidateTree(broadphase.StaticTree, numStaticBodies);
        }

        [Test]
        public void BuildBroadphase_AndOverlap([Values] bool multiThreaded, [Values] bool incrementalDynamic, [Values] bool incrementalStatic)
        {
            using var world = new World("Test world");

            // Create the system
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 100;
            var incremental = incrementalDynamic || incrementalStatic;
            CreateRigidBodies(world, numGroups, sphereCollider, incremental);

            // trigger system update
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // obtain physics world
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var collisionWorld = worldData.PhysicsData.PhysicsWorld.CollisionWorld;
            // validate broadphase integrity
            ValidateBroadphase(collisionWorld.Broadphase, numGroups, incrementalStatic);

            // check CollisionWorld content
            var overlapList = new NativeList<int>(Allocator.Temp);
            var allBodiesOverlap = new OverlapAabbInput
            {
                Aabb = new Aabb
                {
                    Min = new float3(0, 0, 0),
                    Max = new float3(numGroups - 1, 0, 0)
                },
                Filter = CollisionFilter.Default
            };
            collisionWorld.OverlapAabb(allBodiesOverlap, ref overlapList);
            Assert.AreEqual(numGroups * 3, overlapList.Length);
            overlapList.Clear();

            int startIndex = 6;
            int endIndex = 42;
            var selectBodiesOverlap = new OverlapAabbInput
            {
                Aabb = new Aabb
                {
                    Min = new float3(startIndex, 0, 0),
                    Max = new float3(endIndex, 0, 0)
                },
                Filter = CollisionFilter.Default
            };
            int expectedOverlapCount = 3 * (endIndex - startIndex + 1);
            collisionWorld.OverlapAabb(selectBodiesOverlap, ref overlapList);
            Assert.AreEqual(expectedOverlapCount, overlapList.Length);
        }

        [Test]
        public void BuildBroadphase_ChangeCollisionFilter([Values] bool multiThreaded, [Values] bool incrementalDynamic, [Values] bool incrementalStatic)
        {
            using var world = new World("Test world");

            // Create the system
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 1;
            var incremental = incrementalDynamic || incrementalStatic;
            using var entities = CreateRigidBodies(world, numGroups, sphereCollider, incremental);

            // trigger system update
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // validate broadphase integrity
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var physicsWorld = worldData.PhysicsData.PhysicsWorld;
            ValidateBroadphase(physicsWorld.CollisionWorld.Broadphase, numGroups, incrementalStatic);

            // change collision filter and responds to collision flag and expect the change to be reflected in the new broadphase

            var newFilter = new CollisionFilter
            {
                BelongsTo = 2,
                CollidesWith = 1,
                GroupIndex = 0
            };
            Assert.AreNotEqual(CollisionFilter.Default, newFilter);

            using var colliderClones = new NativeList<BlobAssetReference<Collider>>(numGroups * 3, Allocator.Temp);
            for (int i = 0; i < numGroups * 3; ++i)
            {
                var entity = entities[i];
                var collider = world.EntityManager.GetComponentData<PhysicsCollider>(entity);

                var colliderBlobClone = collider.Value.Value.Clone();
                colliderClones.Add(colliderBlobClone);
                collider.Value = colliderBlobClone;

                // change collision filter
                Assert.AreEqual(CollisionFilter.Default, collider.Value.Value.GetCollisionFilter());
                collider.Value.Value.SetCollisionFilter(newFilter);

                // change responds to collision flag
                Assert.IsTrue(collider.Value.Value.RespondsToCollision);
                var material = collider.Value.Value.GetMaterial(ColliderKey.Empty);
                material.CollisionResponse = CollisionResponsePolicy.None;
                collider.Value.Value.SetMaterialField(material, ColliderKey.Empty, Material.MaterialField.CollisionResponsePolicy);

                world.EntityManager.SetComponentData(entity, collider);
            }

            // trigger system update following changes
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // validate broadphase integrity
            ValidateBroadphase(physicsWorld.CollisionWorld.Broadphase, numGroups, incrementalStatic);

            // Check BodyFilter and RespondsToCollision flags
            foreach (var entity in entities)
            {
                var index = physicsWorld.GetRigidBodyIndex(entity);
                bool dynamicBody = index < physicsWorld.NumDynamicBodies;
                var tree = dynamicBody ? physicsWorld.CollisionWorld.Broadphase.DynamicTree : physicsWorld.CollisionWorld.Broadphase.StaticTree;
                var adjustedIndex = dynamicBody ? index : index - physicsWorld.NumDynamicBodies;

                var bodyFilter = tree.BodyFilters[adjustedIndex];
                Assert.AreEqual(newFilter, bodyFilter);
                var respondsToCollision = tree.RespondsToCollision[adjustedIndex];
                Assert.IsFalse(respondsToCollision);
            }

            foreach (var collider in colliderClones)
            {
                collider.Dispose();
            }
        }

        [Test]
        public void BuildBroadphase_ChangeBodyControlType([Values] bool multiThreaded, [Values] bool incrementalDynamic, [Values] bool incrementalStatic)
        {
            using var world = new World("Test world");

            // Create the system
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            // create a bunch of rigid bodies, static and dynamic
            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 2;
            var incremental = incrementalDynamic || incrementalStatic;
            using var entities = CreateRigidBodies(world, numGroups, sphereCollider, incremental);

            // trigger system update
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // validate broadphase integrity
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            ValidateBroadphase(worldData.PhysicsData.PhysicsWorld.CollisionWorld.Broadphase, numGroups, incrementalStatic);

            // change body control type of the first rigid body entity, a static one, to dynamic
            // and expect the change to be reflected in the new broadphase
            var entity = entities[0];
            Assert.IsFalse(world.EntityManager.HasComponent<PhysicsVelocity>(entity));
            world.EntityManager.AddComponentData(entity, new PhysicsVelocity());

            // trigger system update following changes
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // obtain physics world
            worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var physicsWorldNew = worldData.PhysicsData.PhysicsWorld;
            // validate broadphase integrity
            var expectedNumDynamicBodies = numGroups * 2 + 1;
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.DynamicTree, expectedNumDynamicBodies);
            var expectedNumStaticBodies = incrementalStatic ? numGroups - 1 : numGroups - 1 + 1; // the default static body should be present only in the fully built tree
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.StaticTree, expectedNumStaticBodies);

            // change body control type of another rigid body entity, a dynamic one, to static
            // and expect the change to be reflected in the new broadphase
            entity = entities[1];
            Assert.IsTrue(world.EntityManager.HasComponent<PhysicsVelocity>(entity));
            world.EntityManager.RemoveComponent<PhysicsVelocity>(entity);

            // trigger system update following changes
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // check for expected number of static and dynamic bodies
            --expectedNumDynamicBodies;
            ++expectedNumStaticBodies;
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.DynamicTree, expectedNumDynamicBodies);
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.StaticTree, expectedNumStaticBodies);

            // change body control type of all bodies to static
            // and expect the change to be reflected in the new broadphase
            foreach (var e in entities)
            {
                if (world.EntityManager.HasComponent<PhysicsVelocity>(e))
                {
                    world.EntityManager.RemoveComponent<PhysicsVelocity>(e);
                }
            }
            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.DynamicTree, 0);
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.StaticTree, incrementalStatic ? numGroups * 3 : numGroups * 3 + 1);

            // change body control type of all bodies to dynamic
            // and expect the change to be reflected in the new broadphase
            foreach (var e in entities)
            {
                if (!world.EntityManager.HasComponent<PhysicsVelocity>(e))
                {
                    world.EntityManager.AddComponent<PhysicsVelocity>(e);
                }
            }
            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.DynamicTree, numGroups * 3);
            ValidateTree(physicsWorldNew.CollisionWorld.Broadphase.StaticTree, incrementalStatic ? 0 : 1);
        }

        [Test]
        public void BuildBroadphase_DeleteBodies([Values] bool multiThreaded, [Values] bool incrementalDynamic, [Values] bool incrementalStatic)
        {
            using var world = new World("Test world");

            // Create the system
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            // create a bunch of rigid bodies
            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 10;
            var incremental = incrementalDynamic || incrementalStatic;
            using var entities = CreateRigidBodies(world, numGroups, sphereCollider, incremental);

            // trigger system update
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // validate broadphase integrity
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var collisionWorld = worldData.PhysicsData.PhysicsWorld.CollisionWorld;
            ValidateBroadphase(collisionWorld.Broadphase, numGroups, incrementalStatic);

            int dynamicBodiesCount = numGroups * 2;
            int staticBodiesCount = incrementalStatic ? numGroups : numGroups + 1;

            int deletedStaticBodies = 1;
            int deletedDynamicBodies = 2;
            int deletedBodies = deletedStaticBodies + deletedDynamicBodies;

            // delete a few bodies and expect the change to be reflected in the new broadphase
            for (int i = 0; i < deletedBodies; ++i)
            {
                world.EntityManager.DestroyEntity(entities[i]);
            }

            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(collisionWorld.Broadphase.DynamicTree, dynamicBodiesCount - deletedDynamicBodies);
            ValidateTree(collisionWorld.Broadphase.StaticTree, staticBodiesCount - deletedStaticBodies);

            // delete all remaining bodies and expect the change to be reflected in the new broadphase
            for (int i = deletedBodies; i < numGroups * 3; ++i)
            {
                world.EntityManager.DestroyEntity(entities[i]);
            }

            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(collisionWorld.Broadphase.DynamicTree, 0);
            ValidateTree(collisionWorld.Broadphase.StaticTree, incrementalStatic ? 0 : 1);
        }

        [Test]
        public void BuildBroadphase_InvalidateBodies([Values] bool multiThreaded, [Values] bool incrementalDynamic, [Values] bool incrementalStatic)
        {
            using var world = new World("Test world");

            // Create the system
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            // create a bunch of rigid bodies
            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 10;
            var incremental = incrementalDynamic || incrementalStatic;
            using var entities = CreateRigidBodies(world, numGroups, sphereCollider, incremental);

            // trigger system update
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // validate broadphase integrity
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var collisionWorld = worldData.PhysicsData.PhysicsWorld.CollisionWorld;
            ValidateBroadphase(collisionWorld.Broadphase, numGroups, incrementalStatic);

            int dynamicBodiesCount = numGroups * 2;
            int staticBodiesCount = incrementalStatic ? numGroups : numGroups + 1;

            // Invalidate a few bodies and expect the change to be reflected in the new broadphase.
            // There are various ways to invalidate a body and we will test them all.
            // 1. Remove any PhysicsCollider and PhysicsVelocity component
            // 2. Remove any LocalToWorld and LocalTransform component
            // 3. Remove the PhysicsWorldIndex component
            // 4. Add Disabled tag

            int invalidatedStaticBodies = 0;
            int invalidatedDynamicBodies = 0;

            // remove PhysicsWorldIndex
            var staticBody = entities[invalidatedStaticBodies++ *3];
            var dynamicBody = entities[invalidatedDynamicBodies++ *3 + 1];
            world.EntityManager.RemoveComponent<PhysicsWorldIndex>(staticBody);
            world.EntityManager.RemoveComponent<PhysicsWorldIndex>(dynamicBody);

            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(collisionWorld.Broadphase.DynamicTree, dynamicBodiesCount - invalidatedDynamicBodies);
            ValidateTree(collisionWorld.Broadphase.StaticTree, staticBodiesCount - invalidatedStaticBodies);

            // remove LocalTransform
            staticBody = entities[invalidatedStaticBodies++ *3];
            dynamicBody = entities[invalidatedDynamicBodies++ *3 + 1];
            world.EntityManager.RemoveComponent<LocalTransform>(staticBody);
            world.EntityManager.RemoveComponent<LocalTransform>(dynamicBody);

            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(collisionWorld.Broadphase.DynamicTree, dynamicBodiesCount - invalidatedDynamicBodies);
            ValidateTree(collisionWorld.Broadphase.StaticTree, staticBodiesCount - invalidatedStaticBodies);

            // remove PhysicsCollider and PhysicsVelocity
            staticBody = entities[invalidatedStaticBodies++ *3];
            dynamicBody = entities[invalidatedDynamicBodies++ *3 + 1];
            world.EntityManager.RemoveComponent<PhysicsCollider>(staticBody);
            world.EntityManager.RemoveComponent<PhysicsCollider>(dynamicBody);
            world.EntityManager.RemoveComponent<PhysicsVelocity>(dynamicBody);

            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(collisionWorld.Broadphase.DynamicTree, dynamicBodiesCount - invalidatedDynamicBodies);
            ValidateTree(collisionWorld.Broadphase.StaticTree, staticBodiesCount - invalidatedStaticBodies);

            // add Disabled tag
            staticBody = entities[invalidatedStaticBodies++ *3];
            dynamicBody = entities[invalidatedDynamicBodies++ *3 + 1];
            world.EntityManager.AddComponent<Disabled>(staticBody);
            world.EntityManager.AddComponent<Disabled>(dynamicBody);

            // trigger system update and validate
            BuildPhysicsWorld(buildPhysicsWorld, world);
            ValidateTree(collisionWorld.Broadphase.DynamicTree, dynamicBodiesCount - invalidatedDynamicBodies);
            ValidateTree(collisionWorld.Broadphase.StaticTree, staticBodiesCount - invalidatedStaticBodies);
        }

        /// Test which functions like the tests above but which does not add temporal
        /// coherence data immediately to the created rigid bodies. Instead, it requires the data to be injected
        /// by either the InjectTemporalCoherenceDataSystem or the InjectTemporalCoherenceDataLastResortSystem.
        [Test]
        public void BuildBroadphase_InjectTemporalCoherenceData([Values] bool multiThreaded, [Values] bool incrementalDynamic,
            [Values] bool incrementalStatic, [Values] bool injectLastResort)
        {
            using var world = new World("Test world");

            // Create the system
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            // create a bunch of rigid bodies
            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 10;
            using var entities =
                    CreateRigidBodies(world, numGroups, sphereCollider, injectTemporalCoherenceData: false);

            // create and add the InjectTemporalCoherenceDataSystem
            SystemHandle injectTemporalCoherenceDataSystem;
            SystemBase ecbSystem = null;
            if (injectLastResort)
            {
                injectTemporalCoherenceDataSystem = world.GetOrCreateSystem<InjectTemporalCoherenceDataLastResortSystem>();
            }
            else
            {
                injectTemporalCoherenceDataSystem = world.GetOrCreateSystem<InjectTemporalCoherenceDataSystem>();
                // injection system uses the following ecb system to inject the data
                ecbSystem = world.GetOrCreateSystemManaged<BeginFixedStepSimulationEntityCommandBufferSystem>();
            }

            injectTemporalCoherenceDataSystem.Update(world.Unmanaged);
            var jobHandle = world.Unmanaged.ResolveSystemStateRef(injectTemporalCoherenceDataSystem).Dependency;
            jobHandle.Complete();
            Assert.IsTrue(jobHandle.IsCompleted);

            // need to also ensure the ecb system is played back for the injection to take effect
            if (!injectLastResort)
            {
                ecbSystem.Update();
                var ecbSystemJobHandle = world.Unmanaged.ResolveSystemStateRef(ecbSystem.SystemHandle).Dependency;
                ecbSystemJobHandle.Complete();
            }

            // trigger system update
            BuildPhysicsWorld(buildPhysicsWorld, world);

            // validate broadphase integrity
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var collisionWorld = worldData.PhysicsData.PhysicsWorld.CollisionWorld;
            ValidateBroadphase(collisionWorld.Broadphase, numGroups, incrementalStatic);
        }

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
        [Test]
        public void BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification_ResetToDefault([Values] bool multiThreaded,
            [Values] bool incrementalDynamic, [Values] bool incrementalStatic) =>
            BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification(multiThreaded, incrementalDynamic, incrementalStatic, (staticBody, dynamicBody, world) =>
            {
                // Invalidate the temporal coherence of both a static and a dynamic body by overwriting the PhysicsTemporalCoherenceInfo
                // with a new default component.
                world.EntityManager.SetComponentData(staticBody, PhysicsTemporalCoherenceInfo.Default);
                world.EntityManager.SetComponentData(dynamicBody, PhysicsTemporalCoherenceInfo.Default);

                var numErrors = 0;
                numErrors += math.select(0, 1, incrementalDynamic);
                numErrors += math.select(0, 1, incrementalStatic);
                return numErrors;
            }
            );

        [Test]
        [Conditional(CompilationSymbols.CollectionsChecksSymbol), Conditional(CompilationSymbols.DebugChecksSymbol)] // only run if SafetyChecks are available / enabled
        public void BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification_Remove([Values] bool multiThreaded,
            [Values] bool incrementalDynamic, [Values] bool incrementalStatic) =>
            BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification(multiThreaded, incrementalDynamic, incrementalStatic, (staticBody, dynamicBody, world) =>
            {
                // Invalidate the temporal coherence of both a static and a dynamic body by removing the PhysicsTemporalCoherenceInfo component.
                world.EntityManager.RemoveComponent<PhysicsTemporalCoherenceInfo>(staticBody);
                world.EntityManager.RemoveComponent<PhysicsTemporalCoherenceInfo>(dynamicBody);

                if (incrementalDynamic && incrementalStatic)
                {
                    return 4;
                }
                // else:

                if (incrementalDynamic || incrementalStatic)
                {
                    return 3;
                }
                // else:

                return 0;
            }
            );

        [Test]
        [Conditional(CompilationSymbols.CollectionsChecksSymbol), Conditional(CompilationSymbols.DebugChecksSymbol)] // only run if SafetyChecks are available / enabled
        public void BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification_ModifyValues([Values] bool multiThreaded,
            [Values] bool incrementalDynamic, [Values] bool incrementalStatic) =>
            BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification(multiThreaded, incrementalDynamic, incrementalStatic, (staticBody, dynamicBody, world) =>
            {
                // Invalidate the temporal coherence of both a static and a dynamic body by overwriting the PhysicsTemporalCoherenceInfo component
                // with invalid values.
                var invalidTemporalCoherenceInfo = new PhysicsTemporalCoherenceInfo
                {
                    LastRigidBodyIndex = 15,
                    LastBvhNodeIndex = -1,
                    LastBvhLeafSlotIndex = 42
                };

                world.EntityManager.SetComponentData(staticBody, invalidTemporalCoherenceInfo);
                world.EntityManager.SetComponentData(dynamicBody, invalidTemporalCoherenceInfo);

                var numErrors = 0;
                numErrors += math.select(0, 1, incrementalDynamic);
                numErrors += math.select(0, 1, incrementalStatic);
                return numErrors;
            }
            );

        /// Tests that the incremental broadphase integrity check (see ExportPhysicsWorld.cs) correctly detects an illegal
        /// modification of the temporal coherence data.
        [Conditional(CompilationSymbols.CollectionsChecksSymbol), Conditional(CompilationSymbols.DebugChecksSymbol)] // only run if SafetyChecks are available / enabled
        static void BuildBroadphase_TestIntegrityCheck_IllegalTemporalCoherenceDataModification(bool multiThreaded,
            bool incrementalDynamic, bool incrementalStatic, Func<Entity, Entity, World, int> invalidTemporalCoherenceDataModification)
        {
            using var world = new World("Test world");

            // Create the systems
            var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();
            var exportPhysicsWorld = world.GetOrCreateSystem<ExportPhysicsWorld>();

            var physicsStep = PhysicsStep.Default;
            physicsStep.MultiThreaded = (byte)(multiThreaded ? 1 : 0);
            physicsStep.IncrementalDynamicBroadphase = incrementalDynamic;
            physicsStep.IncrementalStaticBroadphase = incrementalStatic;

            world.EntityManager.CreateSingleton(physicsStep);

            // create some rigid bodies
            using var sphereCollider = SphereCollider.Create(new SphereGeometry {Radius = 0.25f});
            int numGroups = 10;
            using var entities = CreateRigidBodies(world, numGroups, sphereCollider, injectTemporalCoherenceData: true);

            Action updateSystems = () =>
            {
                BuildPhysicsWorld(buildPhysicsWorld, world);

                exportPhysicsWorld.Update(world.Unmanaged);
                var jobHandle = world.Unmanaged.ResolveSystemStateRef(exportPhysicsWorld).Dependency;
                jobHandle.Complete();
                Assert.IsTrue(jobHandle.IsCompleted);
            };

            // trigger system update and expect the integrity checks to pass (no exception thrown)
            updateSystems();

            // Start by validating broadphase integrity
            var worldData = world.EntityManager.GetComponentData<BuildPhysicsWorldData>(buildPhysicsWorld);
            var collisionWorld = worldData.PhysicsData.PhysicsWorld.CollisionWorld;
            ValidateBroadphase(collisionWorld.Broadphase, numGroups, incrementalStatic);

            // invalidate temporal coherence data for some random bodies and expect this illegal change to be detected
            // by the incremental broadphase integrity check.
            int staticBodyGroup = 3;
            int dynamicBodyGroup = 8;
            var staticBody = entities[3 * staticBodyGroup];
            var dynamicBody = entities[3 * dynamicBodyGroup + 1];
            var numExpectedErrors = invalidTemporalCoherenceDataModification(staticBody, dynamicBody, world);

            // Trigger system update and expect the integrity check to throw an exception if incremental
            // broadphase is enabled:

            updateSystems();

            var errorMessage = ".*";
            for (int i = 0; i < numExpectedErrors; ++i)
            {
                LogAssert.Expect(LogType.Exception, new Regex(errorMessage));
            }
        }

#endif
    }
}
