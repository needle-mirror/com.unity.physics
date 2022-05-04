using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    // A system which copies transforms and velocities from the physics world back to the original entity components.
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class ExportPhysicsWorld : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;

        private PhysicsWorldExporter.SharedData m_SharedData;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();

            m_SharedData = PhysicsWorldExporter.SharedData.Create();
        }

        protected override void OnDestroy()
        {
            m_SharedData.Dispose();
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadOnly();
        }

        protected override void OnUpdate()
        {
            JobHandle handle = Dependency;

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            handle = CheckIntegrity(handle, m_BuildPhysicsWorldSystem.IntegrityCheckMap);
#endif

            handle = PhysicsWorldExporter.SchedulePhysicsWorldExport(this, in m_BuildPhysicsWorldSystem.PhysicsWorld, handle, m_BuildPhysicsWorldSystem.PhysicsData.DynamicEntityGroup);

            handle = PhysicsWorldExporter.ScheduleCollisionWorldCopy(this, ref m_SharedData, in m_BuildPhysicsWorldSystem.PhysicsWorld,
                handle, m_BuildPhysicsWorldSystem.CollisionWorldProxyGroup);

            // Combine implicit output dependency with user one
            Dependency = JobHandle.CombineDependencies(Dependency, handle);
        }

        [Obsolete("AddInputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public void AddInputDependency(JobHandle inputDep) {}

        [Obsolete("GetOutputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public JobHandle GetOutputDependency() => default;

        #region Integrity checks

        internal JobHandle CheckIntegrity(JobHandle inputDeps, NativeParallelHashMap<uint, long> integrityCheckMap)
        {
            var positionType = GetComponentTypeHandle<Translation>(true);
            var rotationType = GetComponentTypeHandle<Rotation>(true);
            var physicsColliderType = GetComponentTypeHandle<PhysicsCollider>(true);
            var physicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true);

            var checkDynamicBodyIntegrity = new CheckDynamicBodyIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PositionType = positionType,
                RotationType = rotationType,
                PhysicsVelocityType = physicsVelocityType,
                PhysicsColliderType = physicsColliderType
            };

            inputDeps = checkDynamicBodyIntegrity.Schedule(m_BuildPhysicsWorldSystem.PhysicsData.DynamicEntityGroup, inputDeps);

            var checkStaticBodyColliderIntegrity = new CheckColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = physicsColliderType
            };

            inputDeps = checkStaticBodyColliderIntegrity.Schedule(m_BuildPhysicsWorldSystem.PhysicsData.StaticEntityGroup, inputDeps);

            var checkTotalIntegrity = new CheckTotalIntegrity
            {
                IntegrityCheckMap = integrityCheckMap
            };

            return checkTotalIntegrity.Schedule(inputDeps);
        }

        [BurstCompile]
        internal struct CheckDynamicBodyIntegrity : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeParallelHashMap<uint, long> IntegrityCheckMap;

            internal static void DecrementIfExists(NativeParallelHashMap<uint, long> integrityCheckMap, uint systemVersion)
            {
                if (integrityCheckMap.TryGetValue(systemVersion, out long occurences))
                {
                    integrityCheckMap.Remove(systemVersion);
                    integrityCheckMap.Add(systemVersion, occurences - 1);
                }
            }

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                DecrementIfExists(IntegrityCheckMap, batchInChunk.GetOrderVersion());
                DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsVelocityType));
                if (batchInChunk.Has(PositionType))
                {
                    DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PositionType));
                }
                if (batchInChunk.Has(RotationType))
                {
                    DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(RotationType));
                }
                if (batchInChunk.Has(PhysicsColliderType))
                {
                    DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));

                    var colliders = batchInChunk.GetNativeArray(PhysicsColliderType);
                    CheckColliderFilterIntegrity(colliders);
                }
            }
        }

        [BurstCompile]
        internal struct CheckColliderIntegrity : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeParallelHashMap<uint, long> IntegrityCheckMap;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                if (batchInChunk.Has(PhysicsColliderType))
                {
                    CheckDynamicBodyIntegrity.DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));

                    var colliders = batchInChunk.GetNativeArray(PhysicsColliderType);
                    CheckColliderFilterIntegrity(colliders);
                }
            }
        }

        [BurstCompile]
        internal struct CheckTotalIntegrity : IJob
        {
            public NativeParallelHashMap<uint, long> IntegrityCheckMap;
            public void Execute()
            {
                var values = IntegrityCheckMap.GetValueArray(Allocator.Temp);
                var validIntegrity = true;
                for (int i = 0; i < values.Length; i++)
                {
                    if (values[i] != 0)
                    {
                        validIntegrity = false;
                        break;
                    }
                }
                if (!validIntegrity)
                {
                    SafetyChecks.ThrowInvalidOperationException("Adding/removing components or changing position/rotation/velocity/collider ECS data" +
                        " on dynamic entities during physics step");
                }
            }
        }

        // Verifies combined collision filter of compound colliders
        // ToDo: add the same for mesh once per-triangle filters are supported
        private static void CheckColliderFilterIntegrity(NativeArray<PhysicsCollider> colliders)
        {
            for (int i = 0; i < colliders.Length; i++)
            {
                var collider = colliders[i];
                if (collider.IsValid && collider.Value.Value.Type == ColliderType.Compound)
                {
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)collider.ColliderPtr;

                        var rootFilter = compoundCollider->Filter;
                        var combinedFilter = CollisionFilter.Zero;
                        for (int childIndex = 0; childIndex < compoundCollider->Children.Length; childIndex++)
                        {
                            ref CompoundCollider.Child c = ref compoundCollider->Children[childIndex];
                            combinedFilter = CollisionFilter.CreateUnion(combinedFilter, c.Collider->Filter);
                        }

                        // GroupIndex has no concept of union. Creating one from children has no guarantees
                        // that it will be the same as the GroupIndex of the root, so we can't compare those two.
                        // Setting combinedFilter's GroupIndex to rootFilter's will exclude GroupIndex from comparing the two filters.
                        combinedFilter.GroupIndex = rootFilter.GroupIndex;

                        // Check that the combined filter (excluding GroupIndex) of all children is the same as root filter.
                        // If not, it means user has forgotten to call RefreshCollisionFilter() on the CompoundCollider.
                        if (!rootFilter.Equals(combinedFilter))
                        {
                            SafetyChecks.ThrowInvalidOperationException("CollisionFilter of a compound collider is not a union of its children. " +
                                "You must call CompoundCollider.RefreshCollisionFilter() to update the root filter after changing child filters.");
                        }
                    }
                }
            }
        }

        #endregion
    }
}
