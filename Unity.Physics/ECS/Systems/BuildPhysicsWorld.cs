using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine.Assertions;

namespace Unity.Physics.Systems
{
    // A system which builds the physics world based on the entity world.
    // The world will contain a rigid body for every entity which has a rigid body component,
    // and a joint for every entity which has a joint component.
    public class BuildPhysicsWorld : JobComponentSystem
    {
        public PhysicsWorld PhysicsWorld = new PhysicsWorld(0, 0, 0);
        public JobHandle FinalJobHandle { get; private set; }

        // Entity group queries
        public EntityQuery DynamicEntityGroup { get; private set; }
        public EntityQuery StaticEntityGroup { get; private set; }
        public EntityQuery JointEntityGroup { get; private set; }

        EndFramePhysicsSystem m_EndFramePhysicsSystem;

        protected override void OnCreate()
        {
            base.OnCreate();

            DynamicEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(Translation),
                    typeof(Rotation)
                }
            });

            StaticEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsCollider)
                },
                Any = new ComponentType[]
                {
                    typeof(LocalToWorld),
                    typeof(Translation),
                    typeof(Rotation)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsVelocity)
                }
            });

            JointEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsJoint)
                }
            });

            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
        }

        protected override void OnDestroy()
        {
            PhysicsWorld.Dispose();
            base.OnDestroy();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            // Make sure last frame's physics jobs are complete
            m_EndFramePhysicsSystem.FinalJobHandle.Complete();

            // Extract types used by initialize jobs
            var entityType = GetArchetypeChunkEntityType();
            var localToWorldType = GetArchetypeChunkComponentType<LocalToWorld>(true);
            var parentType = GetArchetypeChunkComponentType<Parent>(true);
            var positionType = GetArchetypeChunkComponentType<Translation>(true);
            var rotationType = GetArchetypeChunkComponentType<Rotation>(true);
            var physicsColliderType = GetArchetypeChunkComponentType<PhysicsCollider>(true);
            var physicsVelocityType = GetArchetypeChunkComponentType<PhysicsVelocity>(true);
            var physicsMassType = GetArchetypeChunkComponentType<PhysicsMass>(true);
            var physicsDampingType = GetArchetypeChunkComponentType<PhysicsDamping>(true);
            var physicsGravityFactorType = GetArchetypeChunkComponentType<PhysicsGravityFactor>(true);
            var physicsCustomTagsType = GetArchetypeChunkComponentType<PhysicsCustomTags>(true);
            var physicsJointType = GetArchetypeChunkComponentType<PhysicsJoint>(true);

            int numDynamicBodies = DynamicEntityGroup.CalculateEntityCount();
            int numStaticBodies = StaticEntityGroup.CalculateEntityCount();
            int numJoints = JointEntityGroup.CalculateEntityCount();

            int previousStaticBodyCount = PhysicsWorld.NumStaticBodies;

            // Resize the world's native arrays
            PhysicsWorld.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            // Determine if the static bodies have changed in any way that will require the static broadphase tree to be rebuilt
            JobHandle staticBodiesCheckHandle = default;
            var haveStaticBodiesChanged = new NativeArray<int>(1, Allocator.TempJob);
            haveStaticBodiesChanged[0] = 0;
            {
                if (PhysicsWorld.NumStaticBodies != previousStaticBodyCount)
                {
                    haveStaticBodiesChanged[0] = 1;
                }
                else
                {
                    // Make a job to test for changes
                    int numChunks;
                    using (NativeArray<ArchetypeChunk> chunks = StaticEntityGroup.CreateArchetypeChunkArray(Allocator.TempJob))
                    {
                        numChunks = chunks.Length;
                    }
                    var chunksHaveChanges = new NativeArray<int>(numChunks, Allocator.TempJob);

                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        ChunkHasChangesOutput = chunksHaveChanges,
                        m_LastSystemVersion = LastSystemVersion
                    }.Schedule(StaticEntityGroup, inputDeps);

                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesReduceJob
                    {
                        ChunkHasChangesOutput = chunksHaveChanges,
                        Result = haveStaticBodiesChanged
                    }.Schedule(staticBodiesCheckHandle);
                }
            }

            using (var jobHandles = new NativeList<JobHandle>(4, Allocator.Temp))
            {
                // Static body changes check jobs
                jobHandles.Add(staticBodiesCheckHandle);

                // Create the default static body at the end of the body list
                // TODO: could skip this if no joints present
                jobHandles.Add(new Jobs.CreateDefaultStaticRigidBody
                {
                    NativeBodies = PhysicsWorld.Bodies,
                    BodyIndex = PhysicsWorld.Bodies.Length - 1
                }.Schedule(inputDeps));

                // Dynamic bodies. 
                // Create these separately from static bodies to maintain a 1:1 mapping
                // between dynamic bodies and their motions.
                if (numDynamicBodies > 0)
                {
                    jobHandles.Add(new Jobs.CreateRigidBodies
                    {
                        EntityType = entityType,
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomTagsType = physicsCustomTagsType,

                        FirstBodyIndex = 0,
                        RigidBodies = PhysicsWorld.Bodies
                    }.Schedule(DynamicEntityGroup, inputDeps));

                    jobHandles.Add(new Jobs.CreateMotions
                    {
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsVelocityType = physicsVelocityType,
                        PhysicsMassType = physicsMassType,
                        PhysicsDampingType = physicsDampingType,
                        PhysicsGravityFactorType = physicsGravityFactorType,

                        MotionDatas = PhysicsWorld.MotionDatas,
                        MotionVelocities = PhysicsWorld.MotionVelocities
                    }.Schedule(DynamicEntityGroup, inputDeps));
                }

                // Now, schedule creation of static bodies, with FirstBodyIndex pointing after 
                // the dynamic and kinematic bodies
                if (numStaticBodies > 0)
                {
                    jobHandles.Add(new Jobs.CreateRigidBodies
                    {
                        EntityType = entityType,
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomTagsType = physicsCustomTagsType,

                        FirstBodyIndex = numDynamicBodies,
                        RigidBodies = PhysicsWorld.Bodies
                    }.Schedule(StaticEntityGroup, inputDeps));
                }

                var handle = JobHandle.CombineDependencies(jobHandles);
                jobHandles.Clear();

                // Build joints
                if (numJoints > 0)
                {
                    jobHandles.Add(new Jobs.CreateJoints
                    {
                        JointComponentType = physicsJointType,
                        EntityType = entityType,
                        RigidBodies = PhysicsWorld.Bodies,
                        Joints = PhysicsWorld.Joints,
                        DefaultStaticBodyIndex = PhysicsWorld.Bodies.Length - 1,
                        NumDynamicBodies = numDynamicBodies
                    }.Schedule(JointEntityGroup, handle));
                }

                // Build the broadphase
                // TODO: could optimize this by gathering the AABBs and filters at the same time as building the bodies above
#if !UNITY_DOTSPLAYER
                float timeStep = UnityEngine.Time.fixedDeltaTime;
#else
                float timeStep = Time.DeltaTime;
#endif

                PhysicsStep stepComponent = PhysicsStep.Default;
                if (HasSingleton<PhysicsStep>())
                {
                   stepComponent = GetSingleton<PhysicsStep>();
                }

                JobHandle buildBroadphaseHandle = PhysicsWorld.CollisionWorld.ScheduleBuildBroadphaseJobs(
                    ref PhysicsWorld, timeStep, stepComponent.Gravity,
                    haveStaticBodiesChanged, handle, stepComponent.ThreadCountHint);
                jobHandles.Add(haveStaticBodiesChanged.Dispose(buildBroadphaseHandle));

                FinalJobHandle = JobHandle.CombineDependencies(jobHandles);
            }

            return JobHandle.CombineDependencies(FinalJobHandle, inputDeps);
        }

        #region Jobs

        private static class Jobs
        {
            [BurstCompile]
            internal struct CheckStaticBodyChangesJob : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkComponentType<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ArchetypeChunkComponentType<Parent> ParentType;
                [ReadOnly] public ArchetypeChunkComponentType<Translation> PositionType;
                [ReadOnly] public ArchetypeChunkComponentType<Rotation> RotationType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsCollider> PhysicsColliderType;

                public NativeArray<int> ChunkHasChangesOutput;
                public uint m_LastSystemVersion;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    bool didChunkChange =
                        chunk.DidChange(LocalToWorldType, m_LastSystemVersion) ||
                        chunk.DidChange(PositionType, m_LastSystemVersion) ||
                        chunk.DidChange(RotationType, m_LastSystemVersion) ||
                        chunk.DidChange(PhysicsColliderType, m_LastSystemVersion);
                    ChunkHasChangesOutput[chunkIndex] = didChunkChange ? 1 : 0;
                }
            }

            [BurstCompile]
            internal struct CheckStaticBodyChangesReduceJob : IJob
            {
                [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<int> ChunkHasChangesOutput;
                public NativeArray<int> Result;

                public void Execute()
                {
                    for (int i = 0; i < ChunkHasChangesOutput.Length; i++)
                    {
                        if (ChunkHasChangesOutput[i] > 0)
                        {
                            Result[0] = 1;
                            return;
                        }
                    }

                    Result[0] = 0;
                }
            }

            [BurstCompile]
            internal struct CreateDefaultStaticRigidBody : IJob
            {
                [NativeDisableContainerSafetyRestriction]
                public NativeSlice<RigidBody> NativeBodies;
                public int BodyIndex;

                public void Execute()
                {
                    NativeBodies[BodyIndex] = new RigidBody
                    {
                        WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                        Collider = default,
                        Entity = Entity.Null,
                        CustomTags = 0
                    };
                }
            }

            [BurstCompile]
            internal struct CreateRigidBodies : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkEntityType EntityType;
                [ReadOnly] public ArchetypeChunkComponentType<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ArchetypeChunkComponentType<Parent> ParentType;
                [ReadOnly] public ArchetypeChunkComponentType<Translation> PositionType;
                [ReadOnly] public ArchetypeChunkComponentType<Rotation> RotationType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsCollider> PhysicsColliderType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsCustomTags> PhysicsCustomTagsType;
                [ReadOnly] public int FirstBodyIndex;

                [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> RigidBodies;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsCollider> chunkColliders = chunk.GetNativeArray(PhysicsColliderType);
                    NativeArray<LocalToWorld> chunkLocalToWorlds = chunk.GetNativeArray(LocalToWorldType);
                    NativeArray<Translation> chunkPositions = chunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = chunk.GetNativeArray(RotationType);
                    NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);
                    NativeArray<PhysicsCustomTags> chunkCustomTags = chunk.GetNativeArray(PhysicsCustomTagsType);

                    int instanceCount = chunk.Count;
                    int rbIndex = FirstBodyIndex + firstEntityIndex;

                    bool hasChunkPhysicsColliderType = chunk.Has(PhysicsColliderType);
                    bool hasChunkPhysicsCustomTagsType = chunk.Has(PhysicsCustomTagsType);
                    bool hasChunkParentType = chunk.Has(ParentType);
                    bool hasChunkLocalToWorldType = chunk.Has(LocalToWorldType);
                    bool hasChunkPositionType = chunk.Has(PositionType);
                    bool hasChunkRotationType = chunk.Has(RotationType);

                    RigidTransform worldFromBody = RigidTransform.identity;
                    for (int i = 0; i < instanceCount; i++, rbIndex++)
                    {
                        // if entities are in a transform hierarchy then Translation/Rotation are in the space of their parents
                        // in that case, LocalToWorld is the only common denominator for world space
                        if (hasChunkParentType)
                        {
                            if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody = Math.DecomposeRigidBodyTransform(localToWorld.Value);
                            }
                        }
                        else
                        {
                            if (hasChunkPositionType)
                            {
                                worldFromBody.pos = chunkPositions[i].Value;
                            }
                            else if (hasChunkLocalToWorldType)
                            {
                                worldFromBody.pos = chunkLocalToWorlds[i].Position;
                            }

                            if (hasChunkRotationType)
                            {
                                worldFromBody.rot = chunkRotations[i].Value;
                            }
                            else if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody.rot = Math.DecomposeRigidBodyOrientation(localToWorld.Value);
                            }
                        }

                        RigidBodies[rbIndex] = new RigidBody
                        {
                            WorldFromBody = new RigidTransform(worldFromBody.rot, worldFromBody.pos),
                            Collider = hasChunkPhysicsColliderType ? chunkColliders[i].Value : default,
                            Entity = chunkEntities[i],
                            CustomTags = hasChunkPhysicsCustomTagsType ? chunkCustomTags[i].Value : (byte)0
                        };
                    }
                }
            }

            [BurstCompile]
            internal struct CreateMotions : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkComponentType<Translation> PositionType;
                [ReadOnly] public ArchetypeChunkComponentType<Rotation> RotationType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsVelocity> PhysicsVelocityType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsMass> PhysicsMassType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsDamping> PhysicsDampingType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsGravityFactor> PhysicsGravityFactorType;

                [NativeDisableParallelForRestriction] public NativeSlice<MotionData> MotionDatas;
                [NativeDisableParallelForRestriction] public NativeSlice<MotionVelocity> MotionVelocities;

                public unsafe void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    NativeArray<Translation> chunkPositions = chunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = chunk.GetNativeArray(RotationType);
                    NativeArray<PhysicsVelocity> chunkVelocities = chunk.GetNativeArray(PhysicsVelocityType);
                    NativeArray<PhysicsMass> chunkMasses = chunk.GetNativeArray(PhysicsMassType);
                    NativeArray<PhysicsDamping> chunkDampings = chunk.GetNativeArray(PhysicsDampingType);
                    NativeArray<PhysicsGravityFactor> chunkGravityFactors = chunk.GetNativeArray(PhysicsGravityFactorType);

                    int motionStart = firstEntityIndex;
                    int instanceCount = chunk.Count;

                    bool hasChunkPhysicsGravityFactorType = chunk.Has(PhysicsGravityFactorType);
                    bool hasChunkPhysicsDampingType = chunk.Has(PhysicsDampingType);
                    bool hasChunkPhysicsMassType = chunk.Has(PhysicsMassType);

                    // Note: Transform and AngularExpansionFactor could be calculated from PhysicsCollider.MassProperties
                    // However, to avoid the cost of accessing the collider we assume an infinite mass at the origin of a ~1m^3 box.
                    // For better performance with spheres, or better behavior for larger and/or more irregular colliders
                    // you should add a PhysicsMass component to get the true values
                    var defaultPhysicsMass = new PhysicsMass()
                    {
                        Transform = RigidTransform.identity,
                        InverseMass = 0.0f,
                        InverseInertia = float3.zero,
                        AngularExpansionFactor = 1.0f,
                    };

                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        MotionVelocities[motionIndex] = new MotionVelocity
                        {
                            LinearVelocity = chunkVelocities[i].Linear,
                            AngularVelocity = chunkVelocities[i].Angular,
                            InverseInertia = hasChunkPhysicsMassType ? chunkMasses[i].InverseInertia : defaultPhysicsMass.InverseInertia,
                            InverseMass = hasChunkPhysicsMassType ? chunkMasses[i].InverseMass : defaultPhysicsMass.InverseMass,
                            AngularExpansionFactor = hasChunkPhysicsMassType ? chunkMasses[i].AngularExpansionFactor : defaultPhysicsMass.AngularExpansionFactor
                        };

                    }

                    // Note: these defaults assume a dynamic body with infinite mass, hence no damping 
                    var defaultPhysicsDamping = new PhysicsDamping()
                    {
                        Linear = 0.0f,
                        Angular = 0.0f,
                    };

                    // Note: if a dynamic body infinite mass then assume no gravity should be applied
                    float defaultGravityFactor = hasChunkPhysicsMassType ? 1.0f : 0.0f;

                    // Create motion datas
                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        PhysicsMass mass = hasChunkPhysicsMassType ? chunkMasses[i] : defaultPhysicsMass;
                        PhysicsDamping damping = hasChunkPhysicsDampingType ? chunkDampings[i] : defaultPhysicsDamping;
                        float gravityFactor = hasChunkPhysicsGravityFactorType ? chunkGravityFactors[i].Value : defaultGravityFactor;

                        MotionDatas[motionIndex] = new MotionData
                        {
                            WorldFromMotion = new RigidTransform(
                                math.mul(chunkRotations[i].Value, mass.InertiaOrientation),
                                math.rotate(chunkRotations[i].Value, mass.CenterOfMass) + chunkPositions[i].Value
                            ),
                            BodyFromMotion = new RigidTransform(mass.InertiaOrientation, mass.CenterOfMass),
                            LinearDamping = damping.Linear,
                            AngularDamping = damping.Angular,
                            GravityFactor = gravityFactor
                        };
                    }
                }
            }

            [BurstCompile]
            internal struct CreateJoints : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsJoint> JointComponentType;
                [ReadOnly] public ArchetypeChunkEntityType EntityType;
                [ReadOnly] public NativeSlice<RigidBody> RigidBodies;
                [ReadOnly] public int NumDynamicBodies;

                [NativeDisableParallelForRestriction]
                public NativeSlice<Joint> Joints;

                public int DefaultStaticBodyIndex;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsJoint> chunkJoint = chunk.GetNativeArray(JointComponentType);
                    NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);

                    int instanceCount = chunk.Count;
                    for (int i = 0; i < instanceCount; i++)
                    {
                        PhysicsJoint joint = chunkJoint[i];
                        Assert.IsTrue(joint.EntityA != joint.EntityB);

                        // TODO find a reasonable way to look up the constraint body indices
                        // - stash body index in a component on the entity? But we don't have random access to Entity data in a job
                        // - make a map from entity to rigid body index? Sounds bad and I don't think there is any NativeArray-based map data structure yet

                        // If one of the entities is null, use the default static entity
                        var pair = new BodyIndexPair
                        {
                            BodyAIndex = joint.EntityA == Entity.Null ? DefaultStaticBodyIndex : -1,
                            BodyBIndex = joint.EntityB == Entity.Null ? DefaultStaticBodyIndex : -1
                        };

                        // Find the body indices
                        for (int bodyIndex = 0; bodyIndex < RigidBodies.Length; bodyIndex++)
                        {
                            if (joint.EntityA != Entity.Null)
                            {
                                if (RigidBodies[bodyIndex].Entity == joint.EntityA)
                                {
                                    pair.BodyAIndex = bodyIndex;
                                    if (pair.BodyBIndex >= 0)
                                    {
                                        break;
                                    }
                                }
                            }

                            if (joint.EntityB != Entity.Null)
                            {
                                if (RigidBodies[bodyIndex].Entity == joint.EntityB)
                                {
                                    pair.BodyBIndex = bodyIndex;
                                    if (pair.BodyAIndex >= 0)
                                    {
                                        break;
                                    }
                                }
                            }
                        }

                        bool isInvalid = false;
                        // Invalid if we have not found the body indices...
                        isInvalid |= (pair.BodyAIndex == -1 || pair.BodyBIndex == -1);
                        // ... or if we are constraining two static bodies
                        // Mark static-static invalid since they are not going to affect simulation in any way.
                        isInvalid |= (pair.BodyAIndex >= NumDynamicBodies && pair.BodyBIndex >= NumDynamicBodies);
                        if (isInvalid)
                        {
                            pair = BodyIndexPair.Invalid;
                        }

                        Joints[firstEntityIndex + i] = new Joint
                        {
                            JointData = chunkJoint[i].JointData,
                            BodyPair = pair,
                            Entity = chunkEntities[i],
                            EnableCollision = chunkJoint[i].EnableCollision
                        };
                    }
                }
            }
        }

        #endregion
    }
}
