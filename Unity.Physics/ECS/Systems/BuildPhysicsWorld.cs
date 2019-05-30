using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine.Assertions;

namespace Unity.Physics
{
    public struct StaticLayerChangeInfo
    {
        public void Init(Allocator allocator = Allocator.Persistent)
        {
            NumStaticBodiesArray = new NativeArray<int>(1, allocator);
            HaveStaticBodiesChangedArray = new NativeArray<int>(1, allocator);
        }

        public void Deallocate()
        {
            if (NumStaticBodiesArray.IsCreated)
            {
                NumStaticBodiesArray.Dispose();
            }

            if (HaveStaticBodiesChangedArray.IsCreated)
            {
                HaveStaticBodiesChangedArray.Dispose();
            }
        }

        /// <summary>
        /// Number of static bodies if HaveStaticBodiesChanged is true, zero otherwise.
        /// </summary>
        public int NumStaticBodies
        {
            get { return NumStaticBodiesArray[0]; }
            set { NumStaticBodiesArray[0] = value; }
        }

        /// <summary>
        /// "Boolean" value indicating if static layer has changed compared to previous frame.
        /// </summary>
        public int HaveStaticBodiesChanged
        {
            get { return HaveStaticBodiesChangedArray[0]; }
            set { HaveStaticBodiesChangedArray[0] = value; }
        }

        public NativeArray<int> NumStaticBodiesArray;
        public NativeArray<int> HaveStaticBodiesChangedArray;
    }
}

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

        // Array of length 2
        // First element is a number of static bodies in current frame if static broad-phase layer need to be updated, zero otherwise.
        // Second element is "bool" flag indicating if static broad-phase layer needs to be updated.
        //public NativeArray<int> StaticBodiesChangedInfo;
        public StaticLayerChangeInfo m_StaticLayerChangeInfo;

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
                    typeof(PhysicsCollider),
                    typeof(Translation),
                    typeof(Rotation)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsVelocity)
                },
            });

            JointEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsJoint)
                }
            });

            m_StaticLayerChangeInfo.Init();
        }

        protected override void OnDestroyManager()
        {
            m_StaticLayerChangeInfo.Deallocate();

            PhysicsWorld.Dispose();
            base.OnDestroyManager();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            // Extract types used by initialize jobs
            var entityType = GetArchetypeChunkEntityType();
            var positionType = GetArchetypeChunkComponentType<Translation>(true);
            var rotationType = GetArchetypeChunkComponentType<Rotation>(true);
            var physicsColliderType = GetArchetypeChunkComponentType<PhysicsCollider>(true);
            var physicsVelocityType = GetArchetypeChunkComponentType<PhysicsVelocity>(true);
            var physicsMassType = GetArchetypeChunkComponentType<PhysicsMass>(true);
            var physicsDampingType = GetArchetypeChunkComponentType<PhysicsDamping>(true);
            var physicsGravityFactorType = GetArchetypeChunkComponentType<PhysicsGravityFactor>(true);
            var physicsCustomDataType = GetArchetypeChunkComponentType<PhysicsCustomData>(true);
            var physicsJointType = GetArchetypeChunkComponentType<PhysicsJoint>(true);

            int numDynamicBodies = DynamicEntityGroup.CalculateLength();
            int numStaticBodies = StaticEntityGroup.CalculateLength();
            int numJoints = JointEntityGroup.CalculateLength();

            m_StaticLayerChangeInfo.NumStaticBodies = numStaticBodies + 1;
            m_StaticLayerChangeInfo.HaveStaticBodiesChanged = 0;

            if (numStaticBodies != (PhysicsWorld.StaticBodies.Length - 1)) //-1 for fake static body we add
            {
                // Quick test if number of bodies changed
                m_StaticLayerChangeInfo.HaveStaticBodiesChanged = 1;
            }
            else
            {
                // Make a job to test for changes
                int numChunks; // There has to be a better way of doing this...
                using (var chunks = StaticEntityGroup.CreateArchetypeChunkArray(Allocator.TempJob))
                {
                    numChunks = chunks.Length;
                }

                var chunksHaveChanges = new NativeArray<int>(numChunks, Allocator.TempJob);

                inputDeps = new Jobs.CheckStaticBodyChangesJob
                {
                    PositionType = positionType,
                    RotationType = rotationType,
                    PhysicsColliderType = physicsColliderType,
                    ChunkHasChangesOutput = chunksHaveChanges,
                    m_LastSystemVersion = LastSystemVersion
                }.Schedule(StaticEntityGroup, inputDeps);

                inputDeps = new Jobs.CheckStaticBodyChangesReduceJob
                {
                    ChunkHasChangesOutput = chunksHaveChanges,
                    HaveStaticBodiesChanged = m_StaticLayerChangeInfo.HaveStaticBodiesChangedArray,
                    NumStaticBodies = m_StaticLayerChangeInfo.NumStaticBodiesArray
                }.Schedule(inputDeps);
            }

            // Resize the world's native arrays
            PhysicsWorld.Reset(
                numStaticBodies: numStaticBodies + 1,   // +1 for the default static body
                numDynamicBodies: numDynamicBodies,
                numJoints: numJoints);

            using (var jobHandles = new NativeList<JobHandle>(4, Allocator.Temp))
            {
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
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomDataType = physicsCustomDataType,

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
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomDataType = physicsCustomDataType,

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
                        DefaultStaticBodyIndex = PhysicsWorld.Bodies.Length - 1
                    }.Schedule(JointEntityGroup, handle));
                }

                // Build the broadphase
                // TODO: could optimize this by gathering the AABBs and filters at the same time as building the bodies above

                float timeStep = UnityEngine.Time.fixedDeltaTime;

                PhysicsStep stepComponent = PhysicsStep.Default;
                if (HasSingleton<PhysicsStep>())
                {
                    stepComponent = GetSingleton<PhysicsStep>();
                }

                jobHandles.Add(PhysicsWorld.CollisionWorld.Broadphase.ScheduleBuildJobs(ref PhysicsWorld, timeStep, stepComponent.ThreadCountHint, ref m_StaticLayerChangeInfo, handle));

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
                [ReadOnly] public ArchetypeChunkComponentType<Translation> PositionType;
                [ReadOnly] public ArchetypeChunkComponentType<Rotation> RotationType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsCollider> PhysicsColliderType;

                public NativeArray<int> ChunkHasChangesOutput;
                public uint m_LastSystemVersion;

                public unsafe void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    bool didChunkChange =
                        chunk.DidChange(PositionType, m_LastSystemVersion) ||
                        chunk.DidChange(RotationType, m_LastSystemVersion) ||
                        chunk.DidChange(PhysicsColliderType, m_LastSystemVersion);
                    ChunkHasChangesOutput[chunkIndex] = didChunkChange ? 1 : 0;
                }
            }

            internal struct CheckStaticBodyChangesReduceJob : IJob
            {
                [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<int> ChunkHasChangesOutput;
                public NativeArray<int> HaveStaticBodiesChanged;
                public NativeArray<int> NumStaticBodies;

                public void Execute()
                {
                    for (int i = 0; i < ChunkHasChangesOutput.Length; i++)
                    {
                        if (ChunkHasChangesOutput[0] > 0)
                        {
                            HaveStaticBodiesChanged[0] = 1;
                            return;
                        }
                    }

                    NumStaticBodies[0] = 0;
                    HaveStaticBodiesChanged[0] = 0;
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
                        Collider = null,
                        Entity = Entity.Null,
                        CustomData = 0
                    };
                }
            }

            [BurstCompile]
            internal struct CreateRigidBodies : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkEntityType EntityType;
                [ReadOnly] public ArchetypeChunkComponentType<Translation> PositionType;
                [ReadOnly] public ArchetypeChunkComponentType<Rotation> RotationType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsCollider> PhysicsColliderType;
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsCustomData> PhysicsCustomDataType;
                [ReadOnly] public int FirstBodyIndex;

                [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> RigidBodies;

                public unsafe void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    var chunkColliders = chunk.GetNativeArray(PhysicsColliderType);
                    var chunkPositions = chunk.GetNativeArray(PositionType);
                    var chunkRotations = chunk.GetNativeArray(RotationType);
                    var chunkEntities = chunk.GetNativeArray(EntityType);
                    var chunkCustomDatas = chunk.GetNativeArray(PhysicsCustomDataType);

                    int instanceCount = chunk.Count;
                    int rbIndex = FirstBodyIndex + firstEntityIndex;

                    bool hasChunkPhysicsColliderType = chunk.Has(PhysicsColliderType);
                    bool hasChunkPhysicsCustomDataType = chunk.Has(PhysicsCustomDataType);

                    for (int i = 0; i < instanceCount; i++, rbIndex++)
                    {
                        RigidBodies[rbIndex] = new RigidBody
                        {
                            WorldFromBody = new RigidTransform(chunkRotations[i].Value, chunkPositions[i].Value),
                            Collider = hasChunkPhysicsColliderType ? chunkColliders[i].ColliderPtr : null,
                            Entity = chunkEntities[i],
                            CustomData = hasChunkPhysicsCustomDataType ? chunkCustomDatas[i].Value : (byte)0,
                        };
                    }
                }
            }


            [BurstCompile]
            private static MotionData CreateMotionData(
                Translation position, Rotation orientation, 
                PhysicsMass physicsMass, PhysicsDamping damping, 
                float gravityFactor)
            {
                return new MotionData
                {
                    WorldFromMotion = new RigidTransform(
                        math.mul(orientation.Value, physicsMass.InertiaOrientation),
                        math.rotate(orientation.Value, physicsMass.CenterOfMass) + position.Value
                    ),
                    BodyFromMotion = physicsMass.Transform,
                    LinearDamping = damping.Linear,
                    AngularDamping = damping.Angular,
                    GravityFactor = gravityFactor
                };
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
                    var chunkPositions = chunk.GetNativeArray(PositionType);
                    var chunkRotations = chunk.GetNativeArray(RotationType);
                    var chunkVelocities = chunk.GetNativeArray(PhysicsVelocityType);
                    var chunkMasses = chunk.GetNativeArray(PhysicsMassType);
                    var chunkDampings = chunk.GetNativeArray(PhysicsDampingType);
                    var chunkGravityFactors = chunk.GetNativeArray(PhysicsGravityFactorType);

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

                    // Create motion velocities
                    var defaultInverseInertiaAndMass = new float4(defaultPhysicsMass.InverseInertia, defaultPhysicsMass.InverseMass);
                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        MotionVelocities[motionIndex] = new MotionVelocity
                        {
                            LinearVelocity = chunkVelocities[i].Linear,    // world space
                            AngularVelocity = chunkVelocities[i].Angular,  // inertia space
                            InverseInertiaAndMass = hasChunkPhysicsMassType ? new float4(chunkMasses[i].InverseInertia, chunkMasses[i].InverseMass) : defaultInverseInertiaAndMass,
                            AngularExpansionFactor = hasChunkPhysicsMassType ? chunkMasses[i].AngularExpansionFactor : defaultPhysicsMass.AngularExpansionFactor,
                        };
                    }

                    // Note: these defaults assume a dynamic body with infinite mass, hence no damping 
                    var defaultPhysicsDamping = new PhysicsDamping()
                    {
                        Linear = 0.0f,
                        Angular = 0.0f,
                    };

                    // Note: if a dynamic body infinite mass then assume no gravity should be applied
                    var defaultGravityFactor = hasChunkPhysicsMassType ? 1.0f : 0.0f;

                    // Create motion datas
                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        MotionDatas[motionIndex] = CreateMotionData(
                            chunkPositions[i], chunkRotations[i], 
                            hasChunkPhysicsMassType ? chunkMasses[i] : defaultPhysicsMass,
                            hasChunkPhysicsDampingType ? chunkDampings[i] : defaultPhysicsDamping,
                            hasChunkPhysicsGravityFactorType ? chunkGravityFactors[i].Value : defaultGravityFactor);
                    }
                }
            }

            [BurstCompile]
            internal struct CreateJoints : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsJoint> JointComponentType;
                [ReadOnly] public ArchetypeChunkEntityType EntityType;
                [ReadOnly] public NativeSlice<RigidBody> RigidBodies;

                [NativeDisableParallelForRestriction]
                public NativeSlice<Joint> Joints;

                public int DefaultStaticBodyIndex;

                public unsafe void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    var chunkJoint = chunk.GetNativeArray(JointComponentType);
                    var chunkEntities = chunk.GetNativeArray(EntityType);

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

                        Assert.IsTrue(pair.BodyAIndex != -1 && pair.BodyBIndex != -1);

                        Joints[firstEntityIndex + i] = new Joint
                        {
                            JointData = (JointData*)joint.JointData.GetUnsafePtr(),
                            BodyPair = pair,
                            Entity = chunkEntities[i],
                            EnableCollision = joint.EnableCollision,
                        };
                    }
                }
            }
        }

        #endregion
    }
}
