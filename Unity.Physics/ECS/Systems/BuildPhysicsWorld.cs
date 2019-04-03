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
        public ComponentGroup DynamicEntityGroup { get; private set; }
        public ComponentGroup StaticEntityGroup { get; private set; }
        public ComponentGroup JointEntityGroup { get; private set; }

        protected override void OnCreateManager()
        {
            base.OnCreateManager();

            DynamicEntityGroup = GetComponentGroup(new EntityArchetypeQuery
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsCollider),
                    typeof(PhysicsVelocity),
                    typeof(Translation),
                    typeof(Rotation)
                }
            });

            StaticEntityGroup = GetComponentGroup(new EntityArchetypeQuery
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

            JointEntityGroup = GetComponentGroup(new EntityArchetypeQuery
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsJoint)
                }
            });
        }

        protected override void OnDestroyManager()
        {
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

            // Check for static body changes before the reset()
            bool haveStaticBodiesChanged = false;
            {
                // For now, do this before the reset() - otherwise, we need the BuildRigidBodies jobs to finish
                if (numStaticBodies != (PhysicsWorld.StaticBodies.Length - 1)) //-1 for fake static body we add
                {
                    // Quick test if number of bodies changed
                    haveStaticBodiesChanged = true;
                }
                else
                {
                    // Make a job to test for changes

                    int numChunks; // There has to be a better way of doing this...
                    {
                        var chunks = StaticEntityGroup.CreateArchetypeChunkArray(Allocator.TempJob);
                        numChunks = chunks.Length;
                        chunks.Dispose();
                    }

                    var chunksHaveChanges = new NativeArray<int>(numChunks, Allocator.TempJob);
                    var checkStaticChanges = new Jobs.CheckStaticBodyChangesJob
                    {
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        StaticRigidBodies = PhysicsWorld.StaticBodies,
                        ChunkHasChangesOutput = chunksHaveChanges
                    };

                    checkStaticChanges.Schedule(StaticEntityGroup, inputDeps).Complete();
                    for (int i = 0; i < numChunks; i++)
                    {
                        haveStaticBodiesChanged |= chunksHaveChanges[i] != 0;
                    }
                    chunksHaveChanges.Dispose();
                }
            }

            // Resize the world's native arrays
            PhysicsWorld.Reset(
                numStaticBodies: numStaticBodies + 1,   // +1 for the default static body
                numDynamicBodies: numDynamicBodies,
                numJoints: numJoints);

            var jobHandles = new NativeList<JobHandle>(4, Allocator.Temp);

            // Create the default static body at the end of the body list
            // TODO: could skip this if no joints present
            jobHandles.Add(new Jobs.CreateDefaultStaticRigidBody
            {
                NativeBodies = PhysicsWorld.Bodies,
                BodyIndex = PhysicsWorld.Bodies.Length - 1
            }.Schedule(inputDeps));

            // Dynamic bodies. Create these separately from static bodies to maintain a 1:1 mapping
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

            // Now, schedule creation of static bodies, with FirstBodyIndex pointing after the dynamic bodies
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

            jobHandles.Add(PhysicsWorld.CollisionWorld.Broadphase.ScheduleBuildJobs(ref PhysicsWorld, timeStep, stepComponent.ThreadCountHint, haveStaticBodiesChanged, handle));

            FinalJobHandle = JobHandle.CombineDependencies(jobHandles);
            jobHandles.Dispose();

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
                [ReadOnly] public NativeSlice<RigidBody> StaticRigidBodies;

                [NativeDisableContainerSafetyRestriction] public NativeSlice<int> ChunkHasChangesOutput;

                public unsafe void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    var chunkColliders = chunk.GetNativeArray(PhysicsColliderType);
                    var chunkPositions = chunk.GetNativeArray(PositionType);
                    var chunkRotations = chunk.GetNativeArray(RotationType);

                    ChunkHasChangesOutput[chunkIndex] = 0;

                    // Check the contents of collider/positions/rotations and determine if the value changed since we
                    // last built the rigid bodies array
                    int count = chunk.Count;
                    for (int i = 0, rigidBodyIndex = firstEntityIndex; i < count; i++, rigidBodyIndex++)
                    {
                        // ToDo: implement a check to verify if collider was modified in place.
                        bool colliderDifferent = StaticRigidBodies[rigidBodyIndex].Collider != chunkColliders[i].ColliderPtr;
                        bool positionDifferent = !StaticRigidBodies[rigidBodyIndex].WorldFromBody.pos.Equals(chunkPositions[i].Value);
                        bool rotationDifferent = !StaticRigidBodies[rigidBodyIndex].WorldFromBody.rot.Equals(chunkRotations[i].Value);

                        if (positionDifferent || rotationDifferent || colliderDifferent)
                        {
                            ChunkHasChangesOutput[chunkIndex] = 1;
                            return;
                        }
                    }
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

                    int instanceCount = chunk.Count;
                    int rbIndex = FirstBodyIndex + firstEntityIndex;

                    if (!chunk.Has(PhysicsCustomDataType))
                    {
                        for (int i = 0; i < instanceCount; i++, rbIndex++)
                        {
                            RigidBodies[rbIndex] = new RigidBody
                            {
                                WorldFromBody = new RigidTransform(chunkRotations[i].Value, chunkPositions[i].Value),
                                Collider = chunkColliders[i].ColliderPtr,
                                Entity = chunkEntities[i]
                            };
                        }
                    }
                    else
                    {
                        var chunkCustomDatas = chunk.GetNativeArray(PhysicsCustomDataType);
                        for (int i = 0; i < instanceCount; i++, rbIndex++)
                        {
                            RigidBodies[rbIndex] = new RigidBody
                            {
                                WorldFromBody = new RigidTransform(chunkRotations[i].Value, chunkPositions[i].Value),
                                Collider = chunkColliders[i].ColliderPtr,
                                Entity = chunkEntities[i],
                                CustomData = chunkCustomDatas[i].Value
                            };
                        }
                    }
                }
            }

            // Create a native motion data
            private static MotionData CreateMotionData(
                Translation position, Rotation orientation, PhysicsMass massComponent,
                float linearDamping, float angularDamping, float gravityFactor = 1.0f)
            {
                return new MotionData
                {
                    WorldFromMotion = new RigidTransform(
                        math.mul(orientation.Value, massComponent.InertiaOrientation),
                        math.rotate(orientation.Value, massComponent.CenterOfMass) + position.Value
                    ),
                    BodyFromMotion = massComponent.Transform,
                    LinearDamping = linearDamping,
                    AngularDamping = angularDamping,
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

                [NativeDisableContainerSafetyRestriction] public NativeSlice<MotionData> MotionDatas;
                [NativeDisableContainerSafetyRestriction] public NativeSlice<MotionVelocity> MotionVelocities;

                public unsafe void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    var chunkPositions = chunk.GetNativeArray(PositionType);
                    var chunkRotations = chunk.GetNativeArray(RotationType);
                    var chunkVelocities = chunk.GetNativeArray(PhysicsVelocityType);
                    var chunkMasses = chunk.GetNativeArray(PhysicsMassType);

                    int motionStart = firstEntityIndex;
                    int instanceCount = chunk.Count;

                    // Create motion velocities
                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        MotionVelocities[motionIndex] = new MotionVelocity
                        {
                            LinearVelocity = chunkVelocities[i].Linear,    // world space
                            AngularVelocity = chunkVelocities[i].Angular,  // inertia space
                            InverseInertiaAndMass = new float4(chunkMasses[i].InverseInertia, chunkMasses[i].InverseMass),
                            AngularExpansionFactor = chunkMasses[i].AngularExpansionFactor
                        };
                    }

                    // Create motion datas
                    const float defaultLinearDamping = 0.0f;    // TODO: Use non-zero defaults?
                    const float defaultAngularDamping = 0.0f;
                    if (chunk.Has(PhysicsGravityFactorType))
                    {
                        // With gravity factor ...
                        var chunkGravityFactors = chunk.GetNativeArray(PhysicsGravityFactorType);
                        if (chunk.Has(PhysicsDampingType))
                        {
                            // ... with damping
                            var chunkDampings = chunk.GetNativeArray(PhysicsDampingType);
                            for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                            {
                                MotionDatas[motionIndex] = CreateMotionData(
                                    chunkPositions[i], chunkRotations[i], chunkMasses[i],
                                    chunkDampings[i].Linear, chunkDampings[i].Angular, chunkGravityFactors[i].Value);
                            }
                        }
                        else
                        {
                            // ... without damping
                            for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                            {
                                MotionDatas[motionIndex] = CreateMotionData(
                                    chunkPositions[i], chunkRotations[i], chunkMasses[i],
                                    defaultLinearDamping, defaultAngularDamping, chunkGravityFactors[i].Value);
                            }
                        }
                    }
                    else
                    {
                        // Without gravity factor ...
                        if (chunk.Has(PhysicsDampingType))
                        {
                            // ... with damping
                            var chunkDampings = chunk.GetNativeArray(PhysicsDampingType);
                            for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                            {
                                MotionDatas[motionIndex] = CreateMotionData(
                                    chunkPositions[i], chunkRotations[i], chunkMasses[i],
                                    chunkDampings[i].Linear, chunkDampings[i].Angular);
                            }
                        }
                        else
                        {
                            // ... without damping
                            for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                            {
                                MotionDatas[motionIndex] = CreateMotionData(
                                    chunkPositions[i], chunkRotations[i], chunkMasses[i],
                                    defaultLinearDamping, defaultAngularDamping);
                            }
                        }
                    }
                }
            }

            [BurstCompile]
            internal struct CreateJoints : IJobChunk
            {
                [ReadOnly] public ArchetypeChunkComponentType<PhysicsJoint> JointComponentType;
                [ReadOnly] public ArchetypeChunkEntityType EntityType;
                [ReadOnly] public NativeSlice<RigidBody> RigidBodies;

                [NativeDisableContainerSafetyRestriction]
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
