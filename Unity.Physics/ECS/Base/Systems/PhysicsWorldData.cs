using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// Structure containing PhysicsWorld and other data and queries that are necessary for
    /// simulating a physics world. Note: it is important to create <see cref="PhysicsWorldData"/>
    /// and use it (to schedule physics world build) in the same system. Creating it in one system,
    /// and calling the Schedule() methods in another can cause race conditions.
    /// </summary>
    public struct PhysicsWorldData : IDisposable
    {
        /// <summary>   The physics world. </summary>
        public PhysicsWorld PhysicsWorld;

        /// <summary>   A flag indicating if the static bodies have changed in this frame. </summary>
        public NativeReference<int> HaveStaticBodiesChanged;

        /// <summary>   Query for dynamic bodies. </summary>
        public EntityQuery DynamicEntityGroup;
        /// <summary>   Query for bodies. </summary>
        public EntityQuery StaticEntityGroup;
        /// <summary>   Query for joints. </summary>
        public EntityQuery JointEntityGroup;

        /// <summary>   Query for bodies with invalidated temporal coherence info, e.g., due to deletion. </summary>
        public EntityQuery InvalidatedTemporalCoherenceInfoGroup;

        /// <summary>
        /// The component handles. Stores the information about ECS component handles needed for
        /// generating a <see cref="PhysicsWorld"/>
        /// </summary>
        public PhysicsWorldComponentHandles ComponentHandles;

        /// <summary>
        /// The physics world component handles. Stores the information about ECS component handles
        /// needed for generating a <see cref="PhysicsWorld"/>
        /// </summary>
        public struct PhysicsWorldComponentHandles
        {
            /// <summary>   Constructor. </summary>
            ///
            /// <param name="systemState">  [in,out] State of the system. </param>
            public PhysicsWorldComponentHandles(ref SystemState systemState)
            {
                EntityType = systemState.GetEntityTypeHandle();
                LocalToWorldType = systemState.GetComponentTypeHandle<LocalToWorld>(true);
                ParentType = systemState.GetComponentTypeHandle<Parent>(true);

                LocalTransformType = systemState.GetComponentTypeHandle<LocalTransform>(true);
                PhysicsColliderType = systemState.GetComponentTypeHandle<PhysicsCollider>(true);
                PhysicsVelocityType = systemState.GetComponentTypeHandle<PhysicsVelocity>(true);
                PhysicsMassType = systemState.GetComponentTypeHandle<PhysicsMass>(true);
                PhysicsMassOverrideType = systemState.GetComponentTypeHandle<PhysicsMassOverride>(true);
                PhysicsDampingType = systemState.GetComponentTypeHandle<PhysicsDamping>(true);
                PhysicsGravityFactorType = systemState.GetComponentTypeHandle<PhysicsGravityFactor>(true);
                PhysicsCustomTagsType = systemState.GetComponentTypeHandle<PhysicsCustomTags>(true);
                PhysicsConstrainedBodyPairType = systemState.GetComponentTypeHandle<PhysicsConstrainedBodyPair>(true);
                PhysicsJointType = systemState.GetComponentTypeHandle<PhysicsJoint>(true);
                SimulateType = systemState.GetComponentTypeHandle<Simulate>(true);

                PhysicsTemporalCoherenceInfoTypeRW = systemState.GetComponentTypeHandle<PhysicsTemporalCoherenceInfo>(false);
                PhysicsTemporalCoherenceInfoLookupRW = systemState.GetComponentLookup<PhysicsTemporalCoherenceInfo>(false);

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
                PhysicsWorldIndexType = systemState.GetSharedComponentTypeHandle<PhysicsWorldIndex>();
#endif
            }

            /// <summary>
            /// Updates the <see cref="PhysicsWorldComponentHandles"/>. Call this in OnUpdate() methods of
            /// the systems in which you want to store PhysicsWorldData in.
            /// </summary>
            ///
            /// <param name="systemState">  [in,out] State of the system. </param>
            public void Update(ref SystemState systemState)
            {
                EntityType.Update(ref systemState);
                LocalToWorldType.Update(ref systemState);
                ParentType.Update(ref systemState);

                LocalTransformType.Update(ref systemState);

                PhysicsColliderType.Update(ref systemState);
                PhysicsVelocityType.Update(ref systemState);
                PhysicsMassType.Update(ref systemState);
                PhysicsMassOverrideType.Update(ref systemState);
                PhysicsDampingType.Update(ref systemState);
                PhysicsGravityFactorType.Update(ref systemState);
                PhysicsCustomTagsType.Update(ref systemState);
                PhysicsConstrainedBodyPairType.Update(ref systemState);
                PhysicsJointType.Update(ref systemState);
                SimulateType.Update(ref systemState);

                PhysicsTemporalCoherenceInfoTypeRW.Update(ref systemState);
                PhysicsTemporalCoherenceInfoLookupRW.Update(ref systemState);

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
                PhysicsWorldIndexType.Update(ref systemState);
#endif
            }

            internal EntityTypeHandle EntityType;
            internal ComponentTypeHandle<LocalToWorld> LocalToWorldType;
            internal ComponentTypeHandle<Parent> ParentType;

            internal ComponentTypeHandle<LocalTransform> LocalTransformType;

            internal ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            internal ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            internal ComponentTypeHandle<PhysicsMass> PhysicsMassType;
            internal ComponentTypeHandle<PhysicsMassOverride> PhysicsMassOverrideType;
            internal ComponentTypeHandle<PhysicsDamping> PhysicsDampingType;
            internal ComponentTypeHandle<PhysicsGravityFactor> PhysicsGravityFactorType;
            internal ComponentTypeHandle<PhysicsCustomTags> PhysicsCustomTagsType;
            internal ComponentTypeHandle<PhysicsConstrainedBodyPair> PhysicsConstrainedBodyPairType;
            internal ComponentTypeHandle<PhysicsJoint> PhysicsJointType;
            internal ComponentTypeHandle<Simulate> SimulateType;

            internal ComponentTypeHandle<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoTypeRW;
            internal ComponentLookup<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoLookupRW;

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            internal SharedComponentTypeHandle<PhysicsWorldIndex> PhysicsWorldIndexType;
#endif
        }

        /// <summary>   Constructor. </summary>
        ///
        /// <param name="state">      [in,out] The <see cref="SystemState"/> of the system in which you
        /// want to use <see cref="PhysicsWorldData"/>. </param>
        /// <param name="worldIndex">   Zero-based index of the world. </param>
        public PhysicsWorldData(ref SystemState state, in PhysicsWorldIndex worldIndex)
        {
            PhysicsWorld = new PhysicsWorld(0, 0, 0);
            HaveStaticBodiesChanged = new NativeReference<int>(Allocator.Persistent);

            using (var queryBuilder = new EntityQueryBuilder(Allocator.Temp)
                       .WithAll<PhysicsVelocity, LocalTransform, PhysicsWorldIndex>())
            {
                DynamicEntityGroup = state.GetEntityQuery(queryBuilder);
                DynamicEntityGroup.SetSharedComponentFilter(worldIndex);
            }

            using (var queryBuilder = new EntityQueryBuilder(Allocator.Temp)
                       .WithAll<PhysicsCollider, PhysicsWorldIndex>()
                       .WithAny<LocalToWorld, LocalTransform>()
                       .WithNone<PhysicsVelocity>())
            {
                StaticEntityGroup = state.GetEntityQuery(queryBuilder);
                StaticEntityGroup.SetSharedComponentFilter(worldIndex);
            }

            using (var queryBuilder = new EntityQueryBuilder(Allocator.Temp)
                       .WithAll<PhysicsConstrainedBodyPair, PhysicsJoint, PhysicsWorldIndex>())
            {
                JointEntityGroup = state.GetEntityQuery(queryBuilder);
                JointEntityGroup.SetSharedComponentFilter(worldIndex);
            }

            using (var queryBuilder = new EntityQueryBuilder(Allocator.Temp)
                       // neither static nor dynamic
                       .WithAll<PhysicsTemporalCoherenceInfo>()
                       .WithNone<PhysicsVelocity, PhysicsCollider>()
                       .AddAdditionalQuery()
                       // not in any world
                       .WithAll<PhysicsTemporalCoherenceInfo>()
                       .WithNone<PhysicsWorldIndex>()
                       .AddAdditionalQuery()
                       // without valid transform
                       .WithAll<PhysicsTemporalCoherenceInfo>()
                       .WithNone<LocalToWorld, LocalTransform>()
                       .AddAdditionalQuery()
                       // disabled
                       .WithAll<PhysicsTemporalCoherenceInfo, Disabled>())
            {
                InvalidatedTemporalCoherenceInfoGroup = state.GetEntityQuery(queryBuilder);
            }

            ComponentHandles = new PhysicsWorldComponentHandles(ref state);
        }

        /// <summary>
        /// Calls the <see cref="PhysicsWorldComponentHandles.Update(ref SystemState)"/> of the
        /// handles stored in this object. /&gt;
        /// </summary>
        ///
        /// <param name="state">    [in,out] The state. </param>
        public void Update(ref SystemState state)
        {
            ComponentHandles.Update(ref state);
        }

        /// <summary>
        /// Free stored memory.
        /// </summary>
        public void Dispose()
        {
            PhysicsWorld.Dispose();
            if (HaveStaticBodiesChanged.IsCreated)
            {
                HaveStaticBodiesChanged.Dispose();
            }
        }
    }
}
