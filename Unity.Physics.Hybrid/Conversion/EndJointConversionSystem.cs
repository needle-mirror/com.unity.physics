using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using UnityComponent = UnityEngine.Component;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// A system that is updated after all built-in conversion systems that produce <see cref="PhysicsJoint"/>.
    /// </summary>
    [AlwaysUpdateSystem]
    [UpdateAfter(typeof(BeginJointConversionSystem))]
    public class EndJointConversionSystem : GameObjectConversionSystem
    {
        // TODO: move this to a JointConversionSystemGroup when groups are supported for GameObjectConversionSystems
        Dictionary<UnityComponent, NativeList<Entity>> m_JointEntitiesPerAuthoringComponent =
            new Dictionary<UnityComponent, NativeList<Entity>>();

        static readonly ComponentTypes k_JointComponentsSingle = new ComponentTypes(
            typeof(PhysicsConstrainedBodyPair),
            typeof(PhysicsJoint)
        );
        static readonly ComponentTypes k_JointComponentsMultiple = new ComponentTypes(
            typeof(PhysicsConstrainedBodyPair),
            typeof(PhysicsJoint),
            typeof(PhysicsJointCompanion)
        );

        /// <summary>
        /// Create a new entity for a joint associated with the specified authoring component.
        /// Other systems can later find this joint by using <see cref="GetJointEntities"/>.
        /// Use this method when the behavior specified by the authoring component can be described with a single <see cref="PhysicsJoint"/> (see also <seealso cref="CreateJointEntities"/>).
        /// The new entity will have all required component types as well as a readable name in the Entity Debugger.
        /// </summary>
        /// <param name="authoringComponent">An authoring component being converted.</param>
        /// <param name="constrainedBodyPair">A component describing the bodies constrained by the joint.</param>
        /// <param name="joint">The reference frames and set of constraints to apply to the bodies.</param>
        /// <returns>A new entity with all required component types.</returns>
        public Entity CreateJointEntity(
            UnityComponent authoringComponent, PhysicsConstrainedBodyPair constrainedBodyPair, PhysicsJoint joint
        )
        {
            using (var joints = new NativeArray<PhysicsJoint>(1, Allocator.Temp) { [0] = joint })
            using (var jointEntities = new NativeList<Entity>(1, Allocator.Temp))
            {
                CreateJointEntities(authoringComponent, constrainedBodyPair, joints, jointEntities);
                return jointEntities[0];
            }
        }

        /// <summary>
        /// Create several new entities for a joint associated with the specified authoring component.
        /// Other systems can later find these joints by using <see cref="GetJointEntities"/>.
        /// Use this method instead of <see cref="CreateJointEntity"/> when the behavior specified by the authoring component requires multiple <see cref="PhysicsJoint"/> components to describe.
        /// The new entities will have all required component types, <see cref="PhysiscsJointCompanion"/> buffers, as well as readable names in the Entity Debugger.
        /// </summary>
        /// <param name="authoringComponent">An authoring component being converted.</param>
        /// <param name="constrainedBodyPair">A component describing the bodies constrained by the joints.</param>
        /// <param name="joints">The set of reference frames and corresponding constraints to apply to the bodies.</param>
        /// <param name="newJointEntities">An optional list to populate with all of the new entities.</param>
        public void CreateJointEntities(
            UnityComponent authoringComponent,
            PhysicsConstrainedBodyPair constrainedBodyPair,
            NativeArray<PhysicsJoint> joints,
            NativeList<Entity> newJointEntities = default
        )
        {
            if (!joints.IsCreated || joints.Length == 0)
                return;

            if (newJointEntities.IsCreated)
                newJointEntities.Clear();
            else
                newJointEntities = new NativeList<Entity>(joints.Length, Allocator.Temp);

            // find existing joints associated with the authoring component, if any
            if (!m_JointEntitiesPerAuthoringComponent.TryGetValue(authoringComponent, out var allJointEntities))
            {
                m_JointEntitiesPerAuthoringComponent[authoringComponent]
                    = allJointEntities
                        = new NativeList<Entity>(joints.Length, Allocator.Persistent);
            }

            // create all new joints
            var multipleJoints = joints.Length > 1;
#if UNITY_EDITOR
            var nameEntityA = DstEntityManager.GetName(constrainedBodyPair.EntityA);
            var nameEntityB = constrainedBodyPair.EntityB == Entity.Null
                ? "PhysicsWorld"
                : DstEntityManager.GetName(constrainedBodyPair.EntityB);
            var baseName = $"Joining {nameEntityA} + {nameEntityB}";
#endif
            for (var i = 0; i < joints.Length; ++i)
            {
                uint worldIndex = DstEntityManager.GetSharedComponentData<PhysicsWorldIndex>(constrainedBodyPair.EntityA).Value;
                Assertions.Assert.AreEqual(
                    worldIndex,
                    constrainedBodyPair.EntityB == Entity.Null ? worldIndex : DstEntityManager.GetSharedComponentData<PhysicsWorldIndex>(constrainedBodyPair.EntityB).Value);

                var jointEntity = CreateAdditionalEntity(authoringComponent);

                DstEntityManager.AddSharedComponentData(jointEntity, new PhysicsWorldIndex(worldIndex));

#if UNITY_EDITOR
                DstEntityManager.SetName(jointEntity, $"{baseName} ({joints[i].JointType})");
#endif

                DstEntityManager.AddComponents(
                    jointEntity, multipleJoints ? k_JointComponentsMultiple : k_JointComponentsSingle
                );

                DstEntityManager.SetComponentData(jointEntity, constrainedBodyPair);
                DstEntityManager.SetComponentData(jointEntity, joints[i]);

                newJointEntities.Add(jointEntity);
                allJointEntities.Add(jointEntity);
            }

            if (!multipleJoints)
                return;

            // set companion buffers for new joints
            for (var i = 0; i < joints.Length; ++i)
            {
                var companions = DstEntityManager.GetBuffer<PhysicsJointCompanion>(newJointEntities[i]);
                for (var j = 0; j < joints.Length; ++j)
                {
                    if (i == j)
                        continue;
                    companions.Add(new PhysicsJointCompanion { JointEntity = newJointEntities[j] });
                }
            }
        }

        /// <summary>
        /// Get all joint entities that have been produced by the specified authoring component.
        /// In order to work, these entities must have been produced by either <see cref="CreateJointEntity"/> or <see cref="CreateJointEntities"/>.
        /// </summary>
        /// <param name="authoringComponent">The authoring component that produced one or more joint entities.</param>
        /// <param name="jointEntities">A list to populate with all of the joint entities.</param>
        public void GetJointEntities(UnityComponent authoringComponent, NativeList<Entity> jointEntities)
        {
            jointEntities.Clear();
            if (!m_JointEntitiesPerAuthoringComponent.TryGetValue(authoringComponent, out var createdEntities))
                return;

            if (jointEntities.Capacity < createdEntities.Length)
                jointEntities.Capacity = createdEntities.Length;

            foreach (var jointEntity in createdEntities)
                jointEntities.Add(jointEntity);
        }

        protected override void OnUpdate() {}

        protected override void OnDestroy()
        {
            base.OnDestroy();

            foreach (var nativeList in m_JointEntitiesPerAuthoringComponent.Values)
            {
                if (nativeList.IsCreated)
                    nativeList.Dispose();
            }
            m_JointEntitiesPerAuthoringComponent.Clear();
        }
    }
}
