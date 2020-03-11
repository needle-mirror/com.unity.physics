#if LEGACY_PHYSICS
using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using FloatRange = Unity.Physics.Math.FloatRange;
using LegacyCharacter = UnityEngine.CharacterJoint;
using LegacyConfigurable = UnityEngine.ConfigurableJoint;
using LegacyFixed = UnityEngine.FixedJoint;
using LegacyHinge = UnityEngine.HingeJoint;
using LegacyJoint = UnityEngine.Joint;
using LegacySpring = UnityEngine.SpringJoint;

namespace Unity.Physics.Authoring
{
    public sealed class LegacyJointConversionSystem : GameObjectConversionSystem
    {
        void CreateJointEntity(GameObject gameObject, BlobAssetReference<JointData> jointData, Entity entityA, Entity entityB, bool enableCollision)
        {
            var componentData = new PhysicsJoint
            {
                JointData = jointData,
                EntityA = entityA,
                EntityB = entityB,
                EnableCollision = enableCollision ? 1 : 0,
            };

            Entity jointEntity = CreateAdditionalEntity(gameObject);
#if UNITY_EDITOR
            var nameEntityA = DstEntityManager.GetName(entityA);
            var nameEntityB = entityB == Entity.Null ? "PhysicsWorld" : DstEntityManager.GetName(entityB);
            DstEntityManager.SetName(jointEntity, $"Joining {nameEntityA} + {nameEntityB}");
#endif

            DstEntityManager.AddOrSetComponent(jointEntity, componentData);
        }

        BlobAssetReference<JointData> CreateConfigurableJoint(
            quaternion jointFrameOrientation,
            LegacyJoint joint, bool3 linearLocks, bool3 linearLimited, SoftJointLimit linearLimit, SoftJointLimitSpring linearSpring, bool3 angularFree, bool3 angularLocks,
            bool3 angularLimited, SoftJointLimit lowAngularXLimit, SoftJointLimit highAngularXLimit, SoftJointLimitSpring angularXLimitSpring, SoftJointLimit angularYLimit,
            SoftJointLimit angularZLimit, SoftJointLimitSpring angularYZLimitSpring)
        {
            var constraints = new NativeList<Constraint>(Allocator.Temp);

            // TODO: investigate mapping PhysX spring and damping to Unity Physics SpringFrequency and SpringDamping
            var springFrequency = Constraint.DefaultSpringFrequency;
            var springDamping = Constraint.DefaultSpringDamping;
            
            if (angularLimited[0])
            {
                constraints.Add(Constraint.Twist(0, math.radians(new FloatRange(-highAngularXLimit.limit, -lowAngularXLimit.limit)), springFrequency, springDamping));
            }

            if (angularLimited[1])
            {
                constraints.Add(Constraint.Twist(1, math.radians(new FloatRange(-angularYLimit.limit, angularYLimit.limit)), springFrequency, springDamping));
            }

            if (angularLimited[2])
            {
                constraints.Add(Constraint.Twist(2, math.radians(new FloatRange(-angularZLimit.limit, angularZLimit.limit)), springFrequency, springDamping));
            }

            if (math.any(linearLimited))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLimited,
                    Type = ConstraintType.Linear,
                    Min = math.csum((int3)linearLimited) == 1 ? -linearLimit.limit : 0f,
                    Max = linearLimit.limit,
                    SpringFrequency = springFrequency,
                    SpringDamping = springDamping
                });
            }

            if (math.any(linearLocks))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLocks,
                    Type = ConstraintType.Linear,
                    Min = 0,
                    Max = 0,
                    SpringFrequency = springFrequency,
                    SpringDamping = springDamping
                });
            }

            if (math.any(angularLocks))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = angularLocks,
                    Type = ConstraintType.Angular,
                    Min = 0,
                    Max = 0,
                    SpringFrequency = springFrequency,
                    SpringDamping = springDamping
                });
            }

            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            RigidTransform worldFromBodyB = joint.connectedBody == null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var legacyWorldFromJointA = math.mul(
                new RigidTransform(joint.transform.rotation, joint.transform.position),
                new RigidTransform(jointFrameOrientation, joint.anchor)
            );
            var bodyAFromJoint = new JointFrame(math.mul(math.inverse(worldFromBodyA), legacyWorldFromJointA));
            
            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;
           
            RigidTransform bFromA = isConnectedBodyConverted ? math.mul(math.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : worldFromBodyB;

            var bodyBFromJoint = new JointFrame
            {
                Axis = math.mul(bFromA.rot, bodyAFromJoint.Axis),
                PerpendicularAxis = math.mul(bFromA.rot, bodyAFromJoint.PerpendicularAxis),
                Position = math.mul(bFromBSource, new float4(joint.connectedAnchor, 1f)).xyz
            };
            
            var jointData =  JointData.Create(bodyAFromJoint, bodyBFromJoint, constraints);

            constraints.Dispose();
            
            return jointData;
        }

        bool IsMotionFree(ConfigurableJointMotion motion)
        {
            return motion == ConfigurableJointMotion.Free;
        }

        bool IsMotionLocked(ConfigurableJointMotion motion)
        {
            return motion == ConfigurableJointMotion.Locked;
        }

        bool IsMotionLimited(ConfigurableJointMotion motion)
        {
            return motion == ConfigurableJointMotion.Limited;
        }
        
        void ConvertConfigurableJoint(LegacyConfigurable joint)
        {
            var linearLocks = new bool3
            (
                IsMotionLocked(joint.xMotion),
                IsMotionLocked(joint.yMotion),
                IsMotionLocked(joint.zMotion)
            );

            var linearLimited = new bool3
            (
                IsMotionLimited(joint.xMotion),
                IsMotionLimited(joint.yMotion),
                IsMotionLimited(joint.zMotion)
            );
           
            var angularFree = new bool3
            (
                IsMotionFree(joint.angularXMotion),
                IsMotionFree(joint.angularYMotion),
                IsMotionFree(joint.angularZMotion)
            );

            var angularLocks = new bool3
            (
                IsMotionLocked(joint.angularXMotion),
                IsMotionLocked(joint.angularYMotion),
                IsMotionLocked(joint.angularZMotion)
            );

            var angularLimited = new bool3
            (
                IsMotionLimited(joint.angularXMotion),
                IsMotionLimited(joint.angularYMotion),
                IsMotionLimited(joint.angularZMotion)
            );

            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.secondaryAxis);
            var jointData = CreateConfigurableJoint(jointFrameOrientation, joint, linearLocks, linearLimited, joint.linearLimit, joint.linearLimitSpring, angularFree, angularLocks, angularLimited,
                joint.lowAngularXLimit, joint.highAngularXLimit, joint.angularXLimitSpring, joint.angularYLimit, joint.angularZLimit, joint.angularYZLimitSpring);
            
            CreateJointEntity(joint.gameObject, jointData, GetPrimaryEntity(joint.gameObject), joint.connectedBody == null ? Entity.Null : GetPrimaryEntity(joint.connectedBody), joint.enableCollision);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static quaternion GetJointFrameOrientation(float3 axis, float3 secondaryAxis) =>
            new JointFrame { Axis = axis, PerpendicularAxis = secondaryAxis }.AsRigidTransform().rot;

        void ConvertCharacterJoint(LegacyCharacter joint)
        {
            var linearLocks = new bool3(true);
            var linearLimited = new bool3(false);

            var angularFree = new bool3(false);
            var angularLocks = new bool3(false);
            var angularLimited = new bool3(true);

            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.swingAxis);
            var jointData = CreateConfigurableJoint(jointFrameOrientation, joint, linearLocks, linearLimited, new SoftJointLimit { limit = 0f, bounciness = 0f }, new SoftJointLimitSpring { spring = 0f, damper = 0f }, angularFree, angularLocks, angularLimited,
                joint.lowTwistLimit, joint.highTwistLimit, joint.twistLimitSpring, joint.swing1Limit, joint.swing2Limit, joint.swingLimitSpring);

            CreateJointEntity(joint.gameObject, jointData, GetPrimaryEntity(joint.gameObject), joint.connectedBody == null ? Entity.Null : GetPrimaryEntity(joint.connectedBody), joint.enableCollision);
        }

        void ConvertSpringJoint(LegacySpring joint)
        {
            var constraints = new NativeList<Constraint>(Allocator.Temp);
            constraints.Add(new Constraint {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = joint.minDistance,
                Max = joint.maxDistance,
                SpringFrequency = 1f, // ?
                SpringDamping = 0.1f // ?
            });

            var jointFrameA = JointFrame.Identity;
            jointFrameA.Position = joint.anchor;

            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var jointFrameB = JointFrame.Identity;
            jointFrameB.Position = math.mul(bFromBSource, new float4(joint.connectedAnchor, 1f)).xyz;

            var jointData = JointData.Create(jointFrameA, jointFrameB, constraints);

            CreateJointEntity(joint.gameObject, jointData, GetPrimaryEntity(joint.gameObject), joint.connectedBody == null ? Entity.Null : connectedEntity, joint.enableCollision);
            constraints.Dispose();
        }

        void ConvertFixedJoint(LegacyFixed joint)
        {
            var legacyWorldFromJointA = math.mul(
                new RigidTransform(joint.transform.rotation, joint.transform.position),
                new RigidTransform(quaternion.identity, joint.anchor)
            );

            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            RigidTransform worldFromBodyB = connectedEntity == Entity.Null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var bodyAFromJoint = new JointFrame(math.mul(math.inverse(worldFromBodyA), legacyWorldFromJointA));
            var bodyBFromJoint = new JointFrame(math.mul(math.inverse(worldFromBodyB), legacyWorldFromJointA));

            var jointData = JointData.CreateFixed(bodyAFromJoint, bodyBFromJoint);

            CreateJointEntity(joint.gameObject, jointData, GetPrimaryEntity(joint.gameObject), joint.connectedBody == null ? Entity.Null : connectedEntity, joint.enableCollision);
        }

        void ConvertHingeJoint(LegacyHinge joint)
        {
            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            RigidTransform worldFromBodyB = joint.connectedBody == null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            Math.CalculatePerpendicularNormalized(joint.axis, out float3 perpendicularA, out _);
            var bodyAFromJoint = new JointFrame
            {
                Axis = joint.axis,
                PerpendicularAxis = perpendicularA,
                Position = joint.anchor
            };

            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;
           
            RigidTransform bFromA = isConnectedBodyConverted ? math.mul(math.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : worldFromBodyB;

            var bodyBFromJoint = new JointFrame
            {
                Axis = math.mul(bFromA.rot, joint.axis),
                PerpendicularAxis = math.mul(bFromA.rot, perpendicularA),
                Position = math.mul(bFromBSource, new float4(joint.connectedAnchor, 1f)).xyz
            };

            var limits = math.radians(new FloatRange(joint.limits.min, joint.limits.max));
            var jointData = joint.useLimits
                ? JointData.CreateLimitedHinge(bodyAFromJoint, bodyBFromJoint, limits)
                : JointData.CreateHinge(bodyAFromJoint, bodyBFromJoint);

            CreateJointEntity(joint.gameObject, jointData, GetPrimaryEntity(joint.gameObject), joint.connectedBody == null ? Entity.Null : connectedEntity, joint.enableCollision);
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((LegacyConfigurable joint) => { ConvertConfigurableJoint(joint); });
            Entities.ForEach((LegacyCharacter joint) => { ConvertCharacterJoint(joint); });
            Entities.ForEach((LegacySpring joint) => { ConvertSpringJoint(joint); });
            Entities.ForEach((LegacyFixed joint) => { ConvertFixedJoint(joint); });
            Entities.ForEach((LegacyHinge joint) => { ConvertHingeJoint(joint); });
        }
    }

}
#endif
