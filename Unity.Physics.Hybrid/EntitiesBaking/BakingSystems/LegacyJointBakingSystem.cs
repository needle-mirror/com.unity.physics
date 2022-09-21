#if LEGACY_PHYSICS
using System;
using System.Collections.Generic;
using System.ComponentModel;
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
    public abstract class BaseLegacyJointBaker<T> : Baker<T> where T : UnityEngine.Component
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion GetJointFrameOrientation(float3 axis, float3 secondaryAxis)
        {
            // classic Unity uses a different approach than BodyFrame.ValidateAxes() for ortho-normalizing degenerate inputs
            // ortho-normalizing here ensures behavior is consistent with classic Unity
            var a = (Vector3)axis;
            var p = (Vector3)secondaryAxis;
            Vector3.OrthoNormalize(ref a, ref p);
            return new BodyFrame { Axis = a, PerpendicularAxis = p }.AsRigidTransform().rot;
        }

        public bool3 GetAxesWithMotionType(
            ConfigurableJointMotion motionType,
            ConfigurableJointMotion x, ConfigurableJointMotion y, ConfigurableJointMotion z
        ) => new bool3(x == motionType, y == motionType, z == motionType);

        public float CalculateSpringFrequencyFromSpringConstant(float springConstant)
        {
            if (springConstant < Math.Constants.Eps) return 0.0f;

            return math.sqrt(springConstant) * Math.Constants.OneOverTau;
        }

        public PhysicsConstrainedBodyPair GetConstrainedBodyPair(LegacyJoint joint) =>
            new PhysicsConstrainedBodyPair(
                GetEntity(joint.gameObject),
                joint.connectedBody == null ? Entity.Null : GetEntity(joint.connectedBody),
                joint.enableCollision
            );

        public struct CombinedJoint
        {
            public PhysicsJoint LinearJoint;
            public PhysicsJoint AngularJoint;
        }

        public CombinedJoint CreateConfigurableJoint(
            quaternion jointFrameOrientation,
            LegacyJoint joint, bool3 linearLocks, bool3 linearLimited, SoftJointLimit linearLimit, SoftJointLimitSpring linearSpring, bool3 angularFree, bool3 angularLocks,
            bool3 angularLimited, SoftJointLimit lowAngularXLimit, SoftJointLimit highAngularXLimit, SoftJointLimitSpring angularXLimitSpring, SoftJointLimit angularYLimit,
            SoftJointLimit angularZLimit, SoftJointLimitSpring angularYZLimitSpring)
        {
            var angularConstraints = new FixedList512Bytes<Constraint>();
            var linearConstraints = new FixedList512Bytes<Constraint>();

            bool angularBreakable = joint.breakTorque == float.PositiveInfinity ? false : true;
            bool LinearBreakable = joint.breakForce == float.PositiveInfinity ? false : true;

            if (angularLimited[0])
            {
                Constraint constraint = Constraint.Twist(
                    0,
                    math.radians(new FloatRange(-highAngularXLimit.limit, -lowAngularXLimit.limit).Sorted()),
                    joint.breakTorque * Time.fixedDeltaTime,
                    CalculateSpringFrequencyFromSpringConstant(angularXLimitSpring.spring),
                    angularXLimitSpring.damper);
                constraint.EnableImpulseEvents = angularBreakable;
                angularConstraints.Add(constraint);
            }

            if (angularLimited[1])
            {
                Constraint constraint = Constraint.Twist(
                    1,
                    math.radians(new FloatRange(-angularYLimit.limit, angularYLimit.limit).Sorted()),
                    joint.breakTorque * Time.fixedDeltaTime,
                    CalculateSpringFrequencyFromSpringConstant(angularYZLimitSpring.spring),
                    angularYZLimitSpring.damper);

                constraint.EnableImpulseEvents = angularBreakable;
                angularConstraints.Add(constraint);
            }

            if (angularLimited[2])
            {
                Constraint constraint = Constraint.Twist(
                    2,
                    math.radians(new FloatRange(-angularZLimit.limit, angularZLimit.limit).Sorted()),
                    joint.breakTorque * Time.fixedDeltaTime,
                    CalculateSpringFrequencyFromSpringConstant(angularYZLimitSpring.spring),
                    angularYZLimitSpring.damper);

                constraint.EnableImpulseEvents = angularBreakable;
                angularConstraints.Add(constraint);
            }

            if (math.any(linearLimited))
            {
                //If spring=0, then need to treat it and damper as locked. Okay for damper=0 if spring>0
                var spring = Constraint.DefaultSpringFrequency; //stiff spring
                var damping = 1.0f; //critically damped
                if (linearSpring.spring > 0.0f)
                {
                    spring = linearSpring.spring;
                    damping = linearSpring.damper;
                }

                linearConstraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLimited,
                    Type = ConstraintType.Linear,
                    Min = 0f,
                    Max = linearLimit.limit,  //allow movement up to limit from anchor
                    SpringFrequency = CalculateSpringFrequencyFromSpringConstant(spring),
                    SpringDamping = damping,
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime,
                    EnableImpulseEvents = LinearBreakable
                });
            }

            if (math.any(linearLocks))
            {
                linearConstraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLocks,
                    Type = ConstraintType.Linear,
                    Min = linearLimit.limit,    //lock at distance from anchor
                    Max = linearLimit.limit,
                    SpringFrequency =  Constraint.DefaultSpringFrequency, //stiff spring
                    SpringDamping = 1.0f, //critically damped
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime,
                    EnableImpulseEvents = LinearBreakable
                });
            }

            if (math.any(angularLocks))
            {
                angularConstraints.Add(new Constraint
                {
                    ConstrainedAxes = angularLocks,
                    Type = ConstraintType.Angular,
                    Min = 0,
                    Max = 0,
                    SpringFrequency = Constraint.DefaultSpringFrequency, //stiff spring
                    SpringDamping = 1.0f, //critically damped
                    MaxImpulse = joint.breakTorque * Time.fixedDeltaTime,
                    EnableImpulseEvents = angularBreakable
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
            var bodyAFromJoint = new BodyFrame(math.mul(math.inverse(worldFromBodyA), legacyWorldFromJointA));

            var connectedEntity = GetEntity(joint.connectedBody);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromA = isConnectedBodyConverted ? math.mul(math.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : worldFromBodyB;

            var bodyBFromJoint = new BodyFrame
            {
                Axis = math.mul(bFromA.rot, bodyAFromJoint.Axis),
                PerpendicularAxis = math.mul(bFromA.rot, bodyAFromJoint.PerpendicularAxis),
                Position = math.mul(bFromBSource, new float4(joint.connectedAnchor, 1f)).xyz
            };

            var combinedJoint = new CombinedJoint();
            combinedJoint.LinearJoint.SetConstraints(linearConstraints);
            combinedJoint.LinearJoint.BodyAFromJoint = bodyAFromJoint;
            combinedJoint.LinearJoint.BodyBFromJoint = bodyBFromJoint;
            combinedJoint.AngularJoint.SetConstraints(angularConstraints);
            combinedJoint.AngularJoint.BodyAFromJoint = bodyAFromJoint;
            combinedJoint.AngularJoint.BodyBFromJoint = bodyBFromJoint;

            return combinedJoint;
        }

        public uint GetWorldIndex(UnityEngine.Component c)
        {
            uint worldIndex = 0;
            var physicsBody = GetComponent<PhysicsBodyAuthoring>(c);
            if (physicsBody != null)
            {
                worldIndex = physicsBody.WorldIndex;
            }
            return worldIndex;
        }

        public Entity CreateJointEntity(uint worldIndex, PhysicsConstrainedBodyPair constrainedBodyPair, PhysicsJoint joint)
        {
            using (var joints = new NativeArray<PhysicsJoint>(1, Allocator.Temp) { [0] = joint })
            using (var jointEntities = new NativeList<Entity>(1, Allocator.Temp))
            {
                CreateJointEntities(worldIndex, constrainedBodyPair, joints, jointEntities);
                return jointEntities[0];
            }
        }

        public void CreateJointEntities(uint worldIndex, PhysicsConstrainedBodyPair constrainedBodyPair, NativeArray<PhysicsJoint> joints, NativeList<Entity> newJointEntities = default)
        {
            if (!joints.IsCreated || joints.Length == 0)
                return;

            if (newJointEntities.IsCreated)
                newJointEntities.Clear();
            else
                newJointEntities = new NativeList<Entity>(joints.Length, Allocator.Temp);

            // create all new joints
            var multipleJoints = joints.Length > 1;

            // TODO: BAKING - WE CAN'T GET THE NAME AT THE MOMENT IN BAKING, SHOULD WE ALLOW IT?
/*
#if UNITY_EDITOR
            var nameEntityA = DstEntityManager.GetName(constrainedBodyPair.EntityA);
            var nameEntityB = constrainedBodyPair.EntityB == Entity.Null
                ? "PhysicsWorld"
                : DstEntityManager.GetName(constrainedBodyPair.EntityB);
            var baseName = $"Joining {nameEntityA} + {nameEntityB}";
#endif*/

            var entity = GetEntity();
            for (var i = 0; i < joints.Length; ++i)
            {
                var jointEntity = CreateAdditionalEntity();
                AddSharedComponent(jointEntity, new PhysicsWorldIndex(worldIndex));

                // TODO: BAKING - This is not supported in baking yet
/*
#if UNITY_EDITOR
                DstEntityManager.SetName(jointEntity, $"{baseName} ({joints[i].JointType})");
#endif*/

                AddComponent(jointEntity, constrainedBodyPair);
                AddComponent(jointEntity, joints[i]);

                newJointEntities.Add(jointEntity);
            }

            if (multipleJoints)
            {
                // set companion buffers for new joints
                for (var i = 0; i < joints.Length; ++i)
                {
                    var companions = AddBuffer<PhysicsJointCompanion>(newJointEntities[i]);
                    for (var j = 0; j < joints.Length; ++j)
                    {
                        if (i == j)
                            continue;
                        companions.Add(new PhysicsJointCompanion {JointEntity = newJointEntities[j]});
                    }
                }
            }
        }
    }

    class LegacyCharacterBaker : BaseLegacyJointBaker<LegacyCharacter>
    {
        void ConvertCharacterJoint(LegacyCharacter joint)
        {
            var linearLocks = new bool3(true);
            var linearLimited = new bool3(false);

            var angularFree = new bool3(false);
            var angularLocks = new bool3(false);
            var angularLimited = new bool3(true);

            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.swingAxis);
            var jointData = CreateConfigurableJoint(jointFrameOrientation, joint, linearLocks, linearLimited,
                new SoftJointLimit { limit = 0f, bounciness = 0f },
                new SoftJointLimitSpring { spring = 0f, damper = 0f },
                angularFree, angularLocks, angularLimited,
                joint.lowTwistLimit, joint.highTwistLimit, joint.twistLimitSpring,
                joint.swing1Limit, joint.swing2Limit, joint.swingLimitSpring);

            var worldIndex = GetWorldIndex(joint);
            using (var joints = new NativeArray<PhysicsJoint>(2, Allocator.Temp) { [0] = jointData.LinearJoint, [1] = jointData.AngularJoint })
            using (var jointEntities = new NativeList<Entity>(2, Allocator.Temp))
            {
                CreateJointEntities(worldIndex, GetConstrainedBodyPair(joint), joints, jointEntities);
            }
        }

        public override void Bake(LegacyCharacter authoring)
        {
            ConvertCharacterJoint(authoring);
        }
    }

    class LegacyConfigurableBaker : BaseLegacyJointBaker<LegacyConfigurable>
    {
        void ConvertConfigurableJoint(LegacyConfigurable joint)
        {
            var linearLocks =
                GetAxesWithMotionType(ConfigurableJointMotion.Locked, joint.xMotion, joint.yMotion, joint.zMotion);
            var linearLimited =
                GetAxesWithMotionType(ConfigurableJointMotion.Limited, joint.xMotion, joint.yMotion, joint.zMotion);
            var angularFree =
                GetAxesWithMotionType(ConfigurableJointMotion.Free, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);
            var angularLocks =
                GetAxesWithMotionType(ConfigurableJointMotion.Locked, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);
            var angularLimited =
                GetAxesWithMotionType(ConfigurableJointMotion.Limited, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);

            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.secondaryAxis);
            var jointData = CreateConfigurableJoint(jointFrameOrientation, joint, linearLocks, linearLimited,
                joint.linearLimit, joint.linearLimitSpring, angularFree, angularLocks, angularLimited,
                joint.lowAngularXLimit, joint.highAngularXLimit, joint.angularXLimitSpring, joint.angularYLimit,
                joint.angularZLimit, joint.angularYZLimitSpring);

            var worldIndex = GetWorldIndex(joint);
            using (var joints = new NativeArray<PhysicsJoint>(2, Allocator.Temp) { [0] = jointData.LinearJoint, [1] = jointData.AngularJoint })
            using (var jointEntities = new NativeList<Entity>(2, Allocator.Temp))
            {
                CreateJointEntities(worldIndex, GetConstrainedBodyPair(joint), joints, jointEntities);
            }
        }

        public override void Bake(LegacyConfigurable authoring)
        {
            ConvertConfigurableJoint(authoring);
        }
    }

    class LegacyFixedBaker : BaseLegacyJointBaker<LegacyFixed>
    {
        void ConvertFixedJoint(LegacyFixed joint)
        {
            var legacyWorldFromJointA = math.mul(
                new RigidTransform(joint.transform.rotation, joint.transform.position),
                new RigidTransform(quaternion.identity, joint.anchor)
            );

            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            var connectedEntity = GetEntity(joint.connectedBody);
            RigidTransform worldFromBodyB = connectedEntity == Entity.Null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var bodyAFromJoint = new BodyFrame(math.mul(math.inverse(worldFromBodyA), legacyWorldFromJointA));
            var bodyBFromJoint = new BodyFrame(math.mul(math.inverse(worldFromBodyB), legacyWorldFromJointA));

            var jointData = PhysicsJoint.CreateFixed(bodyAFromJoint, bodyBFromJoint);

            bool LinearBreakable = joint.breakForce == float.PositiveInfinity ? false : true;
            bool angularBreakable = joint.breakTorque == float.PositiveInfinity ? false : true;
            var constraints = jointData.GetConstraints();
            for (int i = 0; i < constraints.Length; ++i)
            {
                ref Constraint constraint = ref constraints.ElementAt(i);
                switch (constraint.Type)
                {
                    case ConstraintType.Linear:
                        constraint.MaxImpulse = joint.breakForce * Time.fixedDeltaTime;
                        constraint.EnableImpulseEvents = LinearBreakable;
                        break;
                    case ConstraintType.Angular:
                        constraint.MaxImpulse = joint.breakTorque * Time.fixedDeltaTime;
                        constraint.EnableImpulseEvents = angularBreakable;
                        break;
                }
            }
            jointData.SetConstraints(constraints);

            var worldIndex = GetWorldIndex(joint);
            Entity entity = CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), jointData);
        }

        public override void Bake(LegacyFixed authoring)
        {
            ConvertFixedJoint(authoring);
        }
    }

    class LegacyHingeBaker : BaseLegacyJointBaker<LegacyHinge>
    {
        void ConvertHingeJoint(LegacyHinge joint)
        {
            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            RigidTransform worldFromBodyB = joint.connectedBody == null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            Math.CalculatePerpendicularNormalized(joint.axis, out float3 perpendicularA, out _);
            var bodyAFromJoint = new BodyFrame
            {
                Axis = joint.axis,
                PerpendicularAxis = perpendicularA,
                Position = joint.anchor
            };

            var connectedEntity = GetEntity(joint.connectedBody);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromA = isConnectedBodyConverted ? math.mul(math.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : worldFromBodyB;

            var bodyBFromJoint = new BodyFrame
            {
                Axis = math.mul(bFromA.rot, joint.axis),
                PerpendicularAxis = math.mul(bFromA.rot, perpendicularA),
                Position = math.mul(bFromBSource, new float4(joint.connectedAnchor, 1f)).xyz
            };

            var limits = math.radians(new FloatRange(joint.limits.min, joint.limits.max).Sorted());
            var jointData = joint.useLimits
                ? PhysicsJoint.CreateLimitedHinge(bodyAFromJoint, bodyBFromJoint, limits)
                : PhysicsJoint.CreateHinge(bodyAFromJoint, bodyBFromJoint);

            bool LinearBreakable = joint.breakForce == float.PositiveInfinity ? false : true;
            bool angularBreakable = joint.breakTorque == float.PositiveInfinity ? false : true;
            var constraints = jointData.GetConstraints();
            for (int i = 0; i < constraints.Length; ++i)
            {
                ref Constraint constraint = ref constraints.ElementAt(i);
                switch (constraint.Type)
                {
                    case ConstraintType.Linear:
                        constraint.MaxImpulse = joint.breakForce * Time.fixedDeltaTime;
                        constraint.EnableImpulseEvents = LinearBreakable;
                        break;
                    case ConstraintType.Angular:
                        constraint.MaxImpulse = joint.breakTorque * Time.fixedDeltaTime;
                        constraint.EnableImpulseEvents = angularBreakable;
                        break;
                }
            }
            jointData.SetConstraints(constraints);

            var worldIndex = GetWorldIndex(joint);
            Entity entity = CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), jointData);
        }

        public override void Bake(LegacyHinge authoring)
        {
            ConvertHingeJoint(authoring);
        }
    }

    class LegacySpringBaker : BaseLegacyJointBaker<LegacySpring>
    {
        void ConvertSpringJoint(LegacySpring joint)
        {
            var distanceRange = new FloatRange(joint.minDistance, joint.maxDistance).Sorted();
            var constraint = new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = CalculateSpringFrequencyFromSpringConstant(joint.spring),
                SpringDamping = joint.damper,
                MaxImpulse = joint.breakForce * Time.fixedDeltaTime,
                EnableImpulseEvents =  Single.IsPositiveInfinity(joint.breakForce) ? false : true
            };

            var jointFrameA = BodyFrame.Identity;
            jointFrameA.Position = joint.anchor;

            var connectedEntity = GetEntity(joint.connectedBody);

            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var jointFrameB = BodyFrame.Identity;
            jointFrameB.Position = math.mul(bFromBSource, new float4(joint.connectedAnchor, 1f)).xyz;

            var jointData = new PhysicsJoint
            {
                BodyAFromJoint = jointFrameA,
                BodyBFromJoint = jointFrameB
            };
            jointData.SetConstraints(new FixedList512Bytes<Constraint>
            {
                Length = 1,
                [0] = constraint
            });

            var worldIndex = GetWorldIndex(joint);
            Entity entity = CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), jointData);
        }

        public override void Bake(LegacySpring authoring)
        {
            ConvertSpringJoint(authoring);
        }
    }
}
#endif
