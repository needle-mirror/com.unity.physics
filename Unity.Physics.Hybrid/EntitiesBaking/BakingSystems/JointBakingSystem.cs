using System.Runtime.CompilerServices;
using UnityEngine.Assertions;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using FloatRange = Unity.Physics.Math.FloatRange;

namespace Unity.Physics.Authoring
{
    internal abstract class BaseJointBaker<T> : Baker<T> where T : UnityEngine.Component
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static float3 GetScaledLocalAnchorPosition(in Transform bodyTransform, in float3 anchorPosition)
        {
            // account for scale in the body transform if any
            var kScaleEpsilon = 0.0001f;
            if (math.lengthsq((float3)bodyTransform.lossyScale - new float3(1f)) > kScaleEpsilon)
            {
                var localToWorld = bodyTransform.localToWorldMatrix;
                var rigidBodyTransform = Math.DecomposeRigidBodyTransform(localToWorld);

                // extract local skew matrix if non-identity world scale detected to re-position the local joint anchor position
                var skewMatrix = math.mul(math.inverse(new float4x4(rigidBodyTransform)), localToWorld);
                return math.mul(skewMatrix, new float4(anchorPosition, 1)).xyz;
            }
            // else:

            return anchorPosition;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static quaternion GetJointFrameOrientation(float3 axis, float3 secondaryAxis)
        {
            // classic Unity uses a different approach than BodyFrame.ValidateAxes() for ortho-normalizing degenerate inputs
            // ortho-normalizing here ensures behavior is consistent with classic Unity
            var a = (Vector3)axis;
            var p = (Vector3)secondaryAxis;
            Vector3.OrthoNormalize(ref a, ref p);
            return new BodyFrame { Axis = a, PerpendicularAxis = p }.AsRigidTransform().rot;
        }

        protected bool3 GetAxesWithMotionType(
            ConfigurableJointMotion motionType,
            ConfigurableJointMotion x, ConfigurableJointMotion y, ConfigurableJointMotion z
        ) => new bool3(x == motionType, y == motionType, z == motionType);

        protected void ConvertSpringDamperSettings(float inSpringConstant, float inDampingCoefficient, float inConstrainedMass,
            out float outSpringFrequency, out float outDampingRatio)
        {
            const float threshold = 0.001f;

            if (inSpringConstant <= threshold)
            {
                // Case: k=0, c=0: Want a stiff constraint
                // Case: k=0, c!=0: Velocity motor case. Use damping coefficient as if it as a ratio (an approximation making it easier to tune)
                outSpringFrequency = Constraint.DefaultSpringFrequency;
                outDampingRatio = inDampingCoefficient <= threshold ?
                    Constraint.DefaultDampingRatio : inDampingCoefficient;
            }
            else
            {
                // Case: k!=0, c=0: Calculate for k and use damping ratio as 0
                // Case: k!=0, c!=0: Calculate both terms
                outSpringFrequency = JacobianUtilities.CalculateSpringFrequencyFromSpringConstant(inSpringConstant, inConstrainedMass);
                outDampingRatio = inDampingCoefficient <= threshold ?
                    0.0f : JacobianUtilities.CalculateDampingRatio(inSpringConstant, inDampingCoefficient, inConstrainedMass);
            }
        }

        /// <summary>
        /// Calculates the spring constant from the spring frequency and mass for Unity Physics -> Built-in conversion
        /// k = (2pi * f)^2 * m
        /// - Built-in uses spring as a spring constant
        /// - Unity Physics uses spring as a spring frequency
        /// </summary>
        /// <param name="springFrequency"></param>
        /// <param name="mass"></param>
        /// <returns></returns>
        protected float CalculateSpringConstantFromSpringFrequency(float springFrequency, float mass = 1.0f)
        {
            if (springFrequency < Math.Constants.Eps) return 0.0f;

            var tauXFrequency = Math.Constants.Tau * springFrequency;
            return tauXFrequency * tauXFrequency * mass;
        }

        protected PhysicsConstrainedBodyPair GetConstrainedBodyPair(in UnityEngine.Joint joint) =>
            new PhysicsConstrainedBodyPair(
                GetEntity(joint.gameObject, TransformUsageFlags.Dynamic),
                joint.connectedBody == null ? Entity.Null : GetEntity(joint.connectedBody, TransformUsageFlags.Dynamic),
                joint.enableCollision
            );

        protected float GetConstrainedBodyMass(in UnityEngine.Joint joint)
        {
            var rigidBody = GetComponent<Rigidbody>(joint.gameObject);
            var connectedBody = joint.connectedBody;
            float mass = 0;

            if (rigidBody != null && !rigidBody.isKinematic)
            {
                mass += rigidBody.mass;
            }

            if (connectedBody != null && !connectedBody.isKinematic)
            {
                mass += connectedBody.mass;
            }

            return mass > 0 ? mass : 1.0f;
        }

        protected float GetConstrainedBodyInertia(in UnityEngine.Joint joint)
        {
            var rigidBody = GetComponent<Rigidbody>(joint.gameObject);
            var connectedBody = joint.connectedBody;
            float approxInertia = 0;

            if (rigidBody != null && !rigidBody.isKinematic)
            {
                approxInertia += 1f / 3f * (rigidBody.inertiaTensor.x + rigidBody.inertiaTensor.y + rigidBody.inertiaTensor.z);
            }

            if (connectedBody != null && !connectedBody.isKinematic)
            {
                approxInertia += 1f / 3f * (connectedBody.inertiaTensor.x + connectedBody.inertiaTensor.y + connectedBody.inertiaTensor.z);
            }

            return approxInertia > 0 ? approxInertia : 1.0f;
        }

        protected struct CombinedJoint
        {
            public PhysicsJoint LinearJoint;
            public PhysicsJoint AngularJoint;
        }

        protected void SetupBodyFrames(quaternion jointFrameOrientation, UnityEngine.Joint joint, ref BodyFrame bodyAFromJoint, ref BodyFrame bodyBFromJoint)
        {
            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            RigidTransform worldFromBodyB = joint.connectedBody == null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var anchorPos = GetScaledLocalAnchorPosition(joint.transform, joint.anchor);
            var worldFromJointA = math.mul(
                new RigidTransform(joint.transform.rotation, joint.transform.position),
                new RigidTransform(jointFrameOrientation, anchorPos)
            );
            bodyAFromJoint = new BodyFrame(math.mul(math.inverse(worldFromBodyA), worldFromJointA));

            var connectedEntity = GetEntity(joint.connectedBody, TransformUsageFlags.Dynamic);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromA = isConnectedBodyConverted ? math.mul(math.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : worldFromBodyB;

            float3 connectedAnchorPos = joint.connectedBody
                ? GetScaledLocalAnchorPosition(joint.connectedBody.transform, joint.connectedAnchor)
                : joint.connectedAnchor;

            float3 bJointPosition = math.mul(bFromBSource, new float4(connectedAnchorPos, 1f)).xyz;
#if UNITY_EDITOR
            bool IsPrefabAsset = UnityEditor.PrefabUtility.GetPrefabAssetType(joint) != UnityEditor.PrefabAssetType.NotAPrefab &&
                UnityEditor.PrefabUtility.GetPrefabInstanceStatus(joint) == UnityEditor.PrefabInstanceStatus.NotAPrefab;

            // This correction applies only to prefabs referenced by other Authoring/Baker scripts,
            // which have not yet been instantiated in the scene. Since {joint.connectedAnchor}
            // is not correctly initialized at this point, the Joint Position shown in the Inspector
            // may appear incorrect for these prefabs.
            if (IsPrefabAsset && joint.autoConfigureConnectedAnchor)
            {
                // Resolve the connected transform if one exists
                Transform connectedTransform = null;
                if (joint.connectedBody != null)
                    connectedTransform = joint.connectedBody.transform;
                else if (joint.connectedArticulationBody != null)
                    connectedTransform = joint.connectedArticulationBody.transform;

                // Correct the joint position to account for prefab initialization state
                if (connectedTransform != null)
                {
                    float3 localPos = math.mul(math.inverse(joint.transform.rotation), connectedTransform.position - joint.transform.position);
                    localPos -= anchorPos;
                    localPos *= -1.0f; // Flip the result to match the expected local direction
                    bJointPosition = localPos;
                }
            }
#endif

            bodyBFromJoint = new BodyFrame
            {
                Axis = math.mul(bFromA.rot, bodyAFromJoint.Axis),
                PerpendicularAxis = math.mul(bFromA.rot, bodyAFromJoint.PerpendicularAxis),
                Position = bJointPosition
            };
        }

        protected CombinedJoint CreateConfigurableJoint(
            quaternion jointFrameOrientation,
            UnityEngine.Joint joint, bool3 linearLocks, bool3 linearLimited, SoftJointLimit linearLimit, SoftJointLimitSpring linearSpring, bool3 angularFree, bool3 angularLocks,
            bool3 angularLimited, SoftJointLimit lowAngularXLimit, SoftJointLimit highAngularXLimit, SoftJointLimitSpring angularXLimitSpring, SoftJointLimit angularYLimit,
            SoftJointLimit angularZLimit, SoftJointLimitSpring angularYZLimitSpring)
        {
            var angularConstraints = new FixedList512Bytes<Constraint>();
            var linearConstraints = new FixedList512Bytes<Constraint>();

            var constrainedMass = GetConstrainedBodyMass(joint);

            if (angularLimited[0])
            {
                ConvertSpringDamperSettings(angularXLimitSpring.spring, angularXLimitSpring.damper, constrainedMass, out var springFrequency, out var dampingRatio);

                Constraint constraint = Constraint.Twist(
                    0,
                    math.radians(new FloatRange(-highAngularXLimit.limit, -lowAngularXLimit.limit).Sorted()),
                    joint.breakTorque * Time.fixedDeltaTime,
                    springFrequency,
                    dampingRatio);
                angularConstraints.Add(constraint);
            }

            if (angularLimited[1])
            {
                ConvertSpringDamperSettings(angularYZLimitSpring.spring, angularYZLimitSpring.damper, constrainedMass, out var springFrequency, out var dampingRatio);

                Constraint constraint = Constraint.Twist(
                    1,
                    math.radians(new FloatRange(-angularYLimit.limit, angularYLimit.limit).Sorted()),
                    joint.breakTorque * Time.fixedDeltaTime,
                    springFrequency,
                    dampingRatio);

                angularConstraints.Add(constraint);
            }

            if (angularLimited[2])
            {
                ConvertSpringDamperSettings(angularYZLimitSpring.spring, angularYZLimitSpring.damper, constrainedMass, out var springFrequency, out var dampingRatio);

                Constraint constraint = Constraint.Twist(
                    2,
                    math.radians(new FloatRange(-angularZLimit.limit, angularZLimit.limit).Sorted()),
                    joint.breakTorque * Time.fixedDeltaTime,
                    springFrequency,
                    dampingRatio);

                angularConstraints.Add(constraint);
            }

            if (math.any(linearLimited))
            {
                ConvertSpringDamperSettings(linearSpring.spring, linearSpring.damper, constrainedMass, out var springFrequency, out var dampingRatio);

                linearConstraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLimited,
                    Type = ConstraintType.Linear,
                    Min = 0f,
                    Max = linearLimit.limit,  // allow movement up to limit from anchor
                    SpringFrequency = springFrequency,
                    DampingRatio = dampingRatio,
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime,
                });
            }

            if (math.any(linearLocks))
            {
                linearConstraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLocks,
                    Type = ConstraintType.Linear,
                    Min = linearLimit.limit,    // lock at distance from anchor
                    Max = linearLimit.limit,
                    SpringFrequency = Constraint.DefaultSpringFrequency, // default spring-damper (stiff)
                    DampingRatio = Constraint.DefaultDampingRatio, // default spring-damper (stiff)
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime,
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
                    SpringFrequency = Constraint.DefaultSpringFrequency, // default spring-damper (stiff)
                    DampingRatio = Constraint.DefaultDampingRatio, // default spring-damper (stiff)
                    MaxImpulse = joint.breakTorque * Time.fixedDeltaTime,
                });
            }
            var bodyAFromJoint = new BodyFrame {};
            var bodyBFromJoint = new BodyFrame {};
            SetupBodyFrames(jointFrameOrientation, joint, ref bodyAFromJoint, ref bodyBFromJoint);

            var combinedJoint = new CombinedJoint();
            combinedJoint.LinearJoint.SetConstraints(linearConstraints);
            combinedJoint.LinearJoint.BodyAFromJoint = bodyAFromJoint;
            combinedJoint.LinearJoint.BodyBFromJoint = bodyBFromJoint;
            combinedJoint.AngularJoint.SetConstraints(angularConstraints);
            combinedJoint.AngularJoint.BodyAFromJoint = bodyAFromJoint;
            combinedJoint.AngularJoint.BodyBFromJoint = bodyBFromJoint;

            return combinedJoint;
        }

        protected PhysicsJoint ConvertMotor(quaternion jointFrameOrientation, UnityEngine.Joint joint, JacobianType motorType,
            float target, float maxImpulseForMotor,
            float springFrequencyIn = Constraint.DefaultSpringFrequency, float springDampingIn = Constraint.DefaultDampingRatio)
        {
            var bodyAFromJoint = new BodyFrame {};
            var bodyBFromJoint = new BodyFrame {};
            SetupBodyFrames(jointFrameOrientation, joint, ref bodyAFromJoint, ref bodyBFromJoint);

            // In PhysX Configurable Joint:
            // X Motion and Angular X Motion: applied to Axis (not necessarily the x-axis)
            // Y Motion and Angular Y Motion: applied to Secondary Axis (not necessarily the y-axis)
            // Z Motion and Angular Z Motion: applied to axis perpendicular to Axis & Secondary Axis
            // Therefore when setting Target fields in PhysX, the Target.x is aligned to whatever the Axis field is set
            // and Target.y is aligned to the Secondary Axis > need to convert the target to be aligns to Axis
            // Targets are set local to bodyA as the offset of the final target relative to  the starting bodyA position
            var jointData = new PhysicsJoint();

            var constrainedMass = GetConstrainedBodyMass(joint);
            ConvertSpringDamperSettings(springFrequencyIn, springDampingIn, constrainedMass,
                out var springFrequencyOut, out var springDampingOut);

            switch (motorType)
            {
                case JacobianType.PositionMotor:
                    jointData = PhysicsJoint.CreatePositionMotor(bodyAFromJoint, bodyBFromJoint, target,
                        maxImpulseForMotor, springFrequencyOut, springDampingOut);
                    break;
                case JacobianType.LinearVelocityMotor:
                    jointData = PhysicsJoint.CreateLinearVelocityMotor(bodyAFromJoint, bodyBFromJoint, target,
                        maxImpulseForMotor, springFrequencyOut, springDampingOut);
                    break;
                case JacobianType.RotationMotor:
                    jointData = PhysicsJoint.CreateRotationalMotor(bodyAFromJoint, bodyBFromJoint, target,
                        maxImpulseForMotor, springFrequencyOut, springDampingOut);
                    break;
                case JacobianType.AngularVelocityMotor:
                    jointData = PhysicsJoint.CreateAngularVelocityMotor(bodyAFromJoint, bodyBFromJoint, target,
                        maxImpulseForMotor, springFrequencyOut, springDampingOut);
                    break;
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    break;
            }

            return jointData;
        }

        protected uint GetWorldIndex(UnityEngine.Component c)
        {
            // World Indices are not supported in current built-in physics implementation, which makes it unavailable with legacy baking.
            return 0;
        }

        protected Entity CreateJointEntity(uint worldIndex, PhysicsConstrainedBodyPair constrainedBodyPair, PhysicsJoint joint)
        {
            using (var joints = new NativeArray<PhysicsJoint>(1, Allocator.Temp) { [0] = joint })
            using (var jointEntities = new NativeList<Entity>(1, Allocator.Temp))
            {
                CreateJointEntities(worldIndex, constrainedBodyPair, joints, jointEntities);
                return jointEntities[0];
            }
        }

        protected void CreateJointEntities(uint worldIndex, PhysicsConstrainedBodyPair constrainedBodyPair, NativeArray<PhysicsJoint> joints, NativeList<Entity> newJointEntities = default)
        {
            if (!joints.IsCreated || joints.Length == 0)
                return;

            if (newJointEntities.IsCreated)
                newJointEntities.Clear();
            else
                newJointEntities = new NativeList<Entity>(joints.Length, Allocator.Temp);

            // create all new joints
            var multipleJoints = joints.Length > 1;

            foreach (var joint in joints)
            {
                var jointEntity = CreateAdditionalEntity(TransformUsageFlags.Dynamic);
                AddSharedComponent(jointEntity, new PhysicsWorldIndex(worldIndex));

                AddComponent(jointEntity, constrainedBodyPair);
                AddComponent(jointEntity, joint);

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

    class CharacterBaker : BaseJointBaker<CharacterJoint>
    {
        void ConvertCharacterJoint(CharacterJoint joint)
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

        public override void Bake(CharacterJoint authoring)
        {
            ConvertCharacterJoint(authoring);
        }
    }

    class ConfigurableBaker : BaseJointBaker<ConfigurableJoint>
    {
        private void CreateJointEntityBlocks(UnityEngine.ConfigurableJoint joint, quaternion jointFrameOrientation, FixedList512Bytes<Constraint> constraints)
        {
            if (constraints.IsEmpty)
                return; // don't create empty joints

            var bodyAFromJoint = new BodyFrame {};
            var bodyBFromJoint = new BodyFrame {};
            SetupBodyFrames(jointFrameOrientation, joint, ref bodyAFromJoint, ref bodyBFromJoint);

            var block = new FixedList512Bytes<Constraint>();

            // A block can have up to three constraints, so that's why we're grouping in three.
            for (int i = 0; i < constraints.Length; i += 3)
            {
                block.Clear();

                int countInBlock = math.min(constraints.Length - i, 3);

                for (int j = i; j < i + countInBlock; ++j)
                    block.Add(constraints[j]);

                var thisJoint = new PhysicsJoint();
                thisJoint.SetConstraints(block);
                thisJoint.BodyAFromJoint = bodyAFromJoint;
                thisJoint.BodyBFromJoint = bodyBFromJoint;

                thisJoint.SetImpulseEventThresholdAllConstraints(
                    joint.breakForce * Time.fixedDeltaTime,
                    joint.breakTorque * Time.fixedDeltaTime
                );

                uint worldIndex = GetWorldIndex(joint);
                CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), thisJoint);
            }
        }

        void ConvertLinearDofs(UnityEngine.ConfigurableJoint joint, float constrainedMass, quaternion jointFrameOrientation, ref FixedList512Bytes<Constraint> constraints)
        {
            var linearLocks = GetAxesWithMotionType(ConfigurableJointMotion.Locked, joint.xMotion, joint.yMotion, joint.zMotion);
            var linearLimited = GetAxesWithMotionType(ConfigurableJointMotion.Limited, joint.xMotion, joint.yMotion, joint.zMotion);

            IdentifyIfLinearMotor(joint, out bool3 isPositionMotor, out bool3 isLinearVelocityMotor, out float3 maxForceForLinearMotor);

            Assert.IsTrue(math.csum(new int3(isLinearVelocityMotor)) <= 1, $"Unity.Physics doesn't fully support double and triple motorization of linear velocity at the moment, only one of those motors will work, game object {joint.name}");

            if (math.any(linearLocks))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLocks,
                    Type = ConstraintType.Linear,
                    Min = 0, // if it's locked, then it's locked at zero
                    Max = 0,
                    SpringFrequency = Constraint.DefaultSpringFrequency,
                    DampingRatio = Constraint.DefaultDampingRatio,
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime
                });
            }

            if (math.any(linearLimited))
            {
                ConvertSpringDamperSettings(joint.linearLimitSpring.spring, joint.linearLimitSpring.damper, constrainedMass, out float springFrequency, out float dampingRatio);

                constraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLimited,
                    Type = ConstraintType.Linear,
                    Min = 0,
                    Max = joint.linearLimit.limit,
                    SpringFrequency = springFrequency,
                    DampingRatio = dampingRatio,
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime
                });
            }

            float3 linearStiffness = new float3(joint.xDrive.positionSpring, joint.yDrive.positionSpring, joint.zDrive.positionSpring);
            float3 linearDamping = new float3(joint.xDrive.positionDamper, joint.yDrive.positionDamper, joint.zDrive.positionDamper);

            for (int axis = 0; axis < 3; ++axis)
            {
                if (!isPositionMotor[axis] && !isLinearVelocityMotor[axis])
                    continue; // skip axis if no position and no velocity drive either

                if (linearLocks[axis])
                    continue; // skip if this axis is locked

                ConvertSpringDamperSettings(linearStiffness[axis], linearDamping[axis], constrainedMass, out float motorFrequency, out float motorDampingRatio);

                constraints.Add(new Constraint
                {
                    ConstrainedAxes = new bool3(axis == 0, axis == 1, axis == 2),
                    Type = isPositionMotor[axis] ? ConstraintType.PositionMotor : ConstraintType.LinearVelocityMotor,
                    Min = -math.INFINITY, // looks like the limits should be enforced by the limit spring instead of the hard limit here
                    Max = math.INFINITY,
                    SpringFrequency = motorFrequency,
                    DampingRatio = motorDampingRatio,
                    MaxImpulse = maxForceForLinearMotor * Time.fixedDeltaTime,
                    Target = isPositionMotor[axis] ? -joint.targetPosition : -joint.targetVelocity,
                });
            }
        }

        void ConvertAngularDofs(UnityEngine.ConfigurableJoint joint, float constrainedMass, quaternion jointFrameOrientation, ref FixedList512Bytes<Constraint> constraints)
        {
            var angularLocks = GetAxesWithMotionType(ConfigurableJointMotion.Locked, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);
            var angularLimited = GetAxesWithMotionType(ConfigurableJointMotion.Limited, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);

            IdentifyIfAngularMotor(joint, out bool3 isRotationMotor, out bool3 isAngularVelocityMotor, out float3 maxForceForAngularMotor);

            Assert.IsTrue(joint.rotationDriveMode != RotationDriveMode.Slerp, $"Slerp drive mode is not supported by the conversion at the moment, defaulting to twist and swing instead, game object {joint.name}");

            if (math.any(angularLocks))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = angularLocks,
                    Type = ConstraintType.Angular,
                    Min = 0, // if it's locked, then it's locked at zero
                    Max = 0,
                    SpringFrequency = Constraint.DefaultSpringFrequency,
                    DampingRatio = Constraint.DefaultDampingRatio,
                    MaxImpulse = joint.breakForce * Time.fixedDeltaTime
                });
            }

            if (math.any(angularLimited))
            {
                // have to do angular limits per axis, unfortunately, because of the twist-swing thing that is asymmetric
                if (angularLimited.x)
                {
                    ConvertSpringDamperSettings(joint.angularXLimitSpring.spring, joint.angularXLimitSpring.damper, constrainedMass, out float springFrequencyX, out float dampingRatioX);

                    constraints.Add(new Constraint
                    {
                        ConstrainedAxes = new bool3(true, false, false),
                        Type = ConstraintType.Angular,
                        Min = -math.radians(joint.highAngularXLimit.limit),
                        Max = -math.radians(joint.lowAngularXLimit.limit),
                        SpringFrequency = springFrequencyX,
                        DampingRatio = dampingRatioX,
                        MaxImpulse = joint.breakForce * Time.fixedDeltaTime
                    });
                }

                if (angularLimited.y)
                {
                    ConvertSpringDamperSettings(joint.angularYZLimitSpring.spring, joint.angularYZLimitSpring.damper, constrainedMass, out float springFrequencyYZ, out float dampingRatioYZ);

                    constraints.Add(new Constraint
                    {
                        ConstrainedAxes = new bool3(false, true, false),
                        Type = ConstraintType.Angular,
                        Min = -math.radians(joint.angularYLimit.limit),
                        Max = math.radians(joint.angularYLimit.limit),
                        SpringFrequency = springFrequencyYZ,
                        DampingRatio = dampingRatioYZ,
                        MaxImpulse = joint.breakForce * Time.fixedDeltaTime
                    });
                }

                if (angularLimited.z)
                {
                    ConvertSpringDamperSettings(joint.angularYZLimitSpring.spring, joint.angularYZLimitSpring.damper, constrainedMass, out float springFrequencyYZ, out float dampingRatioYZ);

                    constraints.Add(new Constraint
                    {
                        ConstrainedAxes = new bool3(false, false, true),
                        Type = ConstraintType.Angular,
                        Min = -math.radians(joint.angularZLimit.limit),
                        Max = math.radians(joint.angularZLimit.limit),
                        SpringFrequency = springFrequencyYZ,
                        DampingRatio = dampingRatioYZ,
                        MaxImpulse = joint.breakForce * Time.fixedDeltaTime
                    });
                }
            }

            // the above branch creates up to three constraints, despite having constraint creation in four spots
            // proof: assume an axis is locked, then it can't be limited at the same time

            float3 angularStiffness = new float3(joint.angularXDrive.positionSpring, joint.angularYZDrive.positionSpring, joint.angularYZDrive.positionSpring);
            float3 angularDamping = new float3(joint.angularXDrive.positionDamper, joint.angularYZDrive.positionDamper, joint.angularYZDrive.positionDamper);

            for (int axis = 0; axis < 3; ++axis)
            {
                if (!isRotationMotor[axis] && !isAngularVelocityMotor[axis])
                    continue; // skip axis if no position and no velocity drive either

                if (angularLocks[axis])
                    continue; // skip if this axis is locked

                ConvertSpringDamperSettings(angularStiffness[axis], angularDamping[axis], constrainedMass, out float motorFrequency, out float motorDampingRatio);

                constraints.Add(new Constraint
                {
                    ConstrainedAxes = new bool3(axis == 0, axis == 1, axis == 2),
                    Type = isRotationMotor[axis] ? ConstraintType.RotationMotor : ConstraintType.AngularVelocityMotor,
                    Min = -math.INFINITY,
                    Max = math.INFINITY,
                    SpringFrequency = motorFrequency,
                    DampingRatio = motorDampingRatio,
                    MaxImpulse = maxForceForAngularMotor * Time.fixedDeltaTime,
                    Target = isRotationMotor[axis] ? -math.radians(joint.targetRotation.eulerAngles) : joint.targetAngularVelocity,
                });
            }
        }

        void ConvertConfigurableJoint(UnityEngine.ConfigurableJoint joint)
        {
            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.secondaryAxis);

            // Let's run some estimations.
            // 1. The worst case for linear dofs is all three limited, and all three driven, this results in 1 + 3 constraints.
            // 2. The worst case for angular dofs is all three limited, and all three driven too. This results in 3 + 3 constraints.
            // 3. Thus, in total we can have up to 4 + 6 = 10 constraints.
            // 4. sizeof(Constraint) is 44 bytes aligned as float, so a fixed size list can hold up to 11 constraints max.
            // 5. Conclude that one fixed list is enough for the worst case.
            var allConstraints = new FixedList512Bytes<Constraint>();

            ConvertLinearDofs(joint, GetConstrainedBodyMass(joint), jointFrameOrientation, ref allConstraints);
            ConvertAngularDofs(joint, GetConstrainedBodyInertia(joint), jointFrameOrientation, ref allConstraints);

            // Once constraints are ready, we group them in minimal amount of blocks
            CreateJointEntityBlocks(joint, jointFrameOrientation, allConstraints);
        }

        public override void Bake(UnityEngine.ConfigurableJoint authoring)
        {
            ConvertConfigurableJoint(authoring);
        }

        private void IdentifyIfLinearMotor(UnityEngine.ConfigurableJoint joint, out bool3 isPositionMotor, out bool3 isLinearVelocityMotor, out float3 maxForceForLinearMotor)
        {
            // shortcuts for the code below
            float3 targetPosition = joint.targetPosition;
            float3 targetVelocity = joint.targetVelocity;
            var xDrive = joint.xDrive;
            var yDrive = joint.yDrive;
            var zDrive = joint.zDrive;
            var name = joint.name;

            CheckPerAxis(targetPosition.x, targetVelocity.x, xDrive.positionSpring, xDrive.positionDamper, xDrive.maximumForce, name,
                out isPositionMotor.x, out isLinearVelocityMotor.x);

            CheckPerAxis(targetPosition.y, targetVelocity.y, yDrive.positionSpring, yDrive.positionDamper, yDrive.maximumForce, name,
                out isPositionMotor.y, out isLinearVelocityMotor.y);

            CheckPerAxis(targetPosition.z, targetVelocity.z, zDrive.positionSpring, zDrive.positionDamper, zDrive.maximumForce, name,
                out isPositionMotor.z, out isLinearVelocityMotor.z);

            maxForceForLinearMotor = new float3(xDrive.maximumForce, yDrive.maximumForce, zDrive.maximumForce);
        }

        private void IdentifyIfAngularMotor(UnityEngine.ConfigurableJoint joint, out bool3 isPositionMotor, out bool3 isVelocityMotor, out float3 maxForceForMotor)
        {
            // shortcuts for the code below
            quaternion targetRotation = joint.targetRotation;
            float3 targetVelocity = joint.targetAngularVelocity;
            var xDrive = joint.angularXDrive;
            var yzDrive = joint.angularYZDrive;
            var name = joint.name;

            CheckPerAxis(targetRotation.value.x, targetVelocity.x, xDrive.positionSpring, xDrive.positionDamper, xDrive.maximumForce, name,
                out isPositionMotor.x, out isVelocityMotor.x);

            CheckPerAxis(targetRotation.value.y, targetVelocity.y, yzDrive.positionSpring, yzDrive.positionDamper, yzDrive.maximumForce, name,
                out isPositionMotor.y, out isVelocityMotor.y);

            CheckPerAxis(targetRotation.value.z, targetVelocity.z, yzDrive.positionSpring, yzDrive.positionDamper, yzDrive.maximumForce, name,
                out isPositionMotor.z, out isVelocityMotor.z);

            maxForceForMotor = new float3(xDrive.maximumForce, yzDrive.maximumForce, yzDrive.maximumForce);
        }

        internal static void CheckPerAxis(float targetPosition, float targetVelocity, float spring, float damper, float force, string name,
            out bool isPositionMotor, out bool isVelocityMotor)
        {
            // for comparisons we must use an absolute value
            targetPosition = math.abs(targetPosition);
            targetVelocity = math.abs(targetVelocity);
            spring = math.abs(spring);
            damper = math.abs(damper);
            force = math.abs(force);

            float threshold = 0.0001f;

            // Initialize for early exit / default values
            isPositionMotor = false;
            isVelocityMotor = false;

            if (force <= threshold) return; // if no force applied at all, it isn't a motor

            // If target position==0 AND target velocity==0 AND spring==0 AND damper== 0. This is not a motor.
            if (targetPosition <= threshold && targetVelocity <= threshold &&
                spring <= threshold && damper <= threshold)
            {
                // Nothing is set. This is not a motor. Likely most common case
                return;
            }

            if (targetPosition > threshold && targetVelocity > threshold)
            {
                Assert.IsTrue(true,
                    $"Configurable Joint Baking Failed for {name}: Invalid configuration. Both target position and target velocity are non-zero.");
                return;
            }

            // If the target velocity is set but the target position is not, then it is a velocity motor (this does not guarantee the motor will function)
            if (targetPosition <= threshold && targetVelocity > threshold)
            {
                isVelocityMotor = true;
                return;
            }

            // If the target position is set but the target velocity is not, then it is a position motor (this does not guarantee the motor will function)
            if (targetPosition > threshold && targetVelocity <= threshold)
            {
                isPositionMotor = true;
                return;
            }

            // If the target position and target velocity are both zero, then it depends on the value of spring and damping
            if (targetPosition <= threshold && targetVelocity <= threshold)
            {
                if (spring <= threshold)
                {
                    if (damper > threshold)
                    {
                        isVelocityMotor = true;  // spring=0, damper!=0
                    }
                    //else // case already covered (nothing is set)
                }
                else
                {
                    // Regardless of damping value, if spring!=0, this is a position motor
                    isPositionMotor = true;
                }
            }
        }
    }

    class FixedBaker : BaseJointBaker<FixedJoint>
    {
        void ConvertFixedJoint(FixedJoint joint)
        {
            var worldFromJointA = math.mul(
                new RigidTransform(joint.transform.rotation, joint.transform.position),
                new RigidTransform(quaternion.identity, joint.anchor)
            );

            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            var connectedEntity = GetEntity(joint.connectedBody, TransformUsageFlags.Dynamic);
            RigidTransform worldFromBodyB = connectedEntity == Entity.Null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var bodyAFromJoint = new BodyFrame(math.mul(math.inverse(worldFromBodyA), worldFromJointA));
            var bodyBFromJoint = new BodyFrame(math.mul(math.inverse(worldFromBodyB), worldFromJointA));

            var jointData = PhysicsJoint.CreateFixed(bodyAFromJoint, bodyBFromJoint);
            jointData.SetImpulseEventThresholdAllConstraints(joint.breakForce * Time.fixedDeltaTime, joint.breakTorque * Time.fixedDeltaTime);

            var worldIndex = GetWorldIndex(joint);
            Entity entity = CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), jointData);
        }

        public override void Bake(FixedJoint authoring)
        {
            ConvertFixedJoint(authoring);
        }
    }

    class HingeBaker : BaseJointBaker<HingeJoint>
    {
        void ConvertHingeJoint(HingeJoint joint)
        {
            RigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            RigidTransform worldFromBodyB = joint.connectedBody == null
                ? RigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            Math.CalculatePerpendicularNormalized(joint.axis, out float3 perpendicularA, out _);

            // calculate local joint anchor position in body A space
            var anchorPos = GetScaledLocalAnchorPosition(joint.transform, joint.anchor);
            var bodyAFromJoint = new BodyFrame
            {
                Axis = joint.axis,
                PerpendicularAxis = perpendicularA,
                Position = anchorPos
            };

            var connectedEntity = GetEntity(joint.connectedBody, TransformUsageFlags.Dynamic);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromA = isConnectedBodyConverted ? math.mul(math.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : worldFromBodyB;

            // calculate local joint anchor position in body B space
            float3 connectedAnchorPos = joint.connectedBody ? GetScaledLocalAnchorPosition(joint.connectedBody.transform, joint.connectedAnchor) : joint.connectedAnchor;
            var bodyBFromJoint = new BodyFrame
            {
                Axis = math.mul(bFromA.rot, joint.axis),
                PerpendicularAxis = math.mul(bFromA.rot, perpendicularA),
                Position = math.mul(bFromBSource, new float4(connectedAnchorPos, 1)).xyz
            };

            var limits = math.radians(new FloatRange(joint.limits.min, joint.limits.max).Sorted());

            // Convert spring-damper settings
            ConvertSpringDamperSettings(joint.spring.spring, joint.spring.damper, GetConstrainedBodyInertia(joint),
                out var springFrequency, out var dampingRatio);

            // Create different types of joints based on: are there limits, is there a motor?
            PhysicsJoint jointData;
            if (joint.useSpring && !joint.useMotor) //a rotational motor if Use Spring = T, Spring: Spring, Damper, Target Position are set AND Motor.Force !=0
            {
                var maxImpulseOfMotor = joint.breakTorque * Time.fixedDeltaTime;
                var target = math.radians(joint.spring.targetPosition);
                jointData = joint.useLimits ? PhysicsJoint.CreateRotationalMotor(bodyAFromJoint, bodyBFromJoint, target, limits, maxImpulseOfMotor, springFrequency, dampingRatio)
                    : PhysicsJoint.CreateRotationalMotor(bodyAFromJoint, bodyBFromJoint, target, maxImpulseOfMotor, springFrequency, dampingRatio);
            }
            else if (joint.useMotor) //an angular velocity motor if use Motor = T, Motor: Target Velocity is set
            {
                var maxImpulseOfMotor = joint.motor.force * Time.fixedDeltaTime;
                var targetSpeedInRadians = math.radians(joint.motor.targetVelocity);
                jointData = PhysicsJoint.CreateAngularVelocityMotor(bodyAFromJoint, bodyBFromJoint,
                    targetSpeedInRadians, maxImpulseOfMotor, springFrequency, dampingRatio);
            }
            else //non-motorized
            {
                jointData = joint.useLimits
                    ? PhysicsJoint.CreateLimitedHinge(bodyAFromJoint, bodyBFromJoint, limits)
                    : PhysicsJoint.CreateHinge(bodyAFromJoint, bodyBFromJoint);
            }

            jointData.SetImpulseEventThresholdAllConstraints(joint.breakForce * Time.fixedDeltaTime, joint.breakTorque * Time.fixedDeltaTime);

            var worldIndex = GetWorldIndex(joint);
            Entity entity = CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), jointData);
        }

        public override void Bake(HingeJoint authoring)
        {
            ConvertHingeJoint(authoring);
        }
    }

    class SpringBaker : BaseJointBaker<SpringJoint>
    {
        void ConvertSpringJoint(SpringJoint joint)
        {
            // calculate local joint anchor positions in body A and body B space:

            var jointAnchorA = GetScaledLocalAnchorPosition(joint.transform, joint.anchor);
            float3 jointAnchorB = joint.connectedBody ? GetScaledLocalAnchorPosition(joint.connectedBody.transform, joint.connectedAnchor) : joint.connectedAnchor;

            var connectedEntity = GetEntity(joint.connectedBody, TransformUsageFlags.Dynamic);

            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            RigidTransform bFromBSource =
                isConnectedBodyConverted ? RigidTransform.identity : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            jointAnchorB = math.mul(bFromBSource, new float4(jointAnchorB, 1)).xyz;

            // create the joint:

            var distanceRange = new FloatRange(joint.minDistance, joint.maxDistance).Sorted();
            var mass = GetConstrainedBodyMass(joint);
            var impulseEventThreshold = joint.breakForce * Time.fixedDeltaTime;
            var springFrequency = JacobianUtilities.CalculateSpringFrequencyFromSpringConstant(joint.spring, GetConstrainedBodyMass(joint));
            var dampingRatio = JacobianUtilities.CalculateDampingRatio(joint.spring, joint.damper, mass);

            var jointData = PhysicsJoint.CreateLimitedDistance(jointAnchorA, jointAnchorB, distanceRange, impulseEventThreshold, springFrequency, dampingRatio);

            var worldIndex = GetWorldIndex(joint);
            Entity entity = CreateJointEntity(worldIndex, GetConstrainedBodyPair(joint), jointData);
        }

        public override void Bake(UnityEngine.SpringJoint authoring)
        {
            ConvertSpringJoint(authoring);
        }
    }
}
