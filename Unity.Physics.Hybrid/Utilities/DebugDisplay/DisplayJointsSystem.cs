using System;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using static Unity.Physics.Math;
using Joint = Unity.Physics.Joint;

namespace Unity.Physics.Authoring
{
    /// Job which draws every joint
    public struct DisplayJointsJob : IJob
    {
        private const float k_Scale = 0.5f;

        public DebugStream.Context OutputStream;
        [ReadOnly] public NativeSlice<RigidBody> Bodies;
        [ReadOnly] public NativeSlice<Joint> Joints;

        public unsafe void Execute()
        {
            // Color palette
            Color colorA = Color.cyan;
            Color colorB = Color.magenta;
            Color colorError = Color.red;
            Color colorRange = Color.yellow;

            OutputStream.Begin(0);

            for (int iJoint = 0; iJoint < Joints.Length; iJoint++)
            {
                Joint joint = Joints[iJoint];
                JointData* jointData = joint.JointData;

                RigidBody bodyA = Bodies[joint.BodyPair.BodyAIndex];
                RigidBody bodyB = Bodies[joint.BodyPair.BodyBIndex];

                MTransform worldFromA, worldFromB;
                MTransform worldFromJointA, worldFromJointB;
                {
                    worldFromA = new MTransform(bodyA.WorldFromBody);
                    worldFromB = new MTransform(bodyB.WorldFromBody);

                    worldFromJointA = Mul(worldFromA, jointData->AFromJoint);
                    worldFromJointB = Mul(worldFromB, jointData->BFromJoint);
                }

                float3 pivotA = worldFromJointA.Translation;
                float3 pivotB = worldFromJointB.Translation;

                for (int iConstraint = 0; iConstraint < jointData->NumConstraints; iConstraint++)
                {
                    Constraint constraint = jointData->Constraints[iConstraint];

                    switch (constraint.Type)
                    {
                        case ConstraintType.Linear:

                            float3 diff = pivotA - pivotB;

                            // Draw the feature on B and find the range for A
                            float3 rangeOrigin;
                            float3 rangeDirection;
                            float rangeDistance;
                            switch (constraint.Dimension)
                            {
                                case 1:
                                    float3 normal = worldFromJointB.Rotation[constraint.ConstrainedAxis1D];
                                    OutputStream.Plane(pivotB, normal * k_Scale, colorB);
                                    rangeDistance = math.dot(normal, diff);
                                    rangeOrigin = pivotA - normal * rangeDistance;
                                    rangeDirection = normal;
                                    break;
                                case 2:
                                    float3 direction = worldFromJointB.Rotation[constraint.FreeAxis2D];
                                    OutputStream.Line(pivotB - direction * k_Scale, pivotB + direction * k_Scale, colorB);
                                    float dot = math.dot(direction, diff);
                                    rangeOrigin = pivotB + direction * dot;
                                    rangeDirection = diff - direction * dot;
                                    rangeDistance = math.length(rangeDirection);
                                    rangeDirection = math.select(rangeDirection / rangeDistance, float3.zero, rangeDistance < 1e-5);
                                    break;
                                case 3:
                                    OutputStream.Point(pivotB, k_Scale, colorB);
                                    rangeOrigin = pivotB;
                                    rangeDistance = math.length(diff);
                                    rangeDirection = math.select(diff / rangeDistance, float3.zero, rangeDistance < 1e-5);
                                    break;
                                default:
                                    throw new NotImplementedException();
                            }

                            // Draw the pivot on A
                            OutputStream.Point(pivotA, k_Scale, colorA);

                            // Draw error
                            float3 rangeA = rangeOrigin + rangeDistance * rangeDirection;
                            float3 rangeMin = rangeOrigin + constraint.Min * rangeDirection;
                            float3 rangeMax = rangeOrigin + constraint.Max * rangeDirection;
                            if (rangeDistance < constraint.Min)
                            {
                                OutputStream.Line(rangeA, rangeMin, colorError);
                            }
                            else if (rangeDistance > constraint.Max)
                            {
                                OutputStream.Line(rangeA, rangeMax, colorError);
                            }
                            if (math.length(rangeA - pivotA) > 1e-5f)
                            {
                                OutputStream.Line(rangeA, pivotA, colorError);
                            }

                            // Draw the range
                            if (constraint.Min != constraint.Max)
                            {
                                OutputStream.Line(rangeMin, rangeMax, colorRange);
                            }

                            break;
                        case ConstraintType.Angular:
                            switch (constraint.Dimension)
                            {
                                case 1:
                                    // Get the limited axis and perpendicular in joint space
                                    int constrainedAxis = constraint.ConstrainedAxis1D;
                                    float3 axisInWorld = worldFromJointA.Rotation[constrainedAxis];
                                    float3 perpendicularInWorld = worldFromJointA.Rotation[(constrainedAxis + 1) % 3] * k_Scale;

                                    // Draw the angle of A
                                    OutputStream.Line(pivotA, pivotA + perpendicularInWorld, colorA);

                                    // Calculate the relative angle
                                    float angle;
                                    {
                                        float3x3 jointBFromA = math.mul(math.inverse(worldFromJointB.Rotation), worldFromJointA.Rotation);
                                        angle = CalculateTwistAngle(new quaternion(jointBFromA), constrainedAxis);
                                    }

                                    // Draw the range in B
                                    float3 axis = worldFromJointA.Rotation[constraint.ConstrainedAxis1D];
                                    OutputStream.Arc(pivotB, axis, math.mul(quaternion.AxisAngle(axis, constraint.Min - angle), perpendicularInWorld), constraint.Max - constraint.Min, colorB);

                                    break;
                                case 2:
                                    // Get axes in world space
                                    int axisIndex = constraint.FreeAxis2D;
                                    float3 axisA = worldFromJointA.Rotation[axisIndex];
                                    float3 axisB = worldFromJointB.Rotation[axisIndex];

                                    // Draw the cones in B
                                    if (constraint.Min == 0.0f)
                                    {
                                        OutputStream.Line(pivotB, pivotB + axisB * k_Scale, colorB);
                                    }
                                    else
                                    {
                                        OutputStream.Cone(pivotB, axisB * k_Scale, constraint.Min, colorB);
                                    }
                                    if (constraint.Max != constraint.Min)
                                    {
                                        OutputStream.Cone(pivotB, axisB * k_Scale, constraint.Max, colorB);
                                    }

                                    // Draw the axis in A
                                    OutputStream.Arrow(pivotA, axisA * k_Scale, colorA);

                                    break;
                                case 3:
                                    // TODO - no idea how to visualize this if the limits are nonzero :)
                                    break;
                                default:
                                    throw new NotImplementedException();
                            }
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                }
            }

            OutputStream.End();
        }
    }

    /// Creates DisplayJointsJobs
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(StepPhysicsWorld))]
    public class DisplayJointsSystem : JobComponentSystem
    {

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawJoints != 0))
            {
                return inputDeps;
            }

            inputDeps = JobHandle.CombineDependencies(inputDeps, m_BuildPhysicsWorldSystem.FinalJobHandle);

            JobHandle handle = new DisplayJointsJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                Bodies = m_BuildPhysicsWorldSystem.PhysicsWorld.Bodies,
                Joints = m_BuildPhysicsWorldSystem.PhysicsWorld.Joints
            }.Schedule(inputDeps);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(handle);

            return handle;
        }
    }
}
