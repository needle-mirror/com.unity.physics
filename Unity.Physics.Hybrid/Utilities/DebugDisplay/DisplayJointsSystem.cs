using System;
using Unity.Burst;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics.Authoring
{
    // Creates DisplayJointsJobs
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class DisplayJointsSystem : SystemBase
    {
        /// Job which draws every joint
        [BurstCompile]
        protected struct DisplayJointsJob : IJob
        {
            const float k_Scale = 0.5f;

            public DebugStream.Context OutputStream;
            [ReadOnly] public NativeArray<RigidBody> Bodies;
            [ReadOnly] public NativeArray<Joint> Joints;

            public unsafe void Execute()
            {
                // Color palette
                var colorA = Unity.DebugDisplay.ColorIndex.Cyan;
                var colorB = Unity.DebugDisplay.ColorIndex.Magenta;
                var colorError = Unity.DebugDisplay.ColorIndex.Red;
                var colorRange = Unity.DebugDisplay.ColorIndex.Yellow;

                OutputStream.Begin(0);

                for (int iJoint = 0; iJoint < Joints.Length; iJoint++)
                {
                    Joint joint = Joints[iJoint];

                    if (!joint.BodyPair.IsValid) continue;

                    RigidBody bodyA = Bodies[joint.BodyPair.BodyIndexA];
                    RigidBody bodyB = Bodies[joint.BodyPair.BodyIndexB];

                    MTransform worldFromA, worldFromB;
                    MTransform worldFromJointA, worldFromJointB;
                    {
                        worldFromA = new MTransform(bodyA.WorldFromBody);
                        worldFromB = new MTransform(bodyB.WorldFromBody);

                        worldFromJointA = Mul(worldFromA, joint.AFromJoint);
                        worldFromJointB = Mul(worldFromB, joint.BFromJoint);
                    }

                    float3 pivotA = worldFromJointA.Translation;
                    float3 pivotB = worldFromJointB.Translation;

                    for (var i = 0; i < joint.Constraints.Length; i++)
                    {
                        Constraint constraint = joint.Constraints[i];
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
                                    case 0:
                                        continue;
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
                                        SafetyChecks.ThrowNotImplementedException();
                                        return;
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
                                    case 0:
                                        continue;
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
                                        SafetyChecks.ThrowNotImplementedException();
                                        return;
                                }
                                break;
                            default:
                                SafetyChecks.ThrowNotImplementedException();
                                return;
                        }
                    }
                }

                OutputStream.End();
            }
        }

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadOnly();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawJoints != 0))
            {
                return;
            }

#pragma warning disable 618
            Dependency = new DisplayJointsJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                Bodies = m_BuildPhysicsWorldSystem.PhysicsWorld.Bodies,
                Joints = m_BuildPhysicsWorldSystem.PhysicsWorld.Joints
            }.Schedule(Dependency);
#pragma warning restore 618
        }
    }
}
