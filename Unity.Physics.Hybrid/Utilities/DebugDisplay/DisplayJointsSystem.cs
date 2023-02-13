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
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    [BurstCompile]
    internal partial struct DisplayJointsSystem : ISystem
    {
        /// Job which draws every joint
        [BurstCompile]
        internal struct DisplayJointsJob : IJobParallelFor
        {
            const float k_Scale = 0.5f;

            [ReadOnly] public NativeArray<RigidBody> Bodies;
            [ReadOnly] public NativeArray<Joint> Joints;

            public unsafe void Execute(int iJoint)
            {
                // Color palette
                var colorA = Unity.DebugDisplay.ColorIndex.Cyan;
                var colorB = Unity.DebugDisplay.ColorIndex.Magenta;
                var colorError = Unity.DebugDisplay.ColorIndex.Red;
                var colorRange = Unity.DebugDisplay.ColorIndex.Yellow;

                Joint joint = Joints[iJoint];

                if (joint.BodyPair.IsValid)
                {
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

                    ref var constraintBlock = ref joint.Constraints;
                    fixed(void* ptr = &constraintBlock)
                    {
                        var constraintPtr = (Constraint*)ptr;
                        for (var i = 0; i < constraintBlock.Length; i++)
                        {
                            Constraint constraint = constraintPtr[i];
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
                                            PhysicsDebugDisplaySystem.Plane(pivotB, normal * k_Scale, colorB);
                                            rangeDistance = math.dot(normal, diff);
                                            rangeOrigin = pivotA - normal * rangeDistance;
                                            rangeDirection = normal;
                                            break;
                                        case 2:
                                            float3 direction = worldFromJointB.Rotation[constraint.FreeAxis2D];
                                            PhysicsDebugDisplaySystem.Line(pivotB - direction * k_Scale, pivotB + direction * k_Scale,
                                                colorB);
                                            float dot = math.dot(direction, diff);
                                            rangeOrigin = pivotB + direction * dot;
                                            rangeDirection = diff - direction * dot;
                                            rangeDistance = math.length(rangeDirection);
                                            rangeDirection = math.select(rangeDirection / rangeDistance, float3.zero,
                                                rangeDistance < 1e-5);
                                            break;
                                        case 3:
                                            PhysicsDebugDisplaySystem.Point(pivotB, k_Scale, colorB);
                                            rangeOrigin = pivotB;
                                            rangeDistance = math.length(diff);
                                            rangeDirection = math.select(diff / rangeDistance, float3.zero,
                                                rangeDistance < 1e-5);
                                            break;
                                        default:
                                            SafetyChecks.ThrowNotImplementedException();
                                            return;
                                    }

                                    // Draw the pivot on A
                                    PhysicsDebugDisplaySystem.Point(pivotA, k_Scale, colorA);

                                    // Draw error
                                    float3 rangeA = rangeOrigin + rangeDistance * rangeDirection;
                                    float3 rangeMin = rangeOrigin + constraint.Min * rangeDirection;
                                    float3 rangeMax = rangeOrigin + constraint.Max * rangeDirection;
                                    if (rangeDistance < constraint.Min)
                                    {
                                        PhysicsDebugDisplaySystem.Line(rangeA, rangeMin, colorError);
                                    }
                                    else if (rangeDistance > constraint.Max)
                                    {
                                        PhysicsDebugDisplaySystem.Line(rangeA, rangeMax, colorError);
                                    }

                                    if (math.length(rangeA - pivotA) > 1e-5f)
                                    {
                                        PhysicsDebugDisplaySystem.Line(rangeA, pivotA, colorError);
                                    }

                                    // Draw the range
                                    if (constraint.Min != constraint.Max)
                                    {
                                        PhysicsDebugDisplaySystem.Line(rangeMin, rangeMax, colorRange);
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
                                            float3 perpendicularInWorld =
                                                worldFromJointA.Rotation[(constrainedAxis + 1) % 3] * k_Scale;

                                            // Draw the angle of A
                                            PhysicsDebugDisplaySystem.Line(pivotA, pivotA + perpendicularInWorld, colorA);

                                            // Calculate the relative angle
                                            float angle;
                                            {
                                                float3x3 jointBFromA = math.mul(math.inverse(worldFromJointB.Rotation),
                                                    worldFromJointA.Rotation);
                                                angle = CalculateTwistAngle(new quaternion(jointBFromA), constrainedAxis);
                                            }

                                            // Draw the range in B
                                            float3 axis = worldFromJointA.Rotation[constraint.ConstrainedAxis1D];
                                            PhysicsDebugDisplaySystem.Arc(pivotB, axis,
                                                math.mul(quaternion.AxisAngle(axis, constraint.Min - angle),
                                                    perpendicularInWorld), constraint.Max - constraint.Min, colorB);

                                            break;
                                        case 2:
                                            // Get axes in world space
                                            int axisIndex = constraint.FreeAxis2D;
                                            float3 axisA = worldFromJointA.Rotation[axisIndex];
                                            float3 axisB = worldFromJointB.Rotation[axisIndex];

                                            // Draw the cones in B
                                            if (constraint.Min == 0.0f)
                                            {
                                                PhysicsDebugDisplaySystem.Line(pivotB, pivotB + axisB * k_Scale, colorB);
                                            }
                                            else
                                            {
                                                PhysicsDebugDisplaySystem.Cone(pivotB, axisB * k_Scale, constraint.Min, colorB);
                                            }

                                            if (constraint.Max != constraint.Min)
                                            {
                                                PhysicsDebugDisplaySystem.Cone(pivotB, axisB * k_Scale, constraint.Max, colorB);
                                            }

                                            // Draw the axis in A
                                            PhysicsDebugDisplaySystem.Arrow(pivotA, axisA * k_Scale, colorA);

                                            break;
                                        case 3:
                                            // TODO - no idea how to visualize this if the limits are nonzero :)
                                            break;
                                        default:
                                            SafetyChecks.ThrowNotImplementedException();
                                            return;
                                    }
                                    break;
                                case ConstraintType.LinearVelocityMotor:
                                    //TODO: implement debug draw for motors
                                    break;
                                case ConstraintType.AngularVelocityMotor:
                                    //TODO: implement debug draw for motors
                                    break;
                                case ConstraintType.PositionMotor:
                                    //TODO: implement debug draw for motors
                                    break;
                                case ConstraintType.RotationMotor:
                                    //TODO: implement debug draw for motors
                                    break;
                                default:
                                    SafetyChecks.ThrowNotImplementedException();
                                    return;
                            }
                        }
                    }
                }
            }
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
#if UNITY_EDITOR
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawJoints == 0)
                return;

            var world = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

            if (world.NumJoints == 0)
            {
                return;
            }

            state.Dependency = new DisplayJointsJob
            {
                Bodies = world.Bodies,
                Joints = world.Joints
            }.Schedule(world.NumJoints, 16, state.Dependency);
#endif
        }
    }
}
