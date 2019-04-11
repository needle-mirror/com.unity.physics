using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace Unity.Physics
{
    public struct SurfaceConstraintInfo
    {
        // Info of interest for the character
        public Plane Plane;
        public float3 Velocity;

        // Hit body info
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public float3 HitPosition;

        // Internal state
        public int Priority;
        public bool Touched;
    }

    public static class SimplexSolver
    {
        public const float c_SimplexSolverEpsilon = 0.0001f;

        public static unsafe void Solve(PhysicsWorld world, float deltaTime, float3 up, int numConstraints,
            ref NativeArray<SurfaceConstraintInfo> constraints, ref float3 position, ref float3 velocity, out float integratedTime)
        {
            // List of planes to solve against (up to 4)
            SurfaceConstraintInfo* supportPlanes = stackalloc SurfaceConstraintInfo[4];
            int numSupportPlanes = 0;

            float remainingTime = deltaTime;
            float currentTime = 0.0f;

            float minDeltaTime = 0.0f;
            if (math.lengthsq(velocity) > c_SimplexSolverEpsilon)
            {
                // Min delta time to travel at least 1cm
                minDeltaTime = 0.01f / math.length(velocity);
            }

            while (remainingTime > 0.0f)
            {
                int hitIndex = -1;
                float minCollisionTime = remainingTime;

                // Iterate over constraints and solve them
                for (int i = 0; i < numConstraints; i++)
                {
                    if (constraints[i].Touched) continue;

                    SurfaceConstraintInfo constraint = constraints[i];

                    float3 relVel = velocity - constraint.Velocity;
                    float relProjVel = -math.dot(relVel, constraint.Plane.Normal);
                    if (relProjVel < c_SimplexSolverEpsilon)
                    {
                        continue;
                    }

                    // Clamp distance to 0, since penetration is handled by constraint.Velocity already
                    float distance = math.max(constraint.Plane.Distance, 0.0f);
                    if (distance < minCollisionTime * relProjVel)
                    {
                        minCollisionTime = distance / relProjVel;
                        hitIndex = i;
                    }
                }

                // Integrate if at least 100 microseconds to hit
                if (minCollisionTime > 1e-4f)
                {
                    currentTime += minCollisionTime;
                    remainingTime -= minCollisionTime;
                    position += minCollisionTime * velocity;
                }

                if (hitIndex < 0 || currentTime > minDeltaTime)
                {
                    break;
                }

                // Mark constraint as touched
                {
                    var constraint = constraints[hitIndex];
                    constraint.Touched = true;
                    constraints[hitIndex] = constraint;
                }

                // Add the hit to the current list of active planes
                supportPlanes[numSupportPlanes++] = constraints[hitIndex];

                // Solve support planes
                ExamineActivePlanes(up, supportPlanes, ref numSupportPlanes, ref velocity);

                // Can't handle more than 4 support planes
                if (numSupportPlanes == 4)
                {
                    break;
                }
            }

            integratedTime = currentTime;
        }

        public static unsafe void ExamineActivePlanes(float3 up, SurfaceConstraintInfo* supportPlanes, ref int numSupportPlanes, ref float3 velocity)
        {
            switch (numSupportPlanes)
            {
                case 1:
                    {
                        Solve1d(supportPlanes[0], ref velocity);
                        return;
                    }
                case 2:
                    {
                        // Test whether we need plane 0 at all
                        float3 tempVelocity = velocity;
                        Solve1d(supportPlanes[1], ref tempVelocity);

                        bool plane0Used = Test1d(supportPlanes[0], tempVelocity);
                        if (!plane0Used)
                        {
                            // Compact the buffer and reduce size
                            supportPlanes[0] = supportPlanes[1];
                            numSupportPlanes = 1;

                            // Write back the result
                            velocity = tempVelocity;
                        }
                        else
                        {
                            Solve2d(up, supportPlanes[0], supportPlanes[1], ref velocity);
                        }

                        return;
                    }
                case 3:
                    {
                        // Try to drop both planes
                        float3 tempVelocity = velocity;
                        Solve1d(supportPlanes[2], ref tempVelocity);

                        bool plane0Used = Test1d(supportPlanes[0], tempVelocity);
                        if (!plane0Used)
                        {
                            bool plane1Used = Test1d(supportPlanes[1], tempVelocity);
                            if (!plane1Used)
                            {
                                // Compact the buffer and reduce size
                                supportPlanes[0] = supportPlanes[2];
                                numSupportPlanes = 1;
                                goto case 1;
                            }
                        }

                        // Try to drop plane 0 or 1
                        for (int testPlane = 0; testPlane < 2; testPlane++)
                        {
                            tempVelocity = velocity;
                            Solve2d(up, supportPlanes[testPlane], supportPlanes[2], ref tempVelocity);

                            bool planeUsed = Test1d(supportPlanes[1 - testPlane], tempVelocity);
                            if (!planeUsed)
                            {
                                supportPlanes[0] = supportPlanes[testPlane];
                                supportPlanes[1] = supportPlanes[2];
                                numSupportPlanes--;
                                goto case 2;
                            }
                        }

                        // Try solve all three
                        Solve3d(up, supportPlanes[0], supportPlanes[1], supportPlanes[2], ref velocity);

                        return;
                    }
                case 4:
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            float3 tempVelocity = velocity;
                            Solve3d(up, supportPlanes[(i + 1) % 3], supportPlanes[(i + 2) % 3], supportPlanes[3], ref tempVelocity);
                            bool planeUsed = Test1d(supportPlanes[i], tempVelocity);
                            if (!planeUsed)
                            {
                                supportPlanes[i] = supportPlanes[2];
                                supportPlanes[2] = supportPlanes[3];
                                numSupportPlanes = 3;
                                goto case 3;
                            }
                        }

                        // Nothing can be dropped so we've failed to solve,
                        // now we do all 3d combinations
                        float3 tempVel = velocity;
                        SurfaceConstraintInfo sp0 = supportPlanes[0];
                        SurfaceConstraintInfo sp1 = supportPlanes[1];
                        SurfaceConstraintInfo sp2 = supportPlanes[2];
                        SurfaceConstraintInfo sp3 = supportPlanes[3];
                        Solve3d(up, sp0, sp1, sp2, ref tempVel);
                        Solve3d(up, sp0, sp1, sp3, ref tempVel);
                        Solve3d(up, sp0, sp2, sp3, ref tempVel);
                        Solve3d(up, sp1, sp2, sp3, ref tempVel);

                        velocity = tempVel;

                        return;
                    }
                default:
                    {
                        // Can't have more than 4 and less than 1 plane
                        Assert.IsTrue(false);
                        break;
                    }
            }
        }

        public static void Solve1d(SurfaceConstraintInfo constraint, ref float3 velocity)
        {
            float3 groundVelocity = constraint.Velocity;
            float3 relVel = velocity - groundVelocity;
            float planeVel = math.dot(relVel, constraint.Plane.Normal);
            relVel -= planeVel * constraint.Plane.Normal;

            velocity = relVel + groundVelocity;
        }

        public static bool Test1d(SurfaceConstraintInfo constraint, float3 velocity)
        {
            float3 relVel = velocity - constraint.Velocity;
            float planeVel = math.dot(relVel, constraint.Plane.Normal);
            return planeVel < -c_SimplexSolverEpsilon;
        }

        public static void Solve2d(float3 up, SurfaceConstraintInfo constraint0, SurfaceConstraintInfo constraint1, ref float3 velocity)
        {
            float3 plane0 = constraint0.Plane.Normal;
            float3 plane1 = constraint1.Plane.Normal;

            // Calculate the free axis
            float3 axis = math.cross(plane0, plane1);
            float axisLen2 = math.lengthsq(axis);

            // Check for parallel planes
            if (axisLen2 < c_SimplexSolverEpsilon)
            {
                // Do the planes sequentially
                Sort2d(ref constraint0, ref constraint1);
                Solve1d(constraint1, ref velocity);
                Solve1d(constraint0, ref velocity);

                return;
            }

            float invAxisLen = math.rsqrt(axisLen2);
            axis *= invAxisLen;

            // Calculate the velocity of the free axis
            float3 axisVel;
            {
                float4x4 m = new float4x4();
                float3 r0 = math.cross(plane0, plane1);
                float3 r1 = math.cross(plane1, axis);
                float3 r2 = math.cross(axis, plane0);
                m.c0 = new float4(r0, 0.0f);
                m.c1 = new float4(r1, 0.0f);
                m.c2 = new float4(r2, 0.0f);
                m.c3 = new float4(0.0f, 0.0f, 0.0f, 1.0f);

                float3 sVel = constraint0.Velocity + constraint1.Velocity;
                float3 t = new float3(
                    math.dot(axis, sVel) * 0.5f,
                    math.dot(plane0, constraint0.Velocity),
                    math.dot(plane1, constraint1.Velocity));

                axisVel = math.rotate(m, t);
                axisVel *= invAxisLen;
            }

            float3 groundVelocity = axisVel;
            float3 relVel = velocity - groundVelocity;

            float vel2 = math.lengthsq(relVel);
            float axisVert = math.dot(up, axis);
            float axisProjVel = math.dot(relVel, axis);

            velocity = groundVelocity + axis * axisProjVel;
        }

        public static void Solve3d(float3 up, SurfaceConstraintInfo constraint0, SurfaceConstraintInfo constraint1, SurfaceConstraintInfo constraint2, ref float3 velocity)
        {
            float3 plane0 = constraint0.Plane.Normal;
            float3 plane1 = constraint1.Plane.Normal;
            float3 plane2 = constraint2.Plane.Normal;

            float4x4 m = new float4x4();
            float3 r0 = math.cross(plane1, plane2);
            float3 r1 = math.cross(plane2, plane0);
            float3 r2 = math.cross(plane0, plane1);
            m.c0 = new float4(r0, 0.0f);
            m.c1 = new float4(r1, 0.0f);
            m.c2 = new float4(r2, 0.0f);
            m.c3 = new float4(0.0f, 0.0f, 0.0f, 1.0f);

            float det = m.c0.x * m.c1.y * m.c2.z;
            float tst = math.abs(det);
            if (tst < c_SimplexSolverEpsilon)
            {
                Sort3d(ref constraint0, ref constraint1, ref constraint2);
                Solve2d(up, constraint1, constraint2, ref velocity);
                Solve2d(up, constraint0, constraint2, ref velocity);
                Solve2d(up, constraint0, constraint1, ref velocity);

                return;
            }

            float3 sVel = constraint0.Velocity + constraint1.Velocity;
            float3 t = new float3(
                math.dot(plane0, constraint0.Velocity),
                math.dot(plane1, constraint1.Velocity),
                math.dot(plane2, constraint2.Velocity));

            float3 pointVel = math.rotate(m, t);
            pointVel /= det;

            velocity = pointVel;
        }

        public static void Sort2d(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1)
        {
            int priority0 = plane0.Priority;
            int priority1 = plane1.Priority;
            if (priority0 > priority1)
            {
                SwapPlanes(ref plane0, ref plane1);
            }
        }

        public static void Sort3d(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1, ref SurfaceConstraintInfo plane2)
        {
            int priority0 = plane0.Priority;
            int priority1 = plane1.Priority;
            int priority2 = plane2.Priority;
            if (priority0 <= priority1)
            {
                if (priority1 <= priority2)
                {
                    // 0, 1, 2
                }
                else if (priority0 <= priority2)
                {
                    // 0, 2, 1
                    SwapPlanes(ref plane1, ref plane2);
                }
                else
                {
                    // 1, 2, 0
                    SwapPlanes(ref plane0, ref plane1);
                    SwapPlanes(ref plane0, ref plane2);
                }
            }
            else
            {
                if (priority2 < priority1)
                {
                    // 2, 1, 0
                    SwapPlanes(ref plane0, ref plane2);
                }
                else if (priority2 > priority0)
                {
                    // 1, 0, 2
                    SwapPlanes(ref plane0, ref plane1);
                }
                else
                {
                    // 2, 0, 1
                    SwapPlanes(ref plane0, ref plane1);
                    SwapPlanes(ref plane1, ref plane2);
                }
            }
        }

        public static void SwapPlanes(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1)
        {
            var temp = plane0;
            plane0 = plane1;
            plane1 = temp;
        }
    }
}
