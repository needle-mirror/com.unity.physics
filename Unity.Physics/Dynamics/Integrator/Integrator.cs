using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Physics
{
    public static class Integrator
    {
        // Schedule a job to integrate the world's motions forward by the given time step.
        public static JobHandle ScheduleIntegrateJobs(ref DynamicsWorld world, float timeStep, float3 gravity, JobHandle inputDeps)
        {
            return new IntegrateMotionsJob
            {
                MotionDatas = world.MotionDatas,
                MotionVelocities = world.MotionVelocities,
                Timestep = timeStep,
                Gravity = gravity
            }.Schedule(world.NumMotions, 64, inputDeps);
        }

        [BurstCompile]
        private struct IntegrateMotionsJob : IJobParallelFor
        {
            public NativeSlice<MotionData> MotionDatas;
            public NativeSlice<MotionVelocity> MotionVelocities;
            public float Timestep;
            public float3 Gravity;

            public void Execute(int i)
            {
                MotionData motionData = MotionDatas[i];
                MotionVelocity motionVelocity = MotionVelocities[i];

                // Update motion space
                {
                    // center of mass
                    motionData.WorldFromMotion.pos += motionVelocity.LinearVelocity * Timestep;

                    // orientation
                    IntegrateOrientation(ref motionData.WorldFromMotion.rot, motionVelocity.AngularVelocity, Timestep);
                }

                // Update velocities
                {
                    // gravity
                    motionVelocity.LinearVelocity += Gravity * motionData.GravityFactor * Timestep;

                    // damping
                    motionVelocity.LinearVelocity *= math.clamp(1.0f - motionData.LinearDamping * Timestep, 0.0f, 1.0f);
                    motionVelocity.AngularVelocity *= math.clamp(1.0f - motionData.AngularDamping * Timestep, 0.0f, 1.0f);
                }

                // Write back
                MotionDatas[i] = motionData;
                MotionVelocities[i] = motionVelocity;
            }
        }

        public static void IntegrateOrientation(ref quaternion orientation, float3 angularVelocity, float timestep)
        {
            quaternion dq = IntegrateAngularVelocity(angularVelocity, timestep);
            quaternion r = math.mul(orientation, dq);
            orientation = math.normalize(r);
        }

        // Returns a non-normalized quaternion that approximates the change in angle angularVelocity * timestep.
        public static quaternion IntegrateAngularVelocity(float3 angularVelocity, float timestep)
        {
            float3 halfDeltaTime = new float3(timestep * 0.5f);
            float3 halfDeltaAngle = angularVelocity * halfDeltaTime;
            return new quaternion(new float4(halfDeltaAngle, 1.0f));
        }
    }
}
