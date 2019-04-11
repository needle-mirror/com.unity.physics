using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// Create and dispatch a DisplayMassPropertiesJob
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayMassPropertiesSystem : JobComponentSystem
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        StepPhysicsWorld m_StepPhysicsWorld;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawMassProperties != 0))
            {
                return inputDeps;
            }

            inputDeps = JobHandle.CombineDependencies(inputDeps, m_StepPhysicsWorld.FinalSimulationJobHandle);

            JobHandle handle = new DisplayMassPropertiesJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                MotionDatas = m_BuildPhysicsWorldSystem.PhysicsWorld.MotionDatas,
                MotionVelocities = m_BuildPhysicsWorldSystem.PhysicsWorld.MotionVelocities
            }.Schedule(inputDeps);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(handle);
            return handle;
        }

        // Job to write mass properties info to a DebugStream for any moving bodies
        // Attempts to build a box which has the same inertia tensor as the body.
        struct DisplayMassPropertiesJob : IJob //<todo.eoin.udebug This can be a parallelfor job
        {
            public DebugStream.Context OutputStream;

            [ReadOnly] public NativeSlice<MotionData> MotionDatas;
            [ReadOnly] public NativeSlice<MotionVelocity> MotionVelocities;

            public void Execute()
            {
                OutputStream.Begin(0);
                for (int m = 0; m < MotionDatas.Length; m++)
                {
                    float3 com = MotionDatas[m].WorldFromMotion.pos;
                    quaternion o = MotionDatas[m].WorldFromMotion.rot;

                    float3 invInertiaLocal = MotionVelocities[m].InverseInertiaAndMass.xyz;
                    float3 il = new float3(1.0f / invInertiaLocal.x, 1.0f / invInertiaLocal.y, 1.0f / invInertiaLocal.z);
                    float invMass = MotionVelocities[m].InverseInertiaAndMass.w;

                    // Reverse the inertia tensor computation to build a box which has the inerta tensor 'il'
                    // The diagonal inertia of a box with dimensions h,w,d and mass m is:
                    // Ix = 1/12 m (ww + dd)
                    // Iy = 1/12 m (dd + hh)
                    // Iz = 1/12 m (ww + hh)
                    //
                    // For simplicity, set K = I * 12 / m
                    // Then K = (ww + dd, dd + hh, ww + hh)
                    // => ww = Kx - dd, dd = Ky - hh, hh = Kz - ww
                    // By manipulation:
                    // 2ww = Kx - Ky + Kz
                    // => w = ((0.5)(Kx - Ky + Kz))^-1
                    // Then, substitution gives h and d.

                    float3 k = new float3(il.x * 12 * invMass, il.y * 12 * invMass, il.z * 12 * invMass);
                    float w = math.sqrt((k.x - k.y + k.z) * 0.5f);
                    float h = math.sqrt(k.z - w * w);
                    float d = math.sqrt(k.y - h * h);

                    float3 boxSize = new float3(h, w, d);
                    OutputStream.Box(boxSize, com, o, Color.magenta);
                }
                OutputStream.End();
            }
        }
    }
}
