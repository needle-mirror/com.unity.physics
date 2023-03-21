using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

namespace Unity.Physics.Authoring
{
#if UNITY_EDITOR

    /// Create and dispatch a DisplayMassPropertiesJob
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    [BurstCompile]
    internal partial struct DisplayMassPropertiesSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawMassProperties == 0)
                return;

            var world = SystemAPI.GetSingleton<PhysicsWorldSingleton>().DynamicsWorld;
            state.Dependency = new DisplayMassPropertiesJob
            {
                MotionDatas = world.MotionDatas,
                MotionVelocities = world.MotionVelocities
            }.Schedule(world.MotionDatas.Length, 16, state.Dependency);
        }

        // Job to write mass properties info to a rendering buffer for any moving bodies
        // Attempts to build a box which has the same inertia tensor as the body.
        [BurstCompile]
        struct DisplayMassPropertiesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<MotionData> MotionDatas;
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;

            public void Execute(int m)
            {
                float3 com = MotionDatas[m].WorldFromMotion.pos;
                quaternion o = MotionDatas[m].WorldFromMotion.rot;

                float3 invInertiaLocal = MotionVelocities[m].InverseInertia;
                float3 il = new float3(1.0f / invInertiaLocal.x, 1.0f / invInertiaLocal.y, 1.0f / invInertiaLocal.z);
                float invMass = MotionVelocities[m].InverseMass;

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
                PhysicsDebugDisplaySystem.Box(boxSize, com, o, DebugDisplay.ColorIndex.Magenta);
            }
        }
    }
#endif
}
