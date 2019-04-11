using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// Job which walks the broadphase tree and writes the
    /// bounding box of leaf nodes to a DebugStream.
    public struct DisplayBroadphaseJob : IJob //<todo.eoin.udebug This can be a parallelfor job
    {
        public DebugStream.Context OutputStream;

        [ReadOnly]
        public NativeArray<BoundingVolumeHierarchy.Node> StaticNodes;

        [ReadOnly]
        public NativeArray<BoundingVolumeHierarchy.Node> DynamicNodes;

        public void DrawLeavesRecursive(NativeArray<BoundingVolumeHierarchy.Node> nodes, Color color, int nodeIndex)
        {
            if (nodes[nodeIndex].IsLeaf)
            {
                bool4 leavesValid = nodes[nodeIndex].AreLeavesValid;
                for (int l = 0; l < 4; l++)
                {
                    if (leavesValid[l])
                    {
                        Aabb aabb = nodes[nodeIndex].Bounds.GetAabb(l);
                        float3 center = aabb.Center;
                        OutputStream.Box(aabb.Extents, center, Quaternion.identity, color);
                    }
                }

                return;
            }

            for (int i = 0; i < 4; i++)
            {
                if (nodes[nodeIndex].IsChildValid(i))
                {
                    DrawLeavesRecursive(nodes, color, nodes[nodeIndex].Data[i]);
                }
            }
        }
        public void Execute()
        {
            OutputStream.Begin(0);
            DrawLeavesRecursive(StaticNodes, Color.yellow, 1);
            DrawLeavesRecursive(DynamicNodes, Color.red, 1);
            OutputStream.End();
        }
    }

    /// Creates DisplayBroadphaseJobs
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayBroadphaseAabbsSystem : JobComponentSystem
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
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawBroadphase != 0))
            {
                return inputDeps;
            }

            inputDeps = JobHandle.CombineDependencies(inputDeps, m_BuildPhysicsWorldSystem.FinalJobHandle);

            ref Broadphase broadphase = ref m_BuildPhysicsWorldSystem.PhysicsWorld.CollisionWorld.Broadphase;

            JobHandle handle = new DisplayBroadphaseJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                StaticNodes = broadphase.StaticTree.Nodes,
                DynamicNodes = broadphase.DynamicTree.Nodes,
            }.Schedule(inputDeps);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(handle);

            return handle;
        }
    }
}
