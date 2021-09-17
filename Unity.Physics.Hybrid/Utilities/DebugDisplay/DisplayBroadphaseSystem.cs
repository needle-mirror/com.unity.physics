using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    /// Job which walks the broadphase tree and writes the
    /// bounding box of leaf nodes to a DebugStream.
    [BurstCompile]
    public struct DisplayBroadphaseJob : IJob //<todo.eoin.udebug This can be a parallelfor job
    {
        public DebugStream.Context OutputStream;

        [ReadOnly]
        public NativeArray<BoundingVolumeHierarchy.Node> StaticNodes;

        [ReadOnly]
        public NativeArray<BoundingVolumeHierarchy.Node> DynamicNodes;

        internal void DrawLeavesRecursive(NativeArray<BoundingVolumeHierarchy.Node> nodes, Unity.DebugDisplay.ColorIndex color, int nodeIndex)
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
                        OutputStream.Box(aabb.Extents, center, quaternion.identity, color);
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
            DrawLeavesRecursive(StaticNodes, Unity.DebugDisplay.ColorIndex.Yellow, 1);
            DrawLeavesRecursive(DynamicNodes, Unity.DebugDisplay.ColorIndex.Red, 1);
            OutputStream.End();
        }
    }

    // Creates DisplayBroadphaseJobs
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class DisplayBroadphaseAabbsSystem : SystemBase
    {
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
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawBroadphase != 0))
            {
                return;
            }

            ref Broadphase broadphase = ref m_BuildPhysicsWorldSystem.PhysicsWorld.CollisionWorld.Broadphase;

            Dependency = new DisplayBroadphaseJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                StaticNodes = broadphase.StaticTree.Nodes,
                DynamicNodes = broadphase.DynamicTree.Nodes,
            }.Schedule(Dependency);
        }
    }
}
