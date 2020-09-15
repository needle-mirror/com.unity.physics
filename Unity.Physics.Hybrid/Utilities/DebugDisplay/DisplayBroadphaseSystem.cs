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
    // Update before end frame system as well as some test systems might have disabled the step system
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayBroadphaseAabbsSystem : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        StepPhysicsWorld m_StepPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawBroadphase != 0))
            {
                return;
            }

            var handle = JobHandle.CombineDependencies(Dependency, m_BuildPhysicsWorldSystem.GetOutputDependency());

            ref Broadphase broadphase = ref m_BuildPhysicsWorldSystem.PhysicsWorld.CollisionWorld.Broadphase;

            handle = new DisplayBroadphaseJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                StaticNodes = broadphase.StaticTree.Nodes,
                DynamicNodes = broadphase.DynamicTree.Nodes,
            }.Schedule(handle);

            m_StepPhysicsWorldSystem.AddInputDependency(handle);

            // Add dependency for end frame system as well as some test systems might have disabled the step system
            m_EndFramePhysicsSystem.AddInputDependency(handle);

            Dependency = handle;
        }
    }
}
