using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Burst;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    // Job which iterates over contacts from narrowphase and writes display info to a DebugStream.
    [BurstCompile]
    public struct DisplayContactsJob : IJobParallelFor
    {
        public SimulationData.Contacts.Iterator ManifoldIterator;
        public DebugStream.Context OutputStream;

        public void Execute(int workItemIndex)
        {
            OutputStream.Begin(workItemIndex);

            while (ManifoldIterator.HasItemsLeft())
            {
                ContactHeader contactHeader = ManifoldIterator.GetNextContactHeader();
                for (int c = 0; c < contactHeader.NumContacts; c++)
                {
                    Unity.Physics.ContactPoint contact = ManifoldIterator.GetNextContact();
                    float3 x0 = contact.Position;
                    float3 x1 = contactHeader.Normal * contact.Distance;
                    Color color = Color.green;
                    OutputStream.Arrow(x0, x1, color);
                }
            }

            OutputStream.End();
        }
    }

    // Create DisplayContactsJob
    [UpdateBefore(typeof(StepPhysicsWorld))]
    public class DisplayContactsSystem : JobComponentSystem
    {
        StepPhysicsWorld m_StepWorld;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreateManager()
        {
            m_StepWorld = World.GetOrCreateManager<StepPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateManager<DebugStream>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawContacts != 0))
            {
                return inputDeps;
            }

            SimulationCallbacks.Callback callback = (ref ISimulation simulation, JobHandle inDeps) =>
            {
                inDeps.Complete(); //<todo Necessary to initialize the modifier

                SimulationData.Contacts contacts = simulation.Contacts;
                int numWorkItems = contacts.NumWorkItems;
                if (numWorkItems > 0)
                {
                    return new DisplayContactsJob
                    {
                        ManifoldIterator = contacts.GetIterator(),
                        OutputStream = m_DebugStreamSystem.GetContext(numWorkItems)
                    }.Schedule(numWorkItems, 1, inDeps);
                }
                return inDeps;
            };

            m_StepWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContacts, callback);


            return inputDeps;
        }
    }
}
