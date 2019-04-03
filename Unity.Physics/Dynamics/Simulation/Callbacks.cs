using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;

namespace Unity.Physics
{
    // A container of user callbacks, to run during scheduling of the simulation jobs
    public class SimulationCallbacks
    {
        public enum Phase
        {
            PostCreateDispatchPairs,
            PostCreateContacts,
            PostCreateContactJacobians,
            PostSolveJacobians,
            PostIntegrateMotions
        }

        public delegate JobHandle Callback(ref ISimulation simulation, JobHandle inputDeps);

        private static readonly int k_NumPhases = Enum.GetValues(typeof(Phase)).Length;

        private readonly List<Callback>[] m_Callbacks = new List<Callback>[k_NumPhases];

        public SimulationCallbacks()
        {
            for (int i = 0; i < k_NumPhases; ++i)
            {
                m_Callbacks[i] = new List<Callback>(8);
            }
        }

        public void Enqueue(Phase phase, Callback cb)
        {
            m_Callbacks[(int)phase].Add(cb);
        }

        public JobHandle Execute(Phase phase, ISimulation simulation, JobHandle inputDeps)
        {
            ref List<Callback> cbs = ref m_Callbacks[(int)phase];
            if (m_Callbacks[(int)phase].Count > 0)
            {
                NativeList<JobHandle> handles = new NativeList<JobHandle>(cbs.Count, Allocator.Temp);
                foreach (Callback callback in cbs)
                {
                    JobHandle newTask = callback(ref simulation, inputDeps);
                    handles.Add(newTask);
                    inputDeps = newTask; // Have to assume each callback will modify the same data
                }
                JobHandle handle = JobHandle.CombineDependencies(handles);
                handles.Dispose();
                return handle;
            }
            return inputDeps;
        }

        public void Clear()
        {
            for (int i = 0; i < k_NumPhases; i++)
            {
                m_Callbacks[i].Clear();
            }
        }
    }
}
