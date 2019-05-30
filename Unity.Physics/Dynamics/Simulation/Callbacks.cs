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
            PostSolveJacobians
        }

        public delegate JobHandle Callback(ref ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps);

        private static readonly int k_NumPhases = Enum.GetValues(typeof(Phase)).Length;

        struct CallbackAndDependency
        {
            public Callback Callback;
            public JobHandle Dependency;
        }

        private readonly List<CallbackAndDependency>[] m_Callbacks = new List<CallbackAndDependency>[k_NumPhases];

        public SimulationCallbacks()
        {
            for (int i = 0; i < k_NumPhases; ++i)
            {
                m_Callbacks[i] = new List<CallbackAndDependency>(8);
            }
        }

        public void Enqueue(Phase phase, Callback cb, JobHandle dependency)
        {
            m_Callbacks[(int)phase].Add(new CallbackAndDependency { Callback = cb, Dependency = dependency });
        }

        public bool Any(Phase phase)
        {
            return m_Callbacks[(int)phase].Count > 0;
        }

        public JobHandle Execute(Phase phase, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
        {
            ref List<CallbackAndDependency> cbs = ref m_Callbacks[(int)phase];
            if (m_Callbacks[(int)phase].Count > 0)
            {
                foreach (CallbackAndDependency callback in cbs)
                    inputDeps = callback.Callback(ref simulation, ref world, JobHandle.CombineDependencies(inputDeps, callback.Dependency));

                return inputDeps;
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
