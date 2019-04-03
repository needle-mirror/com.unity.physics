using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics
{
    // Utilities for accessing the data flowing through a simulation step in a consistent way,
    // regardless of which simulation implementation is being used.
    // WORK IN PROGRESS.
    public static class SimulationData
    {
        // Access to the body pairs produced by the broad phase
        public struct BodyPairs
        {
            readonly SimulationType m_SimulationType;

            // Members for Unity simulation
            [NativeDisableContainerSafetyRestriction]
            NativeArray<Scheduler.DispatchPair> m_PhasedDispatchPairs;

            // Constructor for Unity simulation
            public unsafe BodyPairs(NativeArray<Scheduler.DispatchPair> phasedCollisionPairs)
            {
                m_SimulationType = SimulationType.UnityPhysics;
                m_PhasedDispatchPairs = phasedCollisionPairs;
            }
            
            // Create an iterator
            public Iterator GetIterator() => new Iterator(this);

            // An iterator with read-write access to the body pairs
            public struct Iterator
            {
                BodyPairs m_Pairs;
                int m_CurrentPairIndex;

                public unsafe Iterator(BodyPairs pairs)
                {
                    m_Pairs = pairs;
                    m_CurrentPairIndex = 0;
                }

                // Returns true is there are more pairs to iterate through
                public unsafe bool HasPairsLeft()
                {
                    if (m_Pairs.m_SimulationType == SimulationType.UnityPhysics)
                    {
                        return m_CurrentPairIndex <= m_Pairs.m_PhasedDispatchPairs.Length - 1;
                    }
                    return false;
                }

                // Move to the next body pair
                public unsafe BodyIndexPair NextPair()
                {
                    if (m_Pairs.m_SimulationType == SimulationType.UnityPhysics)
                    {
                        Scheduler.DispatchPair pair = m_Pairs.m_PhasedDispatchPairs[m_CurrentPairIndex++];
                        return new BodyIndexPair { BodyAIndex = pair.BodyAIndex, BodyBIndex = pair.BodyBIndex };
                    }
                    return BodyIndexPair.Invalid;
                }

                // Disable the previous body pair
                public void DisableLastPair()
                {
                    if (m_Pairs.m_SimulationType == SimulationType.UnityPhysics)
                    {
                        if (m_CurrentPairIndex != 0)
                        {
                            m_Pairs.m_PhasedDispatchPairs[m_CurrentPairIndex - 1] = Scheduler.DispatchPair.Invalid;
                        }
                    }
                }
            }
        }

        // Access to the contacts produced by the narrow phase
        public struct Contacts
        {
            readonly SimulationType m_SimulationType;
            [NativeDisableContainerSafetyRestriction]
            BlockStream.Writer m_ContactWriter;
            int m_NumContactsAdded;

            // Members for Unity simulation
            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            BlockStream m_ContactStream;
            [NativeDisableContainerSafetyRestriction]
            NativeArray<Scheduler.SolverSchedulerInfo.SolvePhaseInfo> m_PhaseInfo;
            readonly int m_MaxNumWorkItems;

            public int NumWorkItems => m_MaxNumWorkItems;

            // Constructor for Unity simulation
            public unsafe Contacts(BlockStream contactStream, Scheduler.SolverSchedulerInfo ssi)
            {
                m_SimulationType = SimulationType.UnityPhysics;
                m_ContactWriter = new BlockStream.Writer(contactStream);
                m_NumContactsAdded = 0;

                m_ContactStream = contactStream;
                m_PhaseInfo = ssi.PhaseInfo;
                m_MaxNumWorkItems = ssi.NumWorkItems;
            }

            // Create an iterator over the contact manifolds
            public Iterator GetIterator() => new Iterator(this);

            // Add a contact point
            //<todo.eoin.usermod what to do about multiple contact points?
            public void AddContact(ContactHeader header, ContactPoint point)
            {
                if(m_SimulationType == SimulationType.UnityPhysics)
                {
                    if (m_NumContactsAdded == 0)
                    {
                        int lastWorkItem = math.max(1, m_MaxNumWorkItems) - 1;
                        m_ContactWriter.AppendForEachIndex(lastWorkItem);
                    }

                    header.NumContacts = 1;
                    m_ContactWriter.Write(header);
                    m_ContactWriter.Write(point);
                    m_NumContactsAdded++;
                }
            }

            // Must be called after adding all contact points
            //<todo.eoin.usermod Can we avoid this somehow?
            public void CommitAddedContacts()
            {
                if (m_SimulationType == SimulationType.UnityPhysics)
                {
                    m_ContactWriter.EndForEachIndex();

                    // Append the new contacts to the last scheduler phase
                    int lastWorkItem = math.max(1, m_MaxNumWorkItems) - 1;
                    Scheduler.SolverSchedulerInfo.SolvePhaseInfo pi = m_PhaseInfo[lastWorkItem];
                    pi.BatchSize += m_NumContactsAdded;
                    pi.DispatchPairCount += m_NumContactsAdded;
                    pi.NumWorkItems = math.max(1, pi.NumWorkItems);
                    m_PhaseInfo[lastWorkItem] = pi;
                }
            }

            // Iterates over the contact manifolds
            public unsafe struct Iterator
            {
                readonly SimulationType m_SimulationType;

                //@TODO: Unity should have a Allow null safety restriction
                [NativeDisableContainerSafetyRestriction]
                BlockStream.Reader m_ContactReader;
                ContactHeader m_LastHeader;
                int m_NumPointsLeft;
                int m_CurrentWorkItem;
                readonly int m_MaxNumWorkItems;

                public Iterator(Contacts contacts)
                {
                    m_SimulationType = contacts.m_SimulationType;

                    m_ContactReader = new BlockStream.Reader(contacts.m_ContactStream);
                    m_CurrentWorkItem = 0;
                    m_MaxNumWorkItems = contacts.m_MaxNumWorkItems;
                    while (m_ContactReader.RemainingItemCount == 0 && m_CurrentWorkItem < contacts.m_MaxNumWorkItems)
                    {
                        m_ContactReader.BeginForEachIndex(m_CurrentWorkItem);
                        m_CurrentWorkItem++;
                    }

                    m_LastHeader = new ContactHeader();
                    m_NumPointsLeft = 0;
                }
                
                public bool HasItemsLeft()
                {
                    return m_ContactReader.RemainingItemCount > 0;
                }

                public ContactHeader GetNextContactHeader()
                {
                    while (m_NumPointsLeft > 0)
                    {
                        m_ContactReader.Read<ContactPoint>();
                    }

                    m_LastHeader = m_ContactReader.Read<ContactHeader>();
                    m_NumPointsLeft = m_LastHeader.NumContacts;
                    return m_LastHeader;
                }

                public ContactPoint GetNextContact()
                {
                    //<todo.eoin.hpmod Check we haven't read too many points
                    m_NumPointsLeft--;
                    ContactPoint cp = m_ContactReader.Read<ContactPoint>();

                    while (m_ContactReader.RemainingItemCount == 0 && m_CurrentWorkItem < m_MaxNumWorkItems)
                    {
                        m_ContactReader.BeginForEachIndex(m_CurrentWorkItem);
                        m_CurrentWorkItem++;
                    }

                    return cp;
                }

                public void SetManifoldNormal(float3 newNormal)
                {
                    m_LastHeader.Normal = newNormal;
                    m_ContactReader.Write(m_LastHeader);
                }

                public void UpdatePreviousContactHeader(ContactHeader newHeader)
                {
                    //<todo.eoin.hpmod Check last read was a contact
                    m_ContactReader.Write(newHeader);
                }

                public void UpdatePreviousContact(ContactPoint newData)
                {
                    //<todo.eoin.hpmod Check last read was a contact
                    m_ContactReader.Write(newData);
                }
            }
        }

        // Access to the Jacobians before solving
        // WORK IN PROGRESS
        public struct Jacobians
        {
            readonly bool m_IsCreated;
            BlockStream.Reader m_JacobianStreamReader;
            int m_WorkItemIndex;

            public Jacobians(BlockStream.Reader jacobianStreamReader, int workItemIndex)
            {
                m_IsCreated = true;
                m_JacobianStreamReader = jacobianStreamReader;
                m_WorkItemIndex = workItemIndex;
            }

            public JacobianIterator Iterator => m_IsCreated
                ? new JacobianIterator(m_JacobianStreamReader, m_WorkItemIndex, iterateAll : true)
                : new JacobianIterator();
        }
    }
}
