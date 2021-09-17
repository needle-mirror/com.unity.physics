using System;
using Unity.Entities;

namespace Unity.Physics
{
    /// <summary>
    /// An event raised when a pair of bodies interact during solving.
    /// </summary>
    public interface ISimulationEvent<T>: IComparable<T>
    {
        Entity EntityA { get; }
        Entity EntityB { get; }

        int BodyIndexA { get; }
        int BodyIndexB { get; }

        ColliderKey ColliderKeyA { get; }
        ColliderKey ColliderKeyB { get; }
    }

    public static class ISimulationEventUtilities
    {
        public static int CompareEvents<T>(T thisEvent, T otherEvent) where T : struct, ISimulationEvent<T>
        {
            int i = thisEvent.EntityA.CompareTo(otherEvent.EntityA);
            if (i == 0)
            {
                i = thisEvent.EntityB.CompareTo(otherEvent.EntityB);
                if (i == 0)
                {
                    i = thisEvent.ColliderKeyA.CompareTo(otherEvent.ColliderKeyA);
                    if (i == 0)
                    {
                        i = thisEvent.ColliderKeyB.CompareTo(otherEvent.ColliderKeyB);
                    }
                }
            }

            return i;
        }
    }
}
