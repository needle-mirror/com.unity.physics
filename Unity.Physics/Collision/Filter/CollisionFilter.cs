using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Unity.Physics
{
    // Describes which other objects an object can collide with.
    [DebuggerDisplay("Group: {GroupIndex} Category: {CategoryBits} Mask: {MaskBits}")]
    public struct CollisionFilter
    {
        // A bit mask describing which layers this object belongs to.
        public uint CategoryBits;   // TODO rename?

        // A bit mask describing which layers this object can collide with.
        public uint MaskBits;   // TODO rename?

        // An optional override for the bit mask checks.
        // If the value in both objects is equal and positive, the objects always collide.
        // If the value in both objects is equal and negative, the objects never collide.
        public int GroupIndex;

        // Return false if the filter cannot collide with anything,
        // which likely means it was default constructed but not initialized.
        public bool IsValid => CategoryBits > 0 && MaskBits > 0;

        // A collision filter which wants to collide with everything.
        public static readonly CollisionFilter Default = new CollisionFilter
        {
            CategoryBits = 0xffffffff,
            MaskBits = 0xffffffff,
            GroupIndex = 0
        };

        // A collision filter which never collides with against anything (including Default).
        public static readonly CollisionFilter Zero = new CollisionFilter
        {
            CategoryBits = 0,
            MaskBits = 0,
            GroupIndex = 0
        };

        // Return true if the given pair of filters want to collide with each other.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsCollisionEnabled(CollisionFilter filterA, CollisionFilter filterB)
        {
            if (filterA.GroupIndex > 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return true;
            }
            if (filterA.GroupIndex < 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return false;
            }
            return
                (filterA.CategoryBits & filterB.MaskBits) != 0 &&
                (filterB.CategoryBits & filterA.MaskBits) != 0;
        }

        // Return a union of two filters.
        public static CollisionFilter CreateUnion(CollisionFilter filterA, CollisionFilter filterB)
        {
            return new CollisionFilter
            {
                CategoryBits = filterA.CategoryBits | filterB.CategoryBits,
                MaskBits = filterA.MaskBits | filterB.MaskBits,
                GroupIndex = (filterA.GroupIndex == filterB.GroupIndex) ? filterA.GroupIndex : 0
            };
        }
    }
}
