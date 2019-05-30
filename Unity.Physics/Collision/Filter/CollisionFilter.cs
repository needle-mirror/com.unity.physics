using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Unity.Physics
{
    // Describes which other objects an object can collide with.
    [DebuggerDisplay("Group: {GroupIndex} BelongsTo: {BelongsTo} CollidesWith: {CollidesWith}")]
    [StructLayout(LayoutKind.Explicit)] // marked up so obsolete fields are upgradeable; can remove when they expire
    public struct CollisionFilter
    {
        // A bit mask describing which layers this object belongs to.
        [FieldOffset(0)]
        public uint BelongsTo;
        [FieldOffset(0)]
        [Obsolete("CategoryBits has been deprecated. Use BelongsTo instead (RemovedAfter 2019-07-29) (UnityUpgradable) -> BelongsTo")]
        [EditorBrowsable(EditorBrowsableState.Never)]
        public uint CategoryBits;

        // A bit mask describing which layers this object can collide with.
        [FieldOffset(4)]
        public uint CollidesWith;
        [FieldOffset(4)]
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("MaskBits has been deprecated. Use CollidesWith instead (RemovedAfter 2019-07-29) (UnityUpgradable) -> CollidesWith")]
        public uint MaskBits;

        // An optional override for the bit mask checks.
        // If the value in both objects is equal and positive, the objects always collide.
        // If the value in both objects is equal and negative, the objects never collide.
        [FieldOffset(8)]
        public int GroupIndex;

        // Return false if the filter cannot collide with anything,
        // which likely means it was default constructed but not initialized.
        public bool IsValid => BelongsTo > 0 && CollidesWith > 0;

        // A collision filter which wants to collide with everything.
        public static readonly CollisionFilter Default = new CollisionFilter
        {
            BelongsTo = 0xffffffff,
            CollidesWith = 0xffffffff,
            GroupIndex = 0
        };

        // A collision filter which never collides with against anything (including Default).
        public static readonly CollisionFilter Zero = new CollisionFilter
        {
            BelongsTo = 0,
            CollidesWith = 0,
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
                (filterA.BelongsTo & filterB.CollidesWith) != 0 &&
                (filterB.BelongsTo & filterA.CollidesWith) != 0;
        }

        // Return a union of two filters.
        public static CollisionFilter CreateUnion(CollisionFilter filterA, CollisionFilter filterB)
        {
            return new CollisionFilter
            {
                BelongsTo = filterA.BelongsTo | filterB.BelongsTo,
                CollidesWith = filterA.CollidesWith | filterB.CollidesWith,
                GroupIndex = (filterA.GroupIndex == filterB.GroupIndex) ? filterA.GroupIndex : 0
            };
        }
    }
}
