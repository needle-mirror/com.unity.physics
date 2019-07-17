using System;
using System.ComponentModel;
using Unity.Entities;

// all deprecated types in this assembly should go in this file, if possible
// deprecated members in core types should appear in-line rather than making structs partial

namespace Unity.Physics
{
    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("CustomDataPair has been deprecated. Use CustomTagsPair instead. (RemovedAfter 2019-10-03)", true)]
    public struct CustomDataPair { }

    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("PhysicsCustomData has been deprecated. Use PhysicsCustomTags instead. (RemovedAfter 2019-10-03) (UnityUpgradable) -> PhysicsCustomTags", true)]
    public struct PhysicsCustomData : IComponentData { }
}
