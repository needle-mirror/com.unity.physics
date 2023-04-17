using System.ComponentModel;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    public enum ShapeType
    {
        Box        =  0,
        Capsule    =  1,
        Sphere     =  2,

        [Description("Cylinder (Convex Hull)")]
        Cylinder   =  3,

        Plane      =  4,

        // extra space to accommodate other possible primitives in the future

        ConvexHull = 30,
        Mesh       = 31
    }
}
