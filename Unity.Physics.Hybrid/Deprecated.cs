using System;
using System.ComponentModel;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;
using UnityMesh = UnityEngine.Mesh;

// all deprecated API points in this assembly should go in this file, if possible

namespace Unity.Physics.Authoring
{
    public sealed partial class PhysicsShapeAuthoring
    {
        [FormerlySerializedAs("m_ConvexRadius")]
        [SerializeField, HideInInspector]
        float m_ConvexRadius_Deprecated = -1f;

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Use the signature returning CapsuleGeometryAuthoring instead. (RemovedAfter 2020-05-07)")]
        public CapsuleGeometry GetCapsuleProperties(out quaternion orientation)
        {
            var capsule = GetCapsuleProperties();
            orientation = capsule.Orientation;
            return capsule.ToRuntime();
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Use the signature taking CapsuleGeometryAuthoring instead. (RemovedAfter 2020-05-07)")]
        // TODO: Remove GetCapsuleProperties_WhenPointsAndOrientationNotIdentity_OrientationPointsDownAxisOfCompositeRotation()
        public void SetCapsule(CapsuleGeometry geometry, quaternion orientation)
        {
            var geometryAuthoring = geometry.ToAuthoring();
            geometryAuthoring.Orientation = math.mul(orientation, quaternion.LookRotationSafe(geometry.Vertex1 - geometry.Vertex0, math.up()));
            SetCapsule(geometryAuthoring);
        }
    }
}
