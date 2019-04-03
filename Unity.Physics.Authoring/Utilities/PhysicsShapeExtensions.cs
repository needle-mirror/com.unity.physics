using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityCollider = UnityEngine.Collider;

namespace Unity.Physics.Authoring
{
    static class PhysicsShapeExtensions
    {
        internal static int GetDeviantAxis(this float3 v)
        {
            var deviation = math.abs(v - math.csum(v) / 3f);
            return math.cmax(deviation) == deviation.z ? 2 : math.cmax(deviation) == deviation.y ? 1 : 0;
        }

        internal static int GetMaxAxis(this float3 v)
        {
            var cmax = math.cmax(v);
            return cmax == v.z ? 2 : cmax == v.y ? 1 : 0;
        }

        internal static GameObject GetPrimaryBody(this UnityCollider collider) => GetPrimaryBody(collider.gameObject);
        internal static GameObject GetPrimaryBody(this PhysicsShape shape) => GetPrimaryBody(shape.gameObject);

        static readonly List<UnityCollider> s_CollidersBuffer = new List<UnityCollider>(16);
        static readonly List<PhysicsShape> s_ShapesBuffer = new List<PhysicsShape>(16);

        internal static GameObject GetPrimaryBody(GameObject shape)
        {
            var pb = shape.GetComponentInParent<PhysicsBody>();
            var rb = shape.GetComponentInParent<Rigidbody>();
            if (pb != null)
            {
                return rb == null ? pb.gameObject :
                    pb.transform.IsChildOf(rb.transform) ? pb.gameObject : rb.gameObject;
            }
            if (rb != null)
                return rb.gameObject;
            // for implicit static shape, find topmost Collider or PhysicsShape
            shape.gameObject.GetComponentsInParent(false, s_CollidersBuffer);
            shape.gameObject.GetComponentsInParent(false, s_ShapesBuffer);
            var topCollider = s_CollidersBuffer.Count > 0
                ? s_CollidersBuffer[s_CollidersBuffer.Count - 1].gameObject
                : null;
            s_CollidersBuffer.Clear();
            var topShape = s_ShapesBuffer.Count > 0 ? s_ShapesBuffer[s_ShapesBuffer.Count - 1].gameObject : null;
            s_ShapesBuffer.Clear();
            return topCollider == null
                ? topShape == null ? shape.gameObject : topShape
                : topShape == null
                    ? topCollider
                    : topShape.transform.IsChildOf(topCollider.transform)
                        ? topCollider
                        : topShape;
        }

        internal static void GetBakeTransformation(
            this PhysicsShape shape, out float3 linearScalar, out float radiusScalar
        )
        {
            linearScalar = shape.transform.lossyScale;

            radiusScalar = 1f;
            var s = math.abs(linearScalar);
            switch (shape.ShapeType)
            {
                case ShapeType.Box:
                    radiusScalar = math.cmin(s);
                    break;
                case ShapeType.Sphere:
                    radiusScalar = math.cmax(s);
                    break;
                case ShapeType.Capsule:
                    var cmax = math.cmax(s);
                    var cmin = math.cmin(s);
                    var cmaxI = cmax == s.z ? 2 : cmax == s.y ? 1 : 0;
                    var cminI = cmin == s.z ? 2 : cmin == s.y ? 1 : 0;
                    var cmidI = cmaxI == 2 ? (cminI == 1 ? 0 : 1) : (cminI == 1 ? 2 : 0);
                    radiusScalar = s[cmidI];
                    break;
                case ShapeType.Cylinder:
                    radiusScalar = math.cmax(s);
                    break;
                case ShapeType.ConvexHull:
                    radiusScalar = math.cmax(s);
                    break;
            }
        }
    }
}
