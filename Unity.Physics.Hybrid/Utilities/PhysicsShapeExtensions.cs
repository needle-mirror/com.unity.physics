using System;
using System.Collections.Generic;
using Unity.Collections;
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

        static readonly List<PhysicsBody> s_PhysicsBodiesBuffer = new List<PhysicsBody>(16);
        static readonly List<Rigidbody> s_RigidbodiesBuffer = new List<Rigidbody>(16);
        static readonly List<UnityCollider> s_CollidersBuffer = new List<UnityCollider>(16);
        static readonly List<PhysicsShape> s_ShapesBuffer = new List<PhysicsShape>(16);

        internal static GameObject GetPrimaryBody(GameObject shape)
        {
            var pb = FindFirstEnabledAncestorBody(shape, s_PhysicsBodiesBuffer);
            var rb = FindFirstEnabledAncestorBody(shape, s_RigidbodiesBuffer);
            if (pb != null)
            {
                return rb == null ? pb.gameObject :
                    pb.transform.IsChildOf(rb.transform) ? pb.gameObject : rb.gameObject;
            }
            if (rb != null)
                return rb.gameObject;
            // for implicit static shape, find topmost enabled Collider or PhysicsShape
            var topCollider = FindTopmostEnabledAncestorShape(shape, s_CollidersBuffer);
            var topShape = FindTopmostEnabledAncestorShape(shape, s_ShapesBuffer);
            return topCollider == null
                ? topShape == null ? shape.gameObject : topShape
                : topShape == null
                    ? topCollider
                    : topShape.transform.IsChildOf(topCollider.transform)
                        ? topCollider
                        : topShape;
        }

        static GameObject FindFirstEnabledAncestorBody<T>(GameObject shape, List<T> buffer) where T : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            shape.GetComponentsInParent(true, buffer);
            GameObject result = null;
            for (int i = 0, count = buffer.Count; i < count; ++i)
            {
                if ((buffer[i] as MonoBehaviour)?.enabled ?? true)
                {
                    result = buffer[i].gameObject;
                    break;
                }
            }
            buffer.Clear();
            return result;
        }
        
        static GameObject FindTopmostEnabledAncestorShape<T>(GameObject shape, List<T> buffer) where T : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            shape.GetComponentsInParent(true, buffer);
            GameObject result = null;
            for (var i = buffer.Count - 1; i >= 0; --i)
            {
                if ((buffer[i] as UnityCollider)?.enabled ?? (buffer[i] as MonoBehaviour)?.enabled ?? true)
                {
                    result = buffer[i].gameObject;
                    break;
                }
            }
            buffer.Clear();
            return result;
        }
        
        // matrix to transform point from shape's local basis into world space
        static float4x4 GetBasisToWorldMatrix(
            float4x4 localToWorld, float3 center, quaternion orientation, float3 size
        ) =>
            math.mul(localToWorld, float4x4.TRS(center, orientation, size));

        // matrix to transform point on a primitive from bake space into space of the shape
        static float4x4 GetPrimitiveBakeToShapeMatrix(
            PhysicsShape shape, ref float3 center, ref EulerAngles orientation, float3 scale, int3 basisPriority
        )
        {
            if (
                basisPriority.x == basisPriority.y
                || basisPriority.x == basisPriority.z
                || basisPriority.y == basisPriority.z
            )
                throw new ArgumentException(nameof(basisPriority));

            var localToBasis = float4x4.TRS(center, orientation, scale);
            var localToBake = math.mul(shape.transform.localToWorldMatrix, localToBasis);

            // deskew second longest axis with respect to longest axis
            localToBake[basisPriority[1]] =
                DeskewSecondaryAxis(localToBake[basisPriority[0]], localToBake[basisPriority[1]]);
            // recompute third axes from first two
            var n2 = math.normalizesafe(
                new float4(math.cross(localToBake[basisPriority[0]].xyz, localToBake[basisPriority[1]].xyz), 0f)
            );
            localToBake[basisPriority[2]] = n2 * math.dot(localToBake[basisPriority[2]], n2);

            var bakeToShape = math.mul(math.inverse(shape.GetShapeToWorldMatrix()), localToBake);
            // transform baked center/orientation (i.e. primitive basis) into shape space
            orientation.SetValue(
                quaternion.LookRotationSafe(bakeToShape[basisPriority[0]].xyz, bakeToShape[basisPriority[1]].xyz)
            );
            center = bakeToShape.c3.xyz;

            return bakeToShape;
        }

        static float4 DeskewSecondaryAxis(float4 primaryAxis, float4 secondaryAxis)
        {
            var n0 = math.normalizesafe(primaryAxis);
            var dot = math.dot(secondaryAxis, n0);
            return secondaryAxis - n0 * dot;
        }

        static readonly int[] k_NextAxis = { 1, 2, 0 };
        static readonly int[] k_PrevAxis = { 2, 0, 1 };

        // used for de-skewing basis vectors; default priority assumes primary axis is z, secondary axis is y
        static readonly int3 k_DefaultAxisPriority = new int3(2, 1, 0);

        // priority is determined by length of each size dimension in the shape's basis after applying localToWorld transformation 
        static int3 GetBasisAxisPriority(float4x4 basisToWorld)
        {
            var basisAxisLengths =
                new float3(math.length(basisToWorld.c0), math.length(basisToWorld.c1), math.length(basisToWorld.c2));
            var max = math.cmax(basisAxisLengths);
            var min = math.cmin(basisAxisLengths);
            if (max == min)
                return k_DefaultAxisPriority;

            var imax = max == basisAxisLengths.x ? 0 : max == basisAxisLengths.y ? 1 : 2;

            basisToWorld[k_NextAxis[imax]] = DeskewSecondaryAxis(basisToWorld[imax], basisToWorld[k_NextAxis[imax]]);
            basisToWorld[k_PrevAxis[imax]] = DeskewSecondaryAxis(basisToWorld[imax], basisToWorld[k_PrevAxis[imax]]);

            basisAxisLengths =
                new float3(math.length(basisToWorld.c0), math.length(basisToWorld.c1), math.length(basisToWorld.c2));
            min = math.cmin(basisAxisLengths);
            var imin = min == basisAxisLengths.x ? 0 : min == basisAxisLengths.y ? 1 : 2;
            if (imin == imax)
                imin = k_NextAxis[imax];
            var imid = k_NextAxis[imax] == imin ? k_PrevAxis[imax] : k_NextAxis[imax];

            return new int3(imax, imid, imin);
        }

        internal static void GetBakedBoxProperties(
            this PhysicsShape shape,
            out float3 center, out float3 size, out EulerAngles orientation, out float convexRadius
        )
        {
            shape.GetBoxProperties(out center, out size, out orientation);

            var basisToWorld = GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, orientation, size);
            var basisPriority = GetBasisAxisPriority(basisToWorld);

            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, size, basisPriority);

            var s = new float3(
                math.length(bakeToShape[basisPriority[2]]),
                math.length(bakeToShape[basisPriority[1]]),
                math.length(bakeToShape[basisPriority[0]])
            );
            convexRadius = shape.ConvexRadius * math.cmin(s / size);

            size = s;
        }

        static void MakeZAxisPrimaryBasis(ref int3 basisPriority)
        {
            if (basisPriority[1] == 2)
                basisPriority = basisPriority.yxz;
            else if (basisPriority[2] == 2)
                basisPriority = basisPriority.zxy;
        }

        internal static void GetBakedCapsuleProperties(
            this PhysicsShape shape,
            out float3 center, out float height, out float radius, out EulerAngles orientation,
            out float3 vertex0, out float3 vertex1
        )
        {
            shape.GetCapsuleProperties(out center, out height, out radius, out orientation);

            var s = new float3(radius * 2f, radius * 2f, height);
            var basisToWorld = GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, orientation, s);
            var basisPriority = GetBasisAxisPriority(basisToWorld);
            MakeZAxisPrimaryBasis(ref basisPriority);
            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

            height *= math.length(bakeToShape.c2);
            radius *= math.max(math.length(bakeToShape.c0), math.length(bakeToShape.c1));

            var axis = math.mul(orientation, new float3 { z = 1f });
            var endPoint = axis * math.max(0f, 0.5f * height - radius);
            vertex0 = center + endPoint;
            vertex1 = center - endPoint;
        }

        internal static void GetBakedCylinderProperties(
            this PhysicsShape shape,
            out float3 center, out float height, out float radius, out EulerAngles orientation, out float convexRadius
        )
        {
            shape.GetCylinderProperties(out center, out height, out radius, out orientation);

            var size = new float3(radius * 2f, radius * 2f, height);
            var basisToWorld = GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, orientation, size);
            var basisPriority = GetBasisAxisPriority(basisToWorld);
            MakeZAxisPrimaryBasis(ref basisPriority);
            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

            height *= math.length(bakeToShape.c2);
            radius *= math.max(math.length(bakeToShape.c0), math.length(bakeToShape.c1));

            var s = new float3(
                math.length(bakeToShape[basisPriority[2]]),
                math.length(bakeToShape[basisPriority[1]]),
                math.length(bakeToShape[basisPriority[0]])
            );
            convexRadius = math.min(shape.ConvexRadius * math.cmax(s / size), height * 0.5f);
        }

        internal static void GetBakedSphereProperties(
            this PhysicsShape shape, out float3 center, out float radius, out EulerAngles orientation
        )
        {
            shape.GetSphereProperties(out center, out radius, out orientation);

            var basisToWorld = GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, orientation, 1f);
            var basisPriority = GetBasisAxisPriority(basisToWorld);
            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

            radius *= math.cmax(
                new float3(math.length(bakeToShape.c0), math.length(bakeToShape.c1), math.length(bakeToShape.c2))
            );
        }

        internal static void GetPlanePoints(
            this PhysicsShape shape, out float3 vertex0, out float3 vertex1, out float3 vertex2, out float3 vertex3
        )
        {
            shape.GetPlaneProperties(out var center, out var size, out EulerAngles orientation);

            var sizeYUp = math.float3(size.x, 0, size.y);

            vertex0 = center + math.mul(orientation, sizeYUp * math.float3(-0.5f, 0,  0.5f));
            vertex1 = center + math.mul(orientation, sizeYUp * math.float3( 0.5f, 0,  0.5f));
            vertex2 = center + math.mul(orientation, sizeYUp * math.float3( 0.5f, 0, -0.5f));
            vertex3 = center + math.mul(orientation, sizeYUp * math.float3(-0.5f, 0, -0.5f));
        }

        internal static void GetBakedPlaneProperties(
            this PhysicsShape shape, out float3 vertex0, out float3 vertex1, out float3 vertex2, out float3 vertex3
        )
        {
            GetPlanePoints(shape, out vertex0, out vertex1, out vertex2, out vertex3);
            var localToShape = shape.GetLocalToShapeMatrix();
            vertex0 = math.mul(localToShape, new float4(vertex0, 1f)).xyz;
            vertex1 = math.mul(localToShape, new float4(vertex1, 1f)).xyz;
            vertex2 = math.mul(localToShape, new float4(vertex2, 1f)).xyz;
            vertex3 = math.mul(localToShape, new float4(vertex3, 1f)).xyz;
        }

        internal static void GetBakedConvexProperties(
            this PhysicsShape shape, NativeList<float3> pointCloud, out float radius
        )
        {
            shape.GetConvexHullProperties(pointCloud);

            shape.BakePoints(pointCloud);

            // compute convex radius
            var center = float3.zero;
            var orientation = EulerAngles.Default;
            var bakeToShape =
                GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, k_DefaultAxisPriority);
            var s = new float3(math.length(bakeToShape[0]), math.length(bakeToShape[1]), math.length(bakeToShape[2]));
            radius = shape.ConvexRadius * math.cmin(s);
        }

        static readonly List<Vector3> s_Vertices = new List<Vector3>(65535);

        internal static void GetBakedMeshProperties(
            this PhysicsShape shape, NativeList<float3> pointCloud, NativeList<int> triangles
        )
        {
            // TODO: currently only handling a single mesh
            var mesh = shape.GetMesh();
            if (mesh == null)
                return;
            mesh.GetVertices(s_Vertices);
            foreach (var v in s_Vertices)
                pointCloud.Add(v);
            shape.BakePoints(pointCloud);

            // TODO: use non-allocating accessors
            using (var cpy = new NativeArray<int>(mesh.triangles, Allocator.Temp))
                triangles.AddRange(cpy);
        }
    }
}
