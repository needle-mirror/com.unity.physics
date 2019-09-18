using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Scenes;
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

        const float k_HashFloatTolerance = 0.01f;

        // used to hash convex hull generation properties in a way that is robust to imprecision
        internal static uint GetStableHash(
            this ConvexHullGenerationParameters generationParameters,
            ConvexHullGenerationParameters hashedParameters,
            float tolerance = k_HashFloatTolerance
        )
        {
            var differences = new float3(
                generationParameters.BevelRadius - hashedParameters.BevelRadius,
                generationParameters.MinimumAngle - hashedParameters.MinimumAngle,
                generationParameters.SimplificationTolerance - hashedParameters.SimplificationTolerance
            );
            return math.cmax(math.abs(differences)) < tolerance
                ? unchecked((uint)hashedParameters.GetHashCode())
                : unchecked((uint)generationParameters.GetHashCode());
        }

        // used to hash an array of points in a way that is robust to imprecision
        internal static unsafe uint GetStableHash(
            this NativeList<float3> points, NativeArray<float3> hashedPoints, float tolerance = k_HashFloatTolerance
        )
        {
            if (points.Length != hashedPoints.Length)
                return math.hash(points.GetUnsafePtr(), UnsafeUtility.SizeOf<float3>() * points.Length);

            for (int i = 0, count = points.Length; i < count; ++i)
            {
                if (math.cmax(math.abs(points[i] - hashedPoints[i])) > tolerance)
                    return math.hash(points.GetUnsafePtr(), UnsafeUtility.SizeOf<float3>() * points.Length);
            }
            return math.hash(hashedPoints.GetUnsafePtr(), UnsafeUtility.SizeOf<float3>() * hashedPoints.Length);
        }

        static bool HasNonUniformScale(this float4x4 m)
        {
            var s = new float3(math.lengthsq(m.c0.xyz), math.lengthsq(m.c1.xyz), math.lengthsq(m.c2.xyz));
            return math.cmin(s) != math.cmax(s);
        }

        internal static bool HasShear(this float4x4 m)
        {
            // scale each axis by abs of its max component in order to work with very large/small scales
            var rs0 = m.c0.xyz / math.max(math.cmax(math.abs(m.c0.xyz)), float.Epsilon);
            var rs1 = m.c1.xyz / math.max(math.cmax(math.abs(m.c1.xyz)), float.Epsilon);
            var rs2 = m.c2.xyz / math.max(math.cmax(math.abs(m.c2.xyz)), float.Epsilon);
            // verify all axes are orthogonal
            const float k_Zero = 1e-6f;
            return
                math.abs(math.dot(rs0, rs1)) > k_Zero ||
                math.abs(math.dot(rs0, rs2)) > k_Zero ||
                math.abs(math.dot(rs1, rs2)) > k_Zero;
        }

        // TODO: revisit readable requirement when conversion is offline-only
        internal static bool IsValidForConversion(this UnityEngine.Mesh mesh, GameObject host)
        {
            if (mesh.isReadable)
                return true;
            if (Application.isPlaying)
                return false;
            var subScene = host.GetComponentInParent<SubScene>();
            return subScene != null;
        }

        internal static GameObject GetPrimaryBody(this UnityCollider collider) => GetPrimaryBody(collider.gameObject);
        internal static GameObject GetPrimaryBody(this PhysicsShapeAuthoring shape) => GetPrimaryBody(shape.gameObject);

        static readonly List<PhysicsBodyAuthoring> s_PhysicsBodiesBuffer = new List<PhysicsBodyAuthoring>(16);
        static readonly List<Rigidbody> s_RigidbodiesBuffer = new List<Rigidbody>(16);
        static readonly List<UnityCollider> s_CollidersBuffer = new List<UnityCollider>(16);
        static readonly List<PhysicsShapeAuthoring> s_ShapesBuffer = new List<PhysicsShapeAuthoring>(16);
        static readonly List<StaticOptimizeEntity> s_StaticOptimizeEntitiesBuffer = new List<StaticOptimizeEntity>(4);

        internal static GameObject GetPrimaryBody(GameObject shape)
        {
            var pb = FindFirstEnabledAncestor(shape, s_PhysicsBodiesBuffer);
            var rb = FindFirstEnabledAncestor(shape, s_RigidbodiesBuffer);

            if (pb != null)
            {
                return rb == null ? pb.gameObject :
                    pb.transform.IsChildOf(rb.transform) ? pb.gameObject : rb.gameObject;
            }

            if (rb != null)
                return rb.gameObject;

            // for implicit static shape, first see if it is part of static optimized hierarchy
            var topStatic = FindTopmostEnabledAncestor(shape, s_StaticOptimizeEntitiesBuffer);
            if (topStatic != null)
                return topStatic;

            // otherwise, find topmost enabled Collider or PhysicsShapeAuthoring
            var topCollider = FindTopmostEnabledAncestor(shape, s_CollidersBuffer);
            var topShape = FindTopmostEnabledAncestor(shape, s_ShapesBuffer);

            return topCollider == null
                ? topShape == null ? shape.gameObject : topShape
                : topShape == null
                    ? topCollider
                    : topShape.transform.IsChildOf(topCollider.transform)
                        ? topCollider
                        : topShape;
        }

        static GameObject FindFirstEnabledAncestor<T>(GameObject shape, List<T> buffer) where T : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            shape.GetComponentsInParent(true, buffer);
            GameObject result = null;
            for (int i = 0, count = buffer.Count; i < count; ++i)
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
        
        static GameObject FindTopmostEnabledAncestor<T>(GameObject shape, List<T> buffer) where T : Component
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
            PhysicsShapeAuthoring shape, ref float3 center, ref EulerAngles orientation, float3 scale, int3 basisPriority
        )
        {
            if (
                basisPriority.x == basisPriority.y
                || basisPriority.x == basisPriority.z
                || basisPriority.y == basisPriority.z
            )
                throw new ArgumentException(nameof(basisPriority));

            var localToBasis = float4x4.TRS(center, orientation, scale);
            // correct for imprecision in cases of no scale to prevent e.g., convex radius from being altered
            if (scale.Equals(new float3(1f)))
            {
                localToBasis.c0 = math.normalizesafe(localToBasis.c0);
                localToBasis.c1 = math.normalizesafe(localToBasis.c1);
                localToBasis.c2 = math.normalizesafe(localToBasis.c2);
            }
            var localToBake = math.mul(shape.transform.localToWorldMatrix, localToBasis);

            if (localToBake.HasNonUniformScale() || localToBake.HasShear())
            {
                // deskew second longest axis with respect to longest axis
                localToBake[basisPriority[1]] =
                    DeskewSecondaryAxis(localToBake[basisPriority[0]], localToBake[basisPriority[1]]);

                // recompute third axes from first two
                var n2 = math.normalizesafe(
                    new float4(math.cross(localToBake[basisPriority[0]].xyz, localToBake[basisPriority[1]].xyz), 0f)
                );
                localToBake[basisPriority[2]] = n2 * math.dot(localToBake[basisPriority[2]], n2);
            }

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

        internal static BoxGeometry GetBakedBoxProperties(this PhysicsShapeAuthoring shape)
        {
            var box = shape.GetBoxProperties(out var orientation);
            var center = box.Center;
            var size = box.Size;
            var bevelRadius = box.BevelRadius;

            var localToWorld = (float4x4)shape.transform.localToWorldMatrix;
            float4x4 bakeToShape;
            var basisPriority = k_DefaultAxisPriority;
            var sheared = localToWorld.HasShear();
            if (localToWorld.HasNonUniformScale() || sheared)
            {
                if (sheared)
                {
                    var basisToWorld =
                        GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, orientation, size);
                    basisPriority = GetBasisAxisPriority(basisToWorld);
                }

                bakeToShape =
                    GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, size, basisPriority);

                var s = new float3(
                    math.length(bakeToShape[basisPriority[2]]),
                    math.length(bakeToShape[basisPriority[1]]),
                    math.length(bakeToShape[basisPriority[0]])
                );

                bevelRadius *= math.cmin(s / size);
                size = s;
            }
            else
            {
                bakeToShape =
                    GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

                var s = new float3(
                    math.length(bakeToShape.c0),
                    math.length(bakeToShape.c1),
                    math.length(bakeToShape.c2)
                );

                bevelRadius *= math.cmin(s);
                size *= s;
            }

            return new BoxGeometry
            {
                Center = center,
                Orientation = orientation,
                Size = size,
                BevelRadius = bevelRadius
            };
        }

        static void MakeZAxisPrimaryBasis(ref int3 basisPriority)
        {
            if (basisPriority[1] == 2)
                basisPriority = basisPriority.yxz;
            else if (basisPriority[2] == 2)
                basisPriority = basisPriority.zxy;
        }

        internal static float3 GetCenter(this CapsuleGeometry geometry) =>
            math.lerp(geometry.Vertex0, geometry.Vertex1, 0.5f);

        internal static float GetHeight(this CapsuleGeometry geometry) =>
            2f * geometry.Radius + math.length(geometry.Vertex1 - geometry.Vertex0);

        internal static CapsuleGeometry GetBakedCapsuleProperties(
            this PhysicsShapeAuthoring shape,
            out float3 center, out float height, out EulerAngles orientation
        )
        {
            var capsule = shape.GetCapsuleProperties(out orientation);
            var radius = capsule.Radius;
            center = capsule.GetCenter();
            height = capsule.GetHeight();

            var s = new float3(radius * 2f, radius * 2f, height);
            var localToWorld = (float4x4)shape.transform.localToWorldMatrix;
            var basisToWorld = GetBasisToWorldMatrix(localToWorld, center, orientation, s);
            var basisPriority = k_DefaultAxisPriority;
            var sheared = localToWorld.HasShear();
            if (localToWorld.HasNonUniformScale() || sheared)
            {
                if (sheared)
                    basisPriority = GetBasisAxisPriority(basisToWorld);
                MakeZAxisPrimaryBasis(ref basisPriority);
            }
            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

            height *= math.length(bakeToShape.c2);
            radius *= math.max(math.length(bakeToShape.c0), math.length(bakeToShape.c1));

            var axis = math.mul(orientation, new float3 { z = 1f });
            var endPoint = axis * math.max(0f, 0.5f * height - radius);

            return new CapsuleGeometry
            {
                Vertex0 = center + endPoint,
                Vertex1 = center - endPoint,
                Radius = radius
            };
        }

        internal static CylinderGeometry GetBakedCylinderProperties(this PhysicsShapeAuthoring shape)
        {
            var cylinder = shape.GetCylinderProperties(out var orientation);
            var center = cylinder.Center;
            var height = cylinder.Height;
            var radius = cylinder.Radius;

            var size = new float3(radius * 2f, radius * 2f, height);
            var localToWorld = (float4x4)shape.transform.localToWorldMatrix;
            var basisToWorld = GetBasisToWorldMatrix(localToWorld, center, orientation, size);
            var basisPriority = k_DefaultAxisPriority;
            var sheared = localToWorld.HasShear();
            if (localToWorld.HasNonUniformScale() || sheared)
            {
                if (sheared)
                    basisPriority = GetBasisAxisPriority(basisToWorld);
                MakeZAxisPrimaryBasis(ref basisPriority);
            }
            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

            height *= math.length(bakeToShape.c2);
            radius *= math.max(math.length(bakeToShape.c0), math.length(bakeToShape.c1));

            var s = new float3(
                math.length(bakeToShape[basisPriority[2]]),
                math.length(bakeToShape[basisPriority[1]]),
                math.length(bakeToShape[basisPriority[0]])
            );

            return new CylinderGeometry
            {
                Center = center,
                Orientation = orientation,
                Height = height,
                Radius = radius,
                BevelRadius = math.min(cylinder.BevelRadius * math.cmax(s / size), height * 0.5f),
                SideCount = cylinder.SideCount
            };
        }

        internal static SphereGeometry GetBakedSphereProperties(this PhysicsShapeAuthoring shape, out EulerAngles orientation)
        {
            var sphere = shape.GetSphereProperties(out orientation);
            var center = sphere.Center;
            var radius = sphere.Radius;

            var basisToWorld = GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, orientation, 1f);
            var basisPriority = basisToWorld.HasShear() ? GetBasisAxisPriority(basisToWorld) : k_DefaultAxisPriority;
            var bakeToShape = GetPrimitiveBakeToShapeMatrix(shape, ref center, ref orientation, 1f, basisPriority);

            radius *= math.cmax(
                new float3(math.length(bakeToShape.c0), math.length(bakeToShape.c1), math.length(bakeToShape.c2))
            );

            return new SphereGeometry
            {
                Center = center,
                Radius = radius
            };
        }

        internal static void GetPlanePoints(
            this PhysicsShapeAuthoring shape, out float3 vertex0, out float3 vertex1, out float3 vertex2, out float3 vertex3
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
            this PhysicsShapeAuthoring shape, out float3 vertex0, out float3 vertex1, out float3 vertex2, out float3 vertex3
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
            this PhysicsShapeAuthoring shape, NativeList<float3> pointCloud, out ConvexHullGenerationParameters generationParameters
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
            generationParameters = shape.ConvexHullGenerationParameters;
            generationParameters.SimplificationTolerance = math.max(
                ConvexHullGenerationParametersExtensions.k_MinRecommendedSimplificationTolerance,
                math.cmax(s) * generationParameters.SimplificationTolerance
            );
            generationParameters.BevelRadius *= math.cmin(s);
        }

        internal static void GetBakedMeshProperties(
            this PhysicsShapeAuthoring shape, NativeList<float3> pointCloud, NativeList<int> triangles
        )
        {
            shape.GetMeshProperties(pointCloud, triangles);
            shape.BakePoints(pointCloud);
        }
    }
}
