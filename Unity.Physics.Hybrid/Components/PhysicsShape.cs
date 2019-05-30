using System;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityMesh = UnityEngine.Mesh;

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

    [Serializable]
    struct EulerAngles
    {
        public float3 Value;
        [HideInInspector]
        public math.RotationOrder RotationOrder;

        public void SetValue(quaternion value)
        {
            if (RotationOrder != math.RotationOrder.ZXY)
                throw new NotSupportedException();
            // TODO: add generic Euler decomposition to math.quaternion
            Value = ((Quaternion)value).eulerAngles;
        }

        public static implicit operator quaternion(EulerAngles euler) =>
            quaternion.Euler(math.radians(euler.Value), euler.RotationOrder);

        public static EulerAngles Default => new EulerAngles { RotationOrder = math.RotationOrder.ZXY };
    }

    sealed class UnimplementedShapeException : NotImplementedException
    {
        public UnimplementedShapeException(ShapeType shapeType)
            : base($"Unknown shape type {shapeType} requires explicit implementation") { }
    }

    [AddComponentMenu("DOTS/Physics/Physics Shape")]
    [DisallowMultipleComponent]
    [RequiresEntityConversion]
    public sealed class PhysicsShape : MonoBehaviour, IInheritPhysicsMaterialProperties
    {
        [Serializable]
        struct CylindricalProperties
        {
            public float Height;
            public float Radius;
            [HideInInspector]
            public int Axis;
        }

        static readonly int[] k_NextAxis = { 1, 2, 0 };
        internal const float k_DefaultConvexRadius = 0.05f;

        public ShapeType ShapeType => m_ShapeType;
        [SerializeField]
        ShapeType m_ShapeType = ShapeType.Box;

        [SerializeField]
        float3 m_PrimitiveCenter;

        [SerializeField]
        float3 m_PrimitiveSize = new float3(1f, 1f, 1f);

        [SerializeField]
        EulerAngles m_PrimitiveOrientation = EulerAngles.Default;

        [SerializeField]
        [ExpandChildren]
        CylindricalProperties m_Capsule = new CylindricalProperties { Height = 1f, Radius = 0.5f, Axis = 2 };

        [SerializeField]
        [ExpandChildren]
        CylindricalProperties m_Cylinder = new CylindricalProperties { Height = 1f, Radius = 0.5f, Axis = 2 };

        [SerializeField]
        float m_SphereRadius = 0.5f;

        public float ConvexRadius
        {
            get
            {
                switch (m_ShapeType)
                {
                    case ShapeType.Box:
                    case ShapeType.Cylinder:
                    case ShapeType.ConvexHull:
                        return m_ConvexRadius;
                    case ShapeType.Capsule:
                        return m_Capsule.Radius;
                    case ShapeType.Sphere:
                        return 0.5f * math.cmax(m_PrimitiveSize);
                    case ShapeType.Plane:
                    case ShapeType.Mesh:
                        return 0f;
                    default:
                        throw new UnimplementedShapeException(m_ShapeType);
                }
            }
            set
            {
                var maxRadius = float.MaxValue;
                switch (m_ShapeType)
                {
                    case ShapeType.Box:
                        maxRadius = 0.5f * math.cmin(m_PrimitiveSize);
                        break;
                    case ShapeType.Cylinder:
                        GetCylinderProperties(
                            out var center, out var height, out var radius, out EulerAngles orientation
                        );
                        maxRadius = math.min(0.5f * height, radius);
                        break;
                    case ShapeType.ConvexHull:
                    case ShapeType.Mesh:
                        // TODO : any benefit in clamping this e.g. maxRadius = k_DefaultConvexRadius;
                        break;
                    case ShapeType.Plane:
                        maxRadius = 0f;
                        break;
                    case ShapeType.Capsule:
                    case ShapeType.Sphere:
                        break;
                    default:
                        throw new UnimplementedShapeException(m_ShapeType);
                }
                m_ConvexRadius = math.clamp(m_ConvexRadius, 0f, maxRadius);
            }
        }
        [SerializeField]
        [Tooltip("Determines how rounded the corners of the convex shape will be. A value greater than 0 results in more optimized collision, at the cost of some shape detail.")]
        float m_ConvexRadius = k_DefaultConvexRadius;

        // TODO: remove this accessor in favor of GetRawVertices() when blob data is serializable
        internal UnityMesh CustomMesh => m_CustomMesh;
        [SerializeField]
        [Tooltip("If no custom mesh is specified, then one will be generated using this body's rendered meshes.")]
        UnityMesh m_CustomMesh;

        public PhysicsMaterialTemplate MaterialTemplate { get => m_Material.Template; set => m_Material.Template = value; }
        PhysicsMaterialTemplate IInheritPhysicsMaterialProperties.Template
        {
            get => m_Material.Template;
            set => m_Material.Template = value;
        }

        public bool OverrideIsTrigger { get => m_Material.OverrideIsTrigger; set => m_Material.OverrideIsTrigger = value; }
        public bool IsTrigger { get => m_Material.IsTrigger; set => m_Material.IsTrigger = value; }

        public bool OverrideFriction { get => m_Material.OverrideFriction; set => m_Material.OverrideFriction = value; }
        public PhysicsMaterialCoefficient Friction { get => m_Material.Friction; set => m_Material.Friction = value; }

        public bool OverrideRestitution
        {
            get => m_Material.OverrideRestitution;
            set => m_Material.OverrideRestitution = value;
        }
        public PhysicsMaterialCoefficient Restitution
        {
            get => m_Material.Restitution;
            set => m_Material.Restitution = value;
        }

        public bool OverrideBelongsTo
        {
            get => m_Material.OverrideBelongsTo;
            set => m_Material.OverrideBelongsTo = value;
        }
        public int BelongsTo { get => m_Material.BelongsTo; set => m_Material.BelongsTo = value; }
        public bool GetBelongsTo(int categoryIndex) => m_Material.GetBelongsTo(categoryIndex);
        public void SetBelongsTo(int categoryIndex, bool value) => m_Material.SetBelongsTo(categoryIndex, value);

        public bool OverrideCollidesWith
        {
            get => m_Material.OverrideCollidesWith;
            set => m_Material.OverrideCollidesWith = value;
        }
        public int CollidesWith { get => m_Material.CollidesWith; set => m_Material.CollidesWith = value; }
        public bool GetCollidesWith(int categoryIndex) => m_Material.GetCollidesWith(categoryIndex);
        public void SetCollidesWith(int categoryIndex, bool value) => m_Material.SetCollidesWith(categoryIndex, value);

        public bool OverrideRaisesCollisionEvents
        {
            get => m_Material.OverrideRaisesCollisionEvents;
            set => m_Material.OverrideRaisesCollisionEvents = value;
        }
        public bool RaisesCollisionEvents
        {
            get => m_Material.RaisesCollisionEvents;
            set => m_Material.RaisesCollisionEvents = value;
        }

        public bool OverrideCustomFlags
        {
            get => m_Material.OverrideCustomFlags;
            set => m_Material.OverrideCustomFlags = value;
        }
        public byte CustomFlags { get => m_Material.CustomFlags; set => m_Material.CustomFlags = value; }
        public bool GetCustomFlag(int customFlagIndex) => m_Material.GetCustomFlag(customFlagIndex);
        public void SetCustomFlag(int customFlagIndex, bool value) => m_Material.SetCustomFlag(customFlagIndex, value);

        [SerializeField]
        PhysicsMaterialProperties m_Material = new PhysicsMaterialProperties(true);

        public void GetBoxProperties(out float3 center, out float3 size, out quaternion orientation)
        {
            GetBoxProperties(out center, out size, out EulerAngles euler);
            orientation = euler;
        }

        internal void GetBoxProperties(out float3 center, out float3 size, out EulerAngles orientation)
        {
            center = m_PrimitiveCenter;
            size = m_PrimitiveSize;
            orientation = m_PrimitiveOrientation;
            // prefer identity and shuffle size if aligned with basis vectors
            const float tolerance = 0.00001f;
            var fwd = new float3 { z = 1f };
            var dotFwd = math.abs(math.dot(math.mul(orientation, fwd), fwd));
            var up = new float3 { y = 1f };
            var dotUp = math.abs(math.dot(math.mul(orientation, up), up));
            if (
                math.abs(math.dot(orientation, quaternion.identity)) > tolerance
                && (dotFwd < tolerance || 1f - dotFwd < tolerance)
                && (dotUp < tolerance || 1f - dotUp < tolerance)
            )
            {
                size = math.abs(math.mul(orientation, size)); // TODO: handle floating point error
                orientation.SetValue(quaternion.identity);
            }
        }

        void GetCylindricalProperties(
            CylindricalProperties props,
            out float3 center, out float height, out float radius, out EulerAngles orientation,
            bool rebuildOrientation
        )
        {
            center = m_PrimitiveCenter;
            var lookVector = math.mul(m_PrimitiveOrientation, new float3 { [props.Axis] = 1f });
            // use previous axis so forward will prefer up
            var upVector = math.mul(m_PrimitiveOrientation, new float3 { [k_NextAxis[k_NextAxis[props.Axis]]] = 1f });
            orientation = m_PrimitiveOrientation;
            if (rebuildOrientation && props.Axis != 2)
                orientation.SetValue(quaternion.LookRotation(lookVector, upVector));
            radius = props.Radius;
            height = props.Height;
        }

        public void GetCapsuleProperties(
            out float3 center, out float height, out float radius, out quaternion orientation
        )
        {
            GetCapsuleProperties(out center, out height, out radius, out EulerAngles euler);
            orientation = euler;
        }

        internal void GetCapsuleProperties(
            out float3 center, out float height, out float radius, out EulerAngles orientation
        )
        {
            GetCylindricalProperties(
                m_Capsule, out center, out height, out radius, out orientation, m_ShapeType != ShapeType.Capsule
            );
        }

        [Obsolete("RemovedAfter 2019-08-10")]
        public void GetCapsuleProperties(out float3 vertex0, out float3 vertex1, out float radius)
        {
            UpdateCapsuleAxis();
            var axis = math.mul(m_PrimitiveOrientation, new float3 { z = 1f });
            radius = m_Capsule.Radius;
            var endPoint = axis * (0.5f * m_Capsule.Height - radius);
            vertex0 = m_PrimitiveCenter + endPoint;
            vertex1 = m_PrimitiveCenter - endPoint;
        }

        public void GetCylinderProperties(
            out float3 center, out float height, out float radius, out quaternion orientation
        )
        {
            GetCylinderProperties(out center, out height, out radius, out EulerAngles euler);
            orientation = euler;
        }

        internal void GetCylinderProperties(
            out float3 center, out float height, out float radius, out EulerAngles orientation
        )
        {
            GetCylindricalProperties(
                m_Cylinder, out center, out height, out radius, out orientation, m_ShapeType != ShapeType.Cylinder
            );
        }

        public void GetSphereProperties(out float3 center, out float radius, out quaternion orientation)
        {
            GetSphereProperties(out center, out radius, out EulerAngles euler);
            orientation = euler;
        }

        internal void GetSphereProperties(out float3 center, out float radius, out EulerAngles orientation)
        {
            center = m_PrimitiveCenter;
            radius = m_SphereRadius;
            orientation = m_PrimitiveOrientation;
        }

        public void GetPlaneProperties(out float3 center, out float2 size, out quaternion orientation)
        {
            GetPlaneProperties(out center, out size, out EulerAngles euler);
            orientation = euler;
        }

        internal void GetPlaneProperties(out float3 center, out float2 size, out EulerAngles orientation)
        {
            center = m_PrimitiveCenter;
            orientation = m_PrimitiveOrientation;

            if (m_ShapeType == ShapeType.Plane)
            {
                size = m_PrimitiveSize.xz;
                return;
            }

            UpdateCapsuleAxis();
            var look = m_Capsule.Axis;
            var nextAx = k_NextAxis[look];
            var prevAx = k_NextAxis[k_NextAxis[look]];
            var ax2 = m_PrimitiveSize[nextAx] > m_PrimitiveSize[prevAx] ? nextAx : prevAx;
            size = new float2(m_PrimitiveSize[ax2], m_PrimitiveSize[look]);

            var up = k_NextAxis[ax2] == look ? k_NextAxis[look] : k_NextAxis[ax2];
            var offset = quaternion.LookRotation(new float3 { [look] = 1f }, new float3 { [up] = 1f });

            orientation.SetValue(math.mul(m_PrimitiveOrientation, offset));
        }

        [Obsolete("RemovedAfter 2019-08-10")]
        public void GetPlaneProperties(out float3 vertex0, out float3 vertex1, out float3 vertex2, out float3 vertex3)
        {
            this.GetPlanePoints(out vertex0, out vertex1, out vertex2, out vertex3);
        }

        static readonly List<MeshFilter> s_MeshFilters = new List<MeshFilter>(8);
        static readonly List<PhysicsShape> s_PhysicsShapes = new List<PhysicsShape>(8);
        static readonly List<Vector3> s_Vertices = new List<Vector3>(65535);

        public void GetConvexHullProperties(NativeList<float3> pointCloud)
        {
            pointCloud.Clear();

            if (m_CustomMesh != null)
            {
                m_CustomMesh.GetVertices(s_Vertices);
                foreach (var v in s_Vertices)
                    pointCloud.Add(v);
                return;
            }

            // TODO: account for skinned mesh renderers
            var primaryBody = this.GetPrimaryBody();
            GetComponentsInChildren(true, s_MeshFilters);
            foreach (var meshFilter in s_MeshFilters)
            {
                if (
                    meshFilter.sharedMesh == null
                    || PhysicsShapeExtensions.GetPrimaryBody(meshFilter.gameObject) != primaryBody
                )
                    continue;

                // do not simply use GameObject.activeInHierarchy because it will be false when instantiating a prefab
                var t = meshFilter.transform;
                while (t != transform)
                {
                    if (!t.gameObject.activeSelf)
                        continue;
                    t = t.parent;
                }

                var renderer = meshFilter.GetComponent<MeshRenderer>();
                if (renderer == null || !renderer.enabled)
                    continue;

                s_PhysicsShapes.Clear();
                meshFilter.gameObject.GetComponentsInParent(true, s_PhysicsShapes);
                if (s_PhysicsShapes[0] != this)
                    continue;

                meshFilter.sharedMesh.GetVertices(s_Vertices);
                var meshToLocal =
                    math.mul(math.inverse(transform.localToWorldMatrix), meshFilter.transform.localToWorldMatrix);
                foreach (var v in s_Vertices)
                    pointCloud.Add(math.mul(meshToLocal, new float4(v, 1f)).xyz);
            }
            s_MeshFilters.Clear();
            s_PhysicsShapes.Clear();
        }

        public UnityMesh GetMesh()
        {
            if (m_CustomMesh != null)
                return m_CustomMesh;
            var meshFilter = gameObject.GetComponent<MeshFilter>();
            return meshFilter == null ? null : meshFilter.sharedMesh;
        }

        void UpdateCapsuleAxis()
        {
            var cmax = math.cmax(m_PrimitiveSize);
            var cmin = math.cmin(m_PrimitiveSize);
            if (cmin == cmax)
                return;
            m_Capsule.Axis = m_PrimitiveSize.GetMaxAxis();
        }

        void UpdateCylinderAxis() => m_Cylinder.Axis = m_PrimitiveSize.GetDeviantAxis();

        void Sync(ref CylindricalProperties props)
        {
            props.Height = m_PrimitiveSize[props.Axis];
            props.Radius = 0.5f * math.max(
                m_PrimitiveSize[k_NextAxis[props.Axis]],
                m_PrimitiveSize[k_NextAxis[k_NextAxis[props.Axis]]]
            );
        }

        void SyncCapsuleProperties()
        {
            UpdateCapsuleAxis();
            Sync(ref m_Capsule);
        }

        void SyncCylinderProperties()
        {
            UpdateCylinderAxis();
            Sync(ref m_Cylinder);
        }

        void SyncSphereProperties()
        {
            m_SphereRadius = 0.5f * math.cmax(m_PrimitiveSize);
        }

        public void SetBox(float3 center, float3 size, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetBox(center, size, euler);
        }

        internal void SetBox(float3 center, float3 size, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Box;
            m_PrimitiveCenter = center;
            m_PrimitiveSize = math.max(size, new float3());
            m_PrimitiveOrientation = orientation;
            ConvexRadius = m_ConvexRadius;

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetCapsule(float3 center, float height, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetCapsule(center, height, radius, euler);
        }

        internal void SetCapsule(float3 center, float height, float radius, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Capsule;
            m_PrimitiveCenter = center;
            m_PrimitiveOrientation = orientation;

            radius = math.max(0f, radius);
            height = math.max(height, radius * 2f);
            m_PrimitiveSize = new float3(radius * 2f, radius * 2f, height);

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetCylinder(float3 center, float height, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetCylinder(center, height, radius, euler);
        }

        internal void SetCylinder(float3 center, float height, float radius, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Cylinder;
            m_PrimitiveCenter = center;
            m_PrimitiveOrientation = orientation;

            radius = math.max(0f, radius);
            height = math.max(0f, height);
            m_PrimitiveSize = new float3(radius * 2f, radius * 2f, height);

            ConvexRadius = m_ConvexRadius;

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetSphere(float3 center, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetSphere(center, radius, euler);
        }

        internal void SetSphere(float3 center, float radius, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Sphere;
            m_PrimitiveCenter = center;

            radius = math.max(0f, radius);
            m_PrimitiveSize = new float3(2f * radius, 2f * radius, 2f * radius);

            m_PrimitiveOrientation = orientation;

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetPlane(float3 center, float2 size, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetPlane(center, size, euler);
        }

        internal void SetPlane(float3 center, float2 size, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Plane;
            m_PrimitiveCenter = center;
            m_PrimitiveOrientation = orientation;
            m_PrimitiveSize = new float3(size.x, 0f, size.y);

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetConvexHull(UnityMesh convexHull = null)
        {
            m_ShapeType = ShapeType.ConvexHull;
            m_CustomMesh = convexHull;
        }

        public void SetMesh(UnityMesh mesh = null)
        {
            m_ShapeType = ShapeType.Mesh;
            m_CustomMesh = mesh;
        }

        void OnEnable()
        {
            // included so tick box appears in Editor
        }

        static void Validate(ref CylindricalProperties props)
        {
            props.Height = math.max(0f, props.Height);
            props.Radius = math.max(0f, props.Radius);
        }

        void OnValidate()
        {
            m_PrimitiveSize = math.max(m_PrimitiveSize, new float3());
            Validate(ref m_Capsule);
            Validate(ref m_Cylinder);
            switch (m_ShapeType)
            {
                case ShapeType.Box:
                    GetBoxProperties(out var center, out var size, out EulerAngles orientation);
                    SetBox(center, size, orientation);
                    break;
                case ShapeType.Capsule:
                    GetCapsuleProperties(out center, out var height, out var radius, out orientation);
                    SetCapsule(center, height, radius, orientation);
                    break;
                case ShapeType.Cylinder:
                    GetCylinderProperties(out center, out height, out radius, out orientation);
                    SetCylinder(center, height, radius, orientation);
                    break;
                case ShapeType.Sphere:
                    GetSphereProperties(out center, out radius, out orientation);
                    SetSphere(center, radius, orientation);
                    break;
                case ShapeType.Plane:
                    GetPlaneProperties(out center, out var size2D, out orientation);
                    SetPlane(center, size2D, orientation);
                    break;
                case ShapeType.ConvexHull:
                case ShapeType.Mesh:
                    break;
                default:
                    throw new UnimplementedShapeException(m_ShapeType);
            }
            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
            ConvexRadius = m_ConvexRadius;
            PhysicsMaterialProperties.OnValidate(ref m_Material, true);
        }

        // matrix to transform point from shape space into world space
        internal float4x4 GetShapeToWorldMatrix() =>
            float4x4.TRS(transform.position, transform.rotation, 1f);

        // matrix to transform point from object's local transform matrix into shape space
        internal float4x4 GetLocalToShapeMatrix() =>
            math.mul(math.inverse(GetShapeToWorldMatrix()), transform.localToWorldMatrix);

        internal void BakePoints(NativeArray<float3> points)
        {
            var localToShape = GetLocalToShapeMatrix();
            for (int i = 0, count = points.Length; i < count; ++i)
                points[i] = math.mul(localToShape, new float4(points[i], 1f)).xyz;
        }

        [Obsolete("FitToGeometry() has been deprecated. Use FitToEnabledRenderMeshes() instead. (RemovedAfter 2019-08-27) (UnityUpgradable) -> FitToEnabledRenderMeshes(*)")]
        public void FitToGeometry() => FitToEnabledRenderMeshes();

        public void FitToEnabledRenderMeshes()
        {
            var shapeType = m_ShapeType;

            using (var points = new NativeList<float3>(65535, Allocator.TempJob))
            {
                // temporarily un-assign custom mesh and assume this shape is a convex hull
                var customMesh = m_CustomMesh;
                m_CustomMesh = null;
                GetConvexHullProperties(points);
                m_CustomMesh = customMesh;
                if (points.Length == 0)
                    return;

                var bounds = new Bounds(points[0], float3.zero);
                for (int i = 1, count = points.Length; i < count; ++i)
                    bounds.Encapsulate(points[i]);

                SetBox(bounds.center, bounds.size, quaternion.identity);
                m_ConvexRadius = math.min(math.cmin(bounds.size) * 0.1f, k_DefaultConvexRadius);
            }

            switch (shapeType)
            {
                case ShapeType.Capsule:
                    GetCapsuleProperties(out var center, out var height, out var radius, out EulerAngles orientation);
                    SetCapsule(center, height, radius, orientation);
                    break;
                case ShapeType.Cylinder:
                    GetCylinderProperties(out center, out height, out radius, out orientation);
                    SetCylinder(center, height, radius, orientation);
                    break;
                case ShapeType.Sphere:
                    GetSphereProperties(out center, out radius, out orientation);
                    SetSphere(center, radius, orientation);
                    break;
                case ShapeType.Plane:
                    // force recalculation of plane orientation by making it think shape type is out of date
                    m_ShapeType = ShapeType.Box;
                    GetPlaneProperties(out center, out var size2D, out orientation);
                    SetPlane(center, size2D, orientation);
                    break;
                case ShapeType.Box:
                case ShapeType.ConvexHull:
                case ShapeType.Mesh:
                    m_ShapeType = shapeType;
                    break;
                default:
                    throw new UnimplementedShapeException(shapeType);
            }
            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        void Reset()
        {
            FitToEnabledRenderMeshes();
            // TODO: also pick best primitive shape
        }
    }
}
