using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityComponent = UnityEngine.Component;
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
    public sealed partial class PhysicsShapeAuthoring : MonoBehaviour, IInheritPhysicsMaterialProperties, ISerializationCallbackReceiver
    {
        PhysicsShapeAuthoring() { }

        [Serializable]
        struct CylindricalProperties
        {
            public float Height;
            public float Radius;
            [HideInInspector]
            public int Axis;
        }

        static readonly int[] k_NextAxis = { 1, 2, 0 };

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
        [Tooltip("How many sides the convex cylinder shape should have.")]
        [Range(CylinderGeometry.MinSideCount, CylinderGeometry.MaxSideCount)]
        int m_CylinderSideCount = 20;

        [SerializeField]
        float m_SphereRadius = 0.5f;

        float BevelRadius
        {
            get => m_ConvexHullGenerationParameters.BevelRadius;
            set
            {
                var maxRadius = float.MaxValue;
                switch (m_ShapeType)
                {
                    case ShapeType.Box:
                        maxRadius = 0.5f * math.cmin(m_PrimitiveSize);
                        break;
                    case ShapeType.Cylinder:
                        var cylinder = GetCylinderProperties();
                        maxRadius = math.min(0.5f * cylinder.Height, cylinder.Radius);
                        break;
                    case ShapeType.ConvexHull:
                        // TODO: any benefit in clamping this?
                        break;
                    case ShapeType.Capsule:
                    case ShapeType.Mesh:
                    case ShapeType.Plane:
                    case ShapeType.Sphere:
                        break;
                    default:
                        throw new UnimplementedShapeException(m_ShapeType);
                }
                m_ConvexHullGenerationParameters.BevelRadius = math.clamp(value, 0f, maxRadius);
            }
        }

        public ConvexHullGenerationParameters ConvexHullGenerationParameters => m_ConvexHullGenerationParameters;

        [SerializeField]
        [Tooltip(
            "Specifies the minimum weight of a skinned vertex assigned to this shape and/or its transform children required for it to be included for automatic detection. " +
            "A value of 0 will include all points with any weight assigned to this shape's hierarchy."
        )]
        [Range(0f, 1f)]
        float m_MinimumSkinnedVertexWeight = 0.1f;

        [SerializeField]
        [ExpandChildren]
        ConvexHullGenerationParameters m_ConvexHullGenerationParameters = ConvexHullGenerationParameters.Default.ToAuthoring();

        // TODO: remove this accessor in favor of GetRawVertices() when blob data is serializable
        internal UnityMesh CustomMesh => m_CustomMesh;
        [SerializeField]
        [Tooltip("If no custom mesh is specified, then one will be generated using this body's rendered meshes.")]
        UnityMesh m_CustomMesh;

        public bool ForceUnique { get => m_ForceUnique; set => m_ForceUnique = value; }
        [SerializeField]
        bool m_ForceUnique;

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
        public PhysicsCategoryTags BelongsTo
        {
            get => m_Material.BelongsTo;
            set => m_Material.BelongsTo = value;
        }

        public bool OverrideCollidesWith
        {
            get => m_Material.OverrideCollidesWith;
            set => m_Material.OverrideCollidesWith = value;
        }
        public PhysicsCategoryTags CollidesWith
        {
            get => m_Material.CollidesWith;
            set => m_Material.CollidesWith = value;
        }

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

        public bool OverrideCustomTags
        {
            get => m_Material.OverrideCustomTags;
            set => m_Material.OverrideCustomTags = value;
        }
        public CustomPhysicsMaterialTags CustomTags { get => m_Material.CustomTags; set => m_Material.CustomTags = value; }

        [SerializeField]
        PhysicsMaterialProperties m_Material = new PhysicsMaterialProperties(true);

        public BoxGeometry GetBoxProperties() => GetBoxProperties(out _);

        internal BoxGeometry GetBoxProperties(out EulerAngles orientation)
        {
            orientation = m_PrimitiveOrientation;
            return new BoxGeometry
            {
                Center = m_PrimitiveCenter,
                Size = m_PrimitiveSize,
                Orientation = m_PrimitiveOrientation,
                BevelRadius = BevelRadius
            };
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

        public CapsuleGeometry GetCapsuleProperties(out quaternion orientation)
        {
            var result = GetCapsuleProperties(out EulerAngles euler);
            orientation = euler;
            return result;
        }

        internal CapsuleGeometry GetCapsuleProperties(out EulerAngles orientation)
        {
            GetCylindricalProperties(
                m_Capsule, out var center, out var height, out var radius, out orientation, m_ShapeType != ShapeType.Capsule
            );
            var halfDistance = new float3 { z = 0.5f * height - radius };
            return new CapsuleGeometry
            {
                Radius = radius,
                Vertex0 = center + halfDistance,
                Vertex1 = center - halfDistance
            };
        }

        public CylinderGeometry GetCylinderProperties() => GetCylinderProperties(out _);

        internal CylinderGeometry GetCylinderProperties(out EulerAngles orientation)
        {
            GetCylindricalProperties(
                m_Cylinder, out var center, out var height, out var radius, out orientation, m_ShapeType != ShapeType.Cylinder
            );
            return new CylinderGeometry
            {
                Center = center,
                Height = height,
                Radius = radius,
                Orientation = orientation,
                BevelRadius = BevelRadius,
                SideCount = m_CylinderSideCount
            };
        }

        public SphereGeometry GetSphereProperties(out quaternion orientation)
        {
            var result = GetSphereProperties(out EulerAngles euler);
            orientation = euler;
            return result;
        }

        internal SphereGeometry GetSphereProperties(out EulerAngles orientation)
        {
            orientation = m_PrimitiveOrientation;
            return new SphereGeometry
            {
                Center = m_PrimitiveCenter,
                Radius = m_SphereRadius
            };
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

        static readonly HashSet<int> s_BoneIDs = new HashSet<int>();
        static readonly List<Transform> s_Bones = new List<Transform>(8);
        static readonly HashSet<Transform> s_BonesInHierarchy = new HashSet<Transform>();
        static readonly List<MeshFilter> s_MeshFilters = new List<MeshFilter>(8);
        static readonly List<PhysicsShapeAuthoring> s_PhysicsShapes = new List<PhysicsShapeAuthoring>(8);
        static readonly List<SkinnedMeshRenderer> s_SkinnedMeshRenderers = new List<SkinnedMeshRenderer>(8);
        static readonly List<Vector3> s_Vertices = new List<Vector3>(65535);
        static readonly List<int> s_Indices = new List<int>(65535);

        static UnityMesh ReusableBakeMesh =>
            s_ReusableBakeMesh ??
            (s_ReusableBakeMesh = new UnityMesh { hideFlags = HideFlags.HideAndDontSave });
        static UnityMesh s_ReusableBakeMesh;

        public void GetConvexHullProperties(NativeList<float3> pointCloud) =>
            GetConvexHullProperties(pointCloud, true, default, default, default);

        internal unsafe void GetConvexHullProperties(
            NativeList<float3> pointCloud, bool validate,
            NativeList<HashableShapeInputs> inputs, NativeList<int> allSkinIndices, NativeList<float> allBlendShapeWeights
        )
        {
            pointCloud.Clear();
            if (inputs.IsCreated)
                inputs.Clear();
            if (allSkinIndices.IsCreated)
                allSkinIndices.Clear();
            if (allBlendShapeWeights.IsCreated)
                allBlendShapeWeights.Clear();

            var triangles = new NativeList<int3>(pointCloud.Capacity - 2, Allocator.Temp);

            if (m_CustomMesh != null)
            {
                if (validate && !m_CustomMesh.IsValidForConversion(gameObject))
                    return;

                AppendMeshPropertiesToNativeBuffers(
                    transform.localToWorldMatrix, m_CustomMesh, pointCloud, triangles, validate, inputs
                );
            }
            else
            {
                foreach (var meshFilter in GetAllMeshFiltersInHierarchyBelongingToShape(this, validate))
                {
                    AppendMeshPropertiesToNativeBuffers(
                        meshFilter.transform.localToWorldMatrix, meshFilter.sharedMesh, pointCloud, triangles, validate, inputs
                    );
                }

                using (var skinnedPoints = new NativeList<float3>(8192, Allocator.Temp))
                using (var skinnedInputs = new NativeList<HashableShapeInputs>(8, Allocator.Temp))
                {
                    GetAllSkinnedPointsInHierarchyBelongingToShape(
                        this, skinnedPoints, validate, skinnedInputs, allSkinIndices, allBlendShapeWeights
                    );
                    pointCloud.ResizeUninitialized(pointCloud.Length + skinnedPoints.Length);
                    UnsafeUtility.MemCpy(
                        pointCloud.GetUnsafePtr(),
                        skinnedPoints.GetUnsafePtr(),
                        skinnedPoints.Length * UnsafeUtility.SizeOf<float3>()
                    );
                    pointCloud.AddRange(skinnedPoints);
                    if (inputs.IsCreated)
                        inputs.AddRange(skinnedInputs);
                }
            }

            triangles.Dispose();
        }

        static IEnumerable<T> GetAllActiveComponentsInHierarchyBelongingToShape<T>(
            Transform root, PhysicsShapeAuthoring shape, List<T> buffer
        ) where T : UnityComponent
        {
            buffer.Clear();
            root.GetComponentsInChildren(true, buffer);
            var primaryBody = PhysicsShapeExtensions.GetPrimaryBody(root.gameObject);
            var checkIfComponentBelongsToShape = root.transform.IsChildOf(shape.transform);
            foreach (var component in buffer)
            {
                if (checkIfComponentBelongsToShape)
                {
                    if (PhysicsShapeExtensions.GetPrimaryBody(component.gameObject) != primaryBody)
                        continue;

                    s_PhysicsShapes.Clear();
                    component.gameObject.GetComponentsInParent(true, s_PhysicsShapes);
                    if (s_PhysicsShapes[0] != shape)
                        continue;
                }

                // do not simply use GameObject.activeInHierarchy because it will be false when instantiating a prefab
                var t = component.transform;
                var activeInHierarchy = t.gameObject.activeSelf;
                while (activeInHierarchy && t != root)
                {
                    t = t.parent;
                    activeInHierarchy &= t.gameObject.activeSelf;
                }
                if (!activeInHierarchy)
                    continue;

                yield return component;
            }
            s_PhysicsShapes.Clear();
        }

        internal static IEnumerable<MeshFilter> GetAllMeshFiltersInHierarchyBelongingToShape(
            PhysicsShapeAuthoring shape, bool filterOutInvalid
        )
        {
            foreach (var meshFilter in GetAllActiveComponentsInHierarchyBelongingToShape(shape.transform, shape, s_MeshFilters))
            {
                if (meshFilter.sharedMesh == null)
                    continue;

                var renderer = meshFilter.GetComponent<MeshRenderer>();
                if (renderer == null || !renderer.enabled)
                    continue;

                if (filterOutInvalid && !meshFilter.sharedMesh.IsValidForConversion(shape.gameObject))
                    continue;

                yield return meshFilter;
            }
            s_MeshFilters.Clear();
        }

        internal static void GetAllSkinnedPointsInHierarchyBelongingToShape(
            PhysicsShapeAuthoring shape, NativeList<float3> pointCloud, bool validate,
            NativeList<HashableShapeInputs> inputs, NativeList<int> allIncludedIndices, NativeList<float> allBlendShapeWeights
        )
        {
            if (pointCloud.IsCreated)
                pointCloud.Clear();

            if (inputs.IsCreated)
                inputs.Clear();

            // get all the transforms that belong to this shape
            s_BonesInHierarchy.Clear();
            foreach (var bone in GetAllActiveComponentsInHierarchyBelongingToShape(shape.transform, shape, s_Bones))
                s_BonesInHierarchy.Add(bone);
            s_Bones.Clear();

            // find all skinned mesh renderers in which this shape's transform might be a bone
            foreach (var skin in GetAllActiveComponentsInHierarchyBelongingToShape(shape.transform.root, shape, s_SkinnedMeshRenderers))
            {
                var mesh = skin.sharedMesh;
                if (!skin.enabled || mesh == null || validate && !mesh.IsValidForConversion(shape.gameObject))
                    continue;

                // get indices of this shape's transform hierarchy in skinned mesh's bone array
                s_BoneIDs.Clear();
                var bones = skin.bones;
                for (int i = 0, count = bones.Length; i < count; ++i)
                {
                    if (s_BonesInHierarchy.Contains(bones[i]))
                        s_BoneIDs.Add(i);
                }

                if (s_BoneIDs.Count == 0)
                    continue;

                // sample the vertices
                skin.BakeMesh(ReusableBakeMesh);
                ReusableBakeMesh.GetVertices(s_Vertices);

                // add all vertices weighted to at least one bone in this shape's transform hierarchy
                var bonesPerVertex = mesh.GetBonesPerVertex(); // Allocator.None
                var weights = mesh.GetAllBoneWeights();        // Allocator.None
                var vertexIndex = 0;
                var weightsOffset = 0;
                var shapeFromSkin = math.mul(shape.transform.worldToLocalMatrix, skin.transform.localToWorldMatrix);
                var includedIndices = new NativeList<int>(mesh.vertexCount, Allocator.Temp);
                foreach (var weightCount in bonesPerVertex)
                {
                    var totalWeight = 0f;
                    for (var i = 0; i < weightCount; ++i)
                    {
                        var weight = weights[weightsOffset + i];
                        if (s_BoneIDs.Contains(weight.boneIndex))
                            totalWeight += weight.weight;
                    }

                    if (totalWeight > shape.m_MinimumSkinnedVertexWeight)
                    {
                        pointCloud.Add(math.mul(shapeFromSkin, new float4(s_Vertices[vertexIndex], 1f)).xyz);
                        includedIndices.Add(vertexIndex);
                    }

                    weightsOffset += weightCount;
                    ++vertexIndex;
                }

                var blendShapeWeights = new NativeArray<float>(mesh.blendShapeCount, Allocator.Temp);
                for (var i = 0; i < blendShapeWeights.Length; ++i)
                    blendShapeWeights[i] = skin.GetBlendShapeWeight(i);

                if (!inputs.IsCreated)
                    continue;

                var data = HashableShapeInputs.FromSkinnedMesh(
                    mesh, shapeFromSkin, includedIndices, allIncludedIndices, blendShapeWeights, allBlendShapeWeights
                );
                inputs.Add(data);
            }

            s_BonesInHierarchy.Clear();
        }

        public void GetMeshProperties(NativeList<float3> vertices, NativeList<int3> triangles) =>
            GetMeshProperties(vertices, triangles, true, default);

        internal void GetMeshProperties(
            NativeList<float3> vertices, NativeList<int3> triangles, bool validate, NativeList<HashableShapeInputs> inputs
        )
        {
            vertices.Clear();
            triangles.Clear();
            if (inputs.IsCreated)
                inputs.Clear();

            if (m_CustomMesh != null)
            {
                if (validate && !m_CustomMesh.IsValidForConversion(gameObject))
                    return;

                AppendMeshPropertiesToNativeBuffers(
                    transform.localToWorldMatrix, m_CustomMesh, vertices, triangles, validate, inputs
                );
            }
            else
            {
                foreach (var meshFilter in GetAllMeshFiltersInHierarchyBelongingToShape(this, validate))
                {
                    AppendMeshPropertiesToNativeBuffers(
                        meshFilter.transform.localToWorldMatrix, meshFilter.sharedMesh, vertices, triangles, validate, inputs
                    );
                }
            }
        }

        void AppendMeshPropertiesToNativeBuffers(
            float4x4 localToWorld, UnityMesh mesh, NativeList<float3> vertices, NativeList<int3> triangles, bool validate,
            NativeList<HashableShapeInputs> inputs
        )
        {
            if (mesh == null || validate && !mesh.IsValidForConversion(gameObject))
                return;

            var offset = vertices.Length;

            mesh.GetVertices(s_Vertices);
            if (vertices.Capacity < vertices.Length + s_Vertices.Count)
                vertices.Capacity = vertices.Length + s_Vertices.Count;
            var childToShape = math.mul(transform.worldToLocalMatrix, localToWorld);
            foreach (var v in s_Vertices)
                vertices.Add(math.mul(childToShape, new float4(v, 1f)).xyz);
            s_Vertices.Clear();

            for (var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh)
            {
                mesh.GetIndices(s_Indices, subMesh);
                var numTriangles = s_Indices.Count / 3;
                if (triangles.Capacity < triangles.Length + numTriangles)
                    triangles.Capacity = triangles.Length + numTriangles;
                for (var i = 0; i < numTriangles; i++)
                    triangles.Add(new int3(offset + s_Indices[i * 3], offset + s_Indices[i * 3 + 1], offset + s_Indices[i * 3 + 2]));
            }
            s_Indices.Clear();

            if (inputs.IsCreated)
                inputs.Add(HashableShapeInputs.FromMesh(mesh, childToShape));
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

        public void SetBox(BoxGeometry geometry)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(geometry.Orientation);
            SetBox(geometry, euler);
            BevelRadius = geometry.BevelRadius;
        }

        internal void SetBox(BoxGeometry geometry, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Box;
            m_PrimitiveCenter = geometry.Center;
            m_PrimitiveSize = math.max(geometry.Size, new float3());
            m_PrimitiveOrientation = orientation;
            BevelRadius = BevelRadius;

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetCapsule(CapsuleGeometry geometry, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(
                math.mul(
                    orientation,
                    quaternion.LookRotationSafe(geometry.Vertex1 - geometry.Vertex0, math.up())
                )
            );
            var center = math.lerp(geometry.Vertex0, geometry.Vertex1, 0.5f);
            var height = math.length(geometry.Vertex0 - geometry.Vertex1) + 2f * geometry.Radius;
            SetCapsule(geometry, euler);
        }

        internal void SetCapsule(CapsuleGeometry geometry, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Capsule;
            m_PrimitiveCenter = geometry.GetCenter();
            m_PrimitiveOrientation = orientation;

            var radius = math.max(0f, geometry.Radius);
            var height = math.max(geometry.GetHeight(), radius * 2f);
            m_PrimitiveSize = new float3(radius * 2f, radius * 2f, height);

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetCylinder(CylinderGeometry geometry)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(geometry.Orientation);
            SetCylinder(geometry, euler);
        }

        internal void SetCylinder(CylinderGeometry geometry, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Cylinder;
            m_PrimitiveCenter = geometry.Center;
            m_PrimitiveOrientation = orientation;

            geometry.Radius = math.max(0f, geometry.Radius);
            geometry.Height = math.max(0f, geometry.Height);
            m_PrimitiveSize = new float3(geometry.Radius * 2f, geometry.Radius * 2f, geometry.Height);

            BevelRadius = BevelRadius;

            SyncCapsuleProperties();
            SyncCylinderProperties();
            SyncSphereProperties();
        }

        public void SetSphere(SphereGeometry geometry, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetSphere(geometry, euler);
        }

        internal void SetSphere(SphereGeometry geometry, EulerAngles orientation)
        {
            m_ShapeType = ShapeType.Sphere;
            m_PrimitiveCenter = geometry.Center;

            var radius = math.max(0f, geometry.Radius);
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

        public void SetConvexHull(
            ConvexHullGenerationParameters hullGenerationParameters, float minimumSkinnedVertexWeight
        )
        {
            m_MinimumSkinnedVertexWeight = minimumSkinnedVertexWeight;
            SetConvexHull(hullGenerationParameters);
        }

        public void SetConvexHull(ConvexHullGenerationParameters hullGenerationParameters, UnityMesh customMesh = null)
        {
            m_ShapeType = ShapeType.ConvexHull;
            m_CustomMesh = customMesh;
            hullGenerationParameters.OnValidate();
            m_ConvexHullGenerationParameters = hullGenerationParameters;
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

        void ISerializationCallbackReceiver.OnBeforeSerialize() { }

        void ISerializationCallbackReceiver.OnAfterDeserialize()
        {
            // migrate existing serialized data from old m_ConvexRadius field
            if (m_ConvexRadius_Deprecated >= 0f)
                BevelRadius = m_ConvexRadius_Deprecated;
            m_ConvexRadius_Deprecated = -1f;
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
                    SetBox(GetBoxProperties(out var orientation), orientation);
                    break;
                case ShapeType.Capsule:
                    SetCapsule(GetCapsuleProperties(out orientation), orientation);
                    break;
                case ShapeType.Cylinder:
                    SetCylinder(GetCylinderProperties(out orientation), orientation);
                    break;
                case ShapeType.Sphere:
                    SetSphere(GetSphereProperties(out orientation), orientation);
                    break;
                case ShapeType.Plane:
                    GetPlaneProperties(out var center, out var size2D, out orientation);
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
            m_CylinderSideCount =
                math.clamp(m_CylinderSideCount, CylinderGeometry.MinSideCount, CylinderGeometry.MaxSideCount);
            m_ConvexHullGenerationParameters.OnValidate();
            BevelRadius = BevelRadius;
            m_ConvexRadius_Deprecated = -1f; // prevent user from setting a value for deprecated field
            PhysicsMaterialProperties.OnValidate(ref m_Material, true);
        }

        // matrix to transform point from shape space into world space
        internal float4x4 GetShapeToWorldMatrix() =>
            float4x4.TRS(transform.position, transform.rotation, 1f);

        // matrix to transform point from object's local transform matrix into shape space
        internal float4x4 GetLocalToShapeMatrix() =>
            math.mul(math.inverse(GetShapeToWorldMatrix()), transform.localToWorldMatrix);

        internal unsafe void BakePoints(NativeArray<float3> points)
        {
            var localToShapeQuantized = GetLocalToShapeMatrix();
            using (var aabb = new NativeArray<Aabb>(1, Allocator.TempJob))
            {
                new PhysicsShapeExtensions.GetAabbJob { Points = points, Aabb = aabb }.Run();
                HashableShapeInputs.GetQuantizedTransformations(localToShapeQuantized, aabb[0], out localToShapeQuantized);
            }
            using (var bakedPoints = new NativeArray<float3>(points.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory))
            {
                new BakePointsJob
                {
                    Points = points,
                    LocalToShape = localToShapeQuantized,
                    Output = bakedPoints
                }.Schedule(points.Length, 16).Complete();

                UnsafeUtility.MemCpy(points.GetUnsafePtr(), bakedPoints.GetUnsafePtr(), points.Length * UnsafeUtility.SizeOf<float3>());
            }
        }

        [BurstCompile]
        struct BakePointsJob : IJobParallelFor
        {
            [Collections.ReadOnly]
            public NativeArray<float3> Points;
            public float4x4 LocalToShape;
            public NativeArray<float3> Output;

            public void Execute(int index) => Output[index] = math.mul(LocalToShape, new float4(Points[index], 1f)).xyz;
        }

        public void FitToEnabledRenderMeshes(float minimumSkinnedVertexWeight = 0f)
        {
            var shapeType = m_ShapeType;
            m_MinimumSkinnedVertexWeight = minimumSkinnedVertexWeight;

            using (var points = new NativeList<float3>(65535, Allocator.Persistent))
            {
                // temporarily un-assign custom mesh and assume this shape is a convex hull
                var customMesh = m_CustomMesh;
                m_CustomMesh = null;
                GetConvexHullProperties(points, Application.isPlaying, default, default, default);
                m_CustomMesh = customMesh;
                if (points.Length == 0)
                    return;

                // TODO: find best rotation, particularly if any points came from skinned mesh
                var orientation = quaternion.identity;
                var bounds = new Bounds(points[0], float3.zero);
                for (int i = 1, count = points.Length; i < count; ++i)
                    bounds.Encapsulate(points[i]);

                SetBox(
                    new BoxGeometry { Center = bounds.center, Size = bounds.size, Orientation = orientation }
                );
                BevelRadius = math.min(math.cmin(bounds.size) * 0.1f, ConvexHullGenerationParameters.Default.BevelRadius);
            }

            switch (shapeType)
            {
                case ShapeType.Capsule:
                    SetCapsule(GetCapsuleProperties(out EulerAngles orientation), orientation);
                    break;
                case ShapeType.Cylinder:
                    SetCylinder(GetCylinderProperties(out orientation), orientation);
                    break;
                case ShapeType.Sphere:
                    SetSphere(GetSphereProperties(out orientation), orientation);
                    break;
                case ShapeType.Plane:
                    // force recalculation of plane orientation by making it think shape type is out of date
                    m_ShapeType = ShapeType.Box;
                    GetPlaneProperties(out var center, out var size2D, out orientation);
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

        [Conditional("UNITY_EDITOR")]
        void Reset()
        {
            #if UNITY_EDITOR
            UnityEditor.EditorApplication.delayCall += () =>
            {
                if (this == null)
                    return;
                InitializeConvexHullGenerationParameters();
                FitToEnabledRenderMeshes(m_MinimumSkinnedVertexWeight);
                // TODO: also pick best primitive shape
                UnityEditor.SceneView.RepaintAll();
            };
            #endif
        }

        internal void InitializeConvexHullGenerationParameters()
        {
            var pointCloud = new NativeList<float3>(65535, Allocator.Temp);
            GetConvexHullProperties(pointCloud, false, default, default, default);
            m_ConvexHullGenerationParameters.InitializeToRecommendedAuthoringValues(pointCloud);
        }
    }
}
