using Unity.Mathematics;
using Unity.Physics.Systems;
using UnityEngine;
using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// A component system group that contains the PhysicsDebugDisplay systems.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateAfter(typeof(TransformSystemGroup))]
    public partial class PhysicsDisplayDebugGroup : ComponentSystemGroup
    {
    }

    /// <summary>
    /// A system which cleans physics debug display data from the previous frame during the simulation.
    /// In case of using multiple worlds feature, in order for debug display to work properly
    /// on multiple worlds, you need to disable the update of this system in either main physics group (<see cref="PhysicsSystemGroup"/>)
    /// or in the custom physics group (<see cref="BeforePhysicsSystemGroup"/>), whichever updates later in the loop.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(BeforePhysicsSystemGroup))]
    public partial struct CleanPhysicsDebugDataSystem_Default : ISystem
    {
        void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        void OnUpdate(ref SystemState state)
        {
            DebugDisplay.DebugDisplay.Clear();
        }
    }

    /// <summary>
    /// A system which cleans physics debug display data from the previous frame while in edit mode.
    /// In case of using multiple worlds feature, in order for debug display to work properly
    /// on multiple worlds, you need to disable the update of this system in editor display physics group (<see cref="PhysicsDisplayDebugGroup"/>).
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateInGroup(typeof(PhysicsDisplayDebugGroup))]
    public partial struct CleanPhysicsDebugDataSystem_Editor : ISystem
    {
        void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        void OnUpdate(ref SystemState state)
        {
            DebugDisplay.DebugDisplay.Clear();
        }
    }

    /// <summary>
    /// A system which is responsible for drawing physics debug display data.
    /// Create a singleton entity with <see cref="PhysicsDebugDisplayData"/> and select what you want to be drawn.<para/>
    ///
    /// If you want custom debug draw, you need to:<para/>
    /// 1) Create a system which updates before PhysicsDebugDisplaySystem.<br/>
    /// 2) In OnUpdate() of that system, call GetSingleton <see cref="PhysicsDebugDisplayData"/> (even if you are not using it, it is important to do so to properly chain dependencies).<br/>
    /// 3) Afterwards, in OnUpdate() or in scheduled jobs, call one of the exposed draw methods (Line, Arrow, Plane, Triangle, Cone, Box, ...).<br/>
    /// 4) Data will be drawn when PhysicsDebugDisplaySystem's OnUpdate() is called.<para/>
    /// IMPORTANT: Drawing works only in Editor mode.
    /// </summary>
    public abstract partial class PhysicsDebugDisplaySystem : SystemBase
    {
#if UNITY_EDITOR
        DrawComponent m_DrawComponent;
#endif
        class DrawComponent : MonoBehaviour
        {
            public void OnDrawGizmos()
            {
#if UNITY_EDITOR
                if (World.DefaultGameObjectInjectionWorld == null)
                    return;
                World.DefaultGameObjectInjectionWorld.GetExistingSystemManaged<PhysicsDebugDisplaySystem>().CompleteDependency();
                Unity.DebugDisplay.DebugDisplay.Render();
#endif
            }
        }

        protected override void OnCreate()
        {
            RequireForUpdate<PhysicsDebugDisplayData>();
#if UNITY_EDITOR
            Unity.DebugDisplay.DebugDisplay.Reinstantiate();

            if (m_DrawComponent == null)
            {
                m_DrawComponent = new GameObject("DebugDisplay.DrawComponent", typeof(DrawComponent))
                {
                    hideFlags = HideFlags.DontSave
                }.GetComponent<DrawComponent>();
            }
#endif
        }

        protected override void OnDestroy()
        {
#if UNITY_EDITOR
            if (m_DrawComponent != null)
            {
                if (Application.isPlaying)
                    Object.Destroy(m_DrawComponent.gameObject);
                else
                    Object.DestroyImmediate(m_DrawComponent.gameObject);
            }
            m_DrawComponent = null;
#endif
        }

        /// <summary>
        /// Draws a point.
        /// </summary>
        /// <param name="x"> World space position. </param>
        /// <param name="size"> Extents. </param>
        /// <param name="color"> Color. </param>
        public static void Point(float3 x, float size, Unity.DebugDisplay.ColorIndex color)
        {
            var lines = new Unity.DebugDisplay.Lines(3);

            lines.Draw(x - new float3(size, 0, 0), x + new float3(size, 0, 0), color);
            lines.Draw(x - new float3(0, size, 0), x + new float3(0, size, 0), color);
            lines.Draw(x - new float3(0, 0, size), x + new float3(0, 0, size), color);
        }

        /// <summary>
        /// Draws a line between 2 points.
        /// </summary>
        /// <param name="x0"> Point 0 in world space. </param>
        /// <param name="x1"> Point 1 in world space. </param>
        /// <param name="color"> Color. </param>
        public static void Line(float3 x0, float3 x1, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Line.Draw(x0, x1, color);
        }

        /// <summary>
        /// Draws an arrow.
        /// </summary>
        /// <param name="x"> World space position of the arrow base. </param>
        /// <param name="v"> Arrow direction with length. </param>
        /// <param name="color"> Color. </param>
        public static void Arrow(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Arrow.Draw(x, v, color);
        }

        /// <summary>
        /// Draws a plane.
        /// </summary>
        /// <param name="x"> Point in world space. </param>
        /// <param name="v"> Normal. </param>
        /// <param name="color"> Color. </param>
        public static void Plane(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Plane.Draw(x, v, color);
        }

        /// <summary>
        /// Draws an arc.
        /// </summary>
        /// <param name="center"> World space position of the arc center. </param>
        /// <param name="normal"> Arc normal. </param>
        /// <param name="arm"> Arc arm. </param>
        /// <param name="angle"> Arc angle. </param>
        /// <param name="color"> Color. </param>
        public static void Arc(float3 center, float3 normal, float3 arm, float angle,
            Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Arc.Draw(center, normal, arm, angle, color);
        }

        /// <summary>
        /// Draws a cone.
        /// </summary>
        /// <param name="point"> Point in world space. </param>
        /// <param name="axis"> Cone axis. </param>
        /// <param name="angle"> Cone angle. </param>
        /// <param name="color"> Color. </param>
        public static void Cone(float3 point, float3 axis, float angle, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Cone.Draw(point, axis, angle, color);
        }

        /// <summary>
        /// Draws a box.
        /// </summary>
        /// <param name="Size"> Size of the box. </param>
        /// <param name="Center"> Center of the box in world space. </param>
        /// <param name="Orientation"> Orientation of the box in world space. </param>
        /// <param name="color"> Color. </param>
        public static void Box(float3 Size, float3 Center, quaternion Orientation, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Box.Draw(Size, Center, Orientation, color);
        }

        /// <summary>
        /// Draws a triangle.
        /// </summary>
        /// <param name="vertex0"> Vertex 0 in world space. </param>
        /// <param name="vertex1"> Vertex 1 in world space. </param>
        /// <param name="vertex2"> Vertex 2 in world space. </param>
        /// <param name="normal"> Triangle normal. </param>
        /// <param name="color"> Color. </param>
        public static void Triangle(float3 vertex0, float3 vertex1, float3 vertex2, float3 normal, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Triangle.Draw(vertex0, vertex1, vertex2, normal, color);
        }

        private void ResetColliderDisplayData()
        {
            DrawMeshUtility.ClearTRS();
            AppendMeshColliders.GetMeshes.ClearReferenceMeshes();
        }

        protected override void OnUpdate()
        {
#if UNITY_EDITOR
            CompleteDependency();

            DrawMeshUtility.DrawPrimitiveMeshes();
            ResetColliderDisplayData();
#endif
        }
    }

    /// <summary>
    /// Draw physics debug display data during simulation
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsSystemGroup))]
    [RequireMatchingQueriesForUpdate]
    public partial class PhysicsDebugDisplaySystem_Default : PhysicsDebugDisplaySystem
    {}

    /// <summary>
    /// Draw physics debug display data when running in the  Editor
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateAfter(typeof(CleanPhysicsDebugDataSystem_Editor))]
    [UpdateInGroup(typeof(PhysicsDisplayDebugGroup))]
    [RequireMatchingQueriesForUpdate]
    public partial class PhysicsDebugDisplaySystem_Editor : PhysicsDebugDisplaySystem
    {}
}
