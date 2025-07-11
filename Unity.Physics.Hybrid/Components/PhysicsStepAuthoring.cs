using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using static Unity.Physics.PhysicsStep;

namespace Unity.Physics.Authoring
{
    /// <summary>
    ///     <para>Parameters describing how to step the physics simulation.</para>
    ///     <para>If this component is not present, default values will be used.</para>
    /// </summary>
    [AddComponentMenu("Entities/Physics/Physics Step")]
    [DisallowMultipleComponent]
    [HelpURL(HelpURLs.PhysicsStepAuthoring)]
    public sealed class PhysicsStepAuthoring : MonoBehaviour
    {
        PhysicsStepAuthoring() {}

        /// <summary>
        ///     Specifies the type of physics engine to be used.
        /// </summary>
        public SimulationType SimulationType
        {
            get => m_SimulationType;
            set => m_SimulationType = value;
        }
        [SerializeField]
        [Tooltip("Specifies the type of physics engine to be used.")]
        SimulationType m_SimulationType = Default.SimulationType;

        /// <summary>
        ///     Specifies the amount of gravity present in the physics simulation.
        /// </summary>
        public float3 Gravity
        {
            get => m_Gravity;
            set => m_Gravity = value;
        }
        [SerializeField]
        [Tooltip("Specifies the amount of gravity present in the physics simulation.")]
        float3 m_Gravity = Default.Gravity;

        /// <summary>
        ///     Specifies the number of substep iterations the physics engine will perform.
        ///     Higher values mean more stability.
        /// </summary>
        public int SubstepCount
        {
            get => m_SubstepCount;
            set => m_SubstepCount = value;
        }
        [SerializeField]
        [Tooltip("Specifies the number of substep iterations the physics engine will perform.\n" +
            "Higher values mean smaller timesteps will be taken per frame. This can be more \n" +
            "stable up to a point where the timestep becomes so small computational numerical " +
            "errors can be introduced.")]
        int m_SubstepCount = Default.SubstepCount;

        /// <summary>
        ///     Specifies the number of solver iterations the physics engine will perform.
        ///     Higher values mean more stability, but also worse performance.
        /// </summary>
        public int SolverIterationCount
        {
            get => m_SolverIterationCount;
            set => m_SolverIterationCount = value;
        }
        [SerializeField]
        [Tooltip("Specifies the number of solver iterations the physics engine will perform.\n" +
            "Higher values mean more stability, but also worse performance.")]
        int m_SolverIterationCount = Default.SolverIterationCount;

        /// <summary>
        ///    Enables the contact solver stabilization heuristic.
        /// </summary>
        public bool EnableSolverStabilizationHeuristic
        {
            get => m_EnableSolverStabilizationHeuristic;
            set => m_EnableSolverStabilizationHeuristic = value;
        }
        [SerializeField]
        [Tooltip("Enables the contact solver stabilization heuristic.")]
        bool m_EnableSolverStabilizationHeuristic = Default.SolverStabilizationHeuristicSettings.EnableSolverStabilization;

        /// <summary>
        ///     <para>Enables multi-threaded processing.</para>
        ///     <para>Enabling this option will maximize the use of parallelization in the entire simulation pipeline while disabling it will result in minimal thread usage.</para>
        /// </summary>
        public bool MultiThreaded
        {
            get => m_MultiThreaded;
            set => m_MultiThreaded = value;
        }
        [SerializeField]
        [Tooltip("Enables multi-threaded processing.\n" +
            "Enabling this option will maximize the use of parallelization in the entire simulation pipeline while disabling it will result in minimal thread usage.")]
        bool m_MultiThreaded = Default.MultiThreaded > 0 ? true : false;

        /// <summary>
        /// <para>Sets the collision tolerance.</para>
        /// <para>The collision tolerance specifies the minimum distance required for contacts between rigid bodies to be created.<br/>
        /// This value can be increased if undesired collision tunneling is observed in the simulation.</para>
        /// </summary>
        public float CollisionTolerance
        {
            get => m_CollisionTolerance;
            set => m_CollisionTolerance = value;
        }
        [SerializeField]
        [Tooltip("Sets the collision tolerance.\n" +
            "The collision tolerance specifies the minimum distance required for contacts between rigid bodies to be created.\n" +
            "This value can be increased if undesired collision tunneling is observed in the simulation.")]
        float m_CollisionTolerance = Default.CollisionTolerance;

        /// <summary>
        ///     Specifies whether to update the collision world an additional time after the step for more precise collider queries.
        /// </summary>
        public bool SynchronizeCollisionWorld
        {
            get => m_SynchronizeCollisionWorld;
            set => m_SynchronizeCollisionWorld = value;
        }
        [SerializeField]
        [Tooltip("Specifies whether to update the collision world an additional time after the step for more precise collider queries.")]
        bool m_SynchronizeCollisionWorld = Default.SynchronizeCollisionWorld > 0 ? true : false;

        /// <summary>
        /// <para>Enables the incremental dynamic broadphase.</para>
        /// <para>Enabling this option will update the dynamic broadphase incrementally whenever changes between simulation steps occur,
        /// potentially leading to time savings for cases with many dynamic rigid bodies that don't move or otherwise change.</para>
        /// </summary>
        public bool IncrementalDynamicBroadphase
        {
            get => m_IncrementalDynamicBroadphase;
            set => m_IncrementalDynamicBroadphase = value;
        }
        [SerializeField]
        [Tooltip("Enables the incremental dynamic broadphase.\n" +
            "Enabling this option will update the dynamic broadphase incrementally whenever changes between simulation steps occur, " +
            "potentially leading to time savings for cases with many dynamic rigid bodies that don't move or otherwise change.")]
        bool m_IncrementalDynamicBroadphase = Default.IncrementalDynamicBroadphase;

        /// <summary>
        /// <para>Enables the incremental static broadphase.</para>
        /// <para>Enabling this option will update the static broadphase incrementally whenever changes between simulation steps occur,
        /// potentially leading to time savings for cases with many static rigid bodies that don't move or otherwise change.</para>
        /// </summary>
        public bool IncrementalStaticBroadphase
        {
            get => m_IncrementalStaticBroadphase;
            set => m_IncrementalStaticBroadphase = value;
        }
        [SerializeField]
        [Tooltip("Enables the incremental static broadphase.\n" +
            "Enabling this option will update the static broadphase incrementally whenever changes between simulation steps occur, " +
            "potentially leading to time savings for cases with many static rigid bodies that don't move or otherwise change.")]
        bool m_IncrementalStaticBroadphase = Default.IncrementalStaticBroadphase;

        internal PhysicsStep AsComponent => new PhysicsStep
        {
            SimulationType = SimulationType,
            Gravity = Gravity,
            SubstepCount = SubstepCount,
            SolverIterationCount = SolverIterationCount,
            SolverStabilizationHeuristicSettings = EnableSolverStabilizationHeuristic ?
                new Solver.StabilizationHeuristicSettings
            {
                EnableSolverStabilization = true,
                EnableFrictionVelocities = Default.SolverStabilizationHeuristicSettings.EnableFrictionVelocities,
                VelocityClippingFactor = Default.SolverStabilizationHeuristicSettings.VelocityClippingFactor,
                InertiaScalingFactor = Default.SolverStabilizationHeuristicSettings.InertiaScalingFactor
            } :
            Solver.StabilizationHeuristicSettings.Default,
            MultiThreaded = (byte)(MultiThreaded ? 1 : 0),
            CollisionTolerance = CollisionTolerance,
            SynchronizeCollisionWorld = (byte)(SynchronizeCollisionWorld ? 1 : 0),
            IncrementalDynamicBroadphase = IncrementalDynamicBroadphase,
            IncrementalStaticBroadphase = IncrementalStaticBroadphase
        };

        void OnValidate()
        {
            SubstepCount = math.max(1, SubstepCount);
            SolverIterationCount = math.max(1, SolverIterationCount);
        }
    }

    internal class PhysicsStepBaker : Baker<PhysicsStepAuthoring>
    {
        public override void Bake(PhysicsStepAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, authoring.AsComponent);
        }
    }
}
