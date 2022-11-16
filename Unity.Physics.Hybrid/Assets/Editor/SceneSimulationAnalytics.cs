#if UNITY_EDITOR
#if ENABLE_CLOUD_SERVICES_ANALYTICS
using System;
using Unity.Entities;
using Unity.Physics.Systems;
using UnityEditor;
using UnityEngine;
using UnityEngine.Analytics;


namespace Unity.Physics.Hybrid
{
    [InitializeOnLoad]
    public class SceneSimulationAnalytics
    {
        const int k_MaxEventsPerHour = 1000;
        const int k_MaxNumberOfElements = 1000;
        const string k_VendorKey = "unity.entities";
        const string k_EventName = "sceneSimulationData";

        static bool s_EventRegistered = false;
        static SimulationData s_SimulationData;

        // register an event handler when the class is initialized
        static SceneSimulationAnalytics()
        {
            EditorApplication.playModeStateChanged += ModeChanged;
        }

        private static void ModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.ExitingPlayMode)
            {
                SendAnalyticsEvent();
            }
        }

        static bool EnableAnalytics()
        {
            if (!s_EventRegistered)
            {
                AnalyticsResult result = EditorAnalytics.RegisterEventWithLimit(k_EventName, k_MaxEventsPerHour,
                    k_MaxNumberOfElements, k_VendorKey);
                s_EventRegistered = result == AnalyticsResult.Ok;
                s_SimulationData = new SimulationData();
            }

            return s_EventRegistered;
        }

        public static void SendAnalyticsEvent()
        {
            // The event shouldn't be able to report if this is disabled but if we know we're not going to report.
            // Let's early out and not waste time gathering all the data.
            if (!EditorAnalytics.enabled)
                return;

            if (!EnableAnalytics())
                return;

            // Note: we update the physics analytics data in every update (see BuildPhysicsWorld.cs)
            // and send it off here on playmode exit only (see function ModeChanged() above), for each world that
            // contains physics.
            foreach (var world in World.All)
            {
                using var query = world.EntityManager.CreateEntityQuery(typeof(PhysicsAnalyticsSingleton));

                if (query.TryGetSingleton(out PhysicsAnalyticsSingleton physicsAnalyticsSingleton))
                {
                    CopyToSimulationData(physicsAnalyticsSingleton);
                    s_SimulationData.world_name = world.Name;

                    EditorAnalytics.SendEventWithLimit(k_EventName, s_SimulationData);
                    physicsAnalyticsSingleton.Clear();
                    s_SimulationData.Clear();
                }
            }
        }

        static void CopyToSimulationData(PhysicsAnalyticsSingleton physicsSingleton)
        {
            s_SimulationData.simulation_type = physicsSingleton.m_SimulationType.ToString();
            s_SimulationData.static_rigidbody_max_count = physicsSingleton.m_MaxNumberOfStaticBodiesInAScene;
            s_SimulationData.dynamic_rigidbody_max_count = physicsSingleton.m_MaxNumberOfDynamicBodiesInAScene;

            s_SimulationData.box_collider_max_count = physicsSingleton.m_MaxNumberOfBoxesInAScene;
            s_SimulationData.capsule_collider_max_count = physicsSingleton.m_MaxNumberOfCapsulesInAScene;
            s_SimulationData.mesh_collider_max_count = physicsSingleton.m_MaxNumberOfMeshesInAScene;
            s_SimulationData.sphere_collider_max_count = physicsSingleton.m_MaxNumberOfSpheresInAScene;
            s_SimulationData.terrain_collider_max_count = physicsSingleton.m_MaxNumberOfTerrainsInAScene;
            s_SimulationData.convex_collider_max_count = physicsSingleton.m_MaxNumberOfConvexesInAScene;
            s_SimulationData.quad_collider_max_count = physicsSingleton.m_MaxNumberOfQuadsInAScene;
            s_SimulationData.triangle_collider_max_count = physicsSingleton.m_MaxNumberOfTrianglesInAScene;
            s_SimulationData.compound_collider_max_count = physicsSingleton.m_MaxNumberOfCompoundsInAScene;

            s_SimulationData.linear_max_count = physicsSingleton.m_MaxNumberOfLinearConstraintsInAScene;
            s_SimulationData.angular_max_count = physicsSingleton.m_MaxNumberOfAngularConstraintsInAScene;
            s_SimulationData.motor_planar_max_count = physicsSingleton.m_MaxNumberOfPositionMotorsInAScene;
            s_SimulationData.rotation_motor_max_count = physicsSingleton.m_MaxNumberOfRotationMotorsInAScene;
            s_SimulationData.linear_velocity_motor_max_count = physicsSingleton.m_MaxNumberOfLinearVelocityMotorsInAScene;
            s_SimulationData.angular_velocity_motor_max_count = physicsSingleton.m_MaxNumberOfAngularVelocityMotorsInAScene;
        }

        struct SimulationData
        {
            public string world_name;
            public string simulation_type;
            public uint static_rigidbody_max_count;
            public uint dynamic_rigidbody_max_count;

            public uint box_collider_max_count;
            public uint capsule_collider_max_count;
            public uint mesh_collider_max_count;
            public uint sphere_collider_max_count;
            public uint terrain_collider_max_count;
            public uint convex_collider_max_count;
            public uint quad_collider_max_count;
            public uint triangle_collider_max_count;
            public uint compound_collider_max_count;

            public uint linear_max_count;
            public uint angular_max_count;
            public uint motor_planar_max_count;
            public uint rotation_motor_max_count;
            public uint linear_velocity_motor_max_count;
            public uint angular_velocity_motor_max_count;

            public void Clear()
            {
                world_name = "Unknown";
                simulation_type = "Unknown";
                static_rigidbody_max_count = 0;
                dynamic_rigidbody_max_count = 0;

                box_collider_max_count = 0;
                capsule_collider_max_count = 0;
                mesh_collider_max_count = 0;
                sphere_collider_max_count = 0;
                terrain_collider_max_count = 0;
                convex_collider_max_count = 0;
                quad_collider_max_count = 0;
                triangle_collider_max_count = 0;
                compound_collider_max_count = 0;

                linear_max_count = 0;
                motor_planar_max_count = 0;
                linear_velocity_motor_max_count = 0;
                angular_max_count = 0;
                rotation_motor_max_count = 0;
                angular_velocity_motor_max_count = 0;
            }
        }
    }
}
#endif // ENABLE_CLOUD_SERVICES_ANALYTICS
#endif // UNITY_EDITOR
