using NUnit.Framework;
using Unity.Burst;

namespace Unity.Physics.Tests.Dynamics.Simulation
{
    class CallbackJobTests
    {
        Unity.Physics.Simulation m_Simulation;
        Physics.PhysicsWorld m_World;

        [SetUp]
        public void SetUp()
        {
            m_Simulation = Unity.Physics.Simulation.Create();
            m_World = new Physics.PhysicsWorld(10, 10, 10);
        }

        [TearDown]
        public void TearDown()
        {
            m_World.Dispose();
            m_Simulation.Dispose();
        }

        [BurstCompile]
        struct DummyBodyPairsJob : IBodyPairsJobBase
        {
            public void Execute(ref ModifiableBodyPair pair)
            {
                // does nothing
            }
        }

        [BurstCompile]
        struct DummyJacobiansJob : IJacobiansJobBase
        {
            public void Execute(ref ModifiableJacobianHeader header, ref ModifiableContactJacobian jacobian)
            {
                // does nothing
            }

            public void Execute(ref ModifiableJacobianHeader header, ref ModifiableTriggerJacobian jacobian)
            {
                // does nothing
            }
        }

        [BurstCompile]
        struct DummyContactsJob : IContactsJobBase
        {
            public void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint contact)
            {
                // does nothing
            }
        }

        [BurstCompile]
        struct DummyTriggerEventsJob : ITriggerEventsJobBase
        {
            public void Execute(TriggerEvent triggerEvent)
            {
                // does nothing
            }
        }

        [BurstCompile]
        struct DummyImpulseEventJob : IImpulseEventsJobBase
        {
            public void Execute(ImpulseEvent impulseEvent)
            {
                // does nothing
            }
        }

        [BurstCompile]
        struct DummyCollisionEventsJob : ICollisionEventsJobBase
        {
            public void Execute(CollisionEvent collisionEvent)
            {
                // does nothing
            }
        }

        [Test]
        public void Test_ScheduleBodyPairsJob_EmptySimulation()
        {
            // Note: we have to force this stage for the scheduling below to pass the simulation stage check
            m_Simulation.m_SimulationScheduleStage = SimulationScheduleStage.PostCreateBodyPairs;

            var dummyJob = new DummyBodyPairsJob();

            // we expect this to not crash with an empty simulation
            var handle = IBodyPairsJobExtensions.ScheduleUnityPhysicsBodyPairsJob(dummyJob, m_Simulation, ref m_World, default);
            handle.Complete();
        }

        [Test]
        public void Test_ScheduleJacobiansJob_EmptySimulation()
        {
            // Note: we have to force this stage for the scheduling below to pass the simulation stage check
            m_Simulation.m_SimulationScheduleStage = SimulationScheduleStage.PostCreateJacobians;

            var dummyJob = new DummyJacobiansJob();

            // we expect this to not crash with an empty simulation
            var handle = IJacobiansJobExtensions.ScheduleUnityPhysicsJacobiansJob(dummyJob, m_Simulation, ref m_World, default);
            handle.Complete();
        }

        [Test]
        public void Test_ScheduleContactsJob_EmptySimulation()
        {
            // Note: we have to force this stage for the scheduling below to pass the simulation stage check
            m_Simulation.m_SimulationScheduleStage = SimulationScheduleStage.PostCreateContacts;

            var dummyJob = new DummyContactsJob();

            // we expect this to not crash with an empty simulation
            var handle = IContactsJobExtensions.ScheduleUnityPhysicsContactsJob(dummyJob, m_Simulation, ref m_World, default);
            handle.Complete();
        }

        [Test]
        public void Test_ScheduleTriggerEventsJob_EmptySimulation()
        {
            // Note: we have to force this stage for the scheduling below to pass the simulation stage check
            m_Simulation.m_SimulationScheduleStage = SimulationScheduleStage.Idle;

            var dummyJob = new DummyTriggerEventsJob();

            // we expect this to not crash with an empty simulation
            var handle = ITriggerEventJobExtensions.ScheduleUnityPhysicsTriggerEventsJob(dummyJob, m_Simulation, default);
            handle.Complete();
        }

        [Test]
        public void Test_ScheduleImpulseEventsJob_EmptySimulation()
        {
            // Note: we have to force this stage for the scheduling below to pass the simulation stage check
            m_Simulation.m_SimulationScheduleStage = SimulationScheduleStage.Idle;

            var dummyJob = new DummyImpulseEventJob();

            // we expect this to not crash with an empty simulation
            var handle = IImpulseEventJobExtensions.ScheduleUnityPhysicsImpulseEventsJob(dummyJob, m_Simulation, default);
            handle.Complete();
        }

        [Test]
        public void Test_ScheduleCollisionEventsJob_EmptySimulation()
        {
            // Note: we have to force this stage for the scheduling below to pass the simulation stage check
            m_Simulation.m_SimulationScheduleStage = SimulationScheduleStage.Idle;

            var dummyJob = new DummyCollisionEventsJob();

            // we expect this to not crash with an empty simulation
            var handle = ICollisionEventJobExtensions.ScheduleUnityPhysicsCollisionEventsJob(dummyJob, m_Simulation, default);
            handle.Complete();
        }
    }
}
