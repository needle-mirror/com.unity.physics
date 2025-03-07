using NUnit.Framework;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsStep_UnitTests
    {
        PhysicsStepAuthoring m_StepAuthoring;

        [SetUp]
        public void SetUp() => m_StepAuthoring = new GameObject("Step").AddComponent<PhysicsStepAuthoring>();

        [TearDown]
        public void TearDown()
        {
            if (m_StepAuthoring != null)
                GameObject.DestroyImmediate(m_StepAuthoring.gameObject);
        }

        [Test]
        public void VerifyDefaultSubstepValue()
        {
            m_StepAuthoring.SimulationType = SimulationType.UnityPhysics;
            Assert.That(m_StepAuthoring.SubstepCount, Is.EqualTo(1));
        }

        [Test]
        public void VerifyDefaultSolverIterationsValue()
        {
            m_StepAuthoring.SimulationType = SimulationType.UnityPhysics;
            Assert.That(m_StepAuthoring.SolverIterationCount, Is.EqualTo(4));
        }

#if HAVOK_PHYSICS_EXISTS
        [Test]
        public void VerifyHavokDefaultSubstepValue()
        {
            m_StepAuthoring.SimulationType = SimulationType.HavokPhysics;
            Assert.That(m_StepAuthoring.SubstepCount, Is.EqualTo(1));
        }
#endif

        [Test]
        public void ChangeSimulationTypeAndVerifySubstepValue()
        {
            m_StepAuthoring.SimulationType = SimulationType.UnityPhysics;
            Assert.That(m_StepAuthoring.SubstepCount, Is.EqualTo(1));

            m_StepAuthoring.SubstepCount = 2;
            Assert.That(m_StepAuthoring.SubstepCount, Is.EqualTo(2));

#if HAVOK_PHYSICS_EXISTS
            m_StepAuthoring.SimulationType = SimulationType.HavokPhysics;
            Assert.That(m_StepAuthoring.SubstepCount, Is.EqualTo(2));

            // When switch back to Unity Physics, the previous value should be retained
            m_StepAuthoring.SimulationType = SimulationType.UnityPhysics;
            Assert.That(m_StepAuthoring.SubstepCount, Is.EqualTo(2));
#endif
        }
    }
}
