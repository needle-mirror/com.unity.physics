#if LEGACY_PHYSICS
using System;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using LegacyCharacter = UnityEngine.CharacterJoint;
using LegacyConfigurable = UnityEngine.ConfigurableJoint;
using LegacyFixed = UnityEngine.FixedJoint;
using LegacyHinge = UnityEngine.HingeJoint;
using LegacyJoint = UnityEngine.Joint;
using LegacySpring = UnityEngine.SpringJoint;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsJointConversionSystem_IntegrationTests : BaseHierarchyConversionTest
    {
        [Test]
        public void ConversionSystems_WhenGOHasJoint_IsEnableCollisionFlagSet(
            [Values(typeof(LegacyHinge), typeof(LegacyFixed), typeof(LegacySpring), typeof(LegacyCharacter), typeof(LegacyConfigurable))]
            Type jointType
        )
        {
            CreateHierarchy(new[] { jointType }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<LegacyJoint>().enableCollision = true;

            TestConvertedData<PhysicsConstrainedBodyPair>(pair => Assert.That(pair.EnableCollision, Is.Not.EqualTo(0)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasSpringJoint_IsMinDistanceValueSet()
        {
            CreateHierarchy(new[] { typeof(LegacySpring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<LegacySpring>().minDistance = 100f;
            var expectedMin =
                math.min(Root.GetComponent<LegacySpring>().minDistance, Root.GetComponent<LegacySpring>().maxDistance);

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1)); // TODO: is this a correct assumption, or should it be two separate constraints for min/max with different stiffness?
                Assert.That(j[0].Min, Is.EqualTo(expectedMin));
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasSpringJoint_IsMaxDistanceValueSet()
        {
            CreateHierarchy(new[] { typeof(LegacySpring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<LegacySpring>().maxDistance = 100f;
            var expectedMax =
                math.max(Root.GetComponent<LegacySpring>().minDistance, Root.GetComponent<LegacySpring>().maxDistance);

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1)); // TODO: is this a correct assumption, or should it be two separate constraints for min/max with different stiffness?
                Assert.That(j[0].Max, Is.EqualTo(expectedMax));
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasHingeJoint_WhenUseLimitsHasValue_FinalConstraintIsAllLinearAxesLocked([Values] bool useLimits)
        {
            CreateHierarchy(new[] { typeof(LegacyHinge) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyHinge>();
            joint.useLimits = useLimits;

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.GreaterThanOrEqualTo(2), "HingeJoint should always produce at least 2 constraints");
                var linearConstraint = j[j.GetConstraints().Length - 1];
                linearConstraint.SpringDamping = linearConstraint.SpringFrequency = 0f; // ignore spring settings
                Assert.That(
                    linearConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Linear,
                        ConstrainedAxes = new bool3(true),
                        Min = 0f,
                        Max = 0f
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasHingeJoint_AndDoesNotUseLimits_FirstConstraintIsOtherAngularAxesLocked()
        {
            CreateHierarchy(new[] { typeof(LegacyHinge) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyHinge>();
            joint.useLimits = false;

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(2), "Unlimited HingeJoint should always produce exactly 2 constraints");
                var angularConstraint = j[0];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { y = true, z = true },
                        Min = 0f,
                        Max = 0f
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasHingeJoint_AndUsesLimits_FirstConstraintIsLimitedAxisWithProperHandedness()
        {
            CreateHierarchy(new[] { typeof(LegacyHinge) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyHinge>();
            joint.useLimits = true;
            var limits = new float2(-15f, 35f);
            joint.limits = new JointLimits { min = limits.x, max = limits.y };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(3), "Limited HingeJoint should always produce exactly 3 constraints");
                var angularConstraint = j[0];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = math.radians(limits);
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { x = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasCharacterJoint_FinalConstraintIsAllLinearAxesLocked()
        {
            CreateHierarchy(new[] { typeof(CharacterJoint) }, Array.Empty<Type>(), Array.Empty<Type>());

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(4), "CharacterJoint should always produce exactly 4 constraints");
                var linearConstraint = j[j.GetConstraints().Length - 1];
                linearConstraint.SpringDamping = linearConstraint.SpringFrequency = 0f; // ignore spring settings
                Assert.That(
                    linearConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Linear,
                        ConstrainedAxes = new bool3(true),
                        Min = 0f,
                        Max = 0f
                    })
                );
            });
        }

        // TODO: verify proper ordering of angular constraints for optimal stability
        [Test]
        public void ConversionSystems_WhenGOHasCharacterJoint_FirstConstraintIsAngularXLimitsWithProperHandedness()
        {
            CreateHierarchy(new[] { typeof(CharacterJoint) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<CharacterJoint>();
            var limits = new float2(-15f, 35f);
            joint.lowTwistLimit = new SoftJointLimit { limit = limits.x };
            joint.highTwistLimit = new SoftJointLimit { limit = limits.y };
            joint.swing1Limit = new SoftJointLimit { limit = 0f };
            joint.swing2Limit = new SoftJointLimit { limit = 0f };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(4), "CharacterJoint should always produce exactly 4 constraints");
                var angularConstraint = j[0];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = -math.radians(limits).yx;
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { x = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasCharacterJoint_SecondConstraintIsAngularYLimitWithProperHandedness()
        {
            CreateHierarchy(new[] { typeof(CharacterJoint) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<CharacterJoint>();
            const float limit = 45f;
            joint.lowTwistLimit = new SoftJointLimit { limit = 0f };
            joint.highTwistLimit = new SoftJointLimit { limit = 0f };
            joint.swing1Limit = new SoftJointLimit { limit = limit };
            joint.swing2Limit = new SoftJointLimit { limit = 0f };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(4), "CharacterJoint should always produce exactly 4 constraints");
                var angularConstraint = j[1];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = new float2(-math.radians(limit), math.radians(limit));
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { y = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasCharacterJoint_FinalConstraintAngularZLimitWithProperHandedness()
        {
            CreateHierarchy(new[] { typeof(CharacterJoint) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<CharacterJoint>();
            const float limit = 45f;
            joint.lowTwistLimit = new SoftJointLimit { limit = 0f };
            joint.highTwistLimit = new SoftJointLimit { limit = 0f };
            joint.swing1Limit = new SoftJointLimit { limit = 0f };
            joint.swing2Limit = new SoftJointLimit { limit = 45f };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(4), "CharacterJoint should always produce exactly 4 constraints");
                var angularConstraint = j[2];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = new float2(-math.radians(limit), math.radians(limit));
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { z = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasConfigurableJoint_LinearMotionAndAngularMotionFree_JointHasNoConstraints()
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Free;
            joint.angularXMotion = ConfigurableJointMotion.Free;
            joint.angularYMotion = ConfigurableJointMotion.Free;
            joint.angularZMotion = ConfigurableJointMotion.Free;

            TestConvertedData<PhysicsJoint>(j => Assert.That(j.GetConstraints().Length, Is.EqualTo(0), "Unlimited ConfigurableJoint should produce no constraints"));
        }

        [TestCase(ConfigurableJointMotion.Locked, ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, true, false, false, TestName = "Linear locked (x)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Locked, ConfigurableJointMotion.Free, false, true, false, TestName = "Linear locked (y)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, ConfigurableJointMotion.Locked, false, false, true, TestName = "Linear locked (z)")]
        [TestCase(ConfigurableJointMotion.Locked, ConfigurableJointMotion.Locked, ConfigurableJointMotion.Locked, true, true, true, TestName = "Linear locked (all)")]
        [TestCase(ConfigurableJointMotion.Limited, ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, true, false, false, TestName = "Linear limited (x)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, ConfigurableJointMotion.Free, false, true, false, TestName = "Linear limited (y)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, false, false, true, TestName = "Linear limited (z)")]
        [TestCase(ConfigurableJointMotion.Limited, ConfigurableJointMotion.Limited, ConfigurableJointMotion.Limited, true, true, true, TestName = "Linear limited (all)")]
        public void ConversionSystems_WhenGOHasConfigurableJoint_LinearMotionLockedOrLimited_AxesAreConstrained(
            ConfigurableJointMotion linearXMotion, ConfigurableJointMotion linearYMotion, ConfigurableJointMotion linearZMotion,
            bool expectedConstrainedX, bool expectedConstrainedY, bool expectedConstrainedZ
        )
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = linearXMotion;
            joint.yMotion = linearYMotion;
            joint.zMotion = linearZMotion;
            joint.angularXMotion = ConfigurableJointMotion.Free;
            joint.angularYMotion = ConfigurableJointMotion.Free;
            joint.angularZMotion = ConfigurableJointMotion.Free;

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1), "ConfigurableJoint with uniform limits on a single axis should produce exactly 1 constraint");
                var expectedConstrainedAxes = new bool3(expectedConstrainedX, expectedConstrainedY, expectedConstrainedZ);
                Assert.That(j[0].ConstrainedAxes, Is.EqualTo(expectedConstrainedAxes));
            });
        }

        [TestCase(ConfigurableJointMotion.Locked, ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, true, false, false, TestName = "Angular locked (x)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Locked, ConfigurableJointMotion.Free, false, true, false, TestName = "Angular locked (y)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, ConfigurableJointMotion.Locked, false, false, true, TestName = "Angular locked (z)")]
        [TestCase(ConfigurableJointMotion.Locked, ConfigurableJointMotion.Locked, ConfigurableJointMotion.Locked, true, true, true, TestName = "Angular locked (all)")]
        [TestCase(ConfigurableJointMotion.Limited, ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, true, false, false, TestName = "Angular limited (x)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, ConfigurableJointMotion.Free, false, true, false, TestName = "Angular limited (y)")]
        [TestCase(ConfigurableJointMotion.Free, ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, false, false, true, TestName = "Angular limited (z)")]
        public void ConversionSystems_WhenGOHasConfigurableJoint_AngularMotionLockOrLimited_AxesAreConstrained(
            ConfigurableJointMotion angularXMotion, ConfigurableJointMotion angularYMotion, ConfigurableJointMotion angularZMotion,
            bool expectedConstrainedX, bool expectedConstrainedY, bool expectedConstrainedZ
        )
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Free;
            joint.angularXMotion = angularXMotion;
            joint.angularYMotion = angularYMotion;
            joint.angularZMotion = angularZMotion;

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1), "ConfigurableJoint with limits on only one axis should produce exactly 1 constraint");
                var expectedConstrainedAxes = new bool3(expectedConstrainedX, expectedConstrainedY, expectedConstrainedZ);
                Assert.That(j[0].ConstrainedAxes, Is.EqualTo(expectedConstrainedAxes));
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasConfigurableJoint_AngularMotion__AllAxesAreLimited()
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Free;
            joint.angularXMotion = ConfigurableJointMotion.Limited;
            joint.angularYMotion = ConfigurableJointMotion.Limited;
            joint.angularZMotion = ConfigurableJointMotion.Limited;

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(3), "ConfigurableJoint with angular limits on all axes should produce exactly 3 constraints");
                Assert.That(j[0].ConstrainedAxes, Is.EqualTo(new bool3(true, false, false)));
                Assert.That(j[1].ConstrainedAxes, Is.EqualTo(new bool3(false, true, false)));
                Assert.That(j[2].ConstrainedAxes, Is.EqualTo(new bool3(false, false, true)));
            });
        }

        // TODO: test to verify ConfigurableJoint linear limit converts properly

        // TODO: verify proper ordering of angular constraints for optimal stability
        [Test]
        public void ConversionSystems_WhenGOHasConfigurableJoint_AngularXLimitsConvertToProperHandedness()
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Free;
            joint.angularXMotion = ConfigurableJointMotion.Limited;
            joint.angularYMotion = ConfigurableJointMotion.Free;
            joint.angularZMotion = ConfigurableJointMotion.Free;
            var limits = new float2(-15f, 35f);
            joint.lowAngularXLimit = new SoftJointLimit { limit = limits.x };
            joint.highAngularXLimit = new SoftJointLimit { limit = limits.y };
            joint.angularYLimit = new SoftJointLimit { limit = 0f };
            joint.angularZLimit = new SoftJointLimit { limit = 0f };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1), "ConfigurableJoint with limits on only one axis should produce exactly 1 constraint");
                var angularConstraint = j[0];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = -math.radians(limits).yx;
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { x = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasConfigurableJoint_AngularYLimitConvertsToProperHandedness()
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Free;
            joint.angularXMotion = ConfigurableJointMotion.Free;
            joint.angularYMotion = ConfigurableJointMotion.Limited;
            joint.angularZMotion = ConfigurableJointMotion.Free;
            const float limit = 45f;
            joint.lowAngularXLimit = new SoftJointLimit { limit = 0f };
            joint.highAngularXLimit = new SoftJointLimit { limit = 0f };
            joint.angularYLimit = new SoftJointLimit { limit = limit };
            joint.angularZLimit = new SoftJointLimit { limit = 0f };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1), "ConfigurableJoint with limits on only one axis should produce exactly 1 constraint");
                var angularConstraint = j[0];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = new float2(-math.radians(limit), math.radians(limit));
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { y = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGOHasConfigurableJoint_AngularZLimitConvertsToProperHandedness()
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            var joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Free;
            joint.angularXMotion = ConfigurableJointMotion.Free;
            joint.angularYMotion = ConfigurableJointMotion.Free;
            joint.angularZMotion = ConfigurableJointMotion.Limited;
            const float limit = 45f;
            joint.lowAngularXLimit = new SoftJointLimit { limit = 0f };
            joint.highAngularXLimit = new SoftJointLimit { limit = 0f };
            joint.angularYLimit = new SoftJointLimit { limit = 0f };
            joint.angularZLimit = new SoftJointLimit { limit = limit };

            TestConvertedData<PhysicsJoint>(j =>
            {
                Assume.That(j.GetConstraints().Length, Is.EqualTo(1), "ConfigurableJoint with limits on only one axis should produce exactly 1 constraint");
                var angularConstraint = j[0];
                angularConstraint.SpringDamping = angularConstraint.SpringFrequency = 0f; // ignore spring settings
                var expectedLimits = new float2(-math.radians(limit), math.radians(limit));
                Assert.That(
                    angularConstraint,
                    Is.EqualTo(new Constraint
                    {
                        Type = ConstraintType.Angular,
                        ConstrainedAxes = new bool3 { z = true },
                        Min = expectedLimits.x,
                        Max = expectedLimits.y
                    })
                );
            });
        }

        [Test]
        public void ConversionSystems_WhenGameObjectHasMultipleJointComponents_CreatesMultipleJoints(
            [Values(typeof(LegacyHinge), typeof(LegacyFixed), typeof(LegacySpring), typeof(LegacyCharacter), typeof(LegacyConfigurable))]
            Type jointType
        )
        {
            CreateHierarchy(new[] { jointType, jointType }, Array.Empty<Type>(), Array.Empty<Type>());

            TestConvertedData<PhysicsJoint>(joints => Assert.That(joints, Has.Length.EqualTo(2).And.All.Not.EqualTo(default(PhysicsJoint))), 2);
        }

        /*
        // following are slow tests used for local regression testing only
        [Test]
        public void TestAllConfigurableJointMotionCombinations(
            [Values(ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, ConfigurableJointMotion.Locked)]ConfigurableJointMotion motionX,
            [Values(ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, ConfigurableJointMotion.Locked)]ConfigurableJointMotion motionY,
            [Values(ConfigurableJointMotion.Free, ConfigurableJointMotion.Limited, ConfigurableJointMotion.Locked)]ConfigurableJointMotion motionZ
        )
        {
            CreateHierarchy(new[] { typeof(LegacyConfigurable) }, Array.Empty<Type>(), Array.Empty<Type>());
            LegacyConfigurable joint = Root.GetComponent<LegacyConfigurable>();
            joint.xMotion = motionX;
            joint.yMotion = motionY;
            joint.zMotion = motionZ;

            joint.angularXMotion = motionX;
            joint.angularYMotion = motionY;
            joint.angularZMotion = motionZ;

            TestConvertedData<PhysicsJoint>(j => Assert.Pass());
        }
        */
    }
}
#endif
