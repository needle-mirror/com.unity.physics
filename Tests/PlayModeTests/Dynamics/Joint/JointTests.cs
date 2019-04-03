using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.Entities;
using UnityEngine;
using UnityEngine.TestTools;
using static Unity.Physics.Math;

namespace Unity.Physics.Tests.Dynamics.Joint
{
    public class JointTests
    {
        [Test]
        public void JointDataCreateTest()
        {
            var aFromJoint = MTransform.Identity;
            var bFromJoint = MTransform.Identity;
            var constraints = new Constraint[0];

            var jointDataRef = JointData.Create(aFromJoint, bFromJoint, constraints);

            var jointData = jointDataRef.Value;

            Assert.AreEqual(MTransform.Identity, jointData.AFromJoint);
            Assert.AreEqual(MTransform.Identity, jointData.BFromJoint);
            Assert.AreEqual(1, jointData.Version);
        }

        [Test]
        public void JointDataCreateBallAndSocketTest()
        {
            var positionAinA = float3.zero;
            var positionBinB = float3.zero;

            var jointDataRef = JointData.CreateBallAndSocket(positionAinA, positionBinB);

            var jointData = jointDataRef.Value;
            Assert.AreEqual(MTransform.Identity, jointData.AFromJoint);
            Assert.AreEqual(MTransform.Identity, jointData.BFromJoint);
            Assert.AreEqual(1, jointData.Version);
            Assert.AreEqual(1, jointData.NumConstraints);

            var constraint = jointDataRef.Value.Constraints[0];
            Assert.AreEqual(new bool3(true), constraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Linear, constraint.Type);
            Assert.AreEqual(0.0f, constraint.Min);
            Assert.AreEqual(0.0f, constraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, constraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, constraint.SpringDamping);
        }

        [Test]
        public void JointDataCreateStiffSpringTest()
        {
            var positionAinA = new float3(0.0f, 1.0f, 2.0f);
            var positionBinB = new float3(1.0f, 0.0f, 3.0f);
            var minDistance = 1.0f;
            var maxDistance = 10.0f;

            var jointDataRef = JointData.CreateStiffSpring(positionAinA, positionBinB, minDistance, maxDistance);

            var jointData = jointDataRef.Value;
            Assert.AreEqual(positionAinA, jointData.AFromJoint.Translation);
            Assert.AreEqual(positionBinB, jointData.BFromJoint.Translation);
            Assert.AreEqual(1, jointData.Version);
            Assert.AreEqual(1, jointData.NumConstraints);

            var constraint = jointDataRef.Value.Constraints[0];
            Assert.AreEqual(new bool3(true), constraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Linear, constraint.Type);
            Assert.AreEqual(1.0f, constraint.Min);
            Assert.AreEqual(10.0f, constraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, constraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, constraint.SpringDamping);
        }

        [Test]
        public void JointDataCreatePrismaticTest()
        {
            var positionAinA = new float3(0.0f, 1.0f, 2.0f);
            var positionBinB = new float3(1.0f, 0.0f, 3.0f);
            var axisInB = float3.zero;

            var minDistanceOnAxis = 1.0f;
            var maxDistanceOnAxis = 10.0f;
            var minDistanceFromAxis = 2.0f;
            var maxDistanceFromAxis = 20.0f;

            var jointDataRef = JointData.CreatePrismatic(positionAinA, positionBinB, axisInB, minDistanceOnAxis, maxDistanceOnAxis, minDistanceFromAxis, maxDistanceFromAxis);

            var jointData = jointDataRef.Value;
            Assert.AreEqual(positionAinA, jointData.AFromJoint.Translation);
            Assert.AreEqual(positionBinB, jointData.BFromJoint.Translation);
            Assert.AreEqual(1, jointData.Version);
            Assert.AreEqual(2, jointData.NumConstraints);

            var planarConstraint = jointDataRef.Value.Constraints[0];
            Assert.AreEqual(new bool3(true, false, false), planarConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Linear, planarConstraint.Type);
            Assert.AreEqual(1.0f, planarConstraint.Min);
            Assert.AreEqual(10.0f, planarConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, planarConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, planarConstraint.SpringDamping);

            var cylindricalConstraint = jointDataRef.Value.Constraints[1];
            Assert.AreEqual(new bool3(false, true, true), cylindricalConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Linear, cylindricalConstraint.Type);
            Assert.AreEqual(2.0f, cylindricalConstraint.Min);
            Assert.AreEqual(20.0f, cylindricalConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, cylindricalConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, cylindricalConstraint.SpringDamping);
        }

        [Test]
        public void JointDataCreateHingeTest()
        {
            var positionAinA = new float3(0.0f, 1.0f, 2.0f);
            var positionBinB = new float3(1.0f, 0.0f, 3.0f);
            var axisInA = float3.zero;
            var axisInB = float3.zero;

            var jointDataRef = JointData.CreateHinge(positionAinA, positionBinB, axisInA, axisInB);

            var jointData = jointDataRef.Value;
            Assert.AreEqual(positionAinA, jointData.AFromJoint.Translation);
            Assert.AreEqual(positionBinB, jointData.BFromJoint.Translation);
            Assert.AreEqual(1, jointData.Version);
            Assert.AreEqual(2, jointData.NumConstraints);

            var hingeConstraint = jointDataRef.Value.Constraints[0];
            Assert.AreEqual(new bool3(false, true, true), hingeConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, hingeConstraint.Type);
            Assert.AreEqual(0.0f, hingeConstraint.Min);
            Assert.AreEqual(0.0f, hingeConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, hingeConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, hingeConstraint.SpringDamping);

            var ballAndSocketConstraint = jointDataRef.Value.Constraints[1];
            Assert.AreEqual(new bool3(true, true, true), ballAndSocketConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Linear, ballAndSocketConstraint.Type);
            Assert.AreEqual(0.0f, ballAndSocketConstraint.Min);
            Assert.AreEqual(0.0f, ballAndSocketConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, ballAndSocketConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, ballAndSocketConstraint.SpringDamping);
        }

        [Test]
        public void JointDataCreateLimitedHingeTest()
        {
            var positionAinA = new float3(0.0f, 1.0f, 2.0f);
            var positionBinB = new float3(1.0f, 0.0f, 3.0f);
            var axisInA = new float3(1.0f, 0.0f, 0.0f);
            var axisInB = new float3(1.0f, 0.0f, 0.0f);
            var perpendicularInA = new float3(0.0f, 1.0f, 0.0f);
            var perpendicularInB = new float3(0.0f, 1.0f, 0.0f);
            var minAngle = -0.5f;
            var maxAngle = 0.5f;
            
            var jointDataRef = JointData.CreateLimitedHinge(positionAinA, positionBinB, axisInA, axisInB, perpendicularInA, perpendicularInB, minAngle, maxAngle);

            var jointData = jointDataRef.Value;
            Assert.AreEqual(positionAinA, jointData.AFromJoint.Translation);
            Assert.AreEqual(positionBinB, jointData.BFromJoint.Translation);
            Assert.AreEqual(1, jointData.Version);
            Assert.AreEqual(3, jointData.NumConstraints);

            var twistConstraint = jointDataRef.Value.Constraints[0];
            Assert.AreEqual(new bool3(true, false, false), twistConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, twistConstraint.Type);
            Assert.AreEqual(-0.5f, twistConstraint.Min);
            Assert.AreEqual(0.5f, twistConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, twistConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, twistConstraint.SpringDamping);

            var hingeConstraint = jointDataRef.Value.Constraints[1];
            Assert.AreEqual(new bool3(false, true, true), hingeConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, hingeConstraint.Type);
            Assert.AreEqual(0.0f, hingeConstraint.Min);
            Assert.AreEqual(0.0f, hingeConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, hingeConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, hingeConstraint.SpringDamping);

            var ballAndSocketConstraint = jointDataRef.Value.Constraints[2];
            Assert.AreEqual(new bool3(true, true, true), ballAndSocketConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Linear, ballAndSocketConstraint.Type);
            Assert.AreEqual(0.0f, ballAndSocketConstraint.Min);
            Assert.AreEqual(0.0f, ballAndSocketConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, ballAndSocketConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, ballAndSocketConstraint.SpringDamping);
        }

        [Test]
        public void JointDataCreateRagdollTest()
        {
            var positionAinA = new float3(0.0f, 1.0f, 2.0f);
            var positionBinB = new float3(1.0f, 0.0f, 3.0f);
            var twistAxisInA = new float3(1.0f, 0.0f, 0.0f);
            var twistAxisInB = new float3(1.0f, 0.0f, 0.0f);
            var perpendicularInA = new float3(0.0f, 1.0f, 0.0f);
            var perpendicularInB = new float3(0.0f, 1.0f, 0.0f);
            var maxConeAngle = 0.8f;
            var minPerpendicularAngle = 0.1f;
            var maxPerpendicularAngle = 1.1f;
            var minTwistAngle = 0.2f;
            var maxTwistAngle = 1.2f;

            BlobAssetReference<JointData> jointData0;
            BlobAssetReference<JointData> jointData1;

            JointData.CreateRagdoll(positionAinA, positionBinB, twistAxisInA, twistAxisInB, perpendicularInA, perpendicularInB,
                maxConeAngle, minPerpendicularAngle, maxPerpendicularAngle, minTwistAngle, maxTwistAngle,
                out jointData0, out jointData1);

            var joint0 = jointData0.Value;
            Assert.AreEqual(positionAinA, joint0.AFromJoint.Translation);
            Assert.AreEqual(positionBinB, joint0.BFromJoint.Translation);
            Assert.AreEqual(1, joint0.Version);
            Assert.AreEqual(2, joint0.NumConstraints);

            var twistConstraint = jointData0.Value.Constraints[0];
            Assert.AreEqual(new bool3(true, false, false), twistConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, twistConstraint.Type);
            Assert.AreEqual(0.2f, twistConstraint.Min);
            Assert.AreEqual(1.2f, twistConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, twistConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, twistConstraint.SpringDamping);

            var coneConstraint0 = jointData0.Value.Constraints[1];
            Assert.AreEqual(new bool3(false, true, true), coneConstraint0.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, coneConstraint0.Type);
            Assert.AreEqual(0.0f, coneConstraint0.Min);
            Assert.AreEqual(0.8f, coneConstraint0.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, coneConstraint0.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, coneConstraint0.SpringDamping);

            var joint1 = jointData1.Value;
            Assert.AreEqual(positionAinA, joint1.AFromJoint.Translation);
            Assert.AreEqual(positionBinB, joint1.BFromJoint.Translation);
            Assert.AreEqual(1, joint1.Version);
            Assert.AreEqual(2, joint1.NumConstraints);

            var coneConstraint1 = jointData0.Value.Constraints[0];
            Assert.AreEqual(new bool3(true, false, false), coneConstraint1.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, coneConstraint1.Type);
            Assert.AreEqual(0.2f, coneConstraint1.Min);
            Assert.AreEqual(1.2f, coneConstraint1.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, coneConstraint1.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, coneConstraint1.SpringDamping);

            var ballAndSocketConstraint = jointData0.Value.Constraints[1];
            Assert.AreEqual(new bool3(false, true, true), ballAndSocketConstraint.ConstrainedAxes);
            Assert.AreEqual(ConstraintType.Angular, ballAndSocketConstraint.Type);
            Assert.AreEqual(0.0f, ballAndSocketConstraint.Min);
            Assert.AreEqual(0.8f, ballAndSocketConstraint.Max);
            Assert.AreEqual(Constraint.DefaultSpringFrequency, ballAndSocketConstraint.SpringFrequency);
            Assert.AreEqual(Constraint.DefaultSpringDamping, ballAndSocketConstraint.SpringDamping);
        }

    }
}
