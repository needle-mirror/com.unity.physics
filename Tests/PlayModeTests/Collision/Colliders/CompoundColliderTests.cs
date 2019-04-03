using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Physics.Tests.Utils;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="CompoundCollider"/>
    /// </summary>
    public class CompoundColliderTests
    {
        [Test]
        public void MassProperties_BuiltFromChildren_MatchesExpected()
        {
            void TestCompoundBox(RigidTransform transform)
            {
                // Create a unit box
                var box = BoxCollider.Create(transform.pos, transform.rot, new float3(1, 1, 1), 0.0f);

                // Create a compound of mini boxes, matching the volume of the single box
                var miniBox = BoxCollider.Create(float3.zero, quaternion.identity, new float3(0.5f, 0.5f, 0.5f), 0.0f);
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f,+0.25f,+0.25f))) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f,+0.25f,+0.25f))) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f,-0.25f,+0.25f))) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f,+0.25f,-0.25f))) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f,-0.25f,+0.25f))) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f,-0.25f,-0.25f))) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f,+0.25f,-0.25f))) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f,-0.25f,-0.25f))) }
                };
                var compound = CompoundCollider.Create(children);
                children.Dispose();

                var boxMassProperties = box.Value.MassProperties;
                var compoundMassProperties = compound.Value.MassProperties;

                TestUtils.AreEqual(compoundMassProperties.Volume, boxMassProperties.Volume, 1e-3f);
                TestUtils.AreEqual(compoundMassProperties.AngularExpansionFactor, boxMassProperties.AngularExpansionFactor, 1e-3f);
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.Transform.pos, boxMassProperties.MassDistribution.Transform.pos, 1e-3f);
                //TestUtils.AreEqual(compoundMassProperties.MassDistribution.Orientation, boxMassProperties.MassDistribution.Orientation, 1e-3f);   // TODO: Figure out why this differs, and if that is a problem
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.InertiaTensor, boxMassProperties.MassDistribution.InertiaTensor, 1e-3f);
            }

            // Compare box with compound at various transforms
            TestCompoundBox(RigidTransform.identity);
            TestCompoundBox(new RigidTransform(quaternion.identity, new float3(1.0f, 2.0f, 3.0f)));
            TestCompoundBox(new RigidTransform(quaternion.EulerXYZ(0.5f, 1.0f, 1.5f), float3.zero));
            TestCompoundBox(new RigidTransform(quaternion.EulerXYZ(0.5f, 1.0f, 1.5f), new float3(1.0f, 2.0f, 3.0f)));
        }
    }
}
