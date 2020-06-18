#if UNITY_DATAFLOWGRAPH_EXISTS
using System;
using NUnit.Framework;
using Unity.DataFlowGraph;
using Unity.Burst;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Tests.Utils;
using Unity.Collections;
using Random = Unity.Mathematics.Random;

namespace Unity.Physics.Tests.DFG
{
    internal class ColliderCastNodeTests
    {
        const uint seed = 0x87654321;
        
        private Random m_Rnd = new Random(seed);
        private PhysicsWorld m_World;

        [SetUp]
        public void SetUp()
        {
            m_World = TestUtils.GenerateRandomWorld(ref m_Rnd, 10, 10.0f, 0, true);
        }

        [TearDown]
        public void TearDown()
        {
            m_World.Dispose(); 
        }

        [BurstCompile]
        struct ColliderCastJob : IJob, IDisposable
        {
            private PhysicsWorld World;
            private ColliderCastInput Input;

            [WriteOnly] private NativeArray<ColliderCastHit> HitArray;
            [WriteOnly] private NativeArray<bool> HitSuccessArray;

            public ColliderCastHit Hit { get => HitArray[0]; }
            public bool HitSuccess { get => HitSuccessArray[0]; }
            
            public static ColliderCastJob Create(PhysicsWorld world, ColliderCastInput input)
            {
                return new ColliderCastJob()
                {
                    World = world,
                    Input = input,
                    HitArray = new NativeArray<ColliderCastHit>(1, Allocator.TempJob),
                    HitSuccessArray = new NativeArray<bool>(1, Allocator.TempJob)
                };
            }

            public void Dispose()
            {
                HitArray.Dispose();
                HitSuccessArray.Dispose();
            }
            
            public void Execute()
            {
                HitSuccessArray[0] = World.CastCollider(Input, out ColliderCastHit hit);
                HitArray[0] = hit;
            }
        }

#if !UNITY_EDITOR        
        // Test is only run where Burst is AOT
        [Test]
#endif
        public unsafe void ColliderCastNode_ClosestHit_Matches_ColliderCastQuery_ClosestHit()
        {
            ColliderCastHit hit, hitQuery;
            bool hitSuccess, hitSuccessQuery;
            
            int numTests = 1000;
            for (int iTest = 0; iTest < numTests; iTest++)
            {
                // Generate common random query inputs
                RigidTransform transform = new RigidTransform
                {
                    pos = m_Rnd.NextFloat3(-10.0f, 10.0f),
                    rot = (m_Rnd.NextInt(10) > 0) ? m_Rnd.NextQuaternionRotation() : quaternion.identity,
                };
                var startPos = transform.pos;
                var endPos = startPos + m_Rnd.NextFloat3(-5.0f, 5.0f);

                var collider = TestUtils.GenerateRandomConvex(ref m_Rnd);
                ColliderCastInput input = new ColliderCastInput
                {
                    Collider = (Collider*) collider.GetUnsafePtr(),
                    Start = startPos,
                    End = endPos,
                    Orientation = transform.rot,
                };

                // Build and evaluate node set.
                using (var safetyManager = AtomicSafetyManager.Create())
                {
                    var collisionWorldProxy = new CollisionWorldProxy(m_World.CollisionWorld, &safetyManager);

                    using (var set = new NodeSet())
                    {
                        var colliderCastNode = set.Create<ColliderCastNode>();

                        set.SetData(colliderCastNode, ColliderCastNode.KernelPorts.CollisionWorld, collisionWorldProxy);
                        set.SetData(colliderCastNode, ColliderCastNode.KernelPorts.Input, input);

                        var hitGraphValue = set.CreateGraphValue(colliderCastNode, ColliderCastNode.KernelPorts.Hit);
                        var hitSuccessGraphValue = set.CreateGraphValue(colliderCastNode, ColliderCastNode.KernelPorts.HitSuccess);

                        set.Update();

                        hit = set.GetValueBlocking(hitGraphValue);
                        hitSuccess = set.GetValueBlocking(hitSuccessGraphValue);

                        set.ReleaseGraphValue(hitGraphValue);
                        set.ReleaseGraphValue(hitSuccessGraphValue);

                        set.Destroy(colliderCastNode);
                    }
                }

                // Compare with results obtained from query.
                using (var job = ColliderCastJob.Create(m_World, input))
                {
                    job.Schedule().Complete();

                    hitSuccessQuery = job.HitSuccess;
                    hitQuery = job.Hit;
                }

                if (hitSuccessQuery)
                {
                    Assert.That(hitSuccess, Is.True);
                    Assert.That(hit, Is.EqualTo(hitQuery), $"Iteration {iTest} failed with {input}");
                } 
                else
                {
                    Assert.That(hitSuccess, Is.False);
                }
            }
        }
        
        [Test]
        public unsafe void ColliderCastNode_WithInvalidProxy_Returns_HitSuccess_Equal_To_False()
        {
            bool hitSuccess;
            
            // Empty CollisionWorldProxy
            var collisionWorldProxy = new CollisionWorldProxy();

            var collider = TestUtils.GenerateRandomConvex(ref m_Rnd);
            ColliderCastInput input = new ColliderCastInput
            {
                Collider = (Collider*) collider.GetUnsafePtr(),
                Start = float3.zero,
                End = float3.zero,
                Orientation = quaternion.identity
            };
            
            using (var set = new NodeSet())
            {
                var colliderCastNode = set.Create<ColliderCastNode>();

                set.SetData(colliderCastNode, ColliderCastNode.KernelPorts.CollisionWorld, collisionWorldProxy);
                set.SetData(colliderCastNode, ColliderCastNode.KernelPorts.Input, input);
               
                var hitSuccessGraphValue = set.CreateGraphValue(colliderCastNode, ColliderCastNode.KernelPorts.HitSuccess);
                
                set.Update();
               
                hitSuccess = set.GetValueBlocking(hitSuccessGraphValue);
              
                set.ReleaseGraphValue(hitSuccessGraphValue);
                set.Destroy(colliderCastNode);
            }
           
            Assert.That(hitSuccess, Is.False);
        }
        
    }
}
#endif