#if UNITY_DATAFLOWGRAPH_EXISTS
using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.DataFlowGraph;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.Physics.Tests.Utils;
using Random = Unity.Mathematics.Random;

namespace Unity.Physics.Tests.DFG
{
    internal class RaycastNodeTests
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
        struct RaycastJob : IJob, IDisposable
        {
            private PhysicsWorld World;
            private RaycastInput Input;

            [WriteOnly] private NativeArray<RaycastHit> HitArray;
            [WriteOnly] private NativeArray<bool> HitSuccessArray;

            public RaycastHit Hit { get => HitArray[0]; }
            public bool HitSuccess { get => HitSuccessArray[0]; }
            
            public static RaycastJob Create(PhysicsWorld world, RaycastInput input)
            {
                return new RaycastJob()
                {
                    World = world,
                    Input = input,
                    HitArray = new NativeArray<RaycastHit>(1, Allocator.TempJob),
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
                HitSuccessArray[0] = World.CastRay(Input, out RaycastHit hit);
                HitArray[0] = hit;
            }
        }
        
#if !UNITY_EDITOR        
        // Test is only run where Burst is AOT
        [Test]
#endif
        public unsafe void RayCastNode_ClosestHit_Matches_RayCastQuery_ClosestHit()
        {
            RaycastHit hit, hitQuery;
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

                RaycastInput input = new RaycastInput
                {
                    Start = startPos,
                    End = endPos,
                    Filter = CollisionFilter.Default
                };
               
                // Build and evaluate node set.
                using (var safetyManager = AtomicSafetyManager.Create())
                {
                    var collisionWorldProxy = new CollisionWorldProxy(m_World.CollisionWorld, &safetyManager);

                    using (var set = new NodeSet())
                    {
                        var rayCastNode = set.Create<RaycastNode>();

                        set.SetData(rayCastNode, RaycastNode.KernelPorts.CollisionWorld, collisionWorldProxy);
                        set.SetData(rayCastNode, RaycastNode.KernelPorts.Input, input);

                        var hitGraphValue = set.CreateGraphValue(rayCastNode, RaycastNode.KernelPorts.Hit);
                        var hitSuccessGraphValue = set.CreateGraphValue(rayCastNode, RaycastNode.KernelPorts.HitSuccess);

                        set.Update();

                        hit = set.GetValueBlocking(hitGraphValue);
                        hitSuccess = set.GetValueBlocking(hitSuccessGraphValue);

                        set.ReleaseGraphValue(hitGraphValue);
                        set.ReleaseGraphValue(hitSuccessGraphValue);

                        set.Destroy(rayCastNode);
                    }
                }

                // Compare with results obtained from query.
                using (var job = RaycastJob.Create(m_World, input))
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
        public void RayCastNode_WithInvalidProxy_Returns_HitSuccess_Equals_To_False()
        {
            bool hitSuccess;
            
            // Empty CollisionWorldProxy
            var collisionWorldProxy = new CollisionWorldProxy();

            RaycastInput input = new RaycastInput
            {
                Start = float3.zero,
                End = float3.zero,
                Filter = CollisionFilter.Default
            };
            
            using (var set = new NodeSet())
            {
                var rayCastNode = set.Create<RaycastNode>();

                set.SetData(rayCastNode, RaycastNode.KernelPorts.CollisionWorld, collisionWorldProxy);
                set.SetData(rayCastNode, RaycastNode.KernelPorts.Input, input);
               
                var hitSuccessGraphValue = set.CreateGraphValue(rayCastNode, RaycastNode.KernelPorts.HitSuccess);
                
                set.Update();
               
                hitSuccess = set.GetValueBlocking(hitSuccessGraphValue);
              
                set.ReleaseGraphValue(hitSuccessGraphValue);
                set.Destroy(rayCastNode);
            }
           
            Assert.That(hitSuccess, Is.False);
        }
    }
}
#endif