using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics.LowLevel
{
    // An event raised when a pair of bodies have collided during solving.
    struct CollisionEvent
    {
        public BodyIndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;
        public float3 Normal;

        // The total impulse applied by the solver for this pair
        internal float SolverImpulse;

        // Number of narrow phase contact points
        internal int NumNarrowPhaseContactPoints;

        internal static int CalculateSize(int numContactPoints)
        {
            return UnsafeUtility.SizeOf<CollisionEvent>() + numContactPoints * UnsafeUtility.SizeOf<ContactPoint>();
        }

        internal unsafe ref ContactPoint AccessContactPoint(int pointIndex)
        {
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<CollisionEvent>() + pointIndex * UnsafeUtility.SizeOf<ContactPoint>();
            return ref UnsafeUtilityEx.AsRef<ContactPoint>(ptr);
        }

        // Calculate extra details about the collision, by re-integrating the leaf colliders to the time of collision
        internal unsafe Physics.CollisionEvent.Details CalculateDetails(
            ref PhysicsWorld physicsWorld, float timeStep, Velocity inputVelocityA, Velocity inputVelocityB, NativeArray<ContactPoint> narrowPhaseContactPoints)
        {
            int bodyAIndex = BodyIndices.BodyAIndex;
            int bodyBIndex = BodyIndices.BodyBIndex;
            bool bodyAIsDynamic = bodyAIndex < physicsWorld.MotionVelocities.Length;
            bool bodyBIsDynamic = bodyBIndex < physicsWorld.MotionVelocities.Length;

            MotionVelocity motionVelocityA = bodyAIsDynamic ? physicsWorld.MotionVelocities[bodyAIndex] : MotionVelocity.Zero;
            MotionVelocity motionVelocityB = bodyBIsDynamic ? physicsWorld.MotionVelocities[bodyBIndex] : MotionVelocity.Zero;
            MotionData motionDataA = bodyAIsDynamic ? physicsWorld.MotionDatas[bodyAIndex] : MotionData.Zero;
            MotionData motionDataB = bodyBIsDynamic ? physicsWorld.MotionDatas[bodyBIndex] : MotionData.Zero;

            float estimatedImpulse = SolverImpulse;

            // First calculate minimum time of impact and estimate the impulse
            float toi = timeStep;
            {
                float sumRemainingVelocities = 0.0f;
                float numRemainingVelocities = 0.0f;
                for (int i = 0; i < narrowPhaseContactPoints.Length; i++)
                {
                    var cp = narrowPhaseContactPoints[i];

                    // Collect data for impulse estimation
                    {
                        float3 pointVelA = GetPointVelocity(motionDataA.WorldFromMotion,
                            motionVelocityA.LinearVelocity, motionVelocityA.AngularVelocity, cp.Position + Normal * cp.Distance);
                        float3 pointVelB = GetPointVelocity(motionDataB.WorldFromMotion,
                            motionVelocityB.LinearVelocity, motionVelocityB.AngularVelocity, cp.Position);
                        float projRelVel = math.dot(pointVelB - pointVelA, Normal);
                        if (projRelVel > 0.0f)
                        {
                            sumRemainingVelocities += projRelVel;
                            numRemainingVelocities += 1.0f;
                        }
                    }

                    // Get minimum time of impact
                    {
                        float3 pointVelA = GetPointVelocity(motionDataA.WorldFromMotion,
                            inputVelocityA.Linear, inputVelocityA.Angular, cp.Position + Normal * cp.Distance);
                        float3 pointVelB = GetPointVelocity(motionDataB.WorldFromMotion,
                            inputVelocityB.Linear, inputVelocityB.Angular, cp.Position);
                        float projRelVel = math.dot(pointVelB - pointVelA, Normal);
                        if (projRelVel > 0.0f)
                        {
                            float newToi = math.max(0.0f, cp.Distance / projRelVel);
                            toi = math.min(toi, newToi);
                        }
                        else if (cp.Distance <= 0.0f)
                        {
                            // If in penetration, time of impact is 0 for sure
                            toi = 0.0f;
                        }
                    }
                }

                if (numRemainingVelocities > 0.0f)
                {
                    float sumInvMass = motionVelocityA.InverseInertiaAndMass.w + motionVelocityB.InverseInertiaAndMass.w;
                    estimatedImpulse += sumRemainingVelocities / (numRemainingVelocities * sumInvMass);
                }
            }

            // Then, sub-integrate for time of impact and keep contact points closer than hitDistanceThreshold
            {
                int estimatedContactPointCount = 0;
                for (int i = 0; i < narrowPhaseContactPoints.Length; i++)
                {
                    // Estimate new position
                    var cp = narrowPhaseContactPoints[i];
                    {
                        float3 pointVelA = GetPointVelocity(motionDataA.WorldFromMotion, inputVelocityA.Linear, inputVelocityA.Angular, cp.Position + Normal * cp.Distance);
                        float3 pointVelB = GetPointVelocity(motionDataB.WorldFromMotion, inputVelocityB.Linear, inputVelocityB.Angular, cp.Position);
                        float3 relVel = pointVelB - pointVelA;
                        float projRelVel = math.dot(relVel, Normal);

                        // Position the point on body A
                        cp.Position += Normal * cp.Distance;

                        // Sub integrate the point
                        cp.Position -= relVel * toi;

                        // Reduce the distance
                        cp.Distance -= projRelVel * toi;

                        // Filter out contacts that are still too far away
                        if (cp.Distance <= physicsWorld.CollisionWorld.CollisionTolerance)
                        {
                            narrowPhaseContactPoints[estimatedContactPointCount++] = cp;
                        }
                    }
                }

                // Instantiate collision details and allocate memory
                var details = new Physics.CollisionEvent.Details
                {
                    EstimatedContactPointPositions = new NativeArray<float3>(estimatedContactPointCount, Allocator.Temp),
                    EstimatedImpulse = estimatedImpulse
                };

                // Fill the contact point positions array
                for (int i = 0; i < estimatedContactPointCount; i++)
                {
                    details.EstimatedContactPointPositions[i] = narrowPhaseContactPoints[i].Position;
                }

                return details;
            }
        }

        private static float3 GetPointVelocity(RigidTransform worldFromMotion, float3 linVel, float3 angVel, float3 point)
        {
            float3 angularVelocity = math.rotate(worldFromMotion, angVel);
            float3 linearVelocity = math.cross(angularVelocity, point - worldFromMotion.pos);
            return linVel + linearVelocity;
        }
    }

    // A stream of collision events.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<CollisionEvent>).
    struct CollisionEvents /* : IEnumerable<CollisionEvent> */
    {
        //@TODO: Unity should have a Allow null safety restriction
        [NativeDisableContainerSafetyRestriction]
        private readonly BlockStream m_EventStream;

        public CollisionEvents(BlockStream eventStream)
        {
            m_EventStream = eventStream;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_EventStream);
        }

        public struct Enumerator /* : IEnumerator<CollisionEvent> */
        {
            private BlockStream.Reader m_Reader;
            private int m_CurrentWorkItem;
            private readonly int m_NumWorkItems;
            private unsafe CollisionEvent* m_Current;
            
            public ref CollisionEvent Current
            {
                get
                {
                    unsafe
                    {
                        return ref UnsafeUtilityEx.AsRef<CollisionEvent>(m_Current);
                    }
                }
            }

            public Enumerator(BlockStream stream)
            {
                m_Reader = stream.IsCreated ? stream : new BlockStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;

                unsafe
                {
                    m_Current = default;
                }

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    int currentSize = m_Reader.Read<int>();
                    AdvanceReader();

                    unsafe
                    {
                        m_Current = (CollisionEvent*)m_Reader.Read(currentSize);
                    }

                    AdvanceReader();
                    return true;
                }
                return false;
            }

            private void AdvanceReader()
            {
                while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_NumWorkItems)
                {
                    m_Reader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }
    }
}
