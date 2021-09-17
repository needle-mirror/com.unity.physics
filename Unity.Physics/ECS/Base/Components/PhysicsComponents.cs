using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Extensions;
using Unity.Transforms;

namespace Unity.Physics
{
    /// <summary>
    /// Add this tag to a body or joint to exclude it from the physics world.
    /// This allows you to retain all of its other data, but temporarily ignore it for purposes of physics
    /// </summary>
    [Obsolete("PhysicsExclude will be deprecated soon. Instead of adding PhysicsExclude, please remove PhysicsWorldIndex from entities you want to exclude from all physics worlds. (RemovedAfter 2021-10-01)", false)]
    public struct PhysicsExclude : IComponentData {}

    /// <summary>
    /// Shared component for entities that belong to a physics world. Default physics world is built in <see cref="Systems.BuildPhysicsWorld"/>, from entities that have Value of 0.
    /// </summary>
    public struct PhysicsWorldIndex : ISharedComponentData, IEquatable<PhysicsWorldIndex>
    {
        /// <summary>
        /// Index of the physics world that this entity belongs to.
        /// </summary>
        public uint Value;

        /// <summary>
        /// Constructor taking the physics world index, with default value of 0 (used for default physics world).
        /// </summary>
        public PhysicsWorldIndex(uint worldIndex = 0)
        {
            Value = worldIndex;
        }

        public bool Equals(PhysicsWorldIndex other) => Value == other.Value;

        public override int GetHashCode() => (int)Value;
    }

    // The collision geometry of a rigid body.
    // If not present, the rigid body cannot collide with anything.
    public struct PhysicsCollider : IComponentData
    {
        public BlobAssetReference<Collider> Value;  // null is allowed

        public bool IsValid => Value.IsCreated;
        public unsafe Collider* ColliderPtr => (Collider*)Value.GetUnsafePtr();
        public MassProperties MassProperties => Value.IsCreated ? Value.Value.MassProperties : MassProperties.UnitSphere;
    }

    /// <summary>
    /// A buffer element used to associate an original Entity with a collider key in a <see cref="CompoundCollider"/>.
    /// </summary>
    [InternalBufferCapacity(16)]
    public struct PhysicsColliderKeyEntityPair : IBufferElementData
    {
        public ColliderKey Key;
        public Entity Entity;
    }

    // The mass properties of a rigid body.
    // If not present, the rigid body has infinite mass and inertia.
    public struct PhysicsMass : IComponentData
    {
        public RigidTransform Transform;        // center of mass and orientation of principal axes
        public float InverseMass;               // zero is allowed, for infinite mass
        public float3 InverseInertia;           // zero is allowed, for infinite inertia
        public float AngularExpansionFactor;    // see MassProperties.AngularExpansionFactor

        public float3 CenterOfMass { get => Transform.pos; set => Transform.pos = value; }
        public quaternion InertiaOrientation { get => Transform.rot; set => Transform.rot = value; }

        public bool HasInfiniteMass => InverseMass == 0.0f;
        public bool HasInfiniteInertia => !math.any(InverseInertia);
        public bool IsKinematic => HasInfiniteMass && HasInfiniteInertia;

        public static PhysicsMass CreateDynamic(MassProperties massProperties, float mass)
        {
            SafetyChecks.CheckFiniteAndPositiveAndThrow(mass, nameof(mass));

            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = math.rcp(mass),
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

        public static PhysicsMass CreateKinematic(MassProperties massProperties)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = 0f,
                InverseInertia = float3.zero,
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }
    }

    /// <summary>
    /// Add this component to a dynamic body if it needs to sometimes switch to being kinematic.
    /// This allows you to retain its dynamic mass properties on its <see cref="PhysicsMass"/> component, but have the physics solver temporarily treat it as if it were kinematic.
    /// Kinematic bodies will have infinite mass and inertia. They should also not be affected by gravity.
    /// Hence, if IsKinematic is non-zero the value in an associated <see cref="PhysicsGravityFactor"/> component is also ignored.
    /// If SetVelocityToZero is non-zero then the value in an associated <see cref="PhysicsVelocity"/> component is also ignored.
    /// </summary>
    public struct PhysicsMassOverride : IComponentData
    {
        public byte IsKinematic;
        public byte SetVelocityToZero;
    }

    /// <summary>
    /// The velocity of a rigid body.
    /// If absent, the rigid body is static.
    /// </summary>
    public struct PhysicsVelocity : IComponentData
    {
        /// <summary>
        /// The body's world-space linear velocity in units per second.
        /// </summary>
        public float3 Linear;
        /// <summary>
        /// The body's angular velocity in radians per second about each principal axis specified by <see cref="PhysicsMass.Transform"/>.
        /// In order to get or set world-space values, use <see cref="ComponentExtensions.GetAngularVelocityWorldSpace"/> and <see cref="ComponentExtensions.SetAngularVelocityWorldSpace"/>, respectively.
        /// </summary>
        public float3 Angular;

        /// <summary>
        /// Create a <see cref="PhysicsVelocity"/> required to move a body to a target position and orientation.
        /// Use this method to control kinematic bodies directly if they need to generate contact events when moving to their new positions.
        /// If you need to teleport kinematic bodies you can simply set their Translation and Rotation values directly.
        /// </summary>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="bodyPosition">The body's Translation component.</param>
        /// <param name="bodyOrientation">The body's Rotation component.</param>
        /// <param name="targetTransform">The desired Translation and Rotation values the body should move to in world space.</param>
        /// <param name="stepFrequency">The step frequency in the simulation where the body's motion is solved (i.e., 1 / FixedDeltaTime).</param>
        public static PhysicsVelocity CalculateVelocityToTarget(
            in PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation,
            in RigidTransform targetTransform, in float stepFrequency
        )
        {
            var velocity = new PhysicsVelocity();
            var worldFromBody = new RigidTransform(bodyOrientation.Value, bodyPosition.Value);
            var worldFromMotion = math.mul(worldFromBody, bodyMass.Transform);
            PhysicsWorldExtensions.CalculateVelocityToTargetImpl(
                worldFromBody, math.inverse(worldFromMotion.rot), bodyMass.Transform.pos, targetTransform, stepFrequency,
                out velocity.Linear, out velocity.Angular
            );
            return velocity;
        }
    }

    // Optional damping applied to the rigid body velocities during each simulation step.
    // This scales the velocities using: math.clamp(1 - damping * Timestep, 0, 1)
    public struct PhysicsDamping : IComponentData
    {
        public float Linear;     // damping applied to the linear velocity
        public float Angular;    // damping applied to the angular velocity
    }

    // Optional gravity factor applied to a rigid body during each simulation step.
    // This scales the gravity vector supplied to the simulation step.
    public struct PhysicsGravityFactor : IComponentData
    {
        public float Value;
    }

    // Optional custom tags attached to a rigid body.
    // This will be copied to any contacts and Jacobians involving this rigid body,
    // providing additional context to any user logic operating on those structures.
    public struct PhysicsCustomTags : IComponentData
    {
        public byte Value;
    }

    // Parameters describing how to step the physics world.
    // If none is present in the scene, default values will be used.
    public struct PhysicsStep : IComponentData
    {
        public SimulationType SimulationType;
        public float3 Gravity;
        public int SolverIterationCount;
        public Solver.StabilizationHeuristicSettings SolverStabilizationHeuristicSettings;

        public byte MultiThreaded;

        // Whether to synchronize collision world after physics step to enable precise query results.
        // Note that `BuildPhysicsWorld` will do this work on the following frame anyway, so only use this option when
        // another system must know about the results of the simulation before the end of the frame
        // (e.g., to destroy or create some other body that must be present in the following frame).
        // In most cases, tolerating a frame of latency is easier to work with and is better for performance.
        public byte SynchronizeCollisionWorld;

        public static readonly PhysicsStep Default = new PhysicsStep
        {
            SimulationType = SimulationType.UnityPhysics,
            Gravity = -9.81f * math.up(),
            SolverIterationCount = 4,
            SolverStabilizationHeuristicSettings = Solver.StabilizationHeuristicSettings.Default,
            MultiThreaded = 1,
            SynchronizeCollisionWorld = 0
        };
    }

    /// <summary>
    /// Proxy component data structure for CollisionWorld to use in DataFlowGraph nodes.
    /// </summary>
    /// <seealso cref="ColliderCastNode"/>
    /// <seealso cref="RaycastNode"/>
    public unsafe struct CollisionWorldProxy : IComponentData
    {
        struct NativeArrayProxy
        {
            void* Ptr;
            int Size;

            public bool IsCreated => Ptr != null;

            public static NativeArrayProxy Create<T>(NativeArray<T> src) where T : struct
            {
                return new NativeArrayProxy() { Ptr = src.GetUnsafeReadOnlyPtr(), Size = src.Length };
            }

            public static NativeArrayProxy Create<T>(NativeSlice<T> src) where T : struct
            {
                return new NativeArrayProxy() { Ptr = src.GetUnsafeReadOnlyPtr(), Size = src.Length };
            }
            
            public NativeArray<T> ToArray<T>(AtomicSafetyManager* safetyManager) where T : struct
            {
                var nativeArray = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>((void*)Ptr, Size, Allocator.Invalid);
                safetyManager->MarkNativeArrayAsReadOnly(ref nativeArray);
                return nativeArray;
            }
        }

        struct BroadPhaseTreeProxy
        {
            NativeArrayProxy Nodes;
            NativeArrayProxy NodeFilters;
            NativeArrayProxy BodyFilters;
            NativeArrayProxy Ranges;
            NativeArrayProxy BranchCount;

            public bool IsCreated => Nodes.IsCreated && NodeFilters.IsCreated && BodyFilters.IsCreated && Ranges.IsCreated && BranchCount.IsCreated;

            public BroadPhaseTreeProxy(Broadphase.Tree tree)
            {
                Nodes = NativeArrayProxy.Create(tree.Nodes);
                NodeFilters = NativeArrayProxy.Create(tree.NodeFilters);
                BodyFilters = NativeArrayProxy.Create(tree.BodyFilters);
                Ranges = NativeArrayProxy.Create(tree.Ranges);
                BranchCount = NativeArrayProxy.Create(tree.BranchCount);
            }

            public Broadphase.Tree ToTree(AtomicSafetyManager* safetyManager)
            {
                return new Broadphase.Tree
                {
                    BranchCount = BranchCount.ToArray<int>(safetyManager),
                    Nodes = Nodes.ToArray<BoundingVolumeHierarchy.Node>(safetyManager),
                    NodeFilters = NodeFilters.ToArray<CollisionFilter>(safetyManager),
                    BodyFilters = BodyFilters.ToArray<CollisionFilter>(safetyManager),
                    Ranges = Ranges.ToArray<BoundingVolumeHierarchy.Builder.Range>(safetyManager)
                };
            }
        }

        NativeArrayProxy m_Bodies;
        BroadPhaseTreeProxy m_StaticTree;
        BroadPhaseTreeProxy m_DynamicTree;

        [NativeDisableUnsafePtrRestriction]
        readonly AtomicSafetyManager* m_SafetyManager;

        public bool IsCreated => m_Bodies.IsCreated && m_StaticTree.IsCreated && m_DynamicTree.IsCreated;

        internal CollisionWorldProxy(CollisionWorld collisionWorld, AtomicSafetyManager* safetyManager)
        {
            m_Bodies = NativeArrayProxy.Create(collisionWorld.Bodies);
            m_StaticTree = new BroadPhaseTreeProxy(collisionWorld.Broadphase.StaticTree);
            m_DynamicTree = new BroadPhaseTreeProxy(collisionWorld.Broadphase.DynamicTree);
            m_SafetyManager = safetyManager;
        }

        public CollisionWorld ToCollisionWorld() =>
            new CollisionWorld(m_Bodies.ToArray<RigidBody>(m_SafetyManager), new Broadphase(m_StaticTree.ToTree(m_SafetyManager), m_DynamicTree.ToTree(m_SafetyManager)));
    }
}
