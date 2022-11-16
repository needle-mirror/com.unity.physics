using System;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Extensions
{
    /// <summary>   Different ways to apply changes to a rigid body's velocity. </summary>
    public enum ForceMode
    {
        /// <summary>Apply a continuous force to the rigid body, using its mass.</summary>
        Force = 0,
        /// <summary>Apply a continuous acceleration to the rigid body, ignoring its mass.</summary>
        Impulse = 1,
        /// <summary>Apply an instant force impulse to the rigid body, using its mass.</summary>
        VelocityChange = 2,
        /// <summary>Apply an instant velocity change to the rigid body, ignoring its mass.</summary>
        Acceleration = 5
    }

    /// <summary>   Utility functions acting on physics components. </summary>
    public static class PhysicsComponentExtensions
    {
        /// <summary>
        /// Scale the mass of the body using provided scale.
        /// </summary>
        /// <remarks>
        /// Do not use this function to scale physics mass for simulation purposes, that is done automatically
        /// by physics systems. Use it if you need PhysicsMass in one of your functions if the body has a non-identity
        /// scale component, and do not write back this mass to the body's entity components.
        /// </remarks>
        ///
        /// <param name="pm">       The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="uniformScale"> The body's uniform scale. If this value is approximately 1.0, the function will
        /// early-out with no effect.</param>
        ///
        /// <returns>   A body's physics mass with respect to it's scale. </returns>
        public static PhysicsMass ApplyScale(in this PhysicsMass pm, float uniformScale)
        {
            PhysicsMass scaledBodyMass = pm;
            if (!Math.IsApproximatelyEqual(uniformScale, 1.0f))
            {
                scaledBodyMass.CenterOfMass *= uniformScale;

                float absScale = math.abs(uniformScale);
                scaledBodyMass.InverseInertia *= math.rcp(math.pow(absScale, 5));
                scaledBodyMass.InverseMass *= math.rcp(math.pow(absScale, 3));
                scaledBodyMass.AngularExpansionFactor *= absScale;
            }
            return scaledBodyMass;
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="ApplyScale(Unity.Physics.PhysicsMass,float)"/> instead.
        /// </summary>
        /// <param name="pm">       The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="scale"> The body's uniform scale. If this value is approximately 1.0, the function will
        /// early-out with no effect.</param>
        ///
        /// <returns>   A body's physics mass with respect to it's scale. </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static PhysicsMass ApplyScale(in this PhysicsMass pm, in Scale scale)
        {
            return pm.ApplyScale(scale.Value);
        }

#endif

        /// <summary>
        /// Get a body's effective mass in a given direction and from a particular point in world space.
        /// </summary>
        /// <remarks>
        /// Assumes that there is no scale.
        /// </remarks>
        ///
        /// <param name="bodyMass">         The body mass. </param>
        /// <param name="bodyPosition">     The body position. </param>
        /// <param name="bodyOrientation">  The body orientation. </param>
        /// <param name="impulse">          The impulse. </param>
        /// <param name="point">            The point. </param>
        ///
        /// <returns>   A body's effective mass with respect to the specified point and impulse. </returns>
        public static float GetEffectiveMass(in this PhysicsMass bodyMass, in float3 bodyPosition, in quaternion bodyOrientation, float3 impulse, float3 point) =>
            PhysicsWorldExtensions.GetEffectiveMassImpl(GetCenterOfMassWorldSpace(bodyMass, bodyPosition, bodyOrientation), bodyMass.InverseInertia, impulse, point);
#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetEffectiveMass(Unity.Physics.PhysicsMass,Unity.Mathematics.float3,Unity.Mathematics.quaternion,Unity.Mathematics.float3,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        /// <param name="bodyMass">         The body mass. </param>
        /// <param name="bodyPosition">     The body position. </param>
        /// <param name="bodyOrientation">  The body orientation. </param>
        /// <param name="impulse">          The impulse. </param>
        /// <param name="point">            The point. </param>
        ///
        /// <returns>   A body's effective mass with respect to the specified point and impulse. </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static float GetEffectiveMass(in this PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation, float3 impulse, float3 point) =>
            PhysicsWorldExtensions.GetEffectiveMassImpl(GetCenterOfMassWorldSpace(bodyMass, bodyPosition.Value, bodyOrientation.Value), bodyMass.InverseInertia, impulse, point);
#endif

        /// <summary>
        /// Get a body's effective mass in a given direction and from a particular point in world space.
        /// </summary>
        ///
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="bodyScale">        The body's uniform scale. </param>
        /// <param name="impulse">          An impulse in world space. </param>
        /// <param name="point">            A point in world space. </param>
        ///
        /// <returns>   A body's effective mass with respect to the specified point and impulse. </returns>
        public static float GetEffectiveMass(in this PhysicsMass bodyMass, in float3 bodyPosition, in quaternion bodyOrientation, in float bodyScale, float3 impulse, float3 point)
        {
            float3 com = GetCenterOfMassWorldSpace(bodyMass, bodyScale, bodyPosition, bodyOrientation);
            var scaledMass = bodyMass.ApplyScale(bodyScale);
            return PhysicsWorldExtensions.GetEffectiveMassImpl(com, scaledMass.InverseInertia, impulse, point);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetEffectiveMass(Unity.Physics.PhysicsMass,Unity.Mathematics.float3,Unity.Mathematics.quaternion,float,Unity.Mathematics.float3,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="bodyScale">        The body's uniform scale. </param>
        /// <param name="impulse">          An impulse in world space. </param>
        /// <param name="point">            A point in world space. </param>
        ///
        /// <returns>   A body's effective mass with respect to the specified point and impulse. </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static float GetEffectiveMass(in this PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation, in Scale bodyScale, float3 impulse, float3 point)
        {
            return bodyMass.GetEffectiveMass(bodyPosition.Value, bodyOrientation.Value, bodyScale.Value, impulse, point);
        }

#endif

        /// <summary>   Get the center of mass in world space. </summary>
        ///
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyScale">        The body's uniform scale. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        ///
        /// <returns>   The center of mass in world space. </returns>
        public static float3 GetCenterOfMassWorldSpace(in this PhysicsMass bodyMass, float bodyScale, in float3 bodyPosition, in quaternion bodyOrientation) =>
            math.rotate(bodyOrientation, bodyMass.CenterOfMass * bodyScale) + bodyPosition;
#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetCenterOfMassWorldSpace(Unity.Physics.PhysicsMass,float,Unity.Mathematics.float3,Unity.Mathematics.quaternion)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyScale">        The body's uniform scale. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        ///
        /// <returns>   The center of mass in world space. </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static float3 GetCenterOfMassWorldSpace(in this PhysicsMass bodyMass, in Scale bodyScale, in Translation bodyPosition, in Rotation bodyOrientation) =>
            bodyMass.GetCenterOfMassWorldSpace(bodyScale.Value, bodyPosition.Value, bodyOrientation.Value);
#endif

        /// <summary>
        /// Get the center of mass in world space. Assumes that there is no scale.
        /// </summary>
        /// <seealso cref="GetCenterOfMassWorldSpace(in Unity.Physics.PhysicsMass,float,in Unity.Mathematics.float3,in Unity.Mathematics.quaternion)"/>
        ///
        /// <param name="bodyMass">         The body mass. </param>
        /// <param name="bodyPosition">     The body position. </param>
        /// <param name="bodyOrientation">  The body orientation. </param>
        ///
        /// <returns>   The center of mass world space. </returns>
        public static float3 GetCenterOfMassWorldSpace(in this PhysicsMass bodyMass, in float3 bodyPosition, in quaternion bodyOrientation) =>
            bodyMass.GetCenterOfMassWorldSpace(1.0f, bodyPosition, bodyOrientation);
#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetCenterOfMassWorldSpace(Unity.Physics.PhysicsMass, in Unity.Mathematics.float3, Unity.Mathematics.quaternion)"/> instead.
        /// </summary>
        /// <seealso cref="GetCenterOfMassWorldSpace(in Unity.Physics.PhysicsMass,float,in Unity.Mathematics.float3,in Unity.Mathematics.quaternion)"/>
        ///
        /// <param name="bodyMass">         The body mass. </param>
        /// <param name="bodyPosition">     The body position. </param>
        /// <param name="bodyOrientation">  The body orientation. </param>
        ///
        /// <returns>   The center of mass world space. </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static float3 GetCenterOfMassWorldSpace(in this PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation) =>
            bodyMass.GetCenterOfMassWorldSpace(1.0f, bodyPosition.Value, bodyOrientation.Value);
#endif

        /// <summary>   Set the center of mass in world space. </summary>
        ///
        /// <param name="bodyMass">        [in,out] The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="com">              A position in world space for the new Center Of Mass. </param>
        public static void SetCenterOfMassWorldSpace(ref this PhysicsMass bodyMass, in float3 bodyPosition, in quaternion bodyOrientation, float3 com)
        {
            com -= bodyPosition;
            math.rotate(math.inverse(bodyOrientation), com);
            bodyMass.CenterOfMass = com;
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="SetCenterOfMassWorldSpace(ref Unity.Physics.PhysicsMass, Unity.Mathematics.float3, Unity.Mathematics.quaternion, Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyMass">        [in,out] The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="com">              A position in world space for the new Center Of Mass. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void SetCenterOfMassWorldSpace(ref this PhysicsMass bodyMass, in Translation bodyPosition,
            in Rotation bodyOrientation, float3 com)
        {
            bodyMass.SetCenterOfMassWorldSpace(bodyPosition.Value, bodyOrientation.Value, com);
        }

#endif

        /// <summary>   Get the linear velocity of a rigid body at a given point (in world space) </summary>
        ///
        /// <param name="bodyVelocity">     The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="point">            A reference position in world space. </param>
        ///
        /// <returns>   The linear velocity of a rigid body at a given point (in world space) </returns>
        public static float3 GetLinearVelocity(in this PhysicsVelocity bodyVelocity, PhysicsMass bodyMass, float3 bodyPosition, quaternion bodyOrientation, float3 point)
        {
            var worldFromEntity = new RigidTransform(bodyOrientation, bodyPosition);
            var worldFromMotion = math.mul(worldFromEntity, bodyMass.Transform);

            return PhysicsWorldExtensions.GetLinearVelocityImpl(worldFromMotion, bodyVelocity.Angular, bodyVelocity.Linear, point);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetLinearVelocity(Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Mathematics.float3,Unity.Mathematics.quaternion,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyVelocity">     The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="point">            A reference position in world space. </param>
        ///
        /// <returns>   The linear velocity of a rigid body at a given point (in world space) </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static float3 GetLinearVelocity(in this PhysicsVelocity bodyVelocity, PhysicsMass bodyMass, Translation bodyPosition, Rotation bodyOrientation, float3 point)
        {
            return bodyVelocity.GetLinearVelocity(bodyMass, bodyPosition.Value, bodyOrientation.Value, point);
        }

#endif

        /// <summary>   Get the world-space angular velocity of a rigid body. </summary>
        ///
        /// <param name="bodyVelocity">     The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        ///
        /// <returns>   The angular velocity of a rigid body in world space. </returns>
        public static float3 GetAngularVelocityWorldSpace(in this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in quaternion bodyOrientation)
        {
            quaternion worldFromMotion = math.mul(bodyOrientation, bodyMass.InertiaOrientation);
            return math.rotate(worldFromMotion, bodyVelocity.Angular);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetAngularVelocityWorldSpace(Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Mathematics.quaternion)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyVelocity">     The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        ///
        /// <returns>   The angular velocity of a rigid body in world space. </returns>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static float3 GetAngularVelocityWorldSpace(in this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in Rotation bodyOrientation)
        {
            return bodyVelocity.GetAngularVelocityWorldSpace(bodyMass, bodyOrientation.Value);
        }

#endif

        /// <summary>   Set the world-space angular velocity of a rigid body. </summary>
        ///
        /// <param name="bodyVelocity">    [in,out] The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="angularVelocity"> An angular velocity in world space specifying radians per
        /// second about each axis. </param>
        public static void SetAngularVelocityWorldSpace(ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in quaternion bodyOrientation, in float3 angularVelocity)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(bodyOrientation, bodyMass.InertiaOrientation);
            float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), angularVelocity);
            bodyVelocity.Angular = angularVelocityInertiaSpace;
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="SetAngularVelocityWorldSpace(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Mathematics.quaternion,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyVelocity">    [in,out] The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="angularVelocity"> An angular velocity in world space specifying radians per
        /// second about each axis. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void SetAngularVelocityWorldSpace(ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in Rotation bodyOrientation, in float3 angularVelocity)
        {
            bodyVelocity.SetAngularVelocityWorldSpace(bodyMass, bodyOrientation.Value, angularVelocity);
        }

#endif

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia
        /// properties, and scale.
        /// </summary>
        ///
        /// <param name="bodyMass">     The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyScale">    The body's uniform scale. </param>
        /// <param name="force">        The force to be applied to a body. </param>
        /// <param name="mode">         The method used to apply the force to its targets. </param>
        /// <param name="timestep">     The change in time from the current to the next frame. </param>
        /// <param name="impulse">     [out] A returned impulse proportional to the provided 'force' and
        /// based on the supplied 'mode'. </param>
        /// <param name="impulseMass"> [out] A returned PhysicsMass component to be passed to an Apply
        /// function. </param>
        public static void GetImpulseFromForce(in this PhysicsMass bodyMass, float bodyScale, in float3 force, in ForceMode mode, in float timestep, out float3 impulse, out PhysicsMass impulseMass)
        {
            var scaledBodyMass = bodyMass;
            if (!Math.IsApproximatelyEqual(bodyScale, 1.0f))
            {
                scaledBodyMass = scaledBodyMass.ApplyScale(bodyScale);
            }
            scaledBodyMass.GetImpulseFromForce(force, mode, timestep, out impulse, out impulseMass);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="GetImpulseFromForce(Unity.Physics.PhysicsMass,float,Unity.Mathematics.float3,Unity.Physics.Extensions.ForceMode,float, out Unity.Mathematics.float3, out Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyMass">     The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyScale">    The body's uniform scale. </param>
        /// <param name="force">        The force to be applied to a body. </param>
        /// <param name="mode">         The method used to apply the force to its targets. </param>
        /// <param name="timestep">     The change in time from the current to the next frame. </param>
        /// <param name="impulse">     [out] A returned impulse proportional to the provided 'force' and
        /// based on the supplied 'mode'. </param>
        /// <param name="impulseMass"> [out] A returned PhysicsMass component to be passed to an Apply
        /// function. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void GetImpulseFromForce(in this PhysicsMass bodyMass, in Scale bodyScale, in float3 force, in ForceMode mode, in float timestep, out float3 impulse, out PhysicsMass impulseMass)
        {
            bodyMass.GetImpulseFromForce(bodyScale.Value, force, mode, timestep, out impulse, out impulseMass);
        }

#endif

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia
        /// properties. Assumes that there is no scale.
        /// <see cref="GetImpulseFromForce(in PhysicsMass, float, in float3, in ForceMode, in float, out float3, out PhysicsMass)"/>
        /// </summary>
        ///
        /// <param name="bodyMass">     The body mass. </param>
        /// <param name="force">        The force. </param>
        /// <param name="mode">         The mode. </param>
        /// <param name="timestep">     The timestep. </param>
        /// <param name="impulse">      [out] The impulse. </param>
        /// <param name="impulseMass">  [out] The impulse mass. </param>
        public static void GetImpulseFromForce(in this PhysicsMass bodyMass, in float3 force, in ForceMode mode, in float timestep, out float3 impulse, out PhysicsMass impulseMass)
        {
            var unitMass = new PhysicsMass { InverseInertia = new float3(1.0f), InverseMass = 1.0f, Transform = bodyMass.Transform };
            switch (mode)
            {
                case ForceMode.Force:
                    // Add a continuous force to the rigidbody, using its mass.
                    impulseMass = bodyMass;
                    impulse = force * timestep;
                    break;
                case ForceMode.Acceleration:
                    // Add a continuous acceleration to the rigidbody, ignoring its mass.
                    impulseMass = unitMass;
                    impulse = force * timestep;
                    break;
                case ForceMode.Impulse:
                    // Add an instant force impulse to the rigidbody, using its mass.
                    impulseMass = bodyMass;
                    impulse = force;
                    break;
                case ForceMode.VelocityChange:
                    // Add an instant velocity change to the rigidbody, ignoring its mass.
                    impulseMass = unitMass;
                    impulse = force;
                    break;
                default:
                    impulseMass = bodyMass;
                    impulse = float3.zero;
                    break;
            }
        }

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia
        /// properties. Equivalent to UnityEngine.Rigidbody.AddExplosionForce.
        /// </summary>
        ///
        /// <param name="bodyVelocity">      [in,out] The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">             The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyCollider">         The body's <see cref="PhysicsCollider"/> component. </param>
        /// <param name="bodyPosition">         The body's world-space position. </param>
        /// <param name="bodyOrientation">      The body's world-space rotation. </param>
        /// <param name="bodyScale">            The body's world-space scale. </param>
        /// <param name="explosionForce">    The force of the explosion (which may be modified by
        /// distance). </param>
        /// <param name="explosionPosition"> The centre of the sphere within which the explosion has its
        /// effect. </param>
        /// <param name="explosionRadius">   The radius of the sphere within which the explosion has its
        /// effect. </param>
        /// <param name="timestep">          The change in time from the current to the next frame. </param>
        /// <param name="up">                A vector defining the up direction, generally a unit vector
        /// in the opposite direction to <see cref="PhysicsStep"/>.Gravity. </param>
        /// <param name="explosionFilter">   Filter determining whether an explosion should be applied to
        /// the body. </param>
        /// <param name="upwardsModifier">   (Optional) Adjustment to the apparent position of the
        /// explosion to make it seem to lift objects. </param>
        /// <param name="mode">              (Optional) The method used to apply the force to its targets. </param>
        public static void ApplyExplosionForce(
            ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in PhysicsCollider bodyCollider,
            in float3 bodyPosition, in quaternion bodyOrientation, in float bodyScale,
            float explosionForce, in float3 explosionPosition, in float explosionRadius,
            in float timestep, in float3 up, in CollisionFilter explosionFilter,
            in float upwardsModifier = 0, ForceMode mode = ForceMode.Force)
        {
            // var worldFromBody = new RigidTransform(bodyOrientation.Value, bodyPosition.Value);
            //  The explosion is modelled as a sphere with a certain centre position and radius in world
            //  space;
            //  normally, anything outside the sphere is not affected by the explosion and the force
            //  decreases
            //  in proportion to distance from the centre.
            //  However, if a value of zero is passed for the radius then the full force will be applied
            //  regardless of how far the centre is from the rigidbody.
            bool bExplosionProportionalToDistance = explosionRadius != 0.0f;

            Math.ScaledMTransform worldFromBody = new Math.ScaledMTransform(new RigidTransform(bodyOrientation, bodyPosition), bodyScale);
            var bodyFromWorld = Math.Inverse(worldFromBody);

            var pointDistanceInput = new PointDistanceInput()
            {
                Position = Math.Mul(bodyFromWorld, explosionPosition),
                MaxDistance = math.select(float.MaxValue, math.abs(bodyFromWorld.Scale) * explosionRadius, bExplosionProportionalToDistance),
                Filter = explosionFilter
            };

            // This function applies a force to the object at the point on the surface of the rigidbody
            // that is closest to explosionPosition. The force acts along the direction from
            // explosionPosition to the surface point on the rigidbody. If explosionPosition is inside the
            // rigidbody, or the rigidbody has no active colliders, then the center of mass is used instead
            // of the closest point on the surface.
            if (!bodyCollider.IsValid || !bodyCollider.Value.Value.CalculateDistance(pointDistanceInput, out DistanceHit closestHit))
            {
                // Return now if the collider is invalid or out of range.
                return;
            }

            // The magnitude of the force depends on the distance between explosionPosition and the point
            // where the force was applied. As the distance between explosionPosition and the force point
            // increases, the actual applied force decreases.
            if (bExplosionProportionalToDistance)
            {
                var closestHitFraction = closestHit.Distance / pointDistanceInput.MaxDistance;
                explosionForce *= 1.0f - closestHitFraction;
            }

            var closestHitPositionWorld = Math.Mul(worldFromBody, closestHit.Position);
            var forceDirection = math.normalizesafe(closestHitPositionWorld - explosionPosition);
            var force = explosionForce * forceDirection;

            // The vertical direction of the force can be modified using upwardsModifier. If this parameter
            // is greater than zero, the force is applied at the point on the surface of the Rigidbody that
            // is closest to explosionPosition but shifted along the y-axis by negative
            // upwardsModifier.Using this parameter, you can make the explosion appear to throw objects up
            // into the air, which can give a more dramatic effect rather than a simple outward force.
            // Force can be applied only to an active rigidbody.
            if (0.0f != upwardsModifier)
            {
                closestHitPositionWorld -= up * upwardsModifier;
            }

            bodyMass.GetImpulseFromForce(bodyScale, force, mode, timestep, out float3 impulse, out PhysicsMass impulseMass);

            // Use the option with default scale, since the mass is already scaled
            bodyVelocity.ApplyImpulse(impulseMass, bodyPosition, bodyOrientation, impulse, closestHitPositionWorld);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="ApplyExplosionForce(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Physics.PhysicsCollider,Unity.Mathematics.float3,Unity.Mathematics.quaternion,float,float,Unity.Mathematics.float3,float,float,Unity.Mathematics.float3,Unity.Physics.CollisionFilter,float,Unity.Physics.Extensions.ForceMode)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyVelocity">      [in,out] The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="bodyMass">             The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyCollider">         The body's <see cref="PhysicsCollider"/> component. </param>
        /// <param name="bodyPosition">         The body's world-space position. </param>
        /// <param name="bodyOrientation">      The body's world-space rotation. </param>
        /// <param name="bodyScale">            The body's world-space scale. </param>
        /// <param name="explosionForce">    The force of the explosion (which may be modified by
        /// distance). </param>
        /// <param name="explosionPosition"> The centre of the sphere within which the explosion has its
        /// effect. </param>
        /// <param name="explosionRadius">   The radius of the sphere within which the explosion has its
        /// effect. </param>
        /// <param name="timestep">          The change in time from the current to the next frame. </param>
        /// <param name="up">                A vector defining the up direction, generally a unit vector
        /// in the opposite direction to <see cref="PhysicsStep"/>.Gravity. </param>
        /// <param name="explosionFilter">   Filter determining whether an explosion should be applied to
        /// the body. </param>
        /// <param name="upwardsModifier">   (Optional) Adjustment to the apparent position of the
        /// explosion to make it seem to lift objects. </param>
        /// <param name="mode">              (Optional) The method used to apply the force to its targets. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void ApplyExplosionForce(
            ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in PhysicsCollider bodyCollider,
            in Translation bodyPosition, in Rotation bodyOrientation, in Scale bodyScale,
            float explosionForce, in float3 explosionPosition, in float explosionRadius,
            in float timestep, in float3 up, in CollisionFilter explosionFilter,
            in float upwardsModifier = 0, ForceMode mode = ForceMode.Force)
        {
            bodyVelocity.ApplyExplosionForce(bodyMass, bodyCollider, bodyPosition.Value, bodyOrientation.Value,
                bodyScale.Value,
                explosionForce, explosionPosition, explosionRadius, timestep, up, explosionFilter,
                upwardsModifier, mode);
        }

#endif

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia
        /// properties. Equivalent to UnityEngine.Rigidbody.AddExplosionForce. ExplosionFilter is set to
        /// CollisionFilter.Default Assumes that there is no scale.
        /// <see cref="ApplyExplosionForce(ref PhysicsVelocity, PhysicsMass, PhysicsCollider, float3, quaternion, float, float, float3, float, float, float3, CollisionFilter,float, ForceMode)"/>
        /// </summary>
        ///
        /// <param name="bodyVelocity">         [in,out] The body velocity. </param>
        /// <param name="bodyMass">             The body mass. </param>
        /// <param name="bodyCollider">         The body collider. </param>
        /// <param name="bodyPosition">         The body position. </param>
        /// <param name="bodyOrientation">      The body orientation. </param>
        /// <param name="explosionForce">       The explosion force. </param>
        /// <param name="explosionPosition">    The explosion position. </param>
        /// <param name="explosionRadius">      The explosion radius. </param>
        /// <param name="timestep">             The timestep. </param>
        /// <param name="up">                   The up. </param>
        /// <param name="upwardsModifier">      (Optional) The upwards modifier. </param>
        /// <param name="mode">                 (Optional) The mode. </param>
        public static void ApplyExplosionForce(
            ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in PhysicsCollider bodyCollider,
            in float3 bodyPosition, in quaternion bodyOrientation,
            float explosionForce, in float3 explosionPosition, in float explosionRadius,
            in float timestep, in float3 up,
            in float upwardsModifier = 0, ForceMode mode = ForceMode.Force)
        {
            bodyVelocity.ApplyExplosionForce(bodyMass, bodyCollider, bodyPosition, bodyOrientation, 1.0f, explosionForce,
                explosionPosition, explosionRadius, timestep, up, CollisionFilter.Default, upwardsModifier, mode);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="ApplyExplosionForce(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Physics.PhysicsCollider,Unity.Mathematics.float3,Unity.Mathematics.quaternion,float,Unity.Mathematics.float3,float,float,Unity.Mathematics.float3,float,Unity.Physics.Extensions.ForceMode)"/> instead.
        /// </summary>
        ///
        /// <param name="bodyVelocity">         [in,out] The body velocity. </param>
        /// <param name="bodyMass">             The body mass. </param>
        /// <param name="bodyCollider">         The body collider. </param>
        /// <param name="bodyPosition">         The body position. </param>
        /// <param name="bodyOrientation">      The body orientation. </param>
        /// <param name="explosionForce">       The explosion force. </param>
        /// <param name="explosionPosition">    The explosion position. </param>
        /// <param name="explosionRadius">      The explosion radius. </param>
        /// <param name="timestep">             The timestep. </param>
        /// <param name="up">                   The up. </param>
        /// <param name="upwardsModifier">      (Optional) The upwards modifier. </param>
        /// <param name="mode">                 (Optional) The mode. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void ApplyExplosionForce(
            ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in PhysicsCollider bodyCollider,
            in Translation bodyPosition, in Rotation bodyOrientation,
            float explosionForce, in float3 explosionPosition, in float explosionRadius,
            in float timestep, in float3 up,
            in float upwardsModifier = 0, ForceMode mode = ForceMode.Force)
        {
            bodyVelocity.ApplyExplosionForce(bodyMass, bodyCollider, bodyPosition.Value, bodyOrientation.Value, 1.0f, explosionForce,
                explosionPosition, explosionRadius, timestep, up, CollisionFilter.Default, upwardsModifier, mode);
        }

#endif

        /// <summary>   Applies the impulse. </summary>
        ///
        /// <param name="pv">       [in,out] The velocity. </param>
        /// <param name="pm">       The mass. </param>
        /// <param name="t">        The body position. </param>
        /// <param name="r">        The body rotation. </param>
        /// <param name="impulse">  The impulse. </param>
        /// <param name="point">    The point. </param>
        public static void ApplyImpulse(ref this PhysicsVelocity pv, in PhysicsMass pm, in float3 t, in quaternion r, in float3 impulse, in float3 point)
        {
            pv.ApplyImpulse(pm, t, r, 1.0f, impulse, point);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. use <see cref="ApplyImpulse(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Mathematics.float3,Unity.Mathematics.quaternion,Unity.Mathematics.float3,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="pv">       [in,out] The velocity. </param>
        /// <param name="pm">       The mass. </param>
        /// <param name="t">        The body position. </param>
        /// <param name="r">        The body rotation. </param>
        /// <param name="impulse">  The impulse. </param>
        /// <param name="point">    The point. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void ApplyImpulse(ref this PhysicsVelocity pv, in PhysicsMass pm, in Translation t, in Rotation r, in float3 impulse, in float3 point)
        {
            pv.ApplyImpulse(pm, t.Value, r.Value, 1.0f, impulse, point);
        }

#endif

        /// <summary>   Applies the impulse. </summary>
        ///
        /// <param name="pv">           [in,out] The velocity. </param>
        /// <param name="pm">           The mass. </param>
        /// <param name="t">            The body position. </param>
        /// <param name="r">            Tge body rotation. </param>
        /// <param name="bodyScale">    The body scale. </param>
        /// <param name="impulse">      The impulse. </param>
        /// <param name="point">        The point. </param>
        public static void ApplyImpulse(ref this PhysicsVelocity pv, in PhysicsMass pm, in float3 t, in quaternion r,
            float bodyScale, in float3 impulse, in float3 point)
        {
            var mass = pm.ApplyScale(bodyScale);
            // Linear
            pv.ApplyLinearImpulse(mass, impulse);

            // Angular
            {
                // Calculate point impulse
                var worldFromEntity = new RigidTransform(r, t);
                var worldFromMotion = math.mul(worldFromEntity, mass.Transform);
                float3 angularImpulseWorldSpace = math.cross(point - worldFromMotion.pos, impulse);
                float3 angularImpulseInertiaSpace = math.rotate(math.inverse(worldFromMotion.rot), angularImpulseWorldSpace);

                pv.ApplyAngularImpulse(mass, angularImpulseInertiaSpace);
            }
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="ApplyImpulse(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,Unity.Mathematics.float3,Unity.Mathematics.quaternion,float,Unity.Mathematics.float3,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="pv">           [in,out] The velocity. </param>
        /// <param name="pm">           The mass. </param>
        /// <param name="t">            The body position. </param>
        /// <param name="r">            Tge body rotation. </param>
        /// <param name="bodyScale">    The body scale. </param>
        /// <param name="impulse">      The impulse. </param>
        /// <param name="point">        The point. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void ApplyImpulse(ref this PhysicsVelocity pv, in PhysicsMass pm, in Translation t, in Rotation r,
            in Scale bodyScale, in float3 impulse, in float3 point)
        {
            pv.ApplyImpulse(pm, t.Value, r.Value, bodyScale.Value, impulse, point);
        }

#endif

        /// <summary>   Applies the linear impulse. </summary>
        ///
        /// <param name="velocityData"> [in,out] Information describing the velocity. </param>
        /// <param name="massData">     Information describing the mass. </param>
        /// <param name="impulse">      The impulse. </param>
        public static void ApplyLinearImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, in float3 impulse)
        {
            velocityData.Linear += impulse * massData.InverseMass;
        }

        /// <summary>   Applies the linear impulse. </summary>
        ///
        /// <param name="velocityData"> [in,out] Information describing the velocity. </param>
        /// <param name="massData">     Information describing the mass. </param>
        /// <param name="bodyScale">    The body scale. </param>
        /// <param name="impulse">      The impulse. </param>
        public static void ApplyLinearImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, float bodyScale, in float3 impulse)
        {
            var scaledMass = massData.ApplyScale(bodyScale);
            velocityData.ApplyLinearImpulse(scaledMass, impulse);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="ApplyLinearImpulse(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,float, Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="velocityData"> [in,out] Information describing the velocity. </param>
        /// <param name="massData">     Information describing the mass. </param>
        /// <param name="bodyScale">    The body scale. </param>
        /// <param name="impulse">      The impulse. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void ApplyLinearImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, in Scale bodyScale, in float3 impulse)
        {
            velocityData.ApplyLinearImpulse(massData, bodyScale.Value, impulse);
        }

#endif

        /// <summary>   Applies the angular impulse. </summary>
        ///
        /// <param name="velocityData"> [in,out] Information describing the velocity. </param>
        /// <param name="massData">     Information describing the mass. </param>
        /// <param name="impulse">      The impulse. </param>
        public static void ApplyAngularImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, in float3 impulse)
        {
            velocityData.Angular += impulse * massData.InverseInertia;
        }

        /// <summary>   Applies the angular impulse. </summary>
        ///
        /// <param name="velocityData"> [in,out] Information describing the velocity. </param>
        /// <param name="massData">     Information describing the mass. </param>
        /// <param name="bodyScale">    The body scale. </param>
        /// <param name="impulse">      The impulse. </param>
        public static void ApplyAngularImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, float bodyScale, in float3 impulse)
        {
            var scaledMass = massData.ApplyScale(bodyScale);
            velocityData.ApplyAngularImpulse(scaledMass, impulse);
        }

#if ENABLE_TRANSFORM_V1
        /// <summary>
        /// Obsolete. Use <see cref="ApplyAngularImpulse(ref Unity.Physics.PhysicsVelocity,Unity.Physics.PhysicsMass,float,Unity.Mathematics.float3)"/> instead.
        /// </summary>
        ///
        /// <param name="velocityData"> [in,out] Information describing the velocity. </param>
        /// <param name="massData">     Information describing the mass. </param>
        /// <param name="bodyScale">    The body scale. </param>
        /// <param name="impulse">      The impulse. </param>
        [Obsolete("This variant will be removed with Transform V1; use the V2-compatible version instead.", true)]
        public static void ApplyAngularImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, in Scale bodyScale, in float3 impulse)
        {
            velocityData.ApplyAngularImpulse(massData, bodyScale.Value, impulse);
        }

#endif

        /// <summary>
        /// Compute a future position and orientation for a dynamic rigid body based on its current
        /// trajectory, after a specified amount of time.
        /// </summary>
        ///
        /// <param name="physicsVelocity">  The body's <see cref="PhysicsVelocity"/> component. </param>
        /// <param name="physicsMass">      The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="timestep">         The change in time from the current to the next frame. </param>
        /// <param name="position">         [in,out] The future position of the body. </param>
        /// <param name="orientation">      [in,out] The future orientation of the body. </param>
        public static void Integrate(
            this in PhysicsVelocity physicsVelocity, in PhysicsMass physicsMass, float timestep,
            ref float3 position, ref quaternion orientation)
        {
            var angularVelocityWS =
                physicsVelocity.GetAngularVelocityWorldSpace(physicsMass, orientation);
            Integrator.IntegratePosition(ref position, physicsVelocity.Linear, timestep);
            Integrator.IntegrateOrientation(ref orientation, angularVelocityWS, timestep);
        }
    }
}
