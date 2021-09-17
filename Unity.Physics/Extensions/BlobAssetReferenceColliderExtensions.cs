using System.Runtime.CompilerServices;
using Unity.Entities;

namespace Unity.Physics.Extensions
{
    public static class BlobAssetReferenceColliderExtension
    {
        /// <summary>
        /// Get cast reference to the Collider inside a BlobAssetReference container.
        /// </summary>
        /// <param name="col">The BlobAssetReference<Collider> instance that we're attempting to extract data from.</param>
        /// <returns>A reference to the Collider instance, cast to the specified type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref To As<To>(this BlobAssetReference<Collider> col)
            where To : unmanaged, ICollider
        {
            unsafe
            {
                SafetyChecks.CheckColliderTypeAndThrow<To>(col.Value.Type);
                return ref *(To*)col.GetUnsafePtr();
            }
        }

        /// <summary>
        /// Get cast pointer to the Collider inside a BlobAssetReference container.
        /// </summary>
        /// <param name="col">The BlobAssetReference<Collider> instance that we're attempting to extract data from.</param>
        /// <returns>A pointer to the Collider instance, cast to the specified type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static To* AsPtr<To>(this BlobAssetReference<Collider> col)
            where To : unmanaged, ICollider
        {
            SafetyChecks.CheckColliderTypeAndThrow<To>(col.Value.Type);
            return (To*)col.GetUnsafePtr();
        }

        //specialization for ease of use
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static Collider* AsPtr(this BlobAssetReference<Collider> col)
        {
            return (Collider*)col.GetUnsafePtr();
        }

        /// <summary>
        /// Get a PhysicsComponent instance containing this BlobAssetReference<Collider>
        /// </summary>
        /// <param name="col">The BlobAssetReference<Collider> instance that we're attempting to extract data from.</param>
        /// <returns>A PhysicsComponent instance.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PhysicsCollider AsComponent(this BlobAssetReference<Collider> col)
        {
            return new PhysicsCollider() { Value = col };
        }

        /// <summary>
        /// Set the Collider* property of a ColliderCastInput struct, avoiding the need for an unsafe block in developer code.
        /// </summary>
        /// <param name="input">The ColliderCastInput instance that needs the Collider* property set.</param>
        /// <param name="col">The BlobAssetReference<Collider> instance that we're attempting to extract data from.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetCollider(ref this ColliderCastInput input, BlobAssetReference<Collider> col)
        {
            unsafe
            {
                input.Collider = col.AsPtr();
            }
        }

        /// <summary>
        /// Set the Collider* property of a ColliderDistanceInput struct, avoiding the need for an unsafe block in developer code.
        /// </summary>
        /// <param name="input">The ColliderDistanceInput instance that needs the Collider* property set.</param>
        /// <param name="col">The BlobAssetReference<Collider> instance that we're attempting to extract data from.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetCollider(ref this ColliderDistanceInput input, BlobAssetReference<Collider> col)
        {
            unsafe
            {
                input.Collider = col.AsPtr();
            }
        }
    }
}
