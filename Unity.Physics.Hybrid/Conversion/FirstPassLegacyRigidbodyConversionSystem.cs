using UnityEngine;

namespace Unity.Physics.Authoring
{
    public class FirstPassLegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach(
                (Rigidbody body) =>
                {
                    var entity = GetPrimaryEntity(body.gameObject);
                    DstEntityManager.AddOrSetComponent(entity, new PhysicsCollider());
                }
            );
        }
    }
}
