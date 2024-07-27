using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Collections;
using AlphaTire.Components;

namespace AlphaTire.Systems
{
    [BurstCompile]
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(TireCollisionSystem))]
    //[UpdateAfter(typeof(VehicleFrictionSystem))]
    public partial struct VehicleForceApplySystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency.Complete();
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

            var ecb = new EntityCommandBuffer(Allocator.TempJob);

            foreach (var (forceAccumulationBuffer, vehicleEntity) in SystemAPI.Query<DynamicBuffer<ForceAccumulationBufferElement>>().WithEntityAccess())
            {
                var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(vehicleEntity);

                if (!IsRigidbodyIndexValid(rigidbodyIndex, physicsWorld))
                {
                    return;
                }

                for (int i = 0; i < forceAccumulationBuffer.Length; i++)
                {
                    physicsWorld.ApplyImpulse(rigidbodyIndex, forceAccumulationBuffer[i].force, forceAccumulationBuffer[i].point);
                }

                forceAccumulationBuffer.Clear();
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }

        [BurstCompile]
        private bool IsRigidbodyIndexValid(int rigidbodyIndex, PhysicsWorld physicsWorld)
        {
            return rigidbodyIndex > -1 && rigidbodyIndex < physicsWorld.NumDynamicBodies;
        }
    }
}
