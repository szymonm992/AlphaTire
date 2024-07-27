using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Physics.Systems;
using AlphaTire.Components;

namespace AlphaTire.Systems
{
    [BurstCompile]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsSystemGroup))]
    public partial struct TireCollisionSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            var tireDataHandle = SystemAPI.GetComponentLookup<TireData>(false);
            var colliderLookupHandle = SystemAPI.GetComponentLookup<PhysicsCollider>(true);
            var entityCommandBuffer = new EntityCommandBuffer(Allocator.TempJob);

            var resetPropertiesJob = new ResetTireDataJob();
            state.Dependency = resetPropertiesJob.Schedule(state.Dependency);
            state.Dependency.Complete();

            var detectTireCollisionsJob = new TireCollisionEventJob
            {
                PhysicsWorld = physicsWorld,
                TireDataLookup = tireDataHandle,
                ColliderLookup = colliderLookupHandle,
                EntityCommandBuffer = entityCommandBuffer.AsParallelWriter()
            };

            state.Dependency = detectTireCollisionsJob.Schedule(SystemAPI.GetSingleton<SimulationSingleton>(), state.Dependency);
            state.Dependency.Complete();

            entityCommandBuffer.Playback(state.EntityManager);
            entityCommandBuffer.Dispose();
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        private partial struct ResetTireDataJob : IJobEntity
        {
            [BurstCompile]
            public void Execute(ref TireData tireData)
            {
                UpdateTireProperties(ref tireData, false, default, math.up(), math.forward());
            }
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        private struct TireCollisionEventJob : ICollisionEventsJob
        {
            [ReadOnly] public PhysicsWorld PhysicsWorld;
            [ReadOnly] public ComponentLookup<PhysicsCollider> ColliderLookup;

            public ComponentLookup<TireData> TireDataLookup;
            public EntityCommandBuffer.ParallelWriter EntityCommandBuffer;

            [BurstCompile]
            public void Execute(CollisionEvent collisionEvent)
            {
                var (firstEntity, secondEntity) = (collisionEvent.EntityA, collisionEvent.EntityB);
                var (isFirstEntityCollideable, isSecondEntityCollideable) = GetLookupInfo(ColliderLookup, firstEntity, secondEntity);
                var (isFirstEntityTire, isSecondEntityTire) = GetLookupInfo(TireDataLookup, firstEntity, secondEntity); 

                if (isFirstEntityTire || isSecondEntityTire)
                {
                    var tireEntity = isFirstEntityTire ? firstEntity : secondEntity;
                    var tireProperties = TireDataLookup[tireEntity];

                    var isTireGrounded = (isFirstEntityTire && isSecondEntityCollideable) || (isSecondEntityTire && isFirstEntityCollideable);
                    var contactNormal = isTireGrounded ? collisionEvent.Normal : math.up();
                    var contactSurfaceVector = isTireGrounded ? math.normalize(math.cross(contactNormal, math.right())) : math.forward();

                    if (isTireGrounded)
                    {
                        var collisionDetails = collisionEvent.CalculateDetails(ref PhysicsWorld);
                        UpdateTireProperties(ref tireProperties, true, collisionDetails.EstimatedContactPointPositions, contactNormal, contactSurfaceVector);
                    }
                    else
                    {
                        UpdateTireProperties(ref tireProperties, false, default, math.up(), math.forward());
                    }

                    TireDataLookup[tireEntity] = tireProperties;
                }
            }

            [BurstCompile]
            private readonly (bool hasFirstEntityComponent, bool hasSecondEntityComponent) GetLookupInfo<T>(ComponentLookup<T> DataComponentLookup, Entity firstEntity, Entity secondEntity)
                where T : unmanaged, IComponentData
            {
                return (DataComponentLookup.HasComponent(firstEntity), DataComponentLookup.HasComponent(secondEntity));
            }
        }

        [BurstCompile]
        public static void UpdateTireProperties(ref TireData tireData, bool hasContact, NativeArray<float3> contactPoints, float3 contactNormal, float3 contactSurfaceVector)
        {
            tireData.ContactSurfaceNormal = contactNormal;
            tireData.IsGrounded = hasContact;
            tireData.ContactSurfaceVector = contactSurfaceVector;
        }
    }
}
