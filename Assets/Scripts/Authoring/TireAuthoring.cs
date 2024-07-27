using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using AlphaTire.Components;

namespace AlphaTire.Authoring
{
    public class TireAuthoring : BaseJoint
    {
        public float3 AnchorPosition;
        public float TargetDistance;
        public float MaxImpulseAppliedByMotor = math.INFINITY;

        public float SpringFrequency = Constraint.DefaultSpringFrequency;
        public float SpringDampening = Constraint.DefaultDampingRatio;

        private float3 PositionInConnectedEntity;
        private float3 AxisInConnectedEntity;
        private float3 PerpendicularAxisInConnectedEntity;

        public class TireBaker : JointBaker<TireAuthoring>
        {
            public override void Bake(TireAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                AddComponent(entity, new TireData());

                var springDirection = math.up();
                var aFromB = math.mul(math.inverse(authoring.worldFromA), authoring.worldFromB);
                var springDirectionAxis = math.mul(aFromB.rot, springDirection);
                var bFromA = math.mul(math.inverse(authoring.worldFromB), authoring.worldFromA);

                authoring.PositionInConnectedEntity = math.transform(bFromA, authoring.AnchorPosition);
                authoring.AxisInConnectedEntity = springDirection;

                Math.CalculatePerpendicularNormalized(springDirectionAxis, out var perpendicularLocal, out _);
                authoring.PerpendicularAxisInConnectedEntity = math.mul(bFromA.rot, perpendicularLocal);

                var joint = PhysicsJoint.CreatePositionMotor(
                    new BodyFrame
                    {
                        Axis = springDirectionAxis,
                        PerpendicularAxis = perpendicularLocal,
                        Position = authoring.AnchorPosition
                    },
                    new BodyFrame
                    {
                        Axis = authoring.AxisInConnectedEntity,
                        PerpendicularAxis = authoring.PerpendicularAxisInConnectedEntity,
                        Position = authoring.PositionInConnectedEntity
                    },
                    authoring.TargetDistance,
                    authoring.MaxImpulseAppliedByMotor
                );

                var constrains = joint.GetConstraints();
                constrains[0] = Constraint.MotorPlanar(authoring.TargetDistance, math.abs(authoring.MaxImpulseAppliedByMotor), 
                    math.abs(authoring.SpringFrequency), math.abs(authoring.SpringDampening));

                joint.SetConstraints(constrains);
                joint.SetImpulseEventThresholdAllConstraints(authoring.MaxImpulse);

                var constraintBodyPair = GetConstrainedBodyPair(authoring);
                var worldIndex = GetWorldIndexFromBaseJoint(authoring);

                CreateJointEntity(worldIndex, constraintBodyPair, joint);
            }

        }
    }
}
