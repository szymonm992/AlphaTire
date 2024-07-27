using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using AlphaTire.Components;
using UnityEngine;

namespace AlphaTire.Authoring
{
    public class TireAuthoring : BaseJoint
    {
        public float3 AnchorPosition;
        public float TargetDistance;

        public float MaxImpulseAppliedByMotor = math.INFINITY;
        [Range(0.0f, 1.0f)]
        public float SpringDampeningRatio = 0.7f;
        [Range(0.0f, 1.0f)]
        public float TauCosntraint = 0.6f;
        [Range(2f, 20f)]
        public int Iterations = 8;

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
                JacobianCalculateSpringFrequencyAndDamping(authoring.TauCosntraint, authoring.SpringDampeningRatio, math.rcp(60.0f), authoring.Iterations, out var SpringFrequency, out var DampingRatio);
                Debug.Log($"Calculated spring frequency: {SpringFrequency} damping: {DampingRatio}");

                var motorPlanarConstraint = new Constraint
                {
                    ConstrainedAxes = new bool3(true, false, false),
                    Type = ConstraintType.PositionMotor,
                    Min = 0.0f,
                    Max = 0.0f,
                    SpringFrequency = SpringFrequency,
                    DampingRatio = DampingRatio,
                    MaxImpulse = new float3(authoring.MaxImpulseAppliedByMotor, 0f, 0f), 
                    Target = new float3(authoring.TargetDistance, 0f, 0f),
                };

                constrains[0] = motorPlanarConstraint;
                joint.SetConstraints(constrains);
                joint.SetImpulseEventThresholdAllConstraints(authoring.MaxImpulse);

                var constraintBodyPair = GetConstrainedBodyPair(authoring);
                var worldIndex = GetWorldIndexFromBaseJoint(authoring);

                CreateJointEntity(worldIndex, constraintBodyPair, joint);
            }

            private void JacobianCalculateSpringFrequencyAndDamping(float constraintTau, float constraintDamping,
            float timeStep, int iterations, out float springFrequency, out float dampingRatio)
            {
                int n = iterations;
                float h = timeStep;
                float hh = h * h;
                float a = 1.0f - constraintDamping;
                float aSum = 1.0f;

                for (int i = 1; i < n; i++)
                {
                    aSum += math.pow(a, i);
                }

                float w = math.sqrt(constraintTau * aSum / math.pow(a, n)) / h;
                float ww = w * w;
                springFrequency = w / (2.0f * math.PI);
                dampingRatio = (math.pow(a, -n) - 1 - hh * ww) / (2.0f * h * w);
            }

        }
    }
}
