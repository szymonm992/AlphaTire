using Unity.Entities;
using Unity.Mathematics;

namespace AlphaTire.Components
{
    public struct TireData : IComponentData
    {
        public bool IsGrounded;

        public float3 ContactSurfaceNormal;
        public float3 ContactSurfaceVector;
    }
}
