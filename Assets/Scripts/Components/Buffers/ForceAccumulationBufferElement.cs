using Unity.Entities;
using Unity.Mathematics;

namespace AlphaTire.Components
{
    [InternalBufferCapacity(30)]
    public struct ForceAccumulationBufferElement : IBufferElementData
    {
        public float3 point;
        public float3 force;
    }
}
