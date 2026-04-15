using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

namespace Pathfinding.Components
{
    public struct Pathfinder : IComponentData, IEnableableComponent
    {
        public float3 from, to;

        public float requiredMinDistanceSq;

        public int agentId;

        public int pathId;

        public PathQueryStatus pathStatus;
    }
}