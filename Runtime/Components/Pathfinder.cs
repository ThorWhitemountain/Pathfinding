using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

namespace Pathfinding.Components
{
    //total is 68 bytes, with 16 bytes alignment
    public struct Pathfinder : IComponentData, IEnableableComponent
    {
        [MarshalAs(UnmanagedType.U1)] public bool useFunnel; // 4 bytes

        // -1 if no query assigned
        public int queryIndex; // 4 bytes

        public float requiredMinDistanceSq; // 4 bytes
        public int agentId; // 4 bytes

        public PathQueryStatus pathStatus; //4  bytes

        // Internal state for the NavMeshQuery to track progress. (navmesh snapped positions of from, to)
        public float3 fromLocation; //12 bytes
        public float3 toLocation; //12 bytes

        public float3 from; //12 bytes //todo move everything else away, as this is the only "customer-facing" fields
        public float3 to; //12 bytes //todo move everything else away, as this is the only "customer-facing" fields

    }
}