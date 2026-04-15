using UnityEngine.Experimental.AI;

namespace Pathfinding.Data
{
    public unsafe struct PathQueryResult
    {
        public int pathLength;

        public NavMeshLocation* path;

        public PathQueryStatus status;
    }
}