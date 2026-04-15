using UnityEngine.Experimental.AI;

namespace Pathfinding.Data
{
    public unsafe struct PathQuery
    {
        public int pathRequestId;

        // result data
        public int pathLength;

        //public int id;
        //public int key;
        public int areaMask;

        public NavMeshLocation from;
        public NavMeshLocation to;


        // result data
        public NavMeshLocation* path;
        public PathQueryStatus status;
    }
}