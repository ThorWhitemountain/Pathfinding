using System;
using Pathfinding.Components;
using Pathfinding.Data;
using Pathfinding.Utility;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace Pathfinding.Systems
{
    [UpdateInGroup(typeof(PathfindingSystemGroup))]
    [UpdateAfter(typeof(FindPathSystem))]
    public partial struct PathfinderSystem : ISystem
    {
        private unsafe struct PathQueryPtr
        {
            public PathQuery* pathQuery;
        }

        private const int QueryMaxCount = 100;
        private const int PathMaxSize = 512;
        private const int MaxPathQueueNodes = 4096;

        private NavMeshWorld _navMeshWorld;
        private NativeArray<NavMeshQuery> _queries;

        private NativeWorkQueue<PathQuery> _pathRequests; // only for alloc/dispose
        private NativeArray<PathQueryPtr> _inProgress;

        private NativeParallelHashMap<int, PathQueryResult> _pathResults; // only for alloc/dispose

        [BurstCompile]
        public unsafe void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PathResultsSingleton>();
            state.RequireForUpdate<PathRequestsSingleton>();
            _navMeshWorld = NavMeshWorld.GetDefaultWorld();

            _pathRequests = new NativeWorkQueue<PathQuery>(QueryMaxCount, Allocator.Persistent);
            _inProgress = new NativeArray<PathQueryPtr>(QueryMaxCount, Allocator.Persistent);

            _pathResults = new NativeParallelHashMap<int, PathQueryResult>(QueryMaxCount, Allocator.Persistent);

            for (int i = 0; i < QueryMaxCount; i++)
            {
                ref PathQuery entry = ref _pathRequests.ElementAt(i);
                entry.path = (NavMeshLocation*)UnsafeUtility.Malloc(
                    UnsafeUtility.SizeOf<NavMeshLocation>() * PathMaxSize, UnsafeUtility.AlignOf<NavMeshLocation>(),
                    Allocator.Persistent);
                entry.status = 0;

                _inProgress[i] = new PathQueryPtr
                {
                    pathQuery = null
                };
            }

            state.EntityManager.AddComponentData(state.SystemHandle, new PathRequestsSingleton
            {
                requests = _pathRequests
            });
            state.EntityManager.AddComponentData(state.SystemHandle, new PathResultsSingleton
            {
                results = _pathResults
            });

            int maxWorkers = JobsUtility.JobWorkerMaximumCount;
            _queries = new NativeArray<NavMeshQuery>(maxWorkers, Allocator.Persistent);

            for (int i = 0; i < maxWorkers; i++)
            {
                _queries[i] = new NavMeshQuery(_navMeshWorld, Allocator.Persistent, MaxPathQueueNodes);
            }

            SystemAPI.GetSingletonRW<PathResultsSingleton>();
        }

        [BurstCompile]
        public unsafe void OnDestroy(ref SystemState state)
        {
            for (int i = 0; i < QueryMaxCount; i++)
            {
                UnsafeUtility.Free(_pathRequests.ElementAt(i).path, Allocator.Persistent);
            }

            _pathRequests.Dispose();

            for (int i = 0; i < _queries.Length; i++)
            {
                _queries[i].Dispose();
            }

            _queries.Dispose();

            _pathResults.Dispose();
            _inProgress.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            PathRequestsSingleton requests = SystemAPI.GetSingleton<PathRequestsSingleton>();
            PathResultsSingleton results = SystemAPI.GetSingleton<PathResultsSingleton>();

            JobHandle clearHandle = new ClearResultsHashMap
            {
                results = results.results
            }.Schedule(state.Dependency);

            state.Dependency = new ProcessQueues
            {
                maxIterations = 4096,
                navMeshQueries = _queries,
                pathRequests = requests.requests.AsParallelReader(),
                inProgress = _inProgress,
                results = results.results.AsParallelWriter()
            }.Schedule(JobsUtility.JobWorkerCount, clearHandle);
        }

        [BurstCompile]
        private struct ClearResultsHashMap : IJob
        {
            public NativeParallelHashMap<int, PathQueryResult> results;

            public void Execute()
            {
                results.Clear();
            }
        }

        [BurstCompile]
        private unsafe struct ProcessQueues : IJobFor
        {
            public int maxIterations;

            [ReadOnly] [NativeDisableContainerSafetyRestriction]
            public NativeArray<NavMeshQuery> navMeshQueries;

            public NativeWorkQueue<PathQuery>.ParallelReader pathRequests;
            public NativeArray<PathQueryPtr> inProgress;

            public NativeParallelHashMap<int, PathQueryResult>.ParallelWriter results;

            public void Execute(int index)
            {
                int iterCount = maxIterations;
                NavMeshQuery navMeshQuery = navMeshQueries[index];

                PathQueryPtr* ptr = (PathQueryPtr*)inProgress.GetUnsafePtr();
                ref PathQueryPtr element = ref UnsafeUtility.AsRef<PathQueryPtr>(ptr + index);

                if (element.pathQuery != null)
                {
                    ref PathQuery previousQuery = ref *element.pathQuery;

                    if (!ProcessElement(ref previousQuery, ref navMeshQuery, ref iterCount))
                    {
                        // Didn't finish processing the work
                        return;
                    }
                }

                //ref var previousQuery = ref InProgress.ElementAt(index);

                if (pathRequests.TryGetNext(out PathQuery* pathQuery))
                {
                    if (!ProcessElement(ref *pathQuery, ref navMeshQuery, ref iterCount))
                    {
                        // Didn't finish processing the work, store it for later
                        inProgress[pathQuery->pathRequestId] = new PathQueryPtr { pathQuery = pathQuery };
                        return;
                    }
                }


                element.pathQuery = null;
            }

            // True if we should continue processing
            private bool ProcessElement(ref PathQuery query, ref NavMeshQuery navMeshQuery, ref int iterCount)
            {
                if (iterCount <= 0)
                {
                    return false;
                }

                bool result = Process(ref query, ref navMeshQuery, ref iterCount);
                if (!result)
                {
                    return false;
                }

                // Write our result
                //Debug.Log($"Write path results from id {query.PathRequestId} length {query.PathLength}");
                results.TryAdd(query.pathRequestId, new PathQueryResult
                {
                    status = query.status,
                    path = query.path,
                    pathLength = query.pathLength
                });
                //Check.Assume(added);
                return true;
            }

            private bool Process(ref PathQuery query, ref NavMeshQuery navMeshQuery, ref int iterCount)
            {
                // Handle query start.
                if (query.status == 0)
                {
                    query.status = navMeshQuery.BeginFindPath(query.from, query.to);
                }

                switch (query.status)
                {
                    // Handle query in progress.
                    case PathQueryStatus.InProgress:
                        query.status = navMeshQuery.UpdateFindPath(iterCount, out int iterationsPerformed);
                        iterCount -= iterationsPerformed;
                        return false;
                    // Check query complete
                    case PathQueryStatus.Success:
                    {
                        navMeshQuery.EndFindPath(out int pathLength);

                        NativeArray<PolygonId> polygons = new(pathLength, Allocator.Temp);
                        navMeshQuery.GetPathResult(polygons);

                        NativeArray<StraightPathFlags> straightPathFlags = new(PathMaxSize, Allocator.Temp);
                        NativeArray<float> vertexSide = new(PathMaxSize, Allocator.Temp);

                        int cornerCount = 0;

                        query.status = FindStraightPath(ref navMeshQuery,
                            query.from.position,
                            query.to.position,
                            pathLength,
                            polygons,
                            query.path,
                            ref straightPathFlags,
                            ref vertexSide,
                            ref cornerCount
                        );

                        query.pathLength = cornerCount;

                        return true;
                    }
                    // We've still finished, we just failed in our query
                    case PathQueryStatus.Failure:
                        //Debug.Log($"PathQueryStatus.Failure id {query.PathRequestId}");
                        return true;
                    default: return false;
                }
            }

            private static PathQueryStatus FindStraightPath(
                ref NavMeshQuery query,
                float3 startPos,
                float3 endPos,
                int pathSize,
                NativeArray<PolygonId> polygons,
                NavMeshLocation* straightPath,
                ref NativeArray<StraightPathFlags> straightPathFlags,
                ref NativeArray<float> vertexSide,
                ref int straightPathCount)
            {
                if (!query.IsValid(polygons[0]))
                {
                    //Debug.Log("Query Failure 1");
                    straightPath[0] = new NavMeshLocation(); // empty terminator
                    return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                }

                straightPath[0] = query.CreateLocation(startPos, polygons[0]);

                straightPathFlags[0] = StraightPathFlags.Start;

                int apexIndex = 0;
                int n = 1;
                if (pathSize > 1)
                {
                    Matrix4x4 startPolyWorldToLocal = query.PolygonWorldToLocalMatrix(polygons[0]);

                    float3 apex = startPolyWorldToLocal.MultiplyPoint(startPos);
                    float3 left = new(0, 0,
                        0); // Vector3.zero accesses a static readonly which does not work in burst yet
                    float3 right = new(0, 0, 0);
                    int leftIndex = -1;
                    int rightIndex = -1;

                    for (int i = 1; i <= pathSize; ++i)
                    {
                        Matrix4x4 polyWorldToLocal = query.PolygonWorldToLocalMatrix(polygons[apexIndex]);

                        float3 vl, vr;
                        if (i == pathSize)
                        {
                            vl = vr = polyWorldToLocal.MultiplyPoint(endPos);
                        }
                        else
                        {
                            bool success = query.GetPortalPoints(polygons[i - 1], polygons[i], out Vector3 vecvl,
                                out Vector3 vecvr);
                            if (!success)
                            {
                                //Debug.Log("Query Failure 2");
                                return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                            }

                            vl = polyWorldToLocal.MultiplyPoint(vecvl);
                            vr = polyWorldToLocal.MultiplyPoint(vecvr);
                        }

                        vl -= apex;
                        vr -= apex;

                        // Ensure left/right ordering
                        if (Perp2D(vl, vr) < 0)
                        {
                            Swap(ref vl, ref vr);
                        }

                        // Terminate funnel by turning
                        if (Perp2D(left, vr) < 0)
                        {
                            Matrix4x4 polyLocalToWorld = query.PolygonLocalToWorldMatrix(polygons[apexIndex]);
                            Vector3 termPos = polyLocalToWorld.MultiplyPoint(apex + left);

                            n = RetracePortals(ref query, apexIndex, leftIndex, n, termPos, polygons, straightPath,
                                ref straightPathFlags);
                            if (vertexSide.Length > 0)
                            {
                                vertexSide[n - 1] = -1;
                            }

                            //Debug.Log("LEFT");

                            if (n == PathMaxSize)
                            {
                                straightPathCount = n;
                                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                            }

                            apex = polyWorldToLocal.MultiplyPoint(termPos);
                            left = float3.zero;
                            right = float3.zero;
                            i = apexIndex = leftIndex;
                            continue;
                        }

                        if (Perp2D(right, vl) > 0)
                        {
                            Matrix4x4 polyLocalToWorld = query.PolygonLocalToWorldMatrix(polygons[apexIndex]);
                            Vector3 termPos = polyLocalToWorld.MultiplyPoint(apex + right);

                            n = RetracePortals(ref query, apexIndex, rightIndex, n, termPos, polygons, straightPath,
                                ref straightPathFlags);
                            if (vertexSide.Length > 0)
                            {
                                vertexSide[n - 1] = 1;
                            }

                            //Debug.Log("RIGHT");

                            if (n == PathMaxSize)
                            {
                                straightPathCount = n;
                                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                            }

                            apex = polyWorldToLocal.MultiplyPoint(termPos);
                            left = float3.zero;
                            right = float3.zero;
                            i = apexIndex = rightIndex;
                            continue;
                        }

                        // Narrow funnel
                        if (Perp2D(left, vl) >= 0)
                        {
                            left = vl;
                            leftIndex = i;
                        }

                        if (Perp2D(right, vr) <= 0)
                        {
                            right = vr;
                            rightIndex = i;
                        }
                    }
                }

                // Remove the the next to last if duplicate point - e.g. start and end positions are the same
                // (in which case we have get a single point)
                if (n > 0 && math.all((float3)straightPath[n - 1].position == endPos))
                {
                    n--;
                }

                n = RetracePortals(ref query, apexIndex, pathSize - 1, n, endPos, polygons, straightPath,
                    ref straightPathFlags);
                if (vertexSide.Length > 0)
                {
                    vertexSide[n - 1] = 0;
                }

                if (n == PathMaxSize)
                {
                    straightPathCount = n;
                    return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                }

                // Fix flag for final path point
                straightPathFlags[n - 1] = StraightPathFlags.End;

                straightPathCount = n;
                return PathQueryStatus.Success;
            }

            private static int RetracePortals(
                ref NavMeshQuery query,
                int startIndex,
                int endIndex,
                int n,
                float3 termPos,
                NativeArray<PolygonId> path,
                NavMeshLocation* straightPath,
                ref NativeArray<StraightPathFlags> straightPathFlags)
            {
                for (int k = startIndex; k < (endIndex - 1); ++k)
                {
                    NavMeshPolyTypes type1 = query.GetPolygonType(path[k]);
                    NavMeshPolyTypes type2 = query.GetPolygonType(path[k + 1]);

                    if (type1 != type2)
                    {
                        query.GetPortalPoints(path[k], path[k + 1], out Vector3 vecl, out Vector3 vecr);

                        SegmentSegmentCpa(out float3 cpa1, out float3 _, vecl, vecr, straightPath[n - 1].position, termPos);
                        straightPath[n] = query.CreateLocation(cpa1, path[k + 1]);

                        straightPathFlags[n] = type2 == NavMeshPolyTypes.OffMeshConnection
                            ? StraightPathFlags.OffMeshConnection
                            : 0;
                        if (++n == PathMaxSize)
                        {
                            return PathMaxSize;
                        }
                    }
                }

                straightPath[n] = query.CreateLocation(termPos, path[endIndex]);
                straightPathFlags[n] = query.GetPolygonType(path[endIndex]) == NavMeshPolyTypes.OffMeshConnection
                    ? StraightPathFlags.OffMeshConnection
                    : 0;
                return ++n;
            }

            private static float Perp2D(float3 u, float3 v)
            {
                return (u.z * v.x) - (u.x * v.z);
            }

            private static void Swap(ref float3 a, ref float3 b)
            {
                (a, b) = (b, a);
            }

            private static void SegmentSegmentCpa(out float3 c0, out float3 c1, float3 p0, float3 p1, float3 q0,
                float3 q1)
            {
                float3 u = p1 - p0;
                float3 v = q1 - q0;
                float3 w0 = p0 - q0;

                float a = math.dot(u, u);
                float b = math.dot(u, v);
                float c = math.dot(v, v);
                float d = math.dot(u, w0);
                float e = math.dot(v, w0);

                float den = (a * c) - (b * b);
                float sc, tc;

                if (den == 0)
                {
                    sc = 0;
                    tc = d / b;

                    // todo: handle b = 0 (=> a and/or c is 0)
                }
                else
                {
                    sc = ((b * e) - (c * d)) / ((a * c) - (b * b));
                    tc = ((a * e) - (b * d)) / ((a * c) - (b * b));
                }

                c0 = math.lerp(p0, p1, sc);
                c1 = math.lerp(q0, q1, tc);
            }
        }

        [Flags]
        private enum StraightPathFlags
        {
            Start = 0x01, // The vertex is the start position.
            End = 0x02, // The vertex is the end position.
            OffMeshConnection = 0x04 // The vertex is start of an off-mesh link.
        }
    }
}