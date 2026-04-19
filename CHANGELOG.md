# Change Log:

## 0.6.1

### Changed

- Uses a queue of indices in order to control access to the navMeshQueries needed for pathfinding.
  - Allowing the system to limit the number of pathfinding entities during any one frame, as well as preventing query reuse/overwriting
  - Default Limit is 16 at the moment.

### Fixed

- There can be more than 10 pathfinding entities at one time without throwing errors.

## 0.6.0

- All pointers and unsafe code blocks have been removed
- System structure has been massively simplified to simplify logic. 
  - Find, Process, Export systems are now handled in one job working directly on entities instead of pointers to entity data
- Memory usage reduced by ~1 MB due to no intermediary path result storing
- Option to toggle path processing (path funnel algorithm)
- Removed usage of singleton entities, leading to 2 fewer archetype chunks (32 KB of memory)
- Removed usage of third party nativeWorkQueue (BovineLabs) to instead just use nativeArrays
- This should also allow for finding paths longer than 512 nodes (untested)

## 0.5.3

### Removed

- PathfinderAspect has been removed preemptively since it's been marked as deprecated by Unity and will be deleting in the coming Entities update.

## 0.5.2

### Fixed

- DrawFoundPathSystem to only exist in Editor. (This makes builds possible again, as it was dependent on Editor only code)