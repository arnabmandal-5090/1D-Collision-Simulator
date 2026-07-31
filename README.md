# 1D-Collision-Simulator
An event-driven 1-D collision simulator written in C++. It uses a Min-Heap Priority Queue to predict and jump directly to collision events. By implementing a versioning system for O(1) lazy invalidation of stale states, the engine simulates K elastic collisions among N particles in strictly O(N + K log N) time.
