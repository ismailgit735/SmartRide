# Architecture

The C++ runtime owns a directed immutable adjacency graph (forward and reverse
edge IDs), routing algorithms, spatial matching, worker execution and persistence.
The Python import stage delegates OSM decoding and coordinate resolution to
pyosmium/libosmium; it exports explicit nodes and weighted directed edges.
No Python is needed by the runtime executable.

SRG1 starts with `SRG1 node_count edge_count`, followed by x/y coordinate pairs
and from/to/weight edge records. Internal IDs are zero-based uint32; original
OSM IDs are used only during import. Parallel edges are retained, and paths use
edge IDs so their exact weights and direction can be validated. Graph loading
rejects negative/nonfinite weights, invalid endpoints and truncated input.

The baseline uses Dijkstra as a reference oracle. Unreachable distance is
infinity with an empty path. A zero-length source-to-self route is valid.
Driver simulation samples graph nodes with a recorded mt19937_64 seed.

## Planned routing invariants

A* uses a consistent scaled Euclidean lower bound. For bidirectional A*, use
balanced potentials p(v)=(h(v,t)-h(s,v))/2 and reverse potential -p(v).
Forward key is d(s,v)+p(v)-p(s); reverse key is d(v,t)-p(v)+p(t).
Stop when the sum of frontier keys reaches best_distance+p(t)-p(s).
The reverse search traverses incoming directed edges, not forward adjacency.

Contraction Hierarchies contract nodes, adding directed shortcuts only when a
bounded Dijkstra witness search cannot certify an equal or shorter alternative
avoiding the contracted node. Exhausting a witness work limit must add a shortcut,
never assume a witness. Query searches upward ranks forward and upward ranks
in the reversed graph. Store both child edge IDs for iterative unpacking into
original graph edges. Reference: [Geisberger's thesis](https://ae.iti.kit.edu/download/diploma_thesis_geisberger.pdf).
