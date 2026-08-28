# graph
Graph - definition, layout algorithms, viewer

[![Build Status](https://travis-ci.org/Biosoft-ru/graph-layout.svg?branch=master)](https://travis-ci.org/Biosoft-ru/graph-layout) 
[![Coverage Status](https://coveralls.io/repos/github/Biosoft-ru/graph-layout/badge.svg?branch=master)](https://coveralls.io/github/Biosoft-ru/graph-layout?branch=master) 

## Maven

```xml
<dependency>
    <groupId>ru.biosoft.graph</groupId>
    <artifactId>core</artifactId>
    <version>2.0.0</version>
</dependency>
```

## Gradle

```groovy
implementation group: 'ru.biosoft.graph', name: 'core', version: '2.0.0'
```

## Performance note: `HierarchicLayouter.applyMinNodes`

`HierarchicLayouter.applyMinNodes()` (marked `// TODO: optimize` in the source) is a
known CPU hotspot. It drives its traversal with `nodeSet.iterator().next()` on an
`IdentityHashMap`-backed set, which rescans the whole backing table on every iteration
(O(table) per step, ~O(n²) overall on large graphs). Profiling on a production server
shows it accounts for the majority of the method's cost (~90% of samples).

A work-queue rewrite (FIFO `ArrayDeque` + a set marking each node scheduled exactly once)
removes the scan and is a clean, correct optimization — but it is **not output-equivalent**.
`applyMinNodes` is one pass inside an iterative optimizer
(`applyMedianPositions` / `applyMinEdges` / `applyMinNodes` / `applyMinPath`, repeated for
`maxNodeOptimizationNum` rounds), so changing the traversal order steers the search to a
*different local optimum*: some graphs lay out slightly better, others worse. Measured on
the standard test graphs (hoisting on, `layerDeltaY = 50`), the `simple` graph regressed
from 36 to 55 edge crossings.

Because the scan's iteration order **is** the node-selection order, there is no
order-preserving change that removes the cost — the performance and the exact-layout
compatibility goals are mutually exclusive for this method. The change was therefore
reverted (see PR Biosoft-ru/graph-layout#5, closed). Revisit only if the layout
differences are judged acceptable by the consumers of the layout (e.g. BioUML /
MegaWorkflowBuilder).
