package ru.biosoft.graph;

/**
 * This is a general interface for algorithms that perform a layout process on a given layout graph.
 */
public interface Layouter
{
    /**
     * Assigns a new graph layout to the given graph.
     * @param graph - graph to be arranged.
     */
    public void doLayout(Graph graph, LayoutJobControl jobControl);

    /**
     * Assign a new layout to all edges of graph.
     * @param graph - graph.
     */
    public void layoutEdges(Graph graph, LayoutJobControl jobControl);

    /**
     * Assigns line path for the specified graph edge.
     * @see Path
     *
     * @param graph - graph that contains the edge.
     * @param edge - edge for which line path will be found.
     */
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl jobControl);

    /**
     * Returns meta information about graph layout algorithm.
     */
    public LayouterInfo getInfo();

    /**
     * Estimate number of operations to layout the graph.
     */
    public int estimate(Graph graph, int what);
}
