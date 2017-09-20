package ru.biosoft.graph;

import java.awt.Point;

/**
 * This interface allows to specify constraints on edge start/end points location.
 */
public interface PortFinder
{
    /**
     * This method allows to take into account constraints on edge start/end points for the specified node.
     *
     * @param node - node for which constraints are applied.
     * @param edge - edge that will be started(ended) at this point.
     * @param x - desired point x coordinate
     * @param y - desired point y coordinate
     *
     * @returns point that is nearest to the specified location from which the edge can be started.
     */
    public Point findPort(Node node, Edge edge, int x, int y);

}


