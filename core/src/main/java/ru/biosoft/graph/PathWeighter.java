package ru.biosoft.graph;

import java.awt.*;

/**
 * Utilility methods to calculate path weight. Path with minimal weight is the best.
 * 
 * Path weight is calculated as following:
 * 
 * <pre>
 * weight = edgeIntersections * EdgeIntersectionPenelty + numberOfBends
 *         * BendPenalty + pathLength * LengthPenalty
 * </pre>
 */
public class PathWeighter
{

    /** Penalty for intersection with graph nodes. */
    public static final int nodeIntersectionPenalty = 100000;

    /** Penalty for intersection with other edge line path. */
    public static final int edgeIntersectionPenalty = 5000;

    /** Penalty for each bend on edge path. */
    int bendPenalty = 500;

    /** Penalty for confluence. */
    int confluencePenalty = 5;

    /** Penalty for line path length. */
    int lengthPenalty = 1;

    /**
     * Calculates weight for the specified line path.
     * 
     * To optimise speed, if in some step weight for this line path is greater
     * then bestWeight, the process is terminated and the calculated weight is returned.
     * 
     * @param graph - graph for which line path is estimated
     * @param path - line path
     * @param bestWeight - weight for best line path that is already known
     * 
     * @return weight for the specified line path.
     */
    public int calcPathWeight(Graph graph, Path path, int bestWeight)
    {
        int penalty = 0;

        // penalize bends
        penalty += bendPenalty * ( path.npoints - 2 );
        if( penalty >= bestWeight )
            return bestWeight + 1;

        // penalise length
        for( int i = 0; i < path.npoints - 1; i++ )
        {
            int dx = path.xpoints[i] - path.xpoints[i + 1];
            int dy = path.ypoints[i] - path.ypoints[i + 1];

            penalty += lengthPenalty * (int)Math.sqrt(dx * dx + dy * dy);
        }
        if( penalty >= bestWeight )
            return bestWeight + 1;

        // penalize intersections with other edges and nodes
        for( int i = 0; i < path.npoints - 1; i++ )
        {
            int xFrom = Math.min(path.xpoints[i], path.xpoints[i + 1]) - 1;
            int yFrom = Math.min(path.ypoints[i], path.ypoints[i + 1]) - 1;
            int width = Math.abs(path.xpoints[i] - path.xpoints[i + 1]) + 1;
            int height = Math.abs(path.ypoints[i] - path.ypoints[i + 1]) + 1;

            Rectangle rect = new Rectangle(xFrom, yFrom, width, height);

            for( Edge edge : graph.edgeList )
            {
                if( edge.path != null && edge.path != path )
                {
                    penalty += edgeIntersectionPenalty * calcIntersections(rect, edge.path);

                    if( penalty >= bestWeight )
                        return bestWeight + 1;
                }
            }
        }

        return penalty;
    }

    /**
     * Calculates weight for the specified line (fragment of line path).
     * 
     * The weight is calculated as following:
     * 
     * <pre>
     * weight = NodeIntersectionPenalty (if intersects any graph node)
     *             + edgeIntersections * EdgeIntersectionPenelty
     *             + pathLength * LengthPenalty 
     * </pre>
     * 
     * @param graph - graph for which line weight is estimated
     * @param x1 - line start point x coordinate
     * @param y1 - line start point y coordinate
     * @param x2 - line end point x coordinate
     * @param y2 - line end point y coordinate
     * @param maxWeight - if in some step current weight/penalty more then maxWeigt, the
     *            process is terminated and current weight is returned.
     * 
     * @return weight for the specified line path.
     */
    public int calcLineWeight(Graph graph, int x1, int y1, int x2, int y2, int maxWeight)
    {
        int penalty = 0;

        // penalise node crossings
        if( graph.getIntersectedNode(x1 - 1, y1 - 1, x2 + 1, y2 + 1) != null )
            penalty += nodeIntersectionPenalty;

        if( penalty > maxWeight )
            return penalty;

        // penalize line length
        int dx = x1 - x2;
        int dy = y1 - y2;
        penalty += lengthPenalty * (int)Math.sqrt(dx * dx + dy * dy);

        if( penalty > maxWeight )
            return penalty;

        // penalize edge crossings
        int x = Math.min(x1, x2) - 1;
        int y = Math.min(y1, y2) - 1;
        int width = Math.abs(x2 - x1) + 2;
        int height = Math.abs(y2 - y1) + 2;
        Rectangle rect = new Rectangle(x, y, width, height);

        for( Edge edge : graph.edgeList )
            if( edge.path != null )
            {
                penalty += edgeIntersectionPenalty * calcIntersections(rect, edge.path);
                if( penalty > maxWeight )
                    return maxWeight;
            }

        return penalty;
    }

    /**
     * Calculates number of intersections of the specified line path with the
     * specified rectangle.
     * 
     * @return number of intersection of line path with the specified rectangle.
     */
    protected int calcIntersections(Rectangle rect, Path path)
    {
        int intersections = 0;
        for( int i = 0; i < path.npoints - 1; i++ )
        {
            int xFrom = Math.min(path.xpoints[i], path.xpoints[i + 1]) - 1;
            int yFrom = Math.min(path.ypoints[i], path.ypoints[i + 1]) - 1;
            int width = Math.abs(path.xpoints[i] - path.xpoints[i + 1]) + 1;
            int height = Math.abs(path.ypoints[i] - path.ypoints[i + 1]) + 1;

            if( ( new Rectangle(xFrom, yFrom, width, height) ).intersects(rect) )
            {
                intersections++;

                // penalize confluences
                if( width == rect.width && rect.width == 1 )
                    intersections += confluencePenalty;
                if( height == rect.height && rect.height == 1 )
                    intersections += confluencePenalty;
            }
        }

        return intersections;
    }

}
