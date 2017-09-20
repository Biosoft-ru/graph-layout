package ru.biosoft.graph;

import java.awt.*;

/**
 * Creates simple line paths for all graph edges.
 * 
 * The main idea is to create line connecting centers of two nodes
 * and then remove line segments that are covered by node bounding rectangles.
 **/
public class DiagonalPathLayouter extends AbstractLayouter
{
    /**
    * Maximum distance between parallel edges.
    * Default value is 10.
    **/
    public int lineDistance = 10;

    /**
     * Layouter to make paths for self loops.
     */
    public SelfLoopLayouter selfLoopLayouter = new SelfLoopLayouter();

    /**
     * Assigns line path for the specified graph edge.
     * 
     * @param graph - graph that contains the edge
     * @param edge - edge for which line path will be found
     */
    public void layoutPath(Graph graph, Edge edge, PathWeighter pathWeighter)
    {
        if( edge.getFrom() == edge.getTo() )
        {
            selfLoopLayouter.layoutPath(graph, edge, pathWeighter);
            return;
        }

        if( !edge.master )
            edge = graph.getEdge(edge.from, edge.to);

        if( edge.slaves == null || edge.slaves.size() == 0 )
        {
            makePath(edge, 0, 0);
            return;
        }
        // calc the span between edges
        int n = 1 + edge.slaves.size();

        int dx = edge.getFrom().x + edge.getFrom().width / 2 - edge.getTo().x - edge.getTo().width / 2;
        int dy = edge.getFrom().y + edge.getFrom().height / 2 - edge.getTo().y - edge.getTo().height / 2;
        int len = (int)Math.sqrt(dx * dx + dy * dy);

        if( len == 0 )
            len = 1;

        int sx = 0;
        if( dy != 0 )
            sx = dy / Math.abs(dy) * Math.min(Math.abs(lineDistance * dy / len), Math.min(edge.getFrom().width, edge.getTo().width) / n);
        int sy = 0;
        if( dx != 0 )
            sy = -dx / Math.abs(dx) * Math.min(Math.abs(lineDistance * dx / len), Math.min(edge.getFrom().height, edge.getTo().height) / n);

        for( int i = 0; i < n; i++ )
        {
            dx = (int) ( ( i - n / 2.0 ) * sx );
            dy = (int) ( ( i - n / 2.0 ) * sy );
            if( n / 2 * 2 == n )
            {
                dx += sx / 2;
                dy += sy / 2;
            }

            if( i == 0 )
                makePath(edge, dx, dy);
            else
                makePath(edge.slaves.get(i - 1), dx, dy);
        }
    }


    /**
     * Makes line path for the specified edge.
     * 
     * @param edge - edge for which line path should be created
     * @param dx - shift by x axis for parallel edges
     * @param dy - shift by x axis for parallel edges
    **/
    protected void makePath(Edge edge, int dx, int dy)
    {
        edge.path = new Path();

        int x1 = dx + edge.getFrom().x + edge.getFrom().width / 2;
        int x2 = dx + edge.getTo().x + edge.getTo().width / 2;
        int y1 = dy + edge.getFrom().y + edge.getFrom().height / 2;
        int y2 = dy + edge.getTo().y + edge.getTo().height / 2;

        Point from = intersection(x1, y1, x2, y2, edge.getFrom().getBounds());
        Point to = intersection(x2, y2, x1, y1, edge.getTo().getBounds());

        //checking nearest available point
        Point portFrom = edge.getFrom().findPort(from.x, from.y, edge);
        Point portTo = edge.getTo().findPort(to.x, to.y, edge);

        edge.path.addPoint(portFrom.x, portFrom.y);
        edge.path.addPoint(portTo.x, portTo.y);
    }

    /**
     * Calculates intersection point between the specified line and rectangle.
     * 
     * It is suggested that (x1, y1) point is located inside the rectangle and
     * (x2, y2) is outside the rectangle.
     */
    protected Point intersection(int x1, int y1, int x2, int y2, Rectangle rect)
    {
        int ix = 0;
        int iy = 0;

        int dx = x2 - x1;
        if( dx == 0 )
            dx = 1;

        int dy = y2 - y1;
        if( dy == 0 )
            dy = 1;

        if( x2 >= rect.x && x2 <= rect.x + rect.width )
        {
            if( y2 < rect.y )
                iy = rect.y;
            else
                iy = rect.y + rect.height;

            ix = x1 + ( x2 - x1 ) * ( iy - y1 ) / dy;
        }
        else
        {
            if( x2 < rect.x )
                ix = rect.x;
            else
                ix = rect.x + rect.width;

            iy = y1 + ( y2 - y1 ) * ( ix - x1 ) / dx;

            if( iy < rect.y )
            {
                iy = rect.y;
                ix = x1 + ( x2 - x1 ) * ( iy - y1 ) / dy;
            }
            else if( iy > rect.y + rect.height )
            {
                iy = rect.y + rect.height;
                ix = y1 + ( y2 - y1 ) * ( iy - y1 ) / dy;
            }

            if( iy == rect.y || iy == rect.y + rect.height )
                ix = x1 + ( x2 - x1 ) * ( iy - y1 ) / dy;
        }

        return new Point(ix, iy);
    }


    @Override
    public void layoutEdges(Graph graph, LayoutJobControl jobControl)
    {
        for( Edge edge : graph.edgeList )
        {
            if( !edge.fixed )
                layoutPath(graph, edge, jobControl);
        }
    }


    /**
     * Do nothing because it layouts edges only.
     */
    @Override
    void layoutNodes(Graph graph, LayoutJobControl jobControl)
    {
    }


    @Override
    public int estimate(Graph graph, int what)
    {
        // TODO Auto-generated method stub
        return 0;
    }


    @Override
    public LayouterInfo getInfo()
    {
        return new LayouterInfoSupport(true, false, false, false, false, false);
    }


    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl jobControl)
    {
        layoutPath(graph, edge, pathWeighter);
    }
}