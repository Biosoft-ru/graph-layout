package ru.biosoft.graph;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import ru.biosoft.graph.OrthogonalPathLayouter.Orientation;

/**
 * Utility class to generate some set of point from which
 * edge line path can be started for the specified node.
 * <p/>
 * 
 * Node 'face' consists from two sets of points:
 * <ol>
 * <li><c>points</c> - points located along node bounding rectangle;</li>
 * <li><c>startPoints</c> - points that are located on some distance from
 * node bounding rectangle. This distance is equals 2*gridX from
 * vertical rectangle sides and  2*gridY from horizontal sides.</li>
 * </ol>
 * 
 * <p>Generally, node face consists from 4 points:
 * one point for center of each side of rectangle.
 * However if center point is already occupied,
 * then two points will be included in the face that are located
 * before and after the occupied point.</p>
 * <p/>
 * 
 * Currently this class is used only by {@link OrthogonalPathLayouter}.
 */
public class Face
{
    /**
     * Array of points located along node bounding rectangle.
     */
    List<Point> points = new ArrayList<>();

    /**
     * Hashtable for points that are located on some distance from
     * node bounding rectangle. This distance is equals 2*gridX from
     * vertical rectangle sides and  2*gridY from horizontal sides.
     * <p/>
     * Key - point located along node bounding rectangle.
     * Value - distant point.
     */
    protected HashMap<Point, Point> startPoints = new HashMap<>();

    /**
     * List for occupied points.
     */
    protected List<Point> occupied = new ArrayList<>();

    /**
     * Edge for which node face is build.
     */
    protected Edge edge;

    /**
     * Number of points it the face.
     */
    int size()
    {
        return points.size();
    }


    private boolean oneEdgeToPoint = true;
    /**
     * Constructs the face for the specified node.
     *
     * @param graph - graph to which node belongs. It is used to reveal what points is already occupied.
     * @param node  - node for which face will be generated.
     * @param edge  - edge for which is generated, it is essential for {@see ru.biosoft.graph.Node.findPort} function.
     * @param gridX - distance for remote points.
     * @param gridY - distance for remote points.
     */
    public Face(Graph graph, Node node, Edge edge, int gridX, int gridY, boolean oneEdgeToPoint, Orientation orientation)
    {
        this.edge = edge;
        this.oneEdgeToPoint = oneEdgeToPoint;
        boolean edgeInput = edge.getFrom().equals(node);

        // generates already occupied ports
        for( Edge e : graph.getEdges(node) )
            if( e.getPath() != null && oneEdgeToPoint )
            {
                if( e.from == node )
                    occupied.add(new Point(e.getPath().xpoints[0], e.getPath().ypoints[0]));
                else
                    occupied.add(new Point(e.getPath().xpoints[e.getPath().npoints - 1], e.getPath().ypoints[e.getPath().npoints - 1]));
            }

        int x, y;

        //top point
        if( orientation == Orientation.NONE || ( edgeInput && orientation == Orientation.BOTTOM )
                || ( !edgeInput && orientation == Orientation.TOP ) )
        {
            x = ( node.x + node.width / 2 ) / gridX * gridX;
            y = node.y;
            checkPoint(node, new Point(x, y), false, gridX);
        }

        //right point
        if( orientation == Orientation.NONE || ( edgeInput && orientation == Orientation.RIGHT )
                || ( !edgeInput && orientation == Orientation.LEFT ) )
        {
            x = node.x + node.width;
            y = ( node.y + node.height / 2 ) / gridY * gridY;
            checkPoint(node, new Point(x, y), true, gridY);
        }

        //bottom point
        if( orientation == Orientation.NONE || ( edgeInput && orientation == Orientation.BOTTOM )
                || ( !edgeInput && orientation == Orientation.TOP ) )
        {
            x = ( node.x + node.width / 2 ) / gridX * gridX;
            y = node.y + node.height;
            checkPoint(node, new Point(x, y), false, gridX);
        }

        //left point
        if( orientation == Orientation.NONE || ( edgeInput && orientation == Orientation.LEFT )
                || ( !edgeInput && orientation == Orientation.RIGHT ) )
        {
            x = node.x;
            y = ( node.y + node.height / 2 ) / gridY * gridY;
            checkPoint(node, new Point(x, y), true, gridY);
        }
        //        if ( Logger.isDebugEnabled( log ) )
        //        {
        //            Logger.debug( log, "node face: " + node.getName() + ", n=" + points.size() );
        //            for ( Object point : points )
        //                Logger.debug( log, "   " + point );
        //        }
    }

    /**
     * Checks whether this point is occupied.
     * If occupied then method try to generate two points:
     * one from left(top), and one from right(bottom).
     *
     * @param node     - node for which face is build.
     * @param ip       - initial point located along node bounding rectangle
     * @param vertical - indicates whether initial point located along vertical edge of node bounding rectangle
     * @param step     - distance form the occupied point.
     */
    protected void checkPoint(Node node, Point ip, boolean vertical, int step)
    {
        Point p = node.findPort(ip.x, ip.y, edge);

        // define whether new point located along the same rectangle edge
        // as initial.
        //        if( !p.equals(ip) )
        //        {
        //            if( !vertical
        //                    && ! ( ip.x > node.x && ip.x < node.x + node.width && p.x > node.x && p.x < node.x + node.width && Math.abs(ip.y - p.y) < node.height - 1 ) )
        //                return;
        //
        //            if( vertical
        //                    && ! ( ip.y > node.y && ip.y < node.y + node.height && p.y > node.y && p.y < node.y + node.height && Math.abs(ip.x
        //                            - p.x) < node.width - 1 ) )
        //                return;
        //        }

        if( !isOccupied(p, Math.abs(step)) )
        {
            addPoint(node, p, vertical, step);
            return;
        }

        //        if ( Logger.isDebugEnabled( log ) )
        //            Logger.debug( log, "Face, point is occupied: " + p.x + ", " + p.y + "; node - " + node.x + ", " + node.y );

        // otherwise try to generate two points: one from left,
        // and one from right
        int from = 0;
        int to = 0;
        int center = 0;

        // define direction
        if( vertical )
        {
            from = node.y;
            to = node.y + node.height;
            center = p.y;
        }
        else
        {
            from = node.x;
            to = node.x + node.width;
            center = p.x;
        }

        checkInterval(node, p, center, from, vertical, -step);
        checkInterval(node, p, center, to, vertical, step);
    }

    /**
     * Try to find free point in the specified interval.
     *
     * @param node     - node for which face is build
     * @param p        - point located along node bounding rectangle
     * @param from     - interval start position
     * @param to       - interval end position
     * @param vertical - whether it is vertical side of rectangle
     * @param step     - distance from the occupied point.
     */
    protected void checkInterval(Node node, Point p, int from, int to, boolean vertical, int step)
    {
        //        if ( Logger.isDebugEnabled( log ) )
        //            Logger.debug( log, "  check interval: p=" + p + ", from=" + from + ", to"
        //                    + "vertical=" + vertical + ", step=" + step );

        p = new Point(p.x, p.y);

        int k = ( to - from ) / step;
        if( k < 0 )
            k = -k;

        for( int i = 0; i < k; i++ )
        {
            if( vertical )
                p.y += step;
            else
                p.x += step;

            Point pp = node.findPort(p.x, p.y, edge);
            if( !isOccupied(pp, Math.abs(step)) )
            {
                addPoint(node, pp, vertical, step);
                return;
            }
        }
    }

    protected void addPoint(Node node, Point p, boolean vertical, int grid)
    {
        grid = Math.abs(grid);

        // calculate distant point
        Point dp = new Point(p.x, p.y);
        if( vertical )
        {
            dp.x = dp.x / grid * grid;
            if( dp.x <= node.x )
                dp.x -= 2 * grid; // left
            else
                dp.x += 2 * grid; // right
        }
        else
        {
            dp.y = dp.y / grid * grid;
            if( dp.y <= node.y )
                dp.y -= 2 * grid; // top
            else
                dp.y += 2 * grid; // bottom
        }

        points.add(dp);
        startPoints.put(dp, p);
        occupied.add(p);
    }

    protected boolean isOccupied(Point point, int grid)
    {
        if (oneEdgeToPoint)
            return false;
        for( Point ocPoint : occupied )
        {
            if( Math.abs(point.x - ocPoint.x) < ( grid / 2 ) && Math.abs(point.y - ocPoint.y) < ( grid / 2 ) )
            {
                return true;
            }
        }
        return false;
    }
}
