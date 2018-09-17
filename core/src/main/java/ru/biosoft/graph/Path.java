package ru.biosoft.graph;

import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * This class represents edge path specified by array of x and y coordinates and types for each point.
 */
@SuppressWarnings ("serial")
public class Path extends Polygon
{
    public static final int LINE_TYPE = 0;
    public static final int QUAD_TYPE = 1;
    public static final int CUBIC_TYPE = 2;

    public int pointTypes[];

    public Path()
    {
        super();
    }

    public Path(String fromString)
    {
        String[] values = fromString.split(";");
        int n = Integer.parseInt(values[0]);
        if( values.length >= ( 3 * n + 1 ) )
        {
            for( int i = 0; i < n; i++ )
            {
                addPoint(Integer.parseInt(values[3 * i + 1]), Integer.parseInt(values[3 * i + 2]), Integer
                        .parseInt(values[3 * i + 3]));
            }
        }
    }

    public Path(int xpoints[], int ypoints[], int npoints)
    {
        super(xpoints, ypoints, npoints);
        this.pointTypes = new int[npoints];
    }

    public Path(int xpoints[], int ypoints[], int pointTypes[], int npoints)
    {
        super(xpoints, ypoints, npoints);
        if( pointTypes != null )
        {
            int[] newPointTypes = new int[pointTypes.length];
            System.arraycopy( pointTypes, 0, newPointTypes, 0, npoints );
            this.pointTypes = newPointTypes;
        }
        else
            this.pointTypes = new int[npoints];
    }

    @Override
    public String toString()
    {
        StringBuffer sb = new StringBuffer();
        sb.append(npoints);
        for( int i = 0; i < npoints; i++ )
        {
            sb.append(";");
            sb.append(xpoints[i]);
            sb.append(";");
            sb.append(ypoints[i]);
            sb.append(";");
            sb.append(pointTypes[i]);
        }
        return sb.toString();
    }

    public void removePoint(int n)
    {
        if( n >= 0 && n < npoints )
        {
            for( int i = n; i < npoints - 1; i++ )
            {
                xpoints[i] = xpoints[i + 1];
                ypoints[i] = ypoints[i + 1];
                pointTypes[i] = pointTypes[i + 1];
            }

            npoints--;
        }
    }

    /**
     * Redefine addPoint function for correctly work with pointTypes array.
     */
    @Override
    public void addPoint(int x, int y)
    {
        super.addPoint(x, y);
        if( pointTypes == null )
        {
            pointTypes = new int[npoints];
        }
        else if( pointTypes.length < npoints )
        {
            int[] newPointTypes = new int[npoints];
            System.arraycopy(pointTypes, 0, newPointTypes, 0, pointTypes.length);
            pointTypes = newPointTypes;
        }
    }

    /**
     * addPoint function with type parameter.
     */
    public void addPoint(int x, int y, int type)
    {
        addPoint(x, y);
        pointTypes[npoints - 1] = type;
    }

    public void removeSelfIntersections()
    {
        if( npoints < 5 )
            return;

        ArrayList<Segment> segments = new ArrayList<>();
        for( int i = 1; i < npoints; i++ )
        {
            Segment s = new Segment();
            s.x1 = xpoints[i - 1];
            s.y1 = ypoints[i - 1];
            s.x2 = xpoints[i];
            s.y2 = ypoints[i];
            segments.add(s);
        }

        int j = 0;
        int k = 0;
        Point cross = null;
        for( j = 0; j < npoints - 1; j++ )
        {
            Segment s1 = segments.get(j);
            for( k = 0; k < j; k++ )
            {
                Segment s2 = segments.get(k);
                cross = getCross(s1, s2);
                if( cross != null )
                    break;
            }
            if( cross != null )
                break;
        }

        if( cross == null )
            return;

        for( int r = k + 1; r < j; ++r )
            removePoint(k + 1);

        xpoints[k + 1] = cross.x;
        ypoints[k + 1] = cross.y;

        // recursive call
        removeSelfIntersections();
    }

    private static Point getCross(Segment s1, Segment s2)
    {
        if( s1.isHorizontal() == s2.isHorizontal() )
            return null;

        if( s1.isHorizontal() )
        {
            int x2 = s2.x1; // ==s2.x2
            int y1 = s1.y1; // == s1.y2

            if( ( ( x2 - s1.x1 ) * ( x2 - s1.x2 ) >= 0 ) || ( y1 - s2.y1 ) * ( y1 - s2.y2 ) >= 0 )
                return null;

            return new Point(x2, y1);
        }
        else
        {
            int x1 = s1.x1; // ==s1.x2
            int y2 = s2.y1; // == s2.y2

            if( ( ( x1 - s2.x1 ) * ( x1 - s2.x2 ) >= 0 ) || ( y2 - s1.y1 ) * ( y2 - s1.y2 ) >= 0 )
                return null;

            return new Point(x1, y2);
        }
    }

    class Segment
    {
        int x1;
        int y1;
        int x2;
        int y2;

        public boolean isHorizontal()
        {
            return y1 == y2;
        }

        public Direction getDirection()
        {
            if( isHorizontal() )
            {
                if( x1 < x2 )
                    return Direction.RIGHT;
                else
                    return Direction.LEFT;
            }
            else
            {
                if( y1 < y2 )
                    return Direction.UP;
                else
                    return Direction.DOWN;
            }
        }
    }

    enum Direction
    {
        UP, DOWN, LEFT, RIGHT,
    }

    // Reverse - meaning step BEFORE U-Turn, Forward - meaning step AFTER U-Turn
    // Graph required to check if there's no intersections
    public void removeUTurn(Graph g)
    {
        if( npoints < 5 )
            return;

        ArrayList<Segment> segments = new ArrayList<>();
        for( int i = 1; i < npoints; i++ )
        {
            Segment s = new Segment();
            s.x1 = xpoints[i - 1];
            s.y1 = ypoints[i - 1];
            s.x2 = xpoints[i];
            s.y2 = ypoints[i];
            segments.add(s);
        }

        for( int j = 0; j < npoints - 4; j++ )
        {
            Segment s0 = segments.get(j);
            Segment s1 = segments.get(j + 1);
            Segment s2 = segments.get(j + 2);
            Segment s3 = segments.get(j + 3);

            Direction d0 = s0.getDirection();
            Direction d1 = s1.getDirection();
            Direction d2 = s2.getDirection();
            Direction d3 = s3.getDirection();

            // 1. Forward
            boolean cwForward = isClockwise(d0, d1) && isClockwise(d1, d2) && !isClockwise(d2, d3);
            boolean ccwForward = !isClockwise(d0, d1) && !isClockwise(d1, d2) && isClockwise(d2, d3);
            if( cwForward || ccwForward )
            {
                // Forward U-Turn detected
                // now check if last connection is covered by 1st segment

                if( !s0.isHorizontal() )
                {
                    if( !isCovered(ypoints[j + 3], ypoints[j], s0.getDirection() == Direction.UP) )
                        continue;

                    if( checkIntersections(g, xpoints[j], ypoints[j + 3], xpoints[j + 3], ypoints[j + 3]) )
                        continue;

                    removePoint(j + 1);
                    removePoint(j + 1);
                    xpoints[j + 1] = xpoints[j];

                    removeUTurn(g);
                }
                else
                {
                    if( !isCovered(xpoints[j + 3], xpoints[j], s0.getDirection() == Direction.RIGHT) )
                        continue;

                    if( checkIntersections(g, xpoints[j + 3], ypoints[j], xpoints[j + 3], ypoints[j + 3]) )
                        continue;

                    removePoint(j + 1);
                    removePoint(j + 1);
                    ypoints[j + 1] = ypoints[j];

                    removeUTurn(g);
                }
            }

            // 2. Reverse
            boolean cwReverse = !isClockwise(d0, d1) && isClockwise(d1, d2) && isClockwise(d2, d3);
            boolean ccwReverse = isClockwise(d0, d1) && !isClockwise(d1, d2) && !isClockwise(d2, d3);
            if( cwReverse || ccwReverse )
            {
                // Reverse U-Turn detected
                // now check if first connection is covered by last segment
                if( !s1.isHorizontal() )
                {
                    if( !isCovered(ypoints[j + 1], ypoints[j + 4], s1.getDirection() == Direction.UP) )
                        continue;

                    if( checkIntersections(g, xpoints[j + 1], ypoints[j + 1], xpoints[j + 4], ypoints[j + 1]) )
                        continue;

                    removePoint(j + 2);
                    removePoint(j + 2);
                    xpoints[j + 1] = xpoints[j + 2];

                    removeUTurn(g);
                }
                else
                {
                    if( !isCovered(xpoints[j + 1], xpoints[j + 4], s1.getDirection() == Direction.RIGHT) )
                        continue;

                    if( checkIntersections(g, xpoints[j + 1], ypoints[j + 1], xpoints[j + 1], ypoints[j + 4]) )
                        continue;

                    removePoint(j + 2);
                    removePoint(j + 2);
                    ypoints[j + 1] = ypoints[j + 2];

                    removeUTurn(g);
                }
            }
        }
    }

    private static boolean isCovered(int connection, int cover, boolean positiveDirection)
    {
        return connection != cover && ( ( connection - cover ) > 0 ) == positiveDirection;
    }

    /** Returns true if there will be an intersection with any node/edge. */
    public static boolean checkIntersections(Graph g, int x1, int y1, int x2, int y2)
    {
        int deltaX = 1;
        int deltaY = 1;

        if( x1 > x2 )
        {
            int x = x2;
            x2 = x1;
            x1 = x;
        }

        if( y1 > y2 )
        {
            int y = y2;
            y2 = y1;
            y1 = y;
        }

        x1 -= deltaX;
        x2 += deltaX;
        y1 -= deltaY;
        y2 += deltaY;

        Rectangle rect = new Rectangle(x1, y1, x2 - x1, y2 - y1);
        for( Node node : g.nodeList )
            if( rect.intersects(node.x, node.y, node.width, node.height) )
                return true;

        // TODO: Edge crossings
        return false;
    }

    private static boolean isClockwise(Direction from, Direction to)
    {
        // NO confluence considered !!!
        switch( from )
        {
            case UP:
                return ( to == Direction.RIGHT );
            case RIGHT:
                return ( to == Direction.DOWN );
            case DOWN:
                return ( to == Direction.LEFT );
            case LEFT:
                return ( to == Direction.UP );
            default:
                return false;
        }
    }

    /**
     * Concats two paths into one.
     *
     * @param from - first path.
     * @param to - second path.
     * @returns new path that is concatenation of two specified line paths.
     */
    public static Path concat(Path from, Path to)
    {
        Path result = new Path();

        for( int i = 0; i < from.npoints; i++ )
            result.addPoint(from.xpoints[i], from.ypoints[i], from.pointTypes[i]);

        if( from.xpoints[from.npoints - 1] != to.xpoints[0] || from.ypoints[from.npoints - 1] != to.ypoints[0] )
            result.addPoint(to.xpoints[0], to.ypoints[0], to.pointTypes[0]);

        for( int i = 1; i < to.npoints; i++ )
            result.addPoint(to.xpoints[i], to.ypoints[i], to.pointTypes[i]);

        return result;
    }

    /**
     * Returns angle for the last segment.
     *
     * Might be useful for arrow drawing.
     */
    public double getLastSegmentAngle()
    {
        if( npoints < 2 )
            return 0;
        int n = npoints;
        int x1 = xpoints[n - 2];
        int x2 = xpoints[n - 1];
        int y1 = ypoints[n - 2];
        int y2 = ypoints[n - 1];

        int dx = x2 - x1;
        int dy = y2 - y1;
        double l = Math.sqrt(dx * dx + dy * dy);
        double alpha = Math.asin(dy / l);
        if( dx < 0 )
            alpha = Math.PI - alpha;
        return alpha;
    }

    /**
     * Intersects line with rectangle.
     */
    public static Point intersect(Rectangle rect, Point point1, Point point2, int offset)
    {
        Point result = intersect(point1, point2, new Point(rect.x - offset, rect.y - offset), new Point(rect.x + rect.width + offset,
                rect.y - offset));
        if( result != null )
            return result;
        result = intersect(point1, point2, new Point(rect.x + rect.width + offset, rect.y - offset), new Point(
                rect.x + rect.width + offset, rect.y + rect.height + offset));
        if( result != null )
            return result;
        result = intersect(point1, point2, new Point(rect.x - offset, rect.y + rect.height + offset), new Point(rect.x + rect.width
                + offset, rect.y + rect.height + offset));
        if( result != null )
            return result;
        result = intersect(point1, point2, new Point(rect.x - offset, rect.y - offset), new Point(rect.x - offset, rect.y + rect.height
                + offset));
        return result;
    }

    /**
     * Intersects two lines.
     */
    private static Point intersect(Point line1Point1, Point line1Point2, Point line2Point1, Point line2Point2)
    {
        float q = ( line1Point1.y - line2Point1.y ) * ( line2Point2.x - line2Point1.x ) - ( line1Point1.x - line2Point1.x )
                * ( line2Point2.y - line2Point1.y );
        float d = ( line1Point2.x - line1Point1.x ) * ( line2Point2.y - line2Point1.y ) - ( line1Point2.y - line1Point1.y )
                * ( line2Point2.x - line2Point1.x );

        if( d == 0 ) // parallel lines so no intersection anywhere in space (in curved space, maybe, but not here in Euclidian space.)
        {
            return null;
        }

        float r = q / d;
        q = ( line1Point1.y - line2Point1.y ) * ( line1Point2.x - line1Point1.x ) - ( line1Point1.x - line2Point1.x )
                * ( line1Point2.y - line1Point1.y );
        float s = q / d;

        if( r < 0 || r > 1 || s < 0 || s > 1 )
        {
            return null;
        }

        Point result = new Point();
        result.x = line1Point1.x + (int) ( 0.5f + r * ( line1Point2.x - line1Point1.x ) );
        result.y = line1Point1.y + (int) ( 0.5f + r * ( line1Point2.y - line1Point1.y ) );
        return result;
    }

    /* (non-Javadoc)
     * @see java.lang.Object#hashCode()
     */
    @Override
    public int hashCode()
    {
        final int prime = 31;
        int result = 1;
        result = prime * result + Arrays.hashCode(pointTypes);
        return result;
    }

    /* (non-Javadoc)
     * @see java.lang.Object#equals(java.lang.Object)
     */
    @Override
    public boolean equals(Object obj)
    {
        if( this == obj )
            return true;
        if( obj == null )
            return false;
        if( getClass() != obj.getClass() )
            return false;
        Path other = (Path)obj;
        if( npoints != other.npoints )
            return false;
        for( int i = 0; i < npoints; i++ )
        {
            if( pointTypes[i] != other.pointTypes[i] || xpoints[i] != other.xpoints[i] || ypoints[i] != other.ypoints[i] )
                return false;
        }
        return true;
    }

    @Override
    public Path clone()
    {
        Path clone = new Path();
        clone.xpoints = new int[npoints];
        System.arraycopy( this.xpoints, 0, clone.xpoints, 0, npoints );
        clone.ypoints = new int[npoints];
        System.arraycopy( this.ypoints, 0, clone.ypoints, 0, npoints );
        clone.npoints = this.npoints;
        clone.pointTypes = new int[npoints];
        if( this.pointTypes != null )
            System.arraycopy( this.pointTypes, 0, clone.pointTypes, 0, npoints );
        return clone;
    }
}
