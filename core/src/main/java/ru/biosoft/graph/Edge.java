package ru.biosoft.graph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Represents a directed connection between two nodes in a graph.
 *
 * Parallel edge concept: if there are several parallel edges, then one of them
 * will be marked as master and other as slaves. Master edge contains list of
 * slaves.
 */
public class Edge
{
    /**
     * Creates edge between specified nodes.
     *
     * @param from - edge start node.
     * @param to - edge end node
     */
    public Edge(Node from, Node to)
    {
        this.from = from;
        this.to = to;

        path = new Path();
        path.addPoint(from.x + from.width / 2, from.y + from.height / 2);
        path.addPoint(to.x + to.width / 2, to.y + to.height / 2);
    }

    /**
     * Create simple line path for the edge that connects two specified points.
     *
     * Note: verifies the points order.
     *
     * @param x1 - start point x coordinate
     * @param y1 - start point y coordinate
     * @param x2 - end point x coordinate
     * @param y2 - end point y coordinate
     */
    public void createPath(int x1, int y1, int x2, int y2)
    {
        path = new Path();

        if( x1 >= from.x && x1 <= from.x + from.width && y1 >= from.y && y1 <= from.y + from.height )
        {
            path.addPoint(x1, y1);
            path.addPoint(x2, y2);
        }
        else
        {
            path.addPoint(x2, y2);
            path.addPoint(x1, y1);
        }
    }

    /**
     * Reverses the edge direction and line path.
     */
    protected void reverseDirection()
    {
        Node t = from;
        from = to;
        to = t;
        reversed = !reversed;

        if( path != null )
        {
            Path reversedPath = new Path();
            for( int i = path.npoints - 1; i >= 0; i-- ){
                reversedPath.addPoint(path.xpoints[i], path.ypoints[i], path.pointTypes[i] );
            }

            path = reversedPath;
        }
    }

    /**
     * Creates text string for the edge.
     *
     * @returns text string: <code>from.Name -> toName</code>
     */
    @Override
    public String toString()
    {
        return from.getName() + "->" + to.getName();
    }

    // //////////////////////////////////////////////////////////////////////////
    // properties
    //

    /** Edge start node. */
    protected Node from;

    public Node getFrom()
    {
        return from;
    }

    /** Edge end node. */
    protected Node to;

    public Node getTo()
    {
        return to;
    }

    /** Path to connect edge nodes. */
    protected Path path;

    public Path getPath()
    {
        return path;
    }

    public void setPath(Path path)
    {
        this.path = path;
    }

    /** Indicates whether this edge is reversed. */
    protected boolean reversed = false;

    public boolean isReversed()
    {
        return reversed;
    }

    /** Arbitrary data that can be associated by GraphLayout with the edge. */
    Object data;

    /** Arbitrary data that can be associated by application with the edge. */
    public Object applicationData;


    // //////////////////////////////////////////////////////////////////////////
    // Attribute issues
    //

    protected HashMap<String, String> attributes;

    public boolean hasAttribute ( String key )
    {
        return attributes != null && attributes.containsKey( key );

    }

    /**
     * Finds value for the specified attribute key.
     *
     * @param key - attribute key.
     * @return pointer to value if it was found or null otherwise.
     */
    public String getAttribute ( String key )
    {
        if ( attributes != null )
            return attributes.get ( key );

        return null;
    }

    public void setAttribute ( String key, String value )
    {
        if ( attributes == null )
            attributes = new HashMap<>();

        attributes.put ( key, value );
    }

    // //////////////////////////////////////////////////////////////////////////
    // parallel edges processing issues.
    //

    /** Indicates whether it master edge. */
    public boolean master = true;

    /** List of parallel edges for the master edge. */
    public List<Edge> slaves;

    /**
     * Adds slave (parallel) edge to current master edge.
     *
     * @param slave -
     *            parallel edge to be added to master edge.
     */
    public void addSlave(Edge slave)
    {
        if( slaves == null )
            slaves = new ArrayList<>();

        slaves.add(slave);
    }

    /** Indicates whether this edge path should be preserved during layout. */
    public boolean fixed;
}
