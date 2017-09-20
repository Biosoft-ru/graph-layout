package ru.biosoft.graph;

import java.awt.Point;
import java.awt.Rectangle;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * General definition of graph node.
 */
public class Node implements Cloneable
{
    /**
     * Constructs graph node with the specified name.
     *
     * @param name - node name.
     */
    public Node(String name)
    {
        this( name, 0, 0, 4, 4, false );
    }

    /**
     * Constructs graph node with the specified name, location and size.
     *
     * @param name - node name
     * @param x - node x coordinate
     * @param y - node y coordinate
     * @param width - node width
     * @param height - node height
     */
    public Node(String name, int x, int y, int width, int height)
    {
        this( name, x, y, width, height, false );
    }

    /**
     * Constructs graph node with the specified name, location and size.
     *
     * @param name - node name
     * @param x - node x coordinate
     * @param y - node y coordinate
     * @param width - node width
     * @param height - node height
     * @param fixed - specifies whether this node is fixed.
     */
    public Node(String name, int x, int y, int width, int height, boolean fixed)
    {
        this.name = name;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.fixed = fixed;
    }

    /**
     * Creates text string for the node.
     *
     * @return text string with node name.
     */
    @Override
    public String toString()
    {
        return "N:" + name;
    }

    // //////////////////////////////////////////////////////////////////////////
    // properties
    //

    /** Node name property. */
    protected String name;
    public String getName()
    {
        return name;
    }

    /** Node x coordinate. */
    public int x;

    /** Node y coordinate. */
    public int y;

    /** Node width. */
    public int width;

    /** Node height. */
    public int height;

    /** Indicates whether this node location should be preserved during layout. */
    public boolean fixed;

    /** Arbitrary data that can be associated by GraphLayout with the node. */
    Object data;

    /** Arbitrary data that can be associated by application with the node. */
    public Object applicationData;

    /**
     * Utility method that returns node size and location as rectangle.
     *
     * @returns node bounding rectangle.
     */
    public Rectangle getBounds()
    {
        return new Rectangle( x, y, width, height );

    }

    /**
     * Utility method that set node size and location as in given rectangle.
     */
    public void setBounds(Rectangle rec)
    {
        x = rec.x;
        y = rec.y;
        width = rec.width;
        height = rec.height;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Attribute issues
    //

    protected HashMap<String, String> attributes;

    public static void copyAttributes(Node nodeFrom, Node nodeTo)
    {
        for( Map.Entry<String, String> entry : nodeFrom.attributes.entrySet() )
            nodeTo.setAttribute( entry.getKey(), entry.getValue() );
    }
    public boolean hasAttribute(String key)
    {
        return attributes != null && attributes.containsKey( key );

    }

    /**
     * Finds value for the specified attribute key.
     *
     * @param key - attribute key.
     *
     * @return pointer to value if it was found or null otherwise.
     */
    public String getAttribute(String key)
    {
        if( attributes != null )
            return attributes.get( key );

        return null;
    }

    public void setAttribute(String key, String value)
    {
        if( attributes == null )
            attributes = new HashMap<>();

        attributes.put( key, value );
    }

    // //////////////////////////////////////////////////////////////////////////
    // Port support issues
    //

    /**
     * PortFinder allows to specify constraints on edge start/end points location.
     */
    public PortFinder portFinder = null;

    public void setPortFinder(PortFinder finder)
    {
        this.portFinder = finder;
    }

    /**
     * This method allows to take into account some constraints that are
     * specified by PortFinder.
     *
     * @param x - desired point x coordinate
     * @param y - desired point y coordinate
     * @param edge - edge that will be started(ended) at this point.
     *
     * @return point that is nearest to the specified location from which the
     *          edge can be started.
     */
    public Point findPort(int x, int y, Edge edge)
    {
        if( portFinder != null )
            return portFinder.findPort( this, edge, x, y );

        return new Point( x, y );
    }
    
    public ShapeChanger shapeChanger = null;
    
    public void setShapeChanger(ShapeChanger shapeChanger)
    {
        this.shapeChanger = shapeChanger;
    }
    
    public void adjustSize()
    {
        if (shapeChanger == null)
            return;
        shapeChanger.changeShape(this);
    }

    @Override
    public Node clone()
    {
        Node newNode = new Node( name, x, y, width, height );
        newNode.applicationData = applicationData;
        newNode.fixed = fixed;
        newNode.data = data;
        if( attributes != null )
        {
            Iterator<String> iter = attributes.keySet().iterator();
            while( iter.hasNext() )
            {
                String attr = iter.next();
                newNode.setAttribute( attr, attributes.get( attr ) );
            }
        }
        return newNode;
    }

}
