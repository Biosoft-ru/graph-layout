package ru.biosoft.graph;

import java.awt.Point;
import java.awt.Rectangle;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import ru.biosoft.graph.OrthogonalPathLayouter.Orientation;


/**
 * Different methods for working with ru.biosoft.Graph, ru.biosoft.Node and ru.biosoft.Edge
 */
public class Util
{
    private Util() {}

    /**
     * @return Set<Node> from graph with all nodes connected to node
     */
    public static Set<Node> getNodes(Node node, Graph graph)
    {
        Set<Node> result = new HashSet<>();
        if( graph.getEdges(node) != null )
            for( Edge edge : graph.getEdges(node) )
            {
                result.add(edge.to);
                result.add(edge.from);
            }
        return result;
    }

    /**
     * @return rectangle which surrounds all <b>nodes</b> and is even slightly larger (x+=20, y+=20)
     */
    public static Rectangle getBounds(Set<Node> nodes)
    {
        if( nodes == null )
            return new Rectangle();
        Rectangle rect = null;

        for( Node node : nodes )
        {
            Rectangle r = node.getBounds();
            if( rect == null )
                rect = r;
            else
                rect = rect.union(r);
        }

        if( rect == null )
            rect = new Rectangle();

        rect.x -= 10;
        rect.y -= 10;
        rect.width += 20;
        rect.height += 20;

        return rect;
    }

    /**
     * @return node from <b>graph</b> which is direct (closest) compartment of <b>node</b>
     */
    public static Node getCompartment(Node node, Graph graph)
    {
        String compartmentName = node.getAttribute("compartmentName");
        if( compartmentName == null )
            return null;
        else
        {
//            compartmentName = compartmentName.substring(compartmentName.indexOf(".") + 1);
            return graph.getNode(compartmentName);
        }

    }

    /**
     * @return Set<Node> of all compartments from <b>graph</b> who contains <b>node</b>
     */
    public static Set<Node> getCompartments(Node node, Graph graph)
    {
        Set<Node> compartments = new HashSet<>();

        Node compartment = getCompartment(node, graph);
        while( compartment != null )
        {
            compartments.add(compartment);
            compartment = getCompartment(compartment, graph);
        }
        return compartments;
    }


    /**
     * @return level of this node i.e. how many nested compartments contains <b>node</b>
     */
    public static Integer getLevel(Node node)
    {
        String path = node.getAttribute("compartmentName");
        if( path == null )
            return 0;
        return path.length() - path.replace(".", "").length() + 1; //number of "." in path
    }

    /**
     * @return true if <b>node</b> is a compartment
     */
    public static boolean isCompartment(Node node)
    {
        if( node == null )
            return false;
        Object attr = node.getAttribute("isCompartment");
        return ( attr != null && attr.toString().equals("true") );
    }

    /**
     * @return true if graph contains edge n1 -> n2 or edge n2 -> n1
     */
    public static boolean areConnected(Node n1, Node n2, Graph graph)
    {
        return graph.getEdge(n1, n2) != null || graph.getEdge(n2, n1) != null;
    }

    /**
     * Output graph in text format for debug issues
     * @param path - output file
     * @param graph - graph to be written
     */
    public static void outGraph(String path, Graph graph)
    {
        try (BufferedWriter buf = new BufferedWriter( new FileWriter( path ) ))
        {
            String out = graph.generateText(true);
            buf.write(out);
        }
        catch( Exception ex )
        {
        }
    }

    /**
     * @param p1 - path for first Node
     * @param p2 - path for second Node
     * @return greatest common path for both nodes
     */
    public static String getCommonCompartment(String p1, String p2)
    {
        if( p1 == null || p2 == null )
            return null;

        if( p1.contains(p2) )
            return p2;
        else if( p2.contains(p1) )
            return p1;

        StringBuffer result = new StringBuffer();
        String[] names1 = p1.split("\\.");
        String[] names2 = p2.split("\\.");
        int length = Math.min(names1.length, names2.length);
        for( int i = 0; i < length; i++ )
        {
            if( names1[i].equals(names2[i]) )
            {
                result.append(names1[i]).append(".");
            }
        }
        if( result.length() != 0 )
            result.deleteCharAt(result.length() - 1);
        return result.toString();
    }

    /**
     * Get graph bounds based on fixed nodes and edges and ignore unfixed ones
     */
    public static Rectangle getFixedBounds(Graph g)
    {
        Rectangle rect = null;
        for( Node node : g.nodeList )
        {
            if( !node.fixed )
                continue;

            Rectangle r = node.getBounds();
            if( rect == null )
                rect = r;
            else
                rect = rect.union( r );
        }

        for( Edge edge : g.edgeList )
        {
            if( !edge.fixed )
                continue;
            Path path = edge.path;
            if( path != null )
            {
                Rectangle r = path.getBounds();
                if( rect == null )
                    rect = r;
                else
                    rect = rect.union( r );
            }
        }
        return rect;
    }

    public static void adjustOrientations(Graph graph)
    {
        for( Node node : graph.nodeList )
        {
            if( node.fixed )
                continue;
            Util.calcOrientation(graph, node);
            node.adjustSize();
        }
    }

    public static void calcOrientation(Graph graph, Node node)
    {
        String orientationStr = node.getAttribute("orientation");

        if( orientationStr == null )
            return;

        Orientation orientation = Orientation.fromString(orientationStr);

        int[] rotations = new int[4];
        for( Edge e : graph.getEdges(node) )
        {
            Node otherNode = e.to.equals(node) ? e.from : e.to;

            //get two opposite points on node border and check if port finder will decline both of them and suggest the same (fixed) point
            Point port = node.findPort(node.x, node.y, e);
            Point otherPort = node.findPort(node.x + node.width, node.y + node.height, e);
            if( port.x == otherPort.x && port.y == otherPort.y )  //this edge has _likely_ fixed port on the given node
            {

                Orientation actualPosition = getRelativePosition(node, otherNode); //position of otherNode relative to our node
                Orientation desiredPosition = getPortLocation(node, port); // position of node which is best according to current base node orientation

                //how we should rotate desired position to obtain actual
                //i.e. which is the bet way to rotate base node
                Integer numberOfTurns = numberOfTurnsMap.get(desiredPosition).get(actualPosition);
                rotations[numberOfTurns]++;
            }
        }

        if( rotations[0] > rotations[1] && rotations[0] > rotations[2] && rotations[0] > rotations[3] )
        {
            //do nothing
        }
        else if( rotations[1] > rotations[2] && rotations[1] > rotations[3] )
        {
            orientation = orientation.clockwise(); //rotate 1 time clockwise
        }
        else if( rotations[2] > rotations[3] )
        {
            orientation = orientation.clockwise().clockwise(); //rotate 2 times
        }
        else
        {
            orientation = orientation.clockwise().clockwise().clockwise(); //rotate 3 times
        }

        node.setAttribute("orientation", orientation.toString());
    }


    protected static Map<Orientation, Map<Orientation, Integer>> numberOfTurnsMap = new HashMap<Orientation, Map<Orientation, Integer>>()
    {
        {
            put(Orientation.TOP , new HashMap<Orientation, Integer>()
            {
                {
                    put(Orientation.TOP, 0);
                    put(Orientation.RIGHT, 1);
                    put(Orientation.BOTTOM, 2);
                    put(Orientation.LEFT, 3);
                }
            });

            put(Orientation.RIGHT , new HashMap<Orientation, Integer>()
            {
                {
                    put(Orientation.TOP, 3);
                    put(Orientation.RIGHT, 0);
                    put(Orientation.BOTTOM, 1);
                    put(Orientation.LEFT, 2);
                }
            });

            put(Orientation.BOTTOM , new HashMap<Orientation, Integer>()
            {
                {
                    put(Orientation.TOP, 2);
                    put(Orientation.RIGHT, 3);
                    put(Orientation.BOTTOM, 0);
                    put(Orientation.LEFT, 1);
                }
            });

            put(Orientation.LEFT , new HashMap<Orientation, Integer>()
            {
                {
                    put(Orientation.TOP, 1);
                    put(Orientation.RIGHT, 2);
                    put(Orientation.BOTTOM, 3);
                    put(Orientation.LEFT, 0);
                }
            });
        }
    };

    protected static Orientation getPortLocation(Node base, Point p)
    {

        if( p.x <= base.x )//left
        {
            return Orientation.LEFT;
        }
        else if( p.y <= base.y ) // top
        {
            return Orientation.TOP;
        }
        else if( p.x >= base.x + base.width ) //right
        {
            return Orientation.RIGHT;
        }
        else // bottom
        {
            return Orientation.BOTTOM;
        }
    }

    protected static Orientation getRelativePosition(Node base, Node target)
    {
        int distX = target.x - base.x + (target.width - base.width)/2 ;
        int distY = target.y - base.y + ( target.height - base.height ) / 2;

        if(Math.abs(distX) > Math.abs(distY) )
            return distX > 0 ? Orientation.RIGHT : Orientation.LEFT;
        else
            return distY > 0 ? Orientation.BOTTOM : Orientation.TOP;
    }

}
