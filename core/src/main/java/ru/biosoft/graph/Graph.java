package ru.biosoft.graph;

import java.awt.Point;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.StringTokenizer;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * This class implements a directed graph structure. Consists of a set of nodes
 * and a set of edges (node pairs).
 */
public class Graph implements Cloneable
{
    protected static final Logger log = Logger.getLogger(Graph.class.getName());

    // ///////////////////////////////////////////////////////////////////////////
    // access to nodes and edges
    //

    protected Map<String, Node> nodeMap = new HashMap<>();

    protected List<Node> nodeList = new ArrayList<>();

    protected Map<Node, List<Edge>> edgeMap = new IdentityHashMap<>();

    protected List<Edge> edgeList = new ArrayList<>();

    /** Arbitrary data that can be associated by GraphLayout with the graph. */
    Object data;

    /**
     * Number of nodes in the graph.
     *
     * @return number of nodes in the graph.
     */
    public int nodeCount()
    {
        return nodeMap.size();
    }

    public Node nodeAt(int i)
    {
        return nodeList.get( i );
    }

    public Iterator<Node> nodeIterator()
    {
        return nodeList.iterator();
    }

    public Iterator<Edge> edgeIterator()
    {
        return edgeList.iterator();
    }

    /**
     * Adds new node to the graph.
     *
     * @param node - node to be added.
     * @throws IllegalArgumentException if node name is not specified.
     */
    public void addNode(Node node)
    {
        if( getNode( node.getName() ) != null )
            throw new IllegalArgumentException( "Graph already contains node with name '" + node.getName() + "'" );

        nodeMap.put( node.getName(), node );
        nodeList.add( node );
    }

    /**
     * Adds new edge to the graph.
     *
     * If such edge is already exist in the graph (that is added edge is
     * parallel to some edge in the graph) then edge will be marked as
     * slave(parallel).
     *
     * @param edge - edge to be added.
     */
    public void addEdge(Edge edge)
    {
        if( edge.master )
        {
            // parallel edge processing issues
            for( Edge master : edgeList )
            {
                if( master.master && master.from == edge.from && master.to == edge.to )
                {
                    edge.master = false;
                    master.addSlave( edge );
                    break;
                }
            }
        }

        edgeList.add( edge );
        if( !edge.master )
            return;

        List<Edge> nodes = edgeMap.get( edge.from );
        if( nodes == null )
        {
            nodes = new ArrayList<>();
            edgeMap.put( edge.from, nodes );
        }
        nodes.add( edge );

        nodes = edgeMap.get( edge.to );
        if( nodes == null )
        {
            nodes = new ArrayList<>();
            edgeMap.put( edge.to, nodes );
        }
        nodes.add( edge );
    }

    /**
     * Returns node with the specified name.
     *
     * @param name - node name.
     */
    public Node getNode(String name)
    {
        return nodeMap.get( name );
    }

    /**
     * Returns unmodifieble list of edges for the specified node or empty list if node has no edges
     *
     * Note: the list contains only master edges.
     */
    public List<Edge> getEdges(Node node)
    {
        if( !edgeMap.containsKey( node ) )
            return Collections.emptyList();
        return Collections.unmodifiableList( edgeMap.get( node ) );
    }

    /**
     * Returns list of all graph edges.
     * @return
     */
    public List<Edge> getEdges()
    {
        return edgeList;
    }

    /**
     * Returns list of all nodes
     * @return
     */
    public List<Node> getNodes()
    {
        return nodeList;
    }
    /**
     * Returns the edge connecting the specified nodes.
     *
     * Note: only master edge will be returned. To get parallel edges use
     * {@see ru.biosoft.Edge.slaves} property.
     *
     * @param from - edge start node.
     * @param to - edge end node.
     * @return edge connecting the specified nodes.
     */
    public Edge getEdge(Node from, Node to)
    {
        List<Edge> edges = getEdges( from );
        if( edges == null )
            return null;

        for( Edge edge : edges )
        {
            // reverse edge is parallel to original too
            if( edge.from == from && edge.to == to )
                return edge;
        }

        return null;
    }

    /**
     * Returns the edge connecting the specified nodes.
     *
     * Note: only master edge will be returned. To get parallel edges use
     * {@see ru.biosoft.Edge.slaves} property.
     *
     * @param from - edge start node.
     * @param to - edge end node.
     * @param undirected if true edge starting with to and ending at from may be returned as well
     *
     * @return edge connecting the specified nodes.
     */
    public Edge getEdge(Node from, Node to, boolean undirected)
    {
    	if(!undirected)
    		return getEdge(from, to);
        List<Edge> edges = getEdges( from );

        if( edges == null )
            return null;

        for( Edge edge : edges )
        {
            // reverse edge is parallel to original too
            if( ( edge.from == from && edge.to == to ) || ( edge.from == to && edge.to == from ) )
                return edge;
        }

        return null;
    }

    /**
     * Number of edges in the graph.
     */
    public int edgeCount()
    {
        return edgeList.size();
    }

    /**
     * Removes the specified node from the graph.
     *
     * @param node - node to be removed.
     */
    public void removeNode(Node node)
    {
        // check whether the graph contains such node
        if( getNode( node.getName() ) == null )
            return;

        List<Edge> edges = edgeMap.get( node );
        if( edges != null )
        {
            while( edges.size() > 0 )
                removeEdge( edges.get( 0 ) );
        }

        nodeMap.remove( node.getName() );
        nodeList.remove( node );
        edgeMap.remove( node );
    }

    /**
     * Removes the specified node from the graph.
     *
     * The method supports parallel edge issues. If master edge was removed,
     * then its first slave (if exists) will be marked as master. If slave edge
     * was removed, then it will be removed from slave list of corresponding
     * master edge.
     *
     * @param edge - edge to be removed.
     */
    public void removeEdge(Edge edge)
    {
        // remove edge from node map
        edgeMap.get( edge.from ).remove( edge );
        edgeMap.get( edge.to ).remove( edge );
        edgeList.remove( edge );

        if( !edge.master )
        {
            Edge master = getEdge( edge.from, edge.to );
            master.slaves.remove( edge );
        }
        else if( edge.slaves != null && edge.slaves.size() > 0 )
        {
            Edge master = edge.slaves.get( 0 );
            master.master = true;
            master.slaves = edge.slaves;
            master.slaves.remove( master );

            edgeMap.get( edge.from ).add( master );
            edgeMap.get( edge.to ).add( master );
        }

        edge.master = true;
    }

    // //////////////////////////////////////////////////////////////////////////
    //
    //

    /**
     * Calculates rectangle that bounds all graph nodes.
     * Returns empty rectangle at coordinate space zero position in case no nodes and edges defined
     *
     * @return rectangle that bounds all graph nodes.
     */
    public Rectangle getBounds()
    {
        Rectangle rect = null;

        for( Node node : nodeList )
        {
            Rectangle r = node.getBounds();
            if( rect == null )
                rect = r;
            else
                rect = rect.union( r );
        }

        for( Edge edge : edgeList )
        {
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

        if( rect == null )
            rect = new Rectangle();
        return rect;
    }

    /**
     * Moves all graph nodes such that bounding rectangle (rectangle
     * that bounds all graph nodes) origin will be located in the specified
     * coordinates.
     *
     * @param x - bounding rectangle x coordinate.
     * @param y - bounding rectangle y coordinate.
     */
    public void setLocation(int x, int y)
    {
        Rectangle rect = getBounds();
        move( x - rect.x, y - rect.y );
    }

    /**
     * Moves all graph nodes on the specified distance.
     *
     * @param dx - shift by x axis.
     * @param dy - shift by y axis.
     */
    public void move(int dx, int dy)
    {
        for( Node node : nodeList )
        {
            if( node.fixed )
                continue;

            node.x += dx;
            node.y += dy;
        }

        for ( Edge e : edgeList )
        {
            if( e.fixed )
                continue;

        	Path path = e.path;
        	if ( path != null )
                 path.translate ( dx, dy );
        }
    }

    /** Zoom graph to the specified coefficient. */
    public void zoom(float k)
    {
        for( Node node : nodeList )
        {
            node.x = (int) ( k * node.x );
            node.y = (int) ( k * node.y );
            node.width = (int) ( k * node.width );
            node.height = (int) ( k * node.height );
        }

        for( Edge edge : edgeList )
        {
            Path path = edge.path;
            if( path != null )
                for( int n = 0; n < path.npoints; n++ )
                {
                    path.xpoints[n] = (int) ( k * path.xpoints[n] );
                    path.ypoints[n] = (int) ( k * path.ypoints[n] );
                }
        }
    }

    /**
     * Finds the node (first in nodes list) that will be intersected by the
     * specified line.
     *
     * @param x1 - line start x coordinate
     * @param y1 - line start y coordinate
     * @param x2 - line end x coordinate
     * @param y2 - line end y coordinate.
     *
     * @return node (first in the node list if line intersects several nodes)
     *         intersected by the line or null if there is no intersections.
     */
    public Node getIntersectedNode(int x1, int y1, int x2, int y2)
    {
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

        Rectangle rect = new Rectangle( x1, y1, x2 - x1, y2 - y1 );

        for( Node node : nodeList )
            if( rect.intersects( node.x, node.y, node.width, node.height ) )
                return node;

        return null;
    }

    /**
     * Checks whether the specified point is occupied by edge start or end
     * point.
     *
     * @return true if edge start or end point is equal to the specified point
     *         and false otherwise.
     */
    boolean isOccupied(Edge edge, Point p)
    {
        if( edge.getPath() != null )
        {
            Path path = edge.getPath();
            if( p.x == path.xpoints[0] && p.y == path.ypoints[0] )
                return true;
            if( p.x == path.xpoints[path.npoints - 1] && p.y == path.ypoints[path.npoints - 1] )
                return true;
        }

        return false;
    }

    /**
     * Returns true if the specified point is free, that it is not used as start
     * or end point by other edges for the specified node.
     *
     * @param node - node for which point is checked
     * @param point - point that is checked
     *
     * @return true if the specified point is free and false otherwise.
     */
    boolean isFree(Node node, Point point)
    {
        List<Edge> nodeEdges = getEdges( node );
        for( Edge e : nodeEdges )
        {
            if( isOccupied( e, point ) )
                return false;

            if( e.slaves != null )
                for( Edge slave : e.slaves )
                    if( isOccupied( slave, point ) )
                        return false;
        }

        return true;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Utility methods
    //

    /**
     * Splits the graph to the set of unconnected subgraphs if possible.
     * If graph is connected the method returns initial graph clone.
     *
     * @return list of unconnected subgraphs. If graph is connected the list
     *         will contain only one graph that is initial graph shallow clone.
     */
    public List<Graph> split()
    {
        ArrayList<Graph> graphs = new ArrayList<>();

        for( Node node : nodeList )
            if( !contains( graphs.iterator(), node ) )
                graphs.add( subgraph( node ) );

        return graphs;
    }

    public boolean isConnected()
    {
        if( nodeList.size() <= 1 )
            return true;
        Graph sub = subgraph( nodeList.get( 0 ) );
        return sub.nodeList.size() == nodeList.size();
    }

    /**
     * Returns the connected subgraph including the specified node for the given
     * graph.
     *
     * @param node - node that should be obligatory included in the resulted graph
     * @return connected subgraph including the specified node for the given graph.
     */
    public Graph subgraph(Node node)
    {
        Graph subgraph = new Graph();
        subgraph.addNode( node );

        addEdges( subgraph, getEdges( node ) );

        return subgraph;
    }

    /** Utility method for <code>subgraph</code> method. */
    protected void addEdges(Graph subgraph, List<Edge> edges)
    {
        if( edges == null )
            return;

        for( Edge edge : edges )
        {
            boolean addFrom = false;
            boolean addTo = false;
            if( subgraph.getNode( edge.from.getName() ) == null )
            {
                addFrom = true;
                subgraph.addNode( edge.from );
            }
            if( subgraph.getNode( edge.to.getName() ) == null )
            {
                addTo = true;
                subgraph.addNode( edge.to );
            }

            boolean addEdge = true;
            for( Edge edge_k : subgraph.edgeList )
            {
                if( edge_k.from == edge.from && edge_k.to == edge.to )
                {
                    addEdge = false;
                    break;
                }
            }
            if( addEdge )
            {
                subgraph.addEdge( edge );
                if( edge.slaves != null )
                {
                    // TODO: here maybe problem with lost edges
                    for( Edge slave : edge.slaves.toArray( new Edge[edge.slaves.size()] ) )
                        subgraph.addEdge( slave );
                }
            }

            if( addFrom )
                addEdges( subgraph, getEdges( edge.from ) );

            if( addTo )
                addEdges( subgraph, getEdges( edge.to ) );
        }
    }

    /** Utility method for <code>subgraph</code> method. */
    protected static boolean contains(Iterator<Graph> graphs, Node node)
    {
        while( graphs.hasNext() )
        {
            Graph graph = graphs.next();

            if( graph.getNode( node.getName() ) != null )
                return true;
        }

        return false;
    }

    /**
     * @return List of all possible roots, i.e nodes having only outgoing edges
     * @throws HasCyclesException in case when cycles are found
     */
    public List<Node> getRoots() throws HasCyclesException
    {
        if( nodeCount() == 0 || !isConnected() )
        {
            return null;
        }
        List<Node> roots = new ArrayList<>();
        Set<Node> globallyVisitedNodes = new HashSet<>();
        for( Iterator<Node> nodeIterator = nodeIterator(); nodeIterator.hasNext(); )
        {
            Node node = nodeIterator.next();
            if( !globallyVisitedNodes.contains( node ) )
            {
                fillRoots( node, globallyVisitedNodes, new HashSet<Node>(), roots );
            }
        }
        return roots;
    }

    private void fillRoots(Node node, Set<Node> globallyVisitedNodes, Set<Node> oneStageVisitedNodes, List<Node> roots)
            throws HasCyclesException
    {
        oneStageVisitedNodes.add( node );
        globallyVisitedNodes.add( node );
        List<Edge> edges = getEdges( node );

        boolean found = false;
        if( edges != null )
        {
            for( Edge edge : edges )
            {
                if( edge.to == node )
                {
                    boolean thisStageVisited = oneStageVisitedNodes.contains( edge.from );
                    boolean globallyVisited = globallyVisitedNodes.contains( edge.from );
                    if( thisStageVisited )
                    {
                        throw new HasCyclesException( oneStageVisitedNodes );
                    }
                    if( !globallyVisited )
                    {
                        fillRoots( edge.from, globallyVisitedNodes, oneStageVisitedNodes, roots );
                        oneStageVisitedNodes.remove( edge.from );
                    }
                    found = true;
                }
            }
            if( !found )
            {
                roots.add( node );
            }
        }
    }

    // //////////////////////////////////////////////////////////////////////////
    // Graph read/write utilities
    //

    /**
     * Generates textual description for the specified graph.
     *
     * @return textual description for the specified graph.
     */
    public String generateText()
    {
        return generateText( true );
    }

    /**
     * Generates textual description for the specified graph.
     *
     * @return textual description for the specified graph.
     */
    public String generateText(boolean includeLayout)
    {
        StringBuffer result = new StringBuffer();

        result.append( "// Nodes: " ).append( nodeList.size() ).append( "\n" );

        for( Node node : nodeList )
        {
            result.append( "N: " ).append( node.getName() ).append( ", " ).append( node.width ).append( ", " ).append( node.height );

            if( includeLayout )
                result.append( ", " ).append( node.x ).append( ", " ).append( node.y );


            result.append( "\n" );

            // write node attributes
            if( node.attributes != null )
                for( String key : node.attributes.keySet() )
                    result.append( "A: " + node.getName() + ", " + key + "=" + node.getAttribute( key ) + "\n" );
        }

        result.append( "//\n" );

        result.append( "// Edges: " ).append( edgeList.size() ).append( "\n" );

        for( Edge edge : edgeList )
        {
            result.append( "E: " ).append( edge.from.getName() ).append( ", " ).append( edge.to.getName() );

            if( includeLayout && edge.path != null )
                for( int i = 0; i < edge.path.npoints; i++ )
                    result.append( ", " ).append( edge.path.xpoints[i] ).append( ", " ).append( edge.path.ypoints[i] );

            result.append( "\n" );
        }

        return result.toString();
    }

    /**
     * Fills the graph from its text description.
     *
     * Method automatically detects used graph format and calls corresponding method.
     *
     * @param text - graph textual description.
     */
    public void fillFromText(String text)
    {
        if( text.indexOf( "N:" ) >= 0 )
            fillFromText1( text );
        else
            fillFromText2( text );
    }

    /**
     * Fills the graph from its text description.
     *
     * The format is following:
     *
     * <pre>
     * graph     = node (node | edge | attribute )
     * node      = 'N: ' node_name, width, height (,x ,y)*
     * edge      = 'E: ' node_from, node_to (, pathPoint)*
     * attribute = 'A: ' node_name, attribute_name = attribute_value
     * </pre>
     *
     * @param text - graph textual description.
     */
    public void fillFromText1(String text)
    {
        StringTokenizer lines = new StringTokenizer( text, "\r\n" );
        while( lines.hasMoreTokens() )
        {
            String line = lines.nextToken();
            try
            {
                StringTokenizer tokens = new StringTokenizer( line, " ,=" );

                String token = tokens.nextToken();
                if( "//".equals( token ) )
                {
                }
                else if( "N:".equals( token ) )
                    addNode( parseNode( tokens ) );
                else if( "E:".equals( token ) )
                    addEdge( parseEdge( tokens, this ) );
                else if( "A:".equals( token ) )
                {
                    if( !parseAttribute( tokens ) )
                        log.log(Level.SEVERE, "\n  line: " + line);
                }
                else
                	log.log(Level.SEVERE,"Error: - unknown directive '" + token + "'." + "\n  line: " + line );
            }
            catch( Throwable t )
            {
            	log.log(Level.SEVERE, "Error during parsing: " + t + "\n  line: " + line );
            }
        }
    }

    /**
     * Constructs graph node instance from its textual description.
     *
     * @param tokens -
     *            graph node textual description.
     * @return graph node instance.
     */
    static Node parseNode(StringTokenizer tokens)
    {
        Node node = new Node( tokens.nextToken() );

        node.width = Integer.parseInt( tokens.nextToken() );
        node.height = Integer.parseInt( tokens.nextToken() );

        if( tokens.hasMoreTokens() )
        {
            node.x = Integer.parseInt( tokens.nextToken() );
            node.y = Integer.parseInt( tokens.nextToken() );
        }

        return node;
    }

    /**
     * Assigns attribute to the corresponding graph node.
     *
     * @param tokens - graph node textual description.
     *
     * @return true if attribute was parsed successfully and false otherwise.
     */
    boolean parseAttribute(StringTokenizer tokens)
    {
        String nodeName = tokens.nextToken();
        Node node = getNode( nodeName );
        if( node == null )
        {
        	log.log(Level.SEVERE, "Graph has not node with name '" + nodeName + "'" );
            return false;
        }

        if( !tokens.hasMoreTokens() )
        {
        	log.log(Level.SEVERE, "Attribute key absents" );
            return false;
        }
        String key = tokens.nextToken();

        if( !tokens.hasMoreTokens() )
        {
        	log.log(Level.SEVERE, "Attribute value absents" );
            return false;
        }
        String value = tokens.nextToken();

        node.setAttribute( key, value );
        return true;
    }

    /**
     * Constructs graph edge instance from its textual description.
     *
     * @param tokens - graph edge textual description
     * @param graph - graph which contains edge nodes
     *
     * @return - graph edge instance.
     *
     */
    static Edge parseEdge(StringTokenizer tokens, Graph graph)
    {
        String fromName = tokens.nextToken();
        String toName = tokens.nextToken();

        Node from = graph.getNode( fromName );
        Node to = graph.getNode( toName );

        if( from == null )
            throw new NoSuchElementException( "Graph has not node with name '" + fromName + "'" );
        if( to == null )
            throw new NoSuchElementException( "Graph has not node with name '" + toName + "'" );

        Edge edge = new Edge( from, to );
        if( tokens.hasMoreTokens() )
        {
            edge.path = new Path();

            while( tokens.hasMoreTokens() )
                edge.path.addPoint( Integer.parseInt( tokens.nextToken() ), Integer.parseInt( tokens.nextToken() ) );
        }

        return edge;
    }

    /**
     * Constructs graph edge instance from its textual description. This method
     * can be used for incremental graph layout when you need to add new edge to
     * existing graph.
     *
     * The string format is following:
     *
     * <pre>
     *  edge = edge_name, node_from, node_to (, pathPoint)*
     * </pre>
     *
     * @param text - graph edge textual description
     * @param graph - graph which contains edge nodes
     *
     * @return - graph edge instance.
     */
    public static Edge parseEdge(String text, Graph graph)
    {
        return parseEdge( new StringTokenizer( text, " ," ), graph );
    }

    /**
     * Other variant to read the graph. The format is following:
     *
     * <pre>
     * graph = (node-&gt;node)+
     * </pre>
     */
    public void fillFromText2(String text)
    {
        StringTokenizer tokens = new StringTokenizer( text, ", \n" );
        while( tokens.hasMoreTokens() )
        {
            String edgeStr = tokens.nextToken();

            int d = edgeStr.indexOf( '-' );
            if( d == -1 )
            {
            	log.log(Level.SEVERE, "Invalid edge: '" + edgeStr + "'." );
                continue;
            }

            String in = edgeStr.substring( 0, d );
            String out = edgeStr.substring( d + 2 );

            Node inNode = getNode( in );
            if( inNode == null )
            {
                inNode = new Node( in );
                inNode.width = 30;
                inNode.height = 20;
                addNode( inNode );
            }

            Node outNode = getNode( out );
            if( outNode == null )
            {
                outNode = new Node( out );
                outNode.width = 30;
                outNode.height = 20;
                addNode( outNode );
            }

            Edge edge = new Edge( inNode, outNode );
            addEdge( edge );
        }
    }

    public void addLables(Object[] labelesData, int maxLayer, int width, int height)
    {

        ru.biosoft.graph.Node[] labeles = new ru.biosoft.graph.Node[maxLayer];

        for( int i = maxLayer - 1; i >= 0; i-- )
        {
            labeles[i] = new ru.biosoft.graph.Node( "level." + i );

            labeles[i].width = width;
            labeles[i].height = height;
            this.addNode( labeles[i] );

            labeles[i].setAttribute( "label", "true" );
            labeles[i].setAttribute( "level", String.valueOf( i + 1 ) );
            labeles[i].setAttribute( "score", String.valueOf( 0 ) );
            labeles[i].applicationData = labelesData[i];

        }
    }

    @Override
	public Graph clone()
    {
        Graph result = new Graph();

        for( Node node : nodeList )
        {
            result.addNode( node.clone() );
        }
        for( Edge edge : edgeList )
        {
            Node from = result.getNode( edge.from.name );
            Node to = result.getNode( edge.to.name );
            Edge newEdge = new Edge( from, to );
            newEdge.path = edge.path;
            newEdge.reversed = edge.reversed;
            newEdge.data = edge.data;
            newEdge.master = edge.master;
            newEdge.applicationData = edge.applicationData;
            if( edge.slaves != null )
            {
                for( Edge slave : edge.slaves )
                    newEdge.addSlave( slave );
            }
            result.addEdge( newEdge );
        }
        return result;
    }
}
