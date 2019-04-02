package ru.biosoft.graph;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import ru.biosoft.jobcontrol.JobControl;

/**
 * Orthogonal graph layout algorithm.
 * <p/>
 * The main idea is following:
 * <ul>
 * <li>select central node;</li>
 * <li>select connected node with minimum weight;</li>
 * <li>calculate weight for all possible directions and select best of them;</li>
 * <li>mark node us processed and select next node with minimum degree connected
 * with central node or select new central node;</li>
 * <li>repeat until all nodes will be marked as processed.</li>
 * </ul>
 * <p/>
 * <p>This approach uses so called "greedy" algorithm,
 * that will try to find best solution on each step.
 * However, this algorithm does not guarantee that it
 * will find global optimum.
 * <p/>
 * <p>Additionally the suggested approach can be used
 * for incremental graph nodes layout if it will be necessary.
 */
public class GreedyLayouter extends AbstractLayouter
{
    protected static final Logger log = Logger.getLogger(GreedyLayouter.class.getName());

    public static final int ALIGN_MODE_LEFT = 0;
    public static final int ALIGN_MODE_CENTER = 1;
    public static final int ALIGN_MODE_RIGHT = 2;

    /** Grid x step. */
    protected int gridX = 5;

    /** Grid y step. */
    protected int gridY = 5;

    /** X step for layout  */
    protected int layerDeltaX = 75;

    /** Y step for layout */
    protected int layerDeltaY = 75;

    /** Align mode. */
    public int align = ALIGN_MODE_CENTER;

    private static final int LOCATE_ITERATIONS_MAX_COUNT = 20;

    protected int processedWeight = -1;
    protected int unprocessedWeight = 1;

    protected List<Node> processedNodes = new ArrayList<>();
    protected List<Node> unprocessedNodes = new ArrayList<>();

    // Current central node
    protected Node center;

    // List of rectangles for each path line for processed edges for incremental layout.
    protected List<Rectangle> pathRectangles = new ArrayList<>();

    public GreedyLayouter()
    {
        pathLayouterWrapper = new PathLayouterWrapper( new OrthogonalPathLayouter() );
    }

    public GreedyLayouter(int gridX, int gridY)
    {
        this.gridX = gridX;
        this.gridY = gridY;
    }

    /**
     * Initialize internal variables for processing of the specified graph.
     * <p/>
     * The method do following:
     * <ol>
     * <li>assign initial weight to all graph nodes. Node weight it is degree
     * excluding parallel edges and self loops;</li>
     * <li>puts all nodes into <code>unprocessedNodes</code> list;</li>
     * <li>assign initial center node - node with maximum weight.</li>
     * </ol>
     *
     * @param graph - graph to be laid out.
     */
    protected void init(Graph graph)
    {
        processedNodes.clear();
        unprocessedNodes.clear();
        pathRectangles.clear();
        center = null;

        // sort nodes - fixed are treated as processed,
        // other - as unprocessed
        for( Node node : graph.nodeList )
            if( node.fixed )
            {
                if( node.data == null )
                {
                    LayoutData ld = new LayoutData();
                    ld.fixed = true;
                    ld.processed = true;
                    ld.weight = processedWeight;
                    node.data = ld;
                }
                processedNodes.add(node);
            }
            else
                unprocessedNodes.add(node);

        // calculate nodes weight
        for( Node node : unprocessedNodes )
        {
            LayoutData ld = new LayoutData();
            node.data = ld;

            List<Edge> edges = graph.getEdges(node);
            if( edges != null )
                for( Edge edge : edges )
                    // exclude self loops and parallel edges
                    if( edge.from != edge.to && edge.master )
                    {
                        Node u = ( node == edge.from ) ? edge.to : edge.from;
                        ld.weight += processedNodes.contains(u) ? processedWeight : unprocessedWeight;
                    }

            if( center == null || ( (LayoutData)center.data ).weight < ld.weight )
                center = node;
        }

        // treat unprocessed edges
        for( Edge edge : graph.edgeList )
        {
            if( edge.path != null && edge.path.npoints > 0 && processedNodes.contains(edge.from) && processedNodes.contains(edge.to) )
            {
                for( int i = 0; i < edge.path.npoints - 1; i++ )
                {
                    pathRectangles.add(new Rectangle(Math.min(edge.path.xpoints[i], edge.path.xpoints[i + 1]) - 1, Math.min(
                            edge.path.ypoints[i], edge.path.ypoints[i + 1]) - 1,
                            Math.abs(edge.path.xpoints[i] - edge.path.xpoints[i + 1]) + 2, Math.abs(edge.path.ypoints[i]
                                    - edge.path.ypoints[i + 1]) + 2));
                }
            }
            else
            {
                edge.path = null;
                edge.data = null;
            }
        }
    }

    /**
     * Layout graph nodes.
     * See the algorithm description above.
     *
     * @param graph - graph whose nodes should be laid out
     */
    @Override
    void layoutNodes(Graph graph, LayoutJobControl jobControl)
    {
        int counter = 0;
        init(graph);
        log.log(Level.FINE, "Center node groups:" + center);

        if( center == null )
            return;

        // process central node
        if( processedNodes.size() == 0 )
        {
            center.x = 0;
            center.y = 0;

            ( (LayoutData)center.data ).processed = true;
            unprocessedNodes.remove(center);
            processedNodes.add(center);
        }
        else
        {
            //select first central node as at intermediate step of layout
            Node firstCenter = selectNextCenter( graph );
            if( firstCenter != null )
            {
                center = firstCenter;
                processNode( graph, firstCenter );
            }
        }

        // process other nodes
        while( unprocessedNodes.size() > 0 )
        {
            if( jobControl != null )
            {
                jobControl.done(counter++);
                if( jobControl.getStatus() != JobControl.RUNNING )
                    return;
            }

            Node node = selectNode(graph);
            if( node == null )
            {
            	log.log(Level.SEVERE, "UNPROCESSED nodes: " + unprocessedNodes);
                break;
            }
            processNode(graph, node);
        }

        Util.adjustOrientations(graph);
    }

    // TODO: SYNCRONIZE, why used ONLY in clustered ???
    public void refineNodes(Graph graph, int n)
    {
        boolean changed;
        List<Edge> edgesUnderConsideration = new ArrayList<>();

        for( int k = 0; k < n; k++ )
        {
            changed = false;
            for( Node node : graph.nodeList )
            {
                if( node.fixed )
                    continue;

                edgesUnderConsideration.clear();
                List<Edge> edges = graph.getEdges(node);
                if( edges != null )
                {
                    for( Edge edge : edges )
                    {
                        if( k <= n / 2 )
                        {
                            Node u = edge.getFrom() == node ? edge.getTo() : edge.getFrom();
                            if( graph.getEdges(u).size() == 1 )
                                continue;
                        }
                        edgesUnderConsideration.add(edge);
                    }
                }

                int weight = calcNodeWeight(graph, node, edgesUnderConsideration, Integer.MAX_VALUE);
                int x = node.x;
                int y = node.y;

                if( locateNode(graph, node, edgesUnderConsideration) < weight )
                    changed = true;
                else
                {
                    node.x = x;
                    node.y = y;
                }
            }

            if( k > n / 2 && !changed )
                break;
        }
    }

    /**
     * Select next graph node to be laid out.
     * <p/>
     * First we try select node that is connected with current central
     * node and still unprocessed. Otherwise we select node with minimum
     * weight and set up it as current central node.
     *
     * @param graph - graph from which node will be selected.
     */
    protected Node selectNode(Graph graph)
    {
        // first of all select from nodes that are connected with center
        Node bestNode = selectNode(graph, center);
        if( bestNode != null )
            return bestNode;

        center = selectNextCenter( graph );
        return center;
    }

    protected Node selectNextCenter(Graph graph)
    {
        int weight;
        Node bestNode = null;
        for( Node node : processedNodes )
        {
            Node u = selectNode(graph, node);
            if( u != null )
            {
                weight = ( (LayoutData)u.data ).weight;
                if( bestNode == null || weight < ( (LayoutData)bestNode.data ).weight )
                    bestNode = u;
            }
        }
        return bestNode;
    }

    /**
     * Try to select node that is connected with specified central
     * node and still unprocessed.
     *
     * @param graph      - graph from which node will be selected.
     * @param centerNode - central node.
     */
    protected Node selectNode(Graph graph, Node centerNode)
    {
        Node bestNode = null;
        List<Edge> edges = graph.getEdges(centerNode);
        if( edges == null )
            return null;
        for( Edge edge : edges )
        {
            Node u = edge.from == centerNode ? edge.to : edge.from;
            if( u == null )
                continue;
            if( u.data == null )
                continue;
            if( ( (LayoutData)u.data ).processed )
                continue;

            int weight = ( (LayoutData)u.data ).weight;
            if( bestNode == null || weight < ( (LayoutData)bestNode.data ).weight )
                bestNode = u;
        }

        return bestNode;
    }

    /**
     * Process the specified node.
     * <p/>
     * Node processing includes:
     * <ul>
     * <li>location of node in the best direction;</li>
     * <li>moving node from <code>unprocessedNodes</code> list to
     * <code>processedNodes</code> list;</li>
     * <li>moving the corresponding edges from <code>unprocessedEdges</code>
     * list to <code>processedEdges</code> list.</li>
     * </ul>
     *
     * @param graph - graph to which node belongs
     * @param node  - node to be processed.
     */
    protected void processNode(Graph graph, Node node)
    {
    	log.log(Level.FINE, "Process node:" + node);
        if( node == null )
            return;

        ( (LayoutData)node.data ).weight = 0;
        List<Edge> edgesUnderConsideration = new ArrayList<>();

        List<Edge> edges = graph.getEdges(node);
        for( Edge edge : edges )
        {
            Node u = edge.from == node ? edge.to : edge.from;

            if( u == null || u.data == null )
                continue;

            if( ! ( (LayoutData)u.data ).processed )
                ( (LayoutData)u.data ).weight += processedWeight;
            else
            {
                edgesUnderConsideration.add(edge);
                log.log(Level.FINE, "  edge:" + edge);
            }
        }

        locateNode(graph, node, edgesUnderConsideration);

        ( (LayoutData)node.data ).processed = true;
        unprocessedNodes.remove(node);
        processedNodes.add(node);
    }

    public int locateNode(Graph graph, Node node, List<Edge> edges)
    {
        int bestWeight = Integer.MAX_VALUE;
        Node bestCenter = null;

        for( Edge edge : edges )
        {
            Node center = edge.getFrom() == node ? edge.getTo() : edge.getFrom();

            if( !processedNodes.contains(center) )
                continue;

            int weight = locateNode(graph, node, edges, center);
            if( weight < bestWeight )
            {
                bestWeight = weight;
                bestCenter = center;
            }
        }

        if( bestCenter != null )
            locateNode(graph, node, edges, bestCenter);
        else
        	log.log(Level.SEVERE, "Can not locate node=" + node.getName());

        return bestWeight;
    }

    public int locateNode(Graph graph, Node node, List<Edge> edges, Node center)
    {
        int d = 0;
        int bestWeight = Integer.MAX_VALUE;
        Direction bestDirection = null;

        while( bestWeight > PathWeighter.nodeIntersectionPenalty && d < LOCATE_ITERATIONS_MAX_COUNT )
        {
            d++;
            for( Direction direction : directions )
            {
                locateNode(node, direction, center, d);
                int weight = calcNodeWeight(graph, node, edges, bestWeight);
                if( bestDirection == null || weight < bestWeight )
                {
                    bestDirection = direction;
                    bestWeight = weight;
                }
            }
        }

        locateNode(node, bestDirection, center, d);
        return bestWeight;
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    //

    /**
     * Utility class to specify possible directions for nodes location
     * relative some central node.
     */
    static class Direction
    {
        /**
         * The direction name. Used for debugging purposes only.
         */
        String name;

        /**
         * Horisontal shift relative central node.
         */
        int dx;

        /**
         * Vertical shift relative central node.
         */
        int dy;

        /*
         * Creates the direction with the specified name
         * and shift relative center.
         *
         * @param dx  - horisontal shift
         * @param dy - vertical shift
         * @param name - name.
         */
        public Direction(int dx, int dy, String name)
        {
            this.dx = dx;
            this.dy = dy;
            this.name = name;
        }
    }

    // directions and their priorities how
    // some node can be located relative the central node
    static Direction N = new Direction(0, -1, "N");
    static Direction NE = new Direction(1, -1, "NE");
    static Direction E = new Direction(1, 0, "E");
    static Direction SE = new Direction(1, 1, "SE");
    static Direction S = new Direction(0, 1, "S");
    static Direction SW = new Direction( -1, 1, "SW");
    static Direction W = new Direction( -1, 0, "W");
    static Direction NW = new Direction( -1, -1, "NW");

    static Direction[] directions = {S, E, W, N, SE, SW, NE, NW};

    /**
     * Method to locate node in the specified direction
     * relative the specified central node and with the specified distance between them.
     *
     * @param node   - node to be located
     * @param dir    - direction for location
     * @param center - central node
     * @param d      - vertical/horizontal distance between the specified node and central node.
     */
    protected void locateNode(Node node, Direction dir, Node center, int d)
    {
        if( center == null || node == null )
            return;
        node.x = center.x;
        if( dir.dx != 0 )
        {
            node.x += dir.dx * layerDeltaX * d;

            if( dir.dx < 0 )
                node.x -= node.width;
            else
                node.x += center.width;
        }

        node.y = center.y;
        if( dir.dy != 0 )
        {
            node.y += dir.dy * layerDeltaY * d;

            if( dir.dy < 0 )
                node.y -= node.height;
            else
                node.y += center.height;
        }

        // alignment
        if( align > 0 )//not ALIGN_MODE_CENTER
        {
            if( dir.dx == 0 ) // horisontal alignment
            {
                if( align == ALIGN_MODE_CENTER )
                    node.x = center.x + ( center.width - node.width ) / 2 / gridX * gridX;
                else
                    // ALIGN_MODE_RIGHT
                    node.x = center.x + ( center.width - node.width );
            }

            if( dir.dy == 0 ) // horisintal alignment
            {
                if( align == ALIGN_MODE_CENTER )
                    node.y = center.y + ( center.height - node.height ) / 2 / gridY * gridY;
                else
                    // ALIGN_MODE_RIGHT
                    node.y = center.y + ( center.height - node.height );
            }
        }
    }

    /**
     * Calculates weight for the specified graph node.
     * <p/>
     * The weight is calculated as following:
     * <pre>
     * weight = nodeIntersectionPenalty (if intersects any graph node)
     *          + pathLength
     *          + edgeIntersectionPenaty
     * </pre>
     *
     * @param graph      - graph to which node belongs
     * @param node       - the node
     * @param edges      - unprocessed edges connected with the node
     * @param bestWeight - if in some step current weight/penalty
     *                   more then bestWeight, the process is terminated and current weight is returned.
     * @return weight for the specified node.
     */
    private int calcNodeWeight(Graph graph, Node node, List<Edge> edges, int bestWeight)
    {
        // TODO: SYNCRONIZE WITH PathWeighter
        int weight = 0;

        // calc intersection with other nodes
        int dx = layerDeltaX / 2;
        int dy = layerDeltaY / 2;
        Rectangle rect = new Rectangle(node.x - dx, node.y - dy, node.width + 2 * dx, node.height + 2 * dy);

        for( Node p : processedNodes )
        {
            Rectangle r = new Rectangle(p.x, p.y, p.width, p.height);
            if( rect.intersects(r) )
            {
                weight += PathWeighter.nodeIntersectionPenalty;
                if( weight > bestWeight )
                    return weight;
            }
        }

        // calc edges weight
        for( Edge edge : edges )
        {
            weight += calcEdgeWeight(graph, edge, bestWeight - weight);
            if( weight > bestWeight )
                return weight;
        }

        // calc penalty for intersection with previous edges
        // for incremental layout
        for( Rectangle r : pathRectangles )
            if( rect.intersects(r) )
            {
                weight += PathWeighter.edgeIntersectionPenalty;
                if( weight > bestWeight )
                    return weight;
            }

        return weight;
    }

    /**
     * Calculates weight for the specified edge.
     * Currently edge weight is edge length only.
     *
     * @param graph      - graph to which edge belongs
     * @param edge       - the edge
     * @param bestWeight - if in some step current weight/penalty more
     *                   then bestWeight, the process is terminated and current weight is returned.
     * @return weigth for the specified graph edge.
     */
    protected int calcEdgeWeight(Graph graph, Edge edge, int bestWeight)
    {
        // penalise length
        int dx = edge.from.x + edge.from.width / 2 - ( edge.to.x + edge.to.width / 2 );
        int dy = edge.from.y + edge.from.height / 2 - ( edge.to.y + edge.to.height / 2 );
        int weight = (int)Math.sqrt(dx * dx + dy * dy);

        // force nodes alignment by center
        /*boolean forceAlignByCenter = true;
         if( forceAlignByCenter && align == ALIGN_MODE_CENTER && graph.nodeCount() < 100 )
         {
         Node from = edge.getFrom();
         int x1 = from.x + from.width/2;
         int y1 = from.y + from.height/2;

         Node to   = edge.getTo();
         int x2 = to.x + to.width/2;
         int y2 = to.y + to.height/2;

         if( Math.abs(x1 - x2) < gridX || Math.abs(y1 - y2) < gridY )
         {
         Path path = edge.getPath();

         if(  opl.createHorisontalLinePath(graph, edge) ||
         opl.createVerticalLinePath(graph, edge) )
         weight -= 100;

         edge.setPath(path);
         }
         }*/
        return weight;
    }

    /**
     * Utility class to associate layout information with graph nodes
     * using {@link Node#data} field.
     */
    static class LayoutData
    {
        /**
         * Original node width.
         */
        int width = 0;

        /**
         * Original node height.
         */
        int height = 0;

        /**
         * Whether the original node is fixed.
         */
        boolean fixed = false;

        /**
         * Number of nodes in the cluster.
         */
        float n = 1;

        /**
         * Node weight.
         */
        int weight = 0;

        /**
         * Specifies whether this node already processed.
         */
        boolean processed = false;
    }

    @Override
    public void layoutEdges(Graph graph, LayoutJobControl lJC)
    {
        getPathLayouter().layoutEdges( graph, lJC );
    }

    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl lJC)
    {
        getPathLayouter().layoutPath( graph, edge, lJC );
    }

    //+++++++++++++++++++++++++++++++++++++
    @Override
    public LayouterInfo getInfo()
    {
        LayouterInfoSupport lis = new LayouterInfoSupport(true, true, false, false, false, true);
        return lis;
    }

    @Override
    public int estimate(Graph graph, int what)
    {
        return graph.nodeCount();
    }

    //Getters and setters
    public int getGridX()
    {
        return gridX;
    }
    public void setGridX(int gridX)
    {
        this.gridX = gridX;
    }
    public int getGridY()
    {
        return gridY;
    }
    public void setGridY(int gridY)
    {
        this.gridY = gridY;
    }

    public int getLayerDeltaX()
    {
        return layerDeltaX;
    }
    public void setLayerDeltaX(int deltaX)
    {
        layerDeltaX = deltaX;
    }
    public int getLayerDeltaY()
    {
        return layerDeltaY;
    }
    public void setLayerDeltaY(int deltaY)
    {
        layerDeltaY = deltaY;
    }
}
