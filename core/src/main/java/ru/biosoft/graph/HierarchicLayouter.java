package ru.biosoft.graph;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import java.util.logging.Level;
import java.util.logging.Logger;

import ru.biosoft.graph.OrthogonalPathLayouter.Orientation;
import ru.biosoft.jobcontrol.JobControl;

public class HierarchicLayouter extends AbstractLayouter
{
    protected static final Logger log = Logger.getLogger(HierarchicLayouter.class.getName());

    public HierarchicLayouter()
    {}

    @Override
	protected void layoutNodes(Graph graph, LayoutJobControl lJC)
    {
        initLayoutData( graph );
        normalise( graph );

        assignNodeLevels( graph );
        addDummies( graph );

        // clear edges as they can affect getBounds(). They will be layouted later during layoutEdges stage
        for( Edge edge : graph.getEdges() )
            edge.path = new Path();

        makeLevels( graph );
        initLevelEdges( graph );
        placeNodesInitial();

        for( int i = 0; i < layerOrderIterationNum; i++ )
        {
            orderNodes( graph, i, lJC );
            if( lJC != null )
            {
                if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                    i = layerOrderIterationNum;
            }
        }

        optimizeNodeCoordinates(graph, lJC);

        Util.adjustOrientations(graph);
    }

    /**
     * Returns Omega-value for Edge which is used during edge weight calculation.
     * 
     * Edges between dummy nodes has higher Omega-value, thus they should be more straight.
     * 
     * @param e input edge
     * @return omega value
     */
    protected int getEdgeOmegaValue(Edge e)
    {
        int dummyCount = ( isDummy( e.getFrom() ) ? 1 : 0 ) + ( isDummy( e.getTo() ) ? 1 : 0 );
        switch( dummyCount )
        {
            case 0:
                return 1;
            case 1:
                return 2;
            case 2:
            default:
                return 8;
        }
    }

    protected int calcNodePositionsScore(Graph graph)
    {
        Iterator<Edge> edgeIterator = graph.edgeIterator();
        int weight = 0;
        while( edgeIterator.hasNext() )
        {
            Edge e = edgeIterator.next();
            weight += getEdgeOmegaValue( e ) * Math.abs( getIntralevelCoord( e.from ) - getIntralevelCoord( e.to ) );
        }
        return weight;
    }

    int maxNodeOptimizationNum = 8;
    public int getMaxNodeOptimizationNum()
    {
        return maxNodeOptimizationNum;
    }

    public void setMaxNodeOptimizationNum(int maxNodeOptimizationNum)
    {
        this.maxNodeOptimizationNum = maxNodeOptimizationNum;
    }

    private void optimizeNodeCoordinates(Graph graph, LayoutJobControl lJC)
    {
        preprocessIntralevelCoords( graph );
        fillLinkedStructures( graph );
        int optimalWeight = calcNodePositionsScore( graph );
        int[] optimalPositions = getIntralevelCoords( graph );
        for( int i = 0; i < maxNodeOptimizationNum; i++ )
        {
            applyMedianPositions( graph );
            if( lJC != null )
            {
                operationsDone += 2;
                lJC.done( operationsDone );
                if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                    break;
            }
            applyMinEdges( graph );
            if( lJC != null )
            {
                operationsDone += 2;
                lJC.done( operationsDone );
                if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                    break;
            }
            applyMinNodes( graph );
            if( lJC != null )
            {
                operationsDone += 2;
                lJC.done( operationsDone );
                if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                    break;
            }
            applyMinPath( graph );
            if( lJC != null )
            {
                operationsDone += 2;
                lJC.done( operationsDone );
                if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                    break;
            }
            // TODO: pack
            int weight = calcNodePositionsScore( graph );
            if( weight < optimalWeight )
            {
                optimalWeight = weight;
                optimalPositions = getIntralevelCoords( graph );
            }
            if( lJC != null )
            {
                operationsDone += 2;
                lJC.done( operationsDone );
                if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                    break;
            }
        }
        saveIntralevelCoords( graph, optimalPositions );
        postprocessIntralevelCoords( graph );
    }

    private void applyMinNodes(Graph graph)
    {
        Set<Node> nodeSet = Collections.newSetFromMap( new IdentityHashMap<Node, Boolean>() );
        nodeSet.addAll( graph.getNodes() );
        while( !nodeSet.isEmpty() )
        {
            // TODO: optimize
            Node node = nodeSet.iterator().next();
            nodeSet.remove( node );
            Span span = getNodeSpan( node );
            if( span == null || span.min >= span.max )
                continue;
            Node[] upwardNodes = upwards.get( node );
            Node[] downwardNodes = downwards.get( node );
            Node[] nodes = new Node[upwardNodes.length + downwardNodes.length];
            if( nodes.length == 0 )
                continue;
            System.arraycopy( upwardNodes, 0, nodes, 0, upwardNodes.length );
            System.arraycopy( downwardNodes, 0, nodes, upwardNodes.length, downwardNodes.length );
            int median = span.fitValue( getMedianIntralevelCoord( nodes ) );
            if( median != getIntralevelCoord( node ) )
            {
                if( verticalOrientation )
                    node.x = median;
                else
                    node.y = median;
                nodeSet.addAll( Arrays.asList( nodes ) );
                nodeSet.addAll( levelNodes.get( getLevel( node ) - 1 ) );
                nodeSet.remove( node );
            }
        }
    }


    private void applyMinPath(Graph graph)
    {
        List<List<Node>> chains = getVirtualChains( graph );
        boolean changed;
        do
        {
            changed = false;
            for( List<Node> ch : chains )
            {
                if( ch.size() < 2 )
                    continue;
                int prevIndex = 0;
                Span prevSpan = getNodeSpan( ch.get( 0 ) );
                for( int i = 1; i < ch.size(); i++ )
                {
                    Node n = ch.get( i );
                    Span span = getNodeSpan( n );
                    Span intersected = prevSpan.intersect( span );
                    if( intersected == null )
                    {
                        if( i - prevIndex > 1 )
                        {
                            // TODO: select newPos in more clever way (account real nodes also?)
                            int newPos = prevSpan.min < 0 ? prevSpan.max : prevSpan.min;
                            for( int j = prevIndex; j < i; j++ )
                            {
                                int oldPos = getIntralevelCoord( ch.get( j ) );
                                if( oldPos == newPos )
                                    continue;
                                changed = true;
                                if( verticalOrientation )
                                {
                                    ch.get( j ).x = newPos;
                                }
                                else
                                {
                                    ch.get( j ).y = newPos;
                                }
                            }
                        }
                        prevSpan = span;
                        prevIndex = i;
                    }
                    else
                        prevSpan = intersected;
                }
                if( ch.size() - prevIndex > 1 )
                {
                    // TODO: select newPos in more clever way (account real nodes also?)
                    int newPos = prevSpan.min < 0 ? prevSpan.max : prevSpan.min;
                    for( int j = prevIndex; j < ch.size(); j++ )
                    {
                        int oldPos = getIntralevelCoord( ch.get( j ) );
                        if( oldPos == newPos )
                            continue;
                        changed = true;
                        if( verticalOrientation )
                        {
                            ch.get( j ).x = newPos;
                        }
                        else
                        {
                            ch.get( j ).y = newPos;
                        }
                    }
                }
            }
        }
        while( changed );
    }

    private List<List<Node>> getVirtualChains(Graph graph)
    {
        List<List<Node>> virtualChains = new ArrayList<>();
        unmarkNodes( graph );
        for( Node n : graph.getNodes() )
        {
            if( isDummy( n ) && !isMarked( n ) )
            {
                virtualChains.add( getChain( graph, n ) );
            }
        }
        return virtualChains;
    }

    private List<Node> getChain(Graph graph, Node n)
    {
        List<Node> chain = new ArrayList<>();
        Set<Node> curNodes = Collections.newSetFromMap( new IdentityHashMap<Node, Boolean>() );
        chain.add( n );
        curNodes.add( n );
        while( !curNodes.isEmpty() )
        {
            Node node = curNodes.iterator().next();
            curNodes.remove( node );
            ( (LayoutData) ( node.data ) ).marked = true;
            for( Edge edge : graph.getEdges( node ) )
            {
                Node from = edge.getFrom();
                if( isDummy( from ) && !isMarked( from ) )
                {
                    chain.add( from );
                    curNodes.add( from );
                }
                Node to = edge.getTo();
                if( isDummy( to ) && !isMarked( to ) )
                {
                    chain.add( to );
                    curNodes.add( to );
                }
            }
        }
        Collections.sort( chain, new Comparator<Node>()
        {
            @Override
            public int compare(Node n1, Node n2)
            {
                return getLevel( n1 ) - getLevel( n2 );
            }
        } );
        return chain;
    }

    private void preprocessIntralevelCoords(Graph graph)
    {
        for( Node node : graph.getNodes() )
        {
            if( verticalOrientation )
            {
                node.x = node.x + node.width / 2;
            }
            else
            {
                node.y = node.y + node.height / 2;
            }
        }
    }

    private void postprocessIntralevelCoords(Graph graph)
    {
        int minCoord = Integer.MAX_VALUE;
        for( Node node : graph.getNodes() )
        {
            minCoord = Math.min( minCoord, verticalOrientation ? node.x - node.width / 2 : node.y - node.height / 2 );
        }
        for( Node node : graph.getNodes() )
        {
            if( verticalOrientation )
            {
                node.x = node.x - node.width / 2 - minCoord;
            }
            else
            {
                node.y = node.y - node.height / 2 - minCoord;
            }
        }
        for( int i = 0; i < maxLevel; i++ )
        {
            List<Node> nodes = levelNodes.get( i );
            int levelThickness = 0;
            for( Node node : nodes )
            {
                levelThickness = Math.max( levelThickness, verticalOrientation ? node.height : node.width );
            }
            for( Node node : nodes )
            {
                if( verticalOrientation )
                {
                    node.y = node.y + ( levelThickness - node.height ) / 2;
                }
                else
                {
                    node.x = node.x + ( levelThickness - node.width ) / 2;
                }
            }
        }
    }

    private int[] getIntralevelCoords(Graph graph)
    {
        List<Node> nodes = graph.getNodes();
        int[] coords = new int[nodes.size()];
        for( int i = 0; i < nodes.size(); i++ )
        {
            coords[i] = getIntralevelCoord( nodes.get( i ) );
        }
        return coords;
    }

    private int getIntralevelCoord(Node node)
    {
        return verticalOrientation ? node.x : node.y;
    }

    private int getIntralevelWidth(Node node)
    {
        return verticalOrientation ? node.width + ( isDummy( node ) ? virtualNodesDistance : layerDeltaX ) : node.height
                + ( isDummy( node ) ? virtualNodesDistance : layerDeltaY );
    }

    private void saveIntralevelCoords(Graph graph, int[] coords)
    {
        List<Node> nodes = graph.getNodes();
        if( verticalOrientation )
            for( int i = 0; i < nodes.size(); i++ )
            {
                nodes.get( i ).x = coords[i];
            }
        else
            for( int i = 0; i < nodes.size(); i++ )
            {
                nodes.get( i ).y = coords[i];
            }
    }

    transient Map<Node, Node[]> upwards = null;
    transient Map<Node, Node[]> downwards = null;

    private void fillLinkedStructures(Graph graph)
    {
        List<Node> nodes = graph.getNodes();
        upwards = new IdentityHashMap<>();
        downwards = new IdentityHashMap<>();
        for( int i = 0; i < nodes.size(); i++ )
        {
            Node n = nodes.get( i );
            List<Edge> edges = graph.getEdges( n );
            List<Node> up = new ArrayList<>();
            List<Node> down = new ArrayList<>();
            int upEdgesWeight = 0;
            int downEdgesWeight = 0;
            for( Edge e : edges )
            {
                if( e.getFrom().equals( n ) )
                {
                    up.add( e.getTo() );
                    upEdgesWeight += getEdgeOmegaValue( e );
                }
                else
                {
                    down.add( e.getFrom() );
                    downEdgesWeight += getEdgeOmegaValue( e );
                }
            }
            upwards.put( n, up.toArray( new Node[up.size()] ) );
            downwards.put( n, down.toArray( new Node[down.size()] ) );
            ( (LayoutData)n.data ).upEdgesWeight = upEdgesWeight;
            ( (LayoutData)n.data ).downEdgesWeight = downEdgesWeight;
        }
    }

    private void applyMedianPositions(Graph graph)
    {
        //Downwards
        List<Node> nodes = new ArrayList<>();
        nodes.addAll( graph.getNodes() );
        Collections.sort( nodes, new Comparator<Node>()
        {
            @Override
			public int compare(Node n1, Node n2)
            {
                int c1 = ( (LayoutData)n1.data ).downEdgesWeight;
                int c2 = ( (LayoutData)n2.data ).downEdgesWeight;
                return c1 < c2 ? 1 : c1 > c2 ? -1 : getLevel( n1 ) - getLevel( n2 );
            }
        } );
        unmarkNodes( graph );
        for( int i = 0; i < nodes.size(); i++ )
        {
            Node n = nodes.get( i );
            Node[] adjacent = downwards.get( n );
            int median = verticalOrientation ? n.x : n.y;
            if( adjacent.length != 0 )
                median = getMedianIntralevelCoord( adjacent );
            Span span = getIntralevelNodeSpan( n );
            if( span == null )
                continue;
            median = span.fitValue( median );
            if( verticalOrientation )
                n.x = median;
            else
                n.y = median;
            ( (LayoutData)n.data ).marked = true;
        }

        //Upwards
        nodes.clear();
        nodes.addAll( graph.getNodes() );
        Collections.sort( nodes, new Comparator<Node>()
        {
            @Override
			public int compare(Node n1, Node n2)
            {
                int c1 = ( (LayoutData)n1.data ).upEdgesWeight;
                int c2 = ( (LayoutData)n2.data ).upEdgesWeight;
                return c1 < c2 ? 1 : c1 > c2 ? -1 : getLevel( n2 ) - getLevel( n1 );
            }
        } );
        unmarkNodes( graph );
        for( int i = 0; i < nodes.size(); i++ )
        {
            Node n = nodes.get( i );
            Node[] adjacent = upwards.get( n );
            int median = verticalOrientation ? n.x : n.y;
            if( adjacent.length != 0 )
                median = getMedianIntralevelCoord( adjacent );
            Span span = getIntralevelNodeSpan( n );
            if( span == null )
                continue;
            median = span.fitValue( median );
            if( verticalOrientation )
                n.x = median;
            else
                n.y = median;
            ( (LayoutData)n.data ).marked = true;
        }

    }

    private int snapDown(int value)
    {
        return verticalOrientation ? value / gridX * gridX : value / gridY * gridY;
    }

    private int snapUp(int value)
    {
        return verticalOrientation ? ( value + gridX - 1 ) / gridX * gridX : ( value + gridY - 1 ) / gridY * gridY;
    }

    private Span getIntralevelNodeSpan(Node n)
    {
        List<Node> nodes = levelNodes.get( getLevel( n ) - 1 );
        // TODO: reduce to constant complexity
        for( int i = 0; i < nodes.size(); i++ )
        {
            if( nodes.get( i ) == n )
            {
                Span span = new Span();
                boolean prev = true;
                boolean next = true;
                if( i == 0 )
                {
                    prev = false;
                }
                if( i == nodes.size() - 1 )
                {
                    next = false;
                }
                //move to the right, find nearest marked, count place required for unmarked
                if( next )
                {
                    int j = i + 1;
                    int unmarkedLen = 0;
                    while( j < nodes.size() )
                    {
                        Node nextNode = nodes.get( j );
                        if( ( (LayoutData)nextNode.data ).marked )
                            break;
                        unmarkedLen += getIntralevelWidth( nextNode );
                        unmarkedLen = snapUp( unmarkedLen );
                        j++;
                    }
                    if( j < nodes.size() )//no marked to the left, leave max
                    {
                        int nextMarkedCoord = getIntralevelCoord( nodes.get( j ) ) - getIntralevelWidth( nodes.get( j ) ) / 2;
                        span.max = snapDown( nextMarkedCoord - unmarkedLen - getIntralevelWidth( nodes.get( i ) ) / 2 );
                    }
                }
                if( prev )
                {
                    int j = i - 1;
                    int unmarkedLen = 0;
                    while( j >= 0 )
                    {
                        Node prevNode = nodes.get( j );
                        if( ( (LayoutData)prevNode.data ).marked )
                            break;
                        unmarkedLen += getIntralevelWidth( prevNode );
                        unmarkedLen = snapUp( unmarkedLen );
                        j--;
                    }
                    int prevMarkedCoord = j >= 0 ? getIntralevelCoord( nodes.get( j ) ) + getIntralevelWidth( nodes.get( j ) ) / 2 : 0;
                    span.min = snapUp( prevMarkedCoord + unmarkedLen + getIntralevelWidth( nodes.get( i ) ) / 2 );
                }
                else
                {
                    span.min = 0;
                }
                return span;
            }
        }
        return null;
    }

    /**
     * Returns median intralevel coordinate for given list of nodes.
     * 
     * Warning: this method changes original list of nodes (namely sorts them), so be careful!
     */
    private int getMedianIntralevelCoord(Node[] adjacent)
    {
        Arrays.sort( adjacent, new Comparator<Node>()
        {
            @Override
            public int compare(Node n1, Node n2)
            {
                int c1 = getIntralevelCoord( n1 );
                int c2 = getIntralevelCoord( n2 );
                return c1 > c2 ? 1 : c1 < c2 ? -1 : 0;
            }
        } );
        return ( adjacent.length % 2 == 0 )
                ? ( getIntralevelCoord( adjacent[adjacent.length / 2 - 1] ) + getIntralevelCoord( adjacent[adjacent.length / 2] ) ) / 2
                : getIntralevelCoord( adjacent[adjacent.length / 2] );
    }

    private static class Span
    {
        int min = Integer.MIN_VALUE, max = Integer.MAX_VALUE;

        Span intersect(Span s)
        {
            Span result = new Span();
            result.min = Math.max( min, s.min );
            result.max = Math.min( max, s.max );
            if( result.min > result.max )
                return null;
            return result;
        }

        int fitValue(int value)
        {
            return value > max ? max : value < min ? min : value;
        }
    }

    private Span getNodeSpan(Node n)
    {
        List<Node> nodes = levelNodes.get( getLevel( n ) - 1 );
        // TODO: reduce to constant complexity
        for( int i = 0; i < nodes.size(); i++ )
        {
            if( nodes.get( i ) == n )
            {
                Span span = new Span();
                if( i > 0 )
                    span.min = snapUp( getIntralevelCoord( nodes.get( i - 1 ) )
                            + ( getIntralevelWidth( nodes.get( i - 1 ) ) + getIntralevelWidth( nodes.get( i ) ) ) / 2 );
                if( i < nodes.size() - 1 )
                    span.max = snapDown( getIntralevelCoord( nodes.get( i + 1 ) )
                            - ( getIntralevelWidth( nodes.get( i + 1 ) ) + getIntralevelWidth( nodes.get( i ) ) ) / 2 );
                return span;
            }
        }
        return null;
    }

    private void applyMinEdges(Graph graph)
    {
        for( Edge edge : graph.getEdges() )
        {
            if( isDummy( edge.getFrom() ) || isDummy( edge.getTo() )
                    || getIntralevelCoord( edge.getFrom() ) != getIntralevelCoord( edge.getTo() ) )
                continue;
            Set<Node> adjacentNodes = Collections.newSetFromMap( new IdentityHashMap<Node, Boolean>() );
            List<Edge> adjacentEdges = new ArrayList<>();
            adjacentEdges.addAll( graph.getEdges( edge.getFrom() ) );
            adjacentEdges.addAll( graph.getEdges( edge.getTo() ) );
            for( Edge edge2 : adjacentEdges )
            {
                if( edge == edge2 )
                    continue;
                if( edge2.getFrom() == edge.getFrom() )
                    adjacentNodes.add( edge2.getTo() );
                if( edge2.getTo() == edge.getTo() )
                    adjacentNodes.add( edge2.getFrom() );
            }
            if( adjacentNodes.size() == 0 )
                continue;
            int median = getMedianIntralevelCoord( adjacentNodes.toArray( new Node[adjacentNodes.size()] ) );
            if( getIntralevelCoord( edge.getFrom() ) == median )
                continue;
            Span spanFrom = getNodeSpan( edge.getFrom() );
            if( spanFrom == null )
                continue;
            Span spanTo = getNodeSpan( edge.getTo() );
            if( spanTo == null )
                continue;
            Span span = spanFrom.intersect( spanTo );
            if( span == null )
                continue;
            median = span.fitValue( median );
            if( verticalOrientation )
                edge.getFrom().x = edge.getTo().x = median;
            else
                edge.getFrom().y = edge.getTo().y = median;
        }
    }

    @Override
	public void layoutEdges(Graph graph, LayoutJobControl jobControl)
    {
        // first iteration - remove all pathes
        for( Edge edge : graph.getEdges() )
            edge.path = new Path();

        if( splineEdges )
        {
            layoutSplineEdges( graph );

            // restore reversed edges
            for( int i = 0; i < graph.edgeList.size(); i++ )
            {
                Edge edge = graph.edgeList.get( i );
                if( edge.reversed )
                {
                    edge.reverseDirection();
                }
            }
        }
        else
        {
            // second iteration - create simple pathes
            for( int i = 0; i < maxLevel; i++ )
            {
                List<Node> nodes = levelNodes.get( i );
                for( Node n : nodes )
                {
                    //output edges
                    Point startPoint = ( verticalOrientation ) ? new Point( n.x, n.y + n.height ) : new Point( n.x + n.width, n.y );
                    Point finishPoint;
                    List<List<Edge>> edgeClusters = getOrderedEdgeSet( graph, n, false );
                    int clustersNum = edgeClusters.size();

                    for( int j = 0; j < clustersNum; j++ )
                    {
                        List<Edge> edgeCluster = edgeClusters.get( j );
                        int outEdgeNum = edgeCluster.size();

                        if( outEdgeNum == 1 && isFixed( edgeCluster.get( 0 ), n ) )
                        {
                            Edge edge = edgeCluster.get( 0 );
                            Point outputPoint = ( verticalOrientation ) ? n.findPort( n.x + n.width / 2, n.y + n.height, edge ) : n
                                    .findPort( n.x + n.width, n.y + n.height / 2, edge );
                            edge.getPath().addPoint( outputPoint.x, outputPoint.y );
                            continue;
                        }

                        if( j < clustersNum - 1 )
                        {
                            List<Edge> nextCluster = edgeClusters.get( j + 1 );
                            Edge nextEdge = nextCluster.get( 0 );
                            finishPoint = n.findPort( n.x, n.y, nextEdge );
                        }
                        else
                        {
                            finishPoint = new Point( n.x + n.width, n.y + n.height );
                        }

                        int step = ( verticalOrientation ) ? finishPoint.x - startPoint.x : finishPoint.y - startPoint.y;

                        step /= ( outEdgeNum + 1 );

                        for( int k = 0; k < outEdgeNum; k++ )
                        {
                            if( edgeCluster.get( k ) == null )
                                continue;

                            Edge edge = edgeCluster.get( k );

                            if( !edge.master )
                                continue;

                            Point p = verticalOrientation ? n.findPort(startPoint.x + step * ( k + 1 ), startPoint.y, edge)
                                    : n.findPort(startPoint.x, startPoint.y + step * ( k + 1 ), edge);
                            edge.path.addPoint(p.x, p.y);
                        }
                    }

                    //input edges
                    startPoint = new Point( n.x, n.y );
                    edgeClusters = getOrderedEdgeSet( graph, n, true );
                    clustersNum = edgeClusters.size();
                    for( int j = 0; j < clustersNum; j++ )
                    {
                        List<Edge> edgeCluster = edgeClusters.get( j );
                        int edgeNum = edgeCluster.size();

                        if( edgeNum == 0 )
                            continue;

                        if( edgeNum == 1 && isFixed( edgeCluster.get( 0 ), n ) )
                        {
                            Edge edge = edgeCluster.get( 0 );
                            Point inputPoint = ( verticalOrientation ) ? n.findPort( n.x + n.width / 2, n.y, edge ) : n.findPort( n.x, n.y
                                    + n.height / 2, edge );
                            edge.getPath().addPoint( inputPoint.x, inputPoint.y );
                            continue;
                        }

                        if( j < clustersNum - 1 )
                        {
                            List<Edge> nextCluster = edgeClusters.get( j + 1 );
                            Edge nextEdge = nextCluster.get( 0 );
                            finishPoint = n.findPort( n.x, n.y, nextEdge );
                        }
                        else
                        {
                            finishPoint = ( verticalOrientation ) ? new Point( n.x + n.width, n.y ) : new Point( n.x, n.y + n.height );
                        }

                        int step = ( verticalOrientation ) ? finishPoint.x - startPoint.x : finishPoint.y - startPoint.y;

                        step /= ( edgeNum + 1 );

                        for( int k = 0; k < edgeNum; k++ )
                        {
                            if( edgeCluster.get( k ) == null )
                                continue;

                            Edge edge = edgeCluster.get(k);

                            if( !edge.master )
                                continue;

                            Point p = verticalOrientation ? n.findPort(startPoint.x + step * ( k + 1 ), startPoint.y, edge)
                                    : n.findPort(n.x, startPoint.y + step * ( k + 1 ), edge);
                            edge.path.addPoint(p.x, p.y);
                        }
                    }
                }
            }
            
            Util.outGraph("C:/graphs/g1.txt", graph);

            // third iteration - remove dummies and processing parallel edges
            removeDummies( graph );

            Util.outGraph("C:/graphs/g2.txt", graph);
            
            // processing parallel edges only for neighboring levels
            for( Edge edge : graph.edgeList )
            {
                if( edge.master && edge.slaves != null )
                {
                    if( Math.abs( getLevel( edge.from ) - getLevel( edge.to ) ) == 1 )
                    {
                        for( int i = 0; i < edge.slaves.size(); i++ )
                        {
                            Edge slave = edge.slaves.get( i );

                            // shift parallel edges midway
                            slave.path = shiftParallelMidway( edge, slave, i + 1, graph );
                        }
                        edge.path = shiftParallelMidway( edge, edge, 0, graph );
                    }
                }
            }

            Util.outGraph("C:/graphs/g3.txt", graph);
            
            // restore reversed edges
            for( int i = 0; i < graph.edgeList.size(); i++ )
            {
                Edge edge = graph.edgeList.get( i );
                if( edge.reversed )
                {
                    edge.reverseDirection();
                }
            }
            
            Util.outGraph("C:/graphs/g4.txt", graph);

            if( selfLoopLayouter == null )
                selfLoopLayouter = new SelfLoopLayouter();

            // recover self loops
            for( Edge edge : selfLoops )
            {
                selfLoopLayouter.layoutPath( graph, edge, pathWeighter );
                graph.addEdge( edge );
            }

            Util.outGraph("C:/graphs/g5.txt", graph);
            
            if( ( pathLayouter instanceof HierarchicPathLayouter ) )
            {
                // add to path control points to Bezier splines
                for( Edge edge : graph.edgeList )
                {
                    computingControlPoints( edge );
                }
            }
            else if( ( pathLayouter instanceof OrthogonalPathLayouter ) )
            {
                // TODO: all paths will be clear, so can be optimize
                pathLayouter.layoutEdges( graph, jobControl );
            }
            
            Util.outGraph("C:/graphs/g2.txt", graph);
        }
    }

    // TODO: implement for horizontal orientation
    /** implementation from dot algorithm  */
    protected void layoutSplineEdges(Graph graph)
    {
        // create levels banned space
        fillLevelsHeight( graph );

        removeDummiesForSpline( graph );

        // create lists for all types edges
        List<Edge> neighboringEdges = new LinkedList<>();
        List<Edge> notNeighboringEdges = new LinkedList<>();
        List<Edge> selfEdges = new LinkedList<>();

        for( Edge edge : graph.edgeList )
        {
            if( edge.from == edge.to )
            {
                selfEdges.add( edge );
            }
            else if( Math.abs( getLevel( edge.from ) - getLevel( edge.to ) ) == 1 )
            {
                neighboringEdges.add( edge );
            }
            else if( Math.abs( getLevel( edge.from ) - getLevel( edge.to ) ) > 1 )
            {
                notNeighboringEdges.add( edge );
            }
            else
            {
                // TODO: error class
                throw new RuntimeException( "unrecognized edge" );
            }
        }

        // first - create simple path for same rank edges and neighboring edges
        // TODO: for same rank
        for( Edge edge : neighboringEdges )
        {
            createStraightPath( edge );
        }

        // second - create path for NOT neighboring edges
        for( Edge edge : notNeighboringEdges )
        {
            computeSpline( edge, graph );
        }

        for( Edge edge : selfEdges )
        {
            computeSelfLoop( edge, graph );
        }
    }

    // add for each level empty rectangle with zero width
    protected void fillLevelsHeight(Graph graph)
    {
        List<List<Rectangle>> graphBannedSpace = new ArrayList<>( maxLevel );

        int downSide[] = new int[maxLevel];
        int upSide[] = new int[maxLevel];
        Arrays.fill( upSide, Integer.MAX_VALUE );

        for( Node node : graph.nodeList )
        {
            int level = getLevel( node ) - 1;

            if( ( node.y + node.height > downSide[level] ) )
            {
                downSide[level] = node.y + node.height;
            }
            if( node.y < upSide[level] )
            {
                upSide[level] = node.y;
            }
        }

        for( int i = 0; i < maxLevel; i++ )
        {
            List<Rectangle> levelBannedSpace = new ArrayList<>();
            Rectangle zeroRect = new Rectangle( Integer.MIN_VALUE, upSide[i], 0, downSide[i] - upSide[i] );
            levelBannedSpace.add( zeroRect );
            graphBannedSpace.add( i, levelBannedSpace );
        }

        graph.data = graphBannedSpace;
    }


    // TODO: simplify
    protected void removeDummiesForSpline(Graph graph)
    {
        for( int i = 0; i < graph.nodeCount(); ++i )
        {
            Node from = graph.nodeList.get( i );
            if( Util.isCompartment( from ) )
                continue;
            if( isDummy( from ) )
                continue;

            List<Edge> edges = graph.getEdges( from );
            for( int j = 0; j < edges.size(); j++ )
            {
                Edge edge = edges.get( j );

                // get out dummy edges
                if( edge.from == from && isDummy( edge.to ) )
                {
                    Path line = new Path();
                    line.addPoint( edge.path.xpoints[0], edge.path.ypoints[0] );
                    addDummyNodeToPath( edge.to, line, edge.path.xpoints[1], edge.path.ypoints[1] );

                    List<Node> path = new ArrayList<>();
                    path.add( edge.to );

                    Node to = edge.to;
                    int pathLen = 1;
                    while( true )
                    {
                        List<Edge> dEdges = graph.getEdges( to );
                        for( int d = 0; d < 2; d++ )
                        {
                            Edge e = dEdges.get( d );
                            if( e.from == to )
                            {
                                to = e.to;
                                if( isDummy( to ) )
                                {
                                    addDummyNodeToPath( e.to, line, e.path.xpoints[1], e.path.ypoints[1] );
                                }
                                else
                                    line.addPoint( e.path.xpoints[1], e.path.ypoints[1] );
                                break;
                            }
                        }

                        if( !isDummy( to ) )
                            break;

                        path.add( to );
                        pathLen++;
                    }

                    // restore original edge
                    Edge originalEdge = new Edge( from, to );
                    originalEdge.reversed = edge.reversed;
                    originalEdge.path = line;
                    // check master
                    Edge masterEdge = graph.getEdge( from, to, true );
                    if( masterEdge == null )
                    {
                        originalEdge.master = true;
                    }
                    else
                    {
                        originalEdge.master = false;
                        masterEdge.addSlave( originalEdge );
                    }
                    graph.addEdge( originalEdge );


                    // restore parallel edges
                    List<?> slaves = (List<?>)edge.data;
                    if( slaves != null )
                        for( Object slave : slaves )
                            graph.addEdge( (Edge)slave );

                    List<Rectangle> boxes = new LinkedList<>();
                    for( Node dummy : path )
                    {
                        Rectangle region = getRegionToNode( dummy, graph );
                        boxes.add( region );

                        graph.removeNode( dummy );
                    }
                    originalEdge.data = boxes;

                    j -= pathLen;
                    if( j < 0 )
                        j = 0;
                }
            }
        }
    }

    // TODO: do for horizontal orientation
    // TODO: add indent
    protected Rectangle getRegionToNode(Node node, Graph graph)
    {
        int level = getLevel( node );

        int leftSide = 0;
        int rightSide = Integer.MAX_VALUE;
        for( Node sameLevelNode : graph.nodeList )
        {
            if( !isDummy( sameLevelNode ) && getLevel( sameLevelNode ) == level )
            {
                // divided into left and right
                if( sameLevelNode.x < node.x )
                { // left
                    if( ( sameLevelNode.x + sameLevelNode.width > leftSide ) && ( sameLevelNode.x + sameLevelNode.width < node.x ) )
                    {
                        leftSide = sameLevelNode.x + sameLevelNode.width;
                    }
                }
                else
                {
                    if( ( sameLevelNode.x < rightSide ) && ( sameLevelNode.x > node.x + node.width ) )
                    {
                        rightSide = sameLevelNode.x;
                    }
                }
            }
        }

        Rectangle zeroRect = ( (List<List<Rectangle>>)graph.data ).get(level - 1).get(0);
        int upSide = zeroRect.y;
        int downSide = zeroRect.y + zeroRect.height;

        // TODO: const
        int indent = 10;
        leftSide += indent;
        rightSide -= indent;

        Rectangle region = new Rectangle( leftSide, upSide, rightSide - leftSide, downSide - upSide );

        return region;
    }

    protected void computeSelfLoop(Edge edge, Graph graph)
    {
        // TODO Auto-generated method stub
    }


    protected void computeSpline(Edge edge, Graph graph)
    {
        // for graph with already existing path compute boxes for path
        List<Rectangle> boxes = computeBoxes( edge, graph );

        // create L array, where L[i] - is the line segment that is the intersection of Boxes[i-1] with Boxes[i]
        List<Line2D> LArray = computeLArray( boxes );

        List<Point> controlPoints = new LinkedList<>();
        // TODO: find port for this points
        // add start and end points
        Node fromNode = edge.from;
        Node toNode = edge.to;

        Point start = new Point( fromNode.x + fromNode.width / 2, fromNode.y + fromNode.height / 2 );
        Point end = new Point( toNode.x + toNode.width / 2, toNode.y + toNode.height / 2 );

        controlPoints.add( start );
        controlPoints.add( end );

        // compute control points for future spline ordered ascending main coordinate
        if( !computeControlPoints( start, end, boxes, LArray, controlPoints ) )
        {
            // TODO: need solution
            controlPoints.clear();
            controlPoints.add( start );
            controlPoints.add( end );
        }

        // TODO: it in postprocess
        // if only start and end points then don't compute Bezier
        controlPoints = new ArrayList<>( controlPoints );
        Collections.sort( controlPoints, new Comparator<Point>()
        {
            @Override
            public int compare(Point p1, Point p2)
            {
                return p1.y - p2.y;
            }
        } );

        edge.path = computeBezierSegments( edge, controlPoints, graph );
    }


    protected List<Line2D> computeLArray(List<Rectangle> boxes)
    {
        List<Line2D> LArray = new LinkedList<>();
        Rectangle firstBox = boxes.get( 0 );
        Line2D firstLine = new Line2D.Double( firstBox.getMinX(), firstBox.getMinY(), firstBox.getWidth(), firstBox.getMinY() );
        LArray.add( firstLine );
        for( int i = 1; i < boxes.size(); i++ )
        {
            Rectangle intersection = boxes.get( i - 1 ).intersection( boxes.get( i ) );
            if( intersection.height != 0 )
            {
                // TODO: this is bad
                return null;
            }
            Line2D intersectionLine = new Line2D.Double( intersection.getMinX(), intersection.y, intersection.getMaxX(), intersection.y );
            LArray.add( intersectionLine );
        }
        // add last box line
        Rectangle lastBox = boxes.get( boxes.size() - 1 );
        LArray.add( new Line2D.Double( lastBox.getMinX(), lastBox.getMaxY(), lastBox.getMaxX(), lastBox.getMaxY() ) );

        return new ArrayList<>( LArray );
    }


    protected Path computeBezierSegments(Edge edge, List<Point> controlPoints, Graph graph)
    {
        Path newPath = new Path();

        newPath.addPoint( controlPoints.get( 0 ).x, controlPoints.get( 0 ).y, Path.LINE_TYPE );
        // TODO: add first control
        newPath.addPoint( controlPoints.get( 0 ).x, controlPoints.get( 0 ).y, Path.CUBIC_TYPE );

        // TODO: const
        final double coefLength = 0.3;

        List<Point> oldControlTrio = new ArrayList<>();
        oldControlTrio.add( controlPoints.get( 0 ) );
        oldControlTrio.add( controlPoints.get( 0 ) );

        for( int i = 0; i < controlPoints.size() - 2; i++ )
        {
            List<Point> controlTrio = computeBezierTrio( controlPoints, i, coefLength );

            List<Point> bezierQuad = new ArrayList<>( 4 );
            bezierQuad.add( oldControlTrio.get( oldControlTrio.size() - 2 ) );
            bezierQuad.add( oldControlTrio.get( oldControlTrio.size() - 1 ) );
            bezierQuad.add( controlTrio.get( 0 ) );
            bezierQuad.add( controlTrio.get( 1 ) );

            calculateSpace( bezierQuad, (List<List<Rectangle>>)graph.data, edge );

            newPath.addPoint( controlTrio.get( 0 ).x, controlTrio.get( 0 ).y, Path.LINE_TYPE );
            newPath.addPoint( controlTrio.get( 1 ).x, controlTrio.get( 1 ).y, Path.LINE_TYPE );
            newPath.addPoint( controlTrio.get( 2 ).x, controlTrio.get( 2 ).y, Path.CUBIC_TYPE );

            oldControlTrio = controlTrio;
        }

        List<Point> bezierQuad = new ArrayList<>( 4 );
        bezierQuad.add( oldControlTrio.get( oldControlTrio.size() - 2 ) );
        bezierQuad.add( oldControlTrio.get( oldControlTrio.size() - 1 ) );
        bezierQuad.add( controlPoints.get( controlPoints.size() - 1 ) );
        bezierQuad.add( controlPoints.get( controlPoints.size() - 1 ) );

        calculateSpace( bezierQuad, (List<List<Rectangle>>)graph.data, edge );

        // TODO add last control point
        newPath.addPoint( controlPoints.get( controlPoints.size() - 1 ).x, controlPoints.get( controlPoints.size() - 1 ).y, Path.LINE_TYPE );
        newPath.addPoint( controlPoints.get( controlPoints.size() - 1 ).x, controlPoints.get( controlPoints.size() - 1 ).y, Path.LINE_TYPE );

        return newPath;
    }

    /** Return list Rectangles of each level */
    protected void calculateSpace(List<Point> bezierQuad, List<List<Rectangle>> graphSpaceLevels, Edge edge)
    {
        // create boundary segments
        List<Line2D> boundaryLines = new ArrayList<>( 4 );
        for( int i = 0; i < bezierQuad.size() - 1; i++ )
        {
            boundaryLines.add( new Line2D.Double( bezierQuad.get( i ), bezierQuad.get( i + 1 ) ) );
        }
        boundaryLines.add( new Line2D.Double( bezierQuad.get( 0 ), bezierQuad.get( 3 ) ) );

        // find extreme y points
        int upSidePoint = Integer.MAX_VALUE;
        int downSidePoint = 0;
        for( Point p : bezierQuad )
        {
            if( p.y > downSidePoint )
            {
                downSidePoint = p.y;
            }
            if( p.y < upSidePoint )
            {
                upSidePoint = p.y;
            }
        }

        // find first level after upSidePoint
        int upSideLevelInd = 0;
        for( int i = 0; i < maxLevel; i++ )
        {
            if( ( graphSpaceLevels.get( i ).get( 0 ).y + graphSpaceLevels.get( i ).get( 0 ).height ) > upSidePoint )
            {
                upSideLevelInd = i;
                break;
            }
        }
        // find last level before downSidePoint
        int downSideLevelInd = maxLevel - 1;
        for( int i = maxLevel - 1; i >= 0; i-- )
        {
            if( graphSpaceLevels.get( i ).get( 0 ).y < downSidePoint )
            {
                downSideLevelInd = i;
                break;
            }
        }

        // TODO: optimization
        // check intersections
        for( int i = upSideLevelInd; i <= downSideLevelInd; i++ )
        {
            if( getLevel( edge.from ) - 1 == i || getLevel( edge.to ) - 1 == i )
            {
                continue;
            }

            List<Integer> pretenders = new LinkedList<>();

            Rectangle testRect = new Rectangle( graphSpaceLevels.get( i ).get( 0 ) );
            testRect.x = 0; // TODO: check
            testRect.width = Integer.MAX_VALUE;

            /*three possibilities of leftmost:
             * 1) intersection of upper boundary and segment
             * 2) point inside
             * 3) intersection of lower boundary and segment
             */

            // point inside
            for( Point p : bezierQuad )
            {
                if( testRect.contains( p ) )
                {
                    pretenders.add( p.x );
                }
            }

            // intersection of upper boundary/lower boundary and segment
            for( Line2D segment : boundaryLines )
            {
                if( testRect.intersectsLine( segment ) )
                {
                    // TODO: check
                    // upper
                    if( segment.intersectsLine( new Line2D.Double( testRect.x, testRect.y, testRect.x + testRect.width, testRect.y ) ) )
                    {
                        int xIntersection = (int) ( ( testRect.y - segment.getY1() ) * ( segment.getX2() - segment.getX1() )
                                / ( segment.getY2() - segment.getY1() ) + segment.getX1() );
                        pretenders.add( xIntersection );
                    }

                    // lower
                    if( segment.intersectsLine( new Line2D.Double( testRect.x, testRect.y + testRect.height, testRect.x + testRect.width,
                            testRect.y + testRect.height ) ) )
                    {
                        int xIntersection = (int) ( ( ( testRect.y + testRect.height ) - segment.getY1() )
                                * ( segment.getX2() - segment.getX1() ) / ( segment.getY2() - segment.getY1() ) + segment.getX1() );
                        pretenders.add( xIntersection );
                    }
                }
            }

            // find leftmost and rightmost x
            int leftMost = Integer.MAX_VALUE;
            int rightMost = Integer.MIN_VALUE;
            for( int x : pretenders )
            {
                if( x < leftMost )
                {
                    leftMost = x;
                }
                if( x > rightMost )
                {
                    rightMost = x;
                }
            }

            Rectangle bannedRect = new Rectangle( leftMost, testRect.y, rightMost - leftMost, testRect.height );
            graphSpaceLevels.get( i ).add( bannedRect );
        }
    }


    protected List<Point> computeBezierTrio(List<Point> controlPoints, int ind, double coefLength)
    {
        Line2D parLine = new Line2D.Double( controlPoints.get( ind ), controlPoints.get( ind + 2 ) );
        Line2D seg1 = new Line2D.Double( controlPoints.get( ind ), controlPoints.get( ind + 1 ) );
        Line2D seg2 = new Line2D.Double( controlPoints.get( ind + 1 ), controlPoints.get( ind + 2 ) );

        double lengthParLine = Math.sqrt( Math.pow( parLine.getX1() - parLine.getX2(), 2 )
                + Math.pow( parLine.getY1() - parLine.getY2(), 2 ) );
        double lengthSeg1 = Math.sqrt( Math.pow( seg1.getX1() - seg1.getX2(), 2 ) + Math.pow( seg1.getY1() - seg1.getY2(), 2 ) );
        double lengthSeg2 = Math.sqrt( Math.pow( seg2.getX1() - seg2.getX2(), 2 ) + Math.pow( seg2.getY1() - seg2.getY2(), 2 ) );

        // control 1
        int dy1 = (int) ( seg1.getY2() - parLine.getY2() );
        int dx1 = (int) ( seg1.getX2() - parLine.getX2() );

        // move parLine to end seg1
        Line2D controlLine1 = new Line2D.Double( parLine.getX1() + dx1, parLine.getY1() + dy1, parLine.getX2() + dx1, parLine.getY2() + dy1 );
        double k1 = lengthSeg1 * coefLength / lengthParLine;

        // compression
        int dyParLine = (int) ( parLine.getY2() - parLine.getY1() );
        int dxParLine = (int) ( parLine.getX2() - parLine.getX1() );
        Point2D endSeg1 = controlLine1.getP2();
        controlLine1.setLine( new Point2D.Double( endSeg1.getX() - k1 * dxParLine, endSeg1.getY() - k1 * dyParLine ), endSeg1 );


        // control 2
        int dy2 = (int) ( seg2.getY1() - parLine.getY1() );
        int dx2 = (int) ( seg2.getX1() - parLine.getX1() );

        // move parLine to end seg1
        Line2D controlLine2 = new Line2D.Double( parLine.getX1() + dx2, parLine.getY1() + dy2, parLine.getX2() + dx2, parLine.getY2() + dy2 );
        double k2 = lengthSeg2 * coefLength / lengthParLine;

        // compression
        Point2D startSeg2 = controlLine2.getP1();
        controlLine2.setLine( startSeg2, new Point2D.Double( startSeg2.getX() + k2 * dxParLine, startSeg2.getY() + k2 * dyParLine ) );

        // add points to result
        List<Point> resultTrio = new ArrayList<>( 3 );
        resultTrio.add( new Point( (int)controlLine1.getP1().getX(), (int)controlLine1.getP1().getY() ) );
        resultTrio.add( controlPoints.get( ind + 1 ) );
        resultTrio.add( new Point( (int)controlLine2.getP2().getX(), (int)controlLine2.getP2().getY() ) );

        return resultTrio;
    }


    protected boolean computeControlPoints(Point start, Point end, List<Rectangle> boxes, List<Line2D> lArray, List<Point> controlPoints)
    {
        if( lineFits( start, end, lArray ) )
        {
            return true;
        }

        Point p = computeLineSplit( start, end, boxes, lArray );
        controlPoints.add( p );

        boolean result = computeControlPoints( start, p, boxes, lArray, controlPoints );
        result &= computeControlPoints( p, end, boxes, lArray, controlPoints );

        return result;
    }


    protected Point computeLineSplit(Point start, Point end, List<Rectangle> boxes, List<Line2D> lArray)
    {
        // find index first used L
        int startIndex = 0;
        while( lArray.get( startIndex ).getY1() <= start.y )
        {
            startIndex++;
        }

        // find index last used L
        int endIndex = lArray.size() - 1;
        while( lArray.get( endIndex ).getY1() > end.y )
        {
            endIndex--;
        }

        // finds the L segment that is the furthest from the (q, s) line.
        // p is the one of the two endpoints of the subdivision segment that is closer to the (q, s) line.
        Line2D edgeSegment = new Line2D.Double( start, end );
        Point2D p = null;
        for( int ind = startIndex; ind <= endIndex; ind++ )
        {
            if( !edgeSegment.intersectsLine( lArray.get( ind ) ) )
            {
                if( edgeSegment.ptSegDist( lArray.get( ind ).getP1() ) < edgeSegment.ptSegDist( lArray.get( ind ).getP2() ) )
                {
                    if( p == null || edgeSegment.ptSegDist( lArray.get( ind ).getP1() ) > edgeSegment.ptSegDist( p ) )
                    {
                        p = lArray.get( ind ).getP1();
                    }
                }
                else
                {
                    if( p == null || edgeSegment.ptSegDist( lArray.get( ind ).getP2() ) > edgeSegment.ptSegDist( p ) )
                    {
                        p = lArray.get( ind ).getP2();
                    }
                }
            }
        }

        return new Point( (int)p.getX(), (int)p.getY() );
    }

    // checks if the line defined by q and s lies entirely inside the feasible region
    private boolean lineFits(Point start, Point end, List<Line2D> lArray)
    {
        // find index first used L
        int startIndex = 0;
        while( lArray.get( startIndex ).getY1() <= start.y )
        {
            startIndex++;
        }

        // find index last used L
        int endIndex = lArray.size() - 1;
        while( lArray.get( endIndex ).getY1() > end.y )
        {
            endIndex--;
        }

        // if all line in LArray intersects edgeSegment => can draw edgeSegment directly
        Line2D edgeSegment = new Line2D.Double( start, end );
        for( int ind = startIndex; ind <= endIndex; ind++ )
        {
            if( !edgeSegment.intersectsLine( lArray.get( ind ) ) )
            {
                return false;
            }
        }

        return true;
    }



    // path always downward
    protected List<Rectangle> computeBoxes(Edge edge, Graph graph)
    {
        List<Rectangle> boxes = (List<Rectangle>)edge.data;

        // refresh free space after layout last edge
        updateDummiesBoxes( boxes, (List<List<Rectangle>>)graph.data );

        // add half edge.from node
        Rectangle startRegion = getRegionToNode( edge.from, graph );
        startRegion.y += startRegion.height / 2;
        startRegion.height /= 2;
        boxes.add( 0, startRegion );

        // add boxes between levels
        int endLevel = getLevel( edge.to );
        int startLevel = getLevel( edge.from );
        for( int currentLevel = startLevel; currentLevel < endLevel; currentLevel++ )
        {
            Rectangle regionBetweenLevels = getRegionBetweenLevels( currentLevel, graph );
            // add between box for dummies
            boxes.add( ( currentLevel - startLevel ) * 2 + 1, regionBetweenLevels );
        }

        // add half edge.to node
        Rectangle endRegion = getRegionToNode( edge.to, graph );
        endRegion.height /= 2;
        boxes.add( endRegion );

        return new ArrayList<>( boxes );
    }

    // need to reduce the size of the boxes
    protected void updateDummiesBoxes(List<Rectangle> boxes, List<List<Rectangle>> graphBannedSpaceLevels)
    {
        // TODO: const
        final int indent = 15;

        // leveling
        int shift = 0;
        while( boxes.get( 0 ).y != graphBannedSpaceLevels.get( shift ).get( 0 ).y )
        {
            shift++;
        }

        for( int level = 0; level < boxes.size(); level++ )
        {
            Rectangle curFreeBox = boxes.get( level );
            int startX = curFreeBox.x;
            int endX = curFreeBox.x + curFreeBox.width;

            List<Rectangle> curLevelBannedBoxes = graphBannedSpaceLevels.get( level + shift );

            if( curLevelBannedBoxes.size() > 1 )
            {
                Collections.sort( curLevelBannedBoxes, new Comparator<Rectangle>()
                {
                    @Override
                    public int compare(Rectangle r1, Rectangle r2)
                    {
                        if( r1.x == Integer.MIN_VALUE )
                        {
                            return -1;
                        }
                        if( r2.x == Integer.MIN_VALUE )
                        {
                            return 1;
                        }
                        return r1.x - r2.x;
                    }
                } );

                int averageWidth = 0;
                for( Rectangle rec : curLevelBannedBoxes )
                {
                    averageWidth += rec.width;
                }
                averageWidth = (int) ( (double)averageWidth / ( curLevelBannedBoxes.size() - 1 ) );

                // find left bound
                Iterator<Rectangle> boxItr = curLevelBannedBoxes.iterator();
                boxItr.next();
                int findingInterval = 2 * indent + averageWidth;
                int tmpStartX = startX;
                while( tmpStartX < endX - findingInterval )
                {
                    Rectangle curBox = null;
                    if( boxItr.hasNext() )
                    {
                        curBox = boxItr.next();
                    }
                    else
                    {
                        break;
                    }
                    if( curBox.x < tmpStartX + findingInterval )
                    {
                        tmpStartX = ( tmpStartX > curBox.x + curBox.width ) ? tmpStartX : curBox.x + curBox.width;
                    }
                    else
                    {
                        break;
                    }
                }

                if( tmpStartX < endX - findingInterval )
                {
                    curFreeBox.x = tmpStartX + indent;
                }
                else
                {
                    // TODO:
                    throw new RuntimeException( "no space for path" );
                }
            }
        }
    }


    // TODO: without crossing another edges, all width
    protected Rectangle getRegionBetweenLevels(int currentLevel, Graph graph)
    {
        int upSide = 0;
        int downSide = Integer.MAX_VALUE;
        for( Node node : graph.nodeList )
        {
            if( getLevel( node ) == currentLevel )
            {
                if( node.y + node.height > upSide )
                {
                    upSide = node.y + node.height;
                }
            }
            else if( getLevel( node ) == currentLevel + 1 )
            {
                if( node.y < downSide )
                {
                    downSide = node.y;
                }
            }
        }

        Rectangle region = new Rectangle( 0, upSide, Integer.MAX_VALUE, downSide - upSide );

        return region;
    }


    protected void createStraightPath(Edge edge)
    {
        // TODO choose correct ports

        // TODO: simple path from center nodes yet
        Node fromNode = edge.from;
        Node toNode = edge.to;

        Point start = new Point( fromNode.x + fromNode.width / 2, fromNode.y + fromNode.height / 2 );
        Point end = new Point( toNode.x + toNode.width / 2, toNode.y + toNode.height / 2 );

        edge.path.addPoint( start.x, start.y, Path.LINE_TYPE );
        edge.path.addPoint( end.x, end.y, Path.LINE_TYPE );

        // add edge box to data for later use
        Rectangle box = new Rectangle( start.x, start.y, end.x - start.x, end.y - start.y );
        edge.data = box;
    }


    /** shift in midway parallel edge, only for neighboring levels */
    protected Path shiftParallelMidway(Edge masterEdge, Edge actualEdge, int index, Graph graph)
    {
        Path masterPath = masterEdge.path;
        Path newPath = new Path();

        // calc the span between parallel edges
        int inEdgeNum = 0;
        List<Edge> edgeIn = graph.getEdges( masterEdge.from );
        for( Edge e : edgeIn )
        {
            if( e.from == masterEdge.from )
            {
                inEdgeNum++;
                if( e.slaves != null )
                    inEdgeNum += e.slaves.size();
            }
        }

        int outEdgeNum = 0;
        List<Edge> edgeOut = graph.getEdges( masterEdge.to );
        for( Edge e : edgeOut )
        {
            if( e.to == masterEdge.to )
            {
                outEdgeNum++;
                if( e.slaves != null )
                    outEdgeNum += e.slaves.size();
            }
        }

        float span = 5;
        // center coordinate of parallel edges neighboring levels
        int center;
        int distanceThroughLevel;
        int diffNodes;
        int numberParallelEdges = masterEdge.slaves.size() + 1;
        if( verticalOrientation )
        {
            span = Math.max( gridX, Math.min( masterEdge.from.width / ( inEdgeNum + 1 ), masterEdge.to.width / ( outEdgeNum + 1 ) ) );
            distanceThroughLevel = Math.abs( masterEdge.from.y - masterEdge.to.y );
            diffNodes = masterEdge.from.x - masterEdge.to.x;
            center = ( masterEdge.from.x + masterEdge.to.x ) / 2 + ( masterEdge.from.width + masterEdge.to.width ) / 4;
        }
        else
        {
            span = Math.max( gridY, Math.min( masterEdge.from.height / ( inEdgeNum + 1 ), masterEdge.to.height / ( outEdgeNum + 1 ) ) );
            distanceThroughLevel = Math.abs( masterEdge.from.x - masterEdge.to.x );
            diffNodes = masterEdge.from.y - masterEdge.to.y;
            center = ( masterEdge.from.y + masterEdge.to.y ) / 2 + ( masterEdge.from.height + masterEdge.to.height ) / 4;
        }
        double straightLength = Math.sqrt( distanceThroughLevel * distanceThroughLevel + diffNodes * diffNodes );

        // TODO: magic number 2
        int adaptedDistanceBetweenMidpoint = (int)Math.round( virtualNodesDistance * straightLength * 2 / distanceThroughLevel );
        int startMidway;
        if( numberParallelEdges % 2 != 0 )
        {
            startMidway = center - ( numberParallelEdges / 2 ) * adaptedDistanceBetweenMidpoint;
        }
        else
        {
            startMidway = center - ( ( numberParallelEdges - 1 ) * adaptedDistanceBetweenMidpoint ) / 2;
        }

        // add midway point
        int offset = Math.round( span * ( index ) );
        int midpoint;
        if( verticalOrientation )
        {
            Point start = actualEdge.from.findPort(masterPath.xpoints[0] + offset, masterPath.ypoints[0], actualEdge);
            newPath.addPoint(start.x, start.y, masterPath.pointTypes[0]);

            midpoint = ( masterPath.ypoints[0] + masterPath.ypoints[1] ) / 2;
            newPath.addPoint(startMidway + adaptedDistanceBetweenMidpoint * index, midpoint, Path.LINE_TYPE);

            Point end = actualEdge.to.findPort(masterPath.xpoints[masterPath.npoints - 1] + offset,
                    masterPath.ypoints[masterPath.npoints - 1], actualEdge);
            newPath.addPoint(end.x, end.y, masterPath.pointTypes[masterPath.npoints - 1]);
        }
        else
        {
            Point start = actualEdge.from.findPort(masterPath.xpoints[0], masterPath.ypoints[0] + offset, actualEdge);
            newPath.addPoint(start.x, start.y, masterPath.pointTypes[0]);

            midpoint = ( masterPath.xpoints[0] + masterPath.xpoints[1] ) / 2;
            newPath.addPoint(midpoint, startMidway + adaptedDistanceBetweenMidpoint * index, Path.LINE_TYPE);

            Point end = actualEdge.to.findPort(masterPath.xpoints[masterPath.npoints - 1],
                    masterPath.ypoints[masterPath.npoints - 1] + offset, actualEdge);
            newPath.addPoint(end.x, end.y, masterPath.pointTypes[masterPath.npoints - 1] );
        }

        return newPath;
    }

    /** computing path for Bezier spline */
    private void computingControlPoints(Edge edge)
    {
        Path path = edge.path;
        Path bezierPath = new Path();

        // TODO: replace in selfLoopLayouter
        // if edge self loop
        if( edge.from == edge.to )
        {
            bezierPath.addPoint( path.xpoints[0], path.ypoints[0], Path.LINE_TYPE );

            int dx;
            int dy;

            int Lx;
            int Ly;

            // two orientations of arrow's direction
            if( path.xpoints[2] == path.xpoints[1] )
            {
                dx = ( path.xpoints[3] - path.xpoints[2] ) / 2;
                dy = ( path.ypoints[1] - path.ypoints[2] ) / 2;

                // length of the guide line
                Lx = (int)Math.round( dx * 4.0 / 3 * Math.tan( Math.PI / 8 ) );
                Ly = (int)Math.round( dy * 4.0 / 3 * Math.tan( Math.PI / 8 ) );

                // first arc
                bezierPath.addPoint( path.xpoints[2] + dx, path.ypoints[2] + 2 * dy, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] + dx - Lx, path.ypoints[2] + 2 * dy, Path.CUBIC_TYPE );
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy + Ly, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy, Path.LINE_TYPE );

                // second arc
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy - Ly, Path.CUBIC_TYPE );
                bezierPath.addPoint( path.xpoints[2] + dx - Lx, path.ypoints[2], Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] + dx, path.ypoints[2], Path.LINE_TYPE );

                // third arc
                bezierPath.addPoint( path.xpoints[2] + dx, path.ypoints[2], Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] + dx + Lx, path.ypoints[2], Path.CUBIC_TYPE );
                bezierPath.addPoint( path.xpoints[2] + 2 * dx, path.ypoints[2] + dy - Ly, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] + 2 * dx, path.ypoints[2] + dy, Path.LINE_TYPE );
            }
            else
            {
                dx = ( path.xpoints[2] - path.xpoints[1] ) / 2;
                dy = ( path.ypoints[3] - path.ypoints[2] ) / 2;

                // length of the guide line
                Lx = (int)Math.round( dx * 4.0 / 3 * Math.tan( Math.PI / 8 ) );
                Ly = (int)Math.round( dy * 4.0 / 3 * Math.tan( Math.PI / 8 ) );

                // first arc
                bezierPath.addPoint( path.xpoints[2] - 2 * dx, path.ypoints[2] + dy, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] - 2 * dx, path.ypoints[2] + dy - Ly, Path.CUBIC_TYPE );
                bezierPath.addPoint( path.xpoints[2] - dx - Lx, path.ypoints[2], Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] - dx, path.ypoints[2], Path.LINE_TYPE );

                // second arc
                bezierPath.addPoint( path.xpoints[2] - dx, path.ypoints[2], Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] - dx + Lx, path.ypoints[2], Path.CUBIC_TYPE );
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy - Ly, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy, Path.LINE_TYPE );

                // third arc
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2], path.ypoints[2] + dy + Ly, Path.CUBIC_TYPE );
                bezierPath.addPoint( path.xpoints[2] - dx + Lx, path.ypoints[2] + 2 * dy, Path.LINE_TYPE );
                bezierPath.addPoint( path.xpoints[2] - dx, path.ypoints[2] + 2 * dy, Path.LINE_TYPE );
            }

            bezierPath.addPoint( path.xpoints[4], path.ypoints[4], Path.LINE_TYPE );

            edge.path = bezierPath;

            return;
        }
        if( verticalOrientation )
        {
            // straight line for arrow in Bezier spline
            int distArrow = ( path.ypoints[1] - path.ypoints[0] ) / 4;
            if( path.xpoints[0] != path.xpoints[1] )
            {
                bezierPath.addPoint( path.xpoints[0], path.ypoints[0], Path.LINE_TYPE );
                path.ypoints[0] += distArrow;
            }

            for( int i = 0; i < path.npoints - 1; i++ )
            {
                if( path.xpoints[i] == path.xpoints[i + 1] )
                {
                    // straight section
                    bezierPath.addPoint( path.xpoints[i], path.ypoints[i], Path.LINE_TYPE );
                    // miss the points on a same line
                    int start = i;
                    while( ( i < path.npoints - 3 ) && ( path.xpoints[start] == path.xpoints[i + 2] ) )
                    {
                        i++;
                    }
                }
                else
                {
                    // need control points
                    int dy = ( path.ypoints[i + 1] - path.ypoints[i] ) / 2;
                    bezierPath.addPoint( path.xpoints[i], path.ypoints[i], Path.LINE_TYPE );
                    bezierPath.addPoint( path.xpoints[i], path.ypoints[i] + dy, Path.CUBIC_TYPE );
                    bezierPath.addPoint( path.xpoints[i + 1], path.ypoints[i + 1] - dy, Path.LINE_TYPE );
                }
            }

            // straight line for arrow in Bezier spline
            if( ( bezierPath.npoints > 1 ) && bezierPath.pointTypes[bezierPath.npoints - 2] == Path.CUBIC_TYPE )
            {
                bezierPath.ypoints[bezierPath.npoints - 1] -= distArrow;
                bezierPath.addPoint( path.xpoints[path.npoints - 1], path.ypoints[path.npoints - 1] - distArrow, Path.LINE_TYPE );
            }
            
            adjustArrowTipConnection(bezierPath);
        }
        else
        {
            // straight line for arrow in Bezier spline
            int distArrow = ( path.xpoints[1] - path.xpoints[0] ) / 4;
            if( path.ypoints[0] != path.ypoints[1] )
            {
                bezierPath.addPoint( path.xpoints[0], path.ypoints[0], Path.LINE_TYPE );
                path.xpoints[0] += distArrow;
            }

            for( int i = 0; i < path.npoints - 1; i++ )
            {
                if( path.ypoints[i] == path.ypoints[i + 1] )
                {
                    // straight section
                    bezierPath.addPoint( path.xpoints[i], path.ypoints[i], Path.LINE_TYPE );
                    // miss the points on a same line
                    int start = i;
                    while( ( i < path.npoints - 3 ) && ( path.ypoints[start] == path.ypoints[i + 2] ) )
                    {
                        i++;
                    }
                }
                else
                {
                    int dx = ( path.xpoints[i + 1] - path.xpoints[i] ) / 2;

                    bezierPath.addPoint( path.xpoints[i], path.ypoints[i], Path.LINE_TYPE );
                    bezierPath.addPoint( path.xpoints[i] + dx, path.ypoints[i], Path.CUBIC_TYPE );
                    bezierPath.addPoint( path.xpoints[i + 1] - dx, path.ypoints[i + 1], Path.LINE_TYPE );
                }
            }

            // straight line for arrow in Bezier spline
            if( ( bezierPath.npoints > 1 ) && bezierPath.pointTypes[bezierPath.npoints - 2] == Path.CUBIC_TYPE )
            {
                bezierPath.xpoints[bezierPath.npoints - 1] -= distArrow;
                bezierPath.addPoint( path.xpoints[path.npoints - 1] - distArrow, path.ypoints[path.npoints - 1], Path.LINE_TYPE );
            }
        }

        bezierPath.addPoint( path.xpoints[path.npoints - 1], path.ypoints[path.npoints - 1], Path.LINE_TYPE );

        edge.path = bezierPath;

        // postprocessing
        // if edge is parallel and in neighboring levels
        if( Math.abs( getLevel( edge.from ) - getLevel( edge.to ) ) == 1 && bezierPath.npoints == 9 )
        {
            int pointDist = ( verticalOrientation ) ? bezierPath.ypoints[4] - bezierPath.ypoints[0] : bezierPath.xpoints[4]
                    - bezierPath.xpoints[0];
            int levelDist = ( verticalOrientation ) ? layerDeltaY : layerDeltaX;
            if( ( pointDist <= levelDist / 2 ) )
            { // TODO
              // change tangent line in midpoint
                int pointDiff = ( verticalOrientation ) ? bezierPath.xpoints[8] - bezierPath.xpoints[0] : bezierPath.ypoints[8]
                        - bezierPath.ypoints[0];
                if( pointDiff != 0 )
                {
                    // create Bezier splines
                    Path newPath = new Path();
                    // it is not vertical edge, remove midpoint
                    Point p0 = new Point( ( verticalOrientation ) ? bezierPath.xpoints[1] : bezierPath.ypoints[1], ( verticalOrientation )
                            ? bezierPath.ypoints[1] : bezierPath.xpoints[1] );
                    Point p2 = new Point( ( verticalOrientation ) ? bezierPath.xpoints[7] : bezierPath.ypoints[7], ( verticalOrientation )
                            ? bezierPath.ypoints[7] : bezierPath.xpoints[7] );

                    Point p1 = new Point( ( verticalOrientation ) ? bezierPath.xpoints[4] : bezierPath.ypoints[4], ( verticalOrientation )
                            ? bezierPath.ypoints[4] : bezierPath.xpoints[4] );

                    // control segment length of midpoint
                    final float koefLengthControl = (float) ( 1.0 / 8 );

                    Point controlPoint1 = new Point( Math.round( p1.x - koefLengthControl * ( p2.x - p0.x ) ), Math.round( p1.y
                            - koefLengthControl * ( p2.y - p0.y ) ) );
                    Point controlPoint2 = new Point( Math.round( p1.x - koefLengthControl * ( p0.x - p2.x ) ), Math.round( p1.y
                            - koefLengthControl * ( p0.y - p2.y ) ) );

                    newPath.addPoint( bezierPath.xpoints[0], bezierPath.ypoints[0], Path.LINE_TYPE );
                    newPath.addPoint( bezierPath.xpoints[1], bezierPath.ypoints[1], Path.LINE_TYPE );
                    newPath.addPoint( bezierPath.xpoints[2], bezierPath.ypoints[2], Path.CUBIC_TYPE );

                    if( verticalOrientation )
                    {
                        newPath.addPoint( controlPoint1.x, controlPoint1.y, Path.LINE_TYPE );
                        newPath.addPoint( p1.x, p1.y, Path.LINE_TYPE );
                        newPath.addPoint( controlPoint2.x, controlPoint2.y, Path.CUBIC_TYPE );
                    }
                    else
                    {
                        newPath.addPoint( controlPoint1.y, controlPoint1.x, Path.LINE_TYPE );
                        newPath.addPoint( p1.y, p1.x, Path.LINE_TYPE );
                        newPath.addPoint( controlPoint2.y, controlPoint2.x, Path.CUBIC_TYPE );
                    }

                    newPath.addPoint( bezierPath.xpoints[6], bezierPath.ypoints[6], Path.LINE_TYPE );
                    newPath.addPoint( bezierPath.xpoints[7], bezierPath.ypoints[7], Path.LINE_TYPE );
                    newPath.addPoint( bezierPath.xpoints[8], bezierPath.ypoints[8], Path.CUBIC_TYPE );

                    edge.path = newPath;
                }

                return;
            }
        }
    }
    
    
    /**
     * Spike solution:
     * shifts path in order to free space for arrow tip aligned vertically<br>
     * So edge would connect to arrow tip from the top:
     *        __        
     *         _|_                 ___ 
     *         \ /   instead of  __\ /  
     *          V                   V
     *TODO: rework computingControlPoints method to account this situation
     *@see also bug #6877
     */
    public void adjustArrowTipConnection(Path path)
    {
        int shift = 10;
        
        boolean right = path.xpoints[path.npoints - 1] > path.xpoints[0];
        double middlePoint = (path.xpoints[path.npoints - 1] + path.xpoints[0]) / 2;
                
        for (int i=0; i<path.npoints; i++)
        {
            if (path.xpoints[i] > middlePoint == right)
                path.ypoints[i] -= shift;
        }
    }
    

    public SelfLoopLayouter selfLoopLayouter = new SelfLoopLayouter();

    @Override
	public void layoutPath(Graph graph, Edge edge, LayoutJobControl jobControl)
    {
        if( ! ( pathLayouter instanceof HierarchicPathLayouter ) )
        {
            pathLayouter.layoutPath( graph, edge, jobControl );
            return;
        }

        new DiagonalPathLayouter().layoutPath( graph, edge, pathWeighter );
    }

    // Null element in the result means "dummy"
    protected List<List<Edge>> getOrderedEdgeSet(Graph graph, Node n, boolean in)
    {
        List<List<Edge>> result = new ArrayList<>();

        List<Edge> edges = new ArrayList<>();

        int level = getLevel( n ) + ( ( in ) ? -2 : 0 );

        if( level < 0 || level >= maxLevel )
            return result;

        for( Node u : levelNodes.get( level ) )
        {
            Edge edge = ( in ) ? graph.getEdge( u, n ) : graph.getEdge( n, u );
            if( edge == null )
                continue;

            if( isFixed( edge, n ) )
            {
                if( !edges.isEmpty() )
                    result.add( edges );
                ArrayList<Edge> singleEdge = new ArrayList<>();
                singleEdge.add( edge );
                result.add( singleEdge );
                edges = new ArrayList<>();
            }
            else
            {
                edges.add( edge );
            }

            if( !isDummy( n ) && edge.slaves != null )
            {
                for( Edge slave : edge.slaves )
                {
                    edges.add( slave );
                }
            }
            if( isDummy( n ) && edge.data != null )
            {
                int slavesCount = ( (List<?>)edge.data ).size();
                for( int i = 0; i < slavesCount; i++ )
                {
                    edges.add( null );
                }
            }
        }
        if( !edges.isEmpty() )
            result.add( edges );
        return result;
    }

    protected boolean isFixed(Edge edge, Node node)
    {
        Point port = node.findPort( node.x, node.y, edge );
        if( port.x == node.x && port.y == node.y )
            return false;
        return true;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Properties
    //

    protected boolean verticalOrientation = false;

    public boolean isVerticalOrientation()
    {
        return verticalOrientation;
    }

    public void setVerticalOrientation(boolean verticalOrientation)
    {
        this.verticalOrientation = verticalOrientation;
    }

    protected boolean hoistNodes = false;

    public boolean isHoistNodes()
    {
        return hoistNodes;
    }

    public void setHoistNodes(boolean hoistNodes)
    {
        this.hoistNodes = hoistNodes;
    }

    protected int layerOrderIterationNum = 10;

    public int getLayerOrderIterationNum()
    {
        return layerOrderIterationNum;
    }

    public void setLayerOrderIterationNum(int layerOrderIterationNum)
    {
        this.layerOrderIterationNum = layerOrderIterationNum;
    }

    public static final int STRAIGHTEN_DEFAULT = 0;

    public static final int STRAIGHTEN_FORCE_DIRECTED = 1;

    protected int straightenMethod = STRAIGHTEN_DEFAULT;

    public int getStraightenMethod()
    {
        return straightenMethod;
    }

    public void setStraightenMethod(int straightenMethod)
    {
        this.straightenMethod = straightenMethod;
    }

    protected int straightenIterationNum = 100;

    public int getStraightenIterationNum()
    {
        return straightenIterationNum;
    }

    public void setStraightenIterationNum(int straightenIterationNum)
    {
        this.straightenIterationNum = straightenIterationNum;
    }

    private boolean processNeighbours = false;

    public boolean isProcessNeighbours()
    {
        return processNeighbours;
    }

    public void setProcessNeighbours(boolean processNeighbours)
    {
        this.processNeighbours = processNeighbours;
    }

    protected int virtualNodesDistance = 10;

    public int getVirtualNodesDistance()
    {
        return virtualNodesDistance;
    }

    public void setVirtualNodesDistance(int virtualNodesDistance)
    {
        this.virtualNodesDistance = virtualNodesDistance;
    }

    protected int layerDeltaX = 50;

    public int getLayerDeltaX()
    {
        return layerDeltaX;
    }

    public void setLayerDeltaX(int layerDeltaX)
    {
        this.layerDeltaX = layerDeltaX;
    }

    protected int layerDeltaY = 50;

    public int getLayerDeltaY()
    {
        return layerDeltaY;
    }

    public void setLayerDeltaY(int layerDeltaY)
    {
        this.layerDeltaY = layerDeltaY;
    }

    protected boolean splineEdges = false;

    public boolean isSplineEdges()
    {
        return splineEdges;
    }

    public void setSplineEdges(boolean splineEdges)
    {
        this.splineEdges = splineEdges;
    }

    protected int gridX = 1;

    protected int gridY = 1;

    // //////////////////////////////////////////////////////////////////////////

    public void initLayoutData(Graph graph)
    {
        for( Node n : graph.nodeList )
            n.data = new LayoutData();
    }

    protected void unmarkNodes(Graph graph)
    {
        for( Node n : graph.nodeList )
            ( (LayoutData)n.data ).marked = false;
    }

    protected void printNodeLevels(Graph graph)
    {
        for( Node node : graph.nodeList )
            System.out.print( node.name + ":" + ( (LayoutData)node.data ).level + ", " );
    }

    protected static int getLevel(Node node)
    {
        return ( (LayoutData)node.data ).level;
    }

    protected static void setLevel(Node node, int level)
    {
        ( (LayoutData)node.data ).level = level;
    }

    protected static int getUsageLevel(Node node)
    {
        return ( (LayoutData)node.data ).usageLevel;
    }

    protected static void setUsageLevel(Node node, int level)
    {
        ( (LayoutData)node.data ).usageLevel = level;
    }

    protected boolean isDummy(Node node)
    {
        return ( (LayoutData)node.data ).dummy;
    }

    protected boolean isMarked(Node node)
    {
        return ( (LayoutData)node.data ).marked;
    }

    protected int getBarycenter(Node node)
    {
        return ( (LayoutData)node.data ).barycenter;
    }

    protected int calcBarycenter(Graph graph, Node node, boolean doIn, boolean doOut, boolean processSize)
    {
        int sum = 0;
        int n = 0;

        int center = 0;
        for( Edge edge : graph.getEdges( node ) )
        {
            Node u = null;

            if( edge.to == node && doIn )
                u = edge.from;
            else if( edge.from == node && doOut )
                u = edge.to;

            if( u != null )
            {
                if( verticalOrientation )
                    center = u.x;
                else
                    center = u.y;

                if( processSize )
                {
                    if( verticalOrientation )
                        center += u.width / 2;
                    else
                        center += u.height / 2;
                }

                sum += center;
                n++;

            }
        }

        if( processSize && processNeighbours )
        {
            // get neigthberhood nodes
            List<Node> level = levelNodes.get( getLevel( node ) - 1 );
            int i;
            for( i = 0; i < level.size(); i++ )
            {
                if( level.get( i ) == node )
                    break;
            }

            // process left node
            if( i > 0 )
            {
                Node u = level.get( i - 1 );
                if( verticalOrientation )
                    center = u.x + u.width / 2;
                else
                    center = u.y + u.height / 2;

                sum += center;
                n++;
            }

            // process right node node
            if( i < level.size() - 1 )
            {
                Node u = level.get( i + 1 );
                if( verticalOrientation )
                    center = u.x + u.width / 2;
                else
                    center = u.y + u.height / 2;

                sum += center;
                n++;
            }
        }

        if( n != 0 )
            sum = sum / n;
        else
        {
            if( verticalOrientation )
                sum = node.x + ( processSize ? node.width / 2 : 0 );
            else
                sum = node.y + ( processSize ? node.height / 2 : 0 );
        }

        ( (LayoutData)node.data ).barycenter = sum;
        return sum;
    }

    // //////////////////////////////////////////////////////////////////////////

    transient protected List<Edge> selfLoops;

    protected void normalise(Graph graph)
    {
        // remove self loops
        selfLoops = new ArrayList<>();

        for( int i = 0; i < graph.edgeList.size(); i++ )
        {
            Edge edge = graph.edgeList.get( i );
            if( edge.from == edge.to )
            {
                edge.path = null;
                selfLoops.add( edge );
                graph.removeEdge( edge );
                i--;
            }
        }

        // Break any cycles in the graph by reversing some edges
        unmarkNodes( graph );
        for( Node node : graph.nodeList )
            if( ! ( (LayoutData)node.data ).marked )
                breakCycles( graph, node );
    }

    protected void breakCycles(Graph graph, Node curr)
    {
        ( (LayoutData)curr.data ).marked = true;
        ( (LayoutData)curr.data ).picked = true;

        List<Edge> edges = graph.getEdges( curr );
        if( edges == null )
            return;

        for( int j = 0; j < edges.size(); j++ )
        {
            Edge edge = edges.get( j );
            if( edge.from == curr ) // get out edges
            {
                Node n = edge.to;
                if( ( (LayoutData)n.data ).picked )
                {
                    //Logger.info(cat, "reversed edge: " + edge + "<" + edge.slaves + ">");

                    graph.removeEdge( edge );
                    edge.reverseDirection();
                    graph.addEdge( edge );

                    j--;
                }
                else if( ! ( (LayoutData)n.data ).marked )
                {
                    j = 0;
                    breakCycles( graph, n );
                }
            }
        }

        ( (LayoutData)curr.data ).picked = false;
    }

    protected void assignNodeLevels(Graph graph)
    {
        // Do a topological sort on the meta-graph
        ArrayList<Node> topoSortedNodes = new ArrayList<>();
        Node metaRoot = makeMetaRoot( graph );
        unmarkNodes( graph );
        topoSort( graph, metaRoot, topoSortedNodes );
        graph.removeNode( metaRoot );
        topoSortedNodes.remove( metaRoot );

        // then assign levels to nodes.
        int toposize = topoSortedNodes.size();
        maxLevel = 0;
        for( int i = 0; i < toposize; ++i )
        {
            Node v = topoSortedNodes.get( i );
            int level = 0;

            List<Edge> edges = graph.getEdges( v );
            for( Edge edge : edges )
            {
                if( edge.to == v )
                {
                    Node u = edge.from;
                    if( getLevel( u ) > level )
                        level = getLevel( u );
                }
            }

            setLevel( v, level + 1 );
            if( level + 1 > maxLevel )
                maxLevel = level + 1;
        }

        // Hoist the nodes up to the maximum possible usage level.
        for( int i = topoSortedNodes.size() - 1; i >= 0; --i )
        {
            Node v = topoSortedNodes.get( i );
            int minUsageLevel = maxLevel;

            List<Edge> edges = graph.getEdges( v );
            boolean hasOutEdges = false;
            for( Edge edge : edges )
                if( edge.from == v )
                {
                    hasOutEdges = true;
                    break;
                }

            if( !hoistNodes && !hasOutEdges )
                minUsageLevel = getLevel( v );
            else
            {
                for( Edge edge : edges )
                {
                    if( edge.from == v )
                    {
                        Node u = edge.to;
                        int usageLevel = getUsageLevel( u ) - 1;
                        if( usageLevel < minUsageLevel )
                            minUsageLevel = usageLevel;
                    }
                }
            }

            setUsageLevel( v, minUsageLevel );
        }

        for( int i = 0; i < toposize; ++i )
        {
            Node v = topoSortedNodes.get( i );
            setLevel( v, getUsageLevel( v ) );
        }
    }

    public static void topoSort(Graph graph, Node curr, List<Node> topoSortedNodes)
    {
        ( (LayoutData)curr.data ).marked = true;

        List<Edge> edges = graph.getEdges( curr );
        for( Edge edge : edges )
        {
            if( edge.to == curr )
            {
                Node n = edge.from;
                if( ! ( (LayoutData)n.data ).marked )
                    topoSort( graph, n, topoSortedNodes );
            }
        }

        topoSortedNodes.add( curr );
    }

    protected void addDummies(Graph graph)
    {
        int dummyCount = 1;
        for( int i = 0; i < graph.nodeCount(); ++i )
        {
            Node to = graph.nodeList.get( i );
            if( ( (LayoutData)to.data ).dummy )
                continue;

            List<Edge> edges = graph.getEdges( to );
            for( int j = 0; j < edges.size(); j++ )
            {
                Edge edge = edges.get( j );
                // MUST to clear original path to avoid invalid edge restoration at layoutEdges
                edge.path = new Path();
                if( edge.to == to ) // get in edges
                {
                    if( ( (LayoutData)edge.from.data ).dummy )
                        continue;

                    if( getLevel( to ) > getLevel( edge.from ) + 1 )
                        j--;

                    // create dummy nodes
                    while( getLevel( to ) > getLevel( edge.from ) + 1 )
                    {
                        // create dummy node, assign the level and add it to the graph
                        Node from = edge.from;
                        Node dummy = new Node( "dummy_" + dummyCount );
                        dummyCount++;
                        graph.addNode( dummy );

                        dummy.data = new LayoutData();
                        setLevel( dummy, getLevel( from ) + 1 );
                        ( (LayoutData)dummy.data ).dummy = true;

                        // create edges and add them to the graph
                        Edge in = new Edge( from, dummy );
                        in.data = null;
                        in.reversed = edge.reversed;
                        if( edge.getAttribute( "inputPortName" ) != null )
                            in.setAttribute( "inputPortName", edge.getAttribute( "inputPortName" ) );
                        graph.addEdge( in );

                        Edge out = new Edge( dummy, to );
                        out.data = null;
                        out.reversed = edge.reversed;
                        if( edge.getAttribute( "outputPortName" ) != null )
                            out.setAttribute( "outputPortName", edge.getAttribute( "outputPortName" ) );
                        graph.addEdge( out );

                        // remove edge in the graph
                        graph.removeEdge( edge );

                        edge = out;
                    }
                }
            }
        }
    }

    protected void removeDummies(Graph graph)
    {
        for( int i = 0; i < graph.nodeCount(); ++i )
        {
            Node from = graph.nodeList.get( i );
            if( Util.isCompartment( from ) )
                continue;
            if( isDummy( from ) )
                continue;

            List<Edge> edges = graph.getEdges( from );
            for( int j = 0; j < edges.size(); j++ )
            {
                Edge edge = edges.get( j );

                // get out dummy edges
                if( edge.from == from && isDummy( edge.to ) )
                {
                    Path line = new Path();
                    line.addPoint( edge.path.xpoints[0], edge.path.ypoints[0] );
                    addDummyNodeToPath( edge.to, line, edge.path.xpoints[1], edge.path.ypoints[1] );

                    List<Node> path = new ArrayList<>();
                    path.add( edge.to );

                    Node to = edge.to;
                    int pathLen = 1;
                    while( true )
                    {
                        List<Edge> dEdges = graph.getEdges( to );
                        for( int d = 0; d < 2; d++ )
                        {
                            Edge e = dEdges.get( d );
                            if( e.from == to )
                            {
                                to = e.to;
                                if( isDummy( to ) )
                                {
                                    addDummyNodeToPath( e.to, line, e.path.xpoints[1], e.path.ypoints[1] );
                                }
                                else
                                    line.addPoint( e.path.xpoints[1], e.path.ypoints[1] );
                                break;
                            }
                        }

                        if( !isDummy( to ) )
                            break;

                        path.add( to );
                        pathLen++;
                    }

                    // restore original edge
                    Edge originalEdge = new Edge( from, to );
                    originalEdge.reversed = edge.reversed;
                    originalEdge.path = line;
                    // check master
                    Edge masterEdge = graph.getEdge( from, to, true );
                    if( masterEdge == null )
                    {
                        originalEdge.master = true;
                    }
                    else
                    {
                        originalEdge.master = false;
                        masterEdge.addSlave( originalEdge );
                    }
                    graph.addEdge( originalEdge );


                    // restore parallel edges
                    List<?> slaves = (List<?>)edge.data;
                    if( slaves != null )
                        for( Object slave : slaves )
                            graph.addEdge( (Edge)slave );

                    for( Node aPath : path )
                        graph.removeNode( aPath );

                    j -= pathLen;
                    if( j < 0 )
                        j = 0;
                }
            }
        }
    }

    private void addDummyNodeToPath(Node dummyNode, Path path, int x, int y)
    {
        int levelSize = levelSizes[getLevel( dummyNode ) - 1];

        if( levelSize > 20 )
        {
            int virtualPointDeltaY = verticalOrientation ? levelSize / 2 : 0;
            int virtualPointDeltaX = verticalOrientation ? 0 : levelSize / 2;

            path.addPoint( x - virtualPointDeltaX, y - virtualPointDeltaY );
            path.addPoint( x + virtualPointDeltaX, y + virtualPointDeltaY );
        }
        else
        {
            path.addPoint( x, y, Path.CUBIC_TYPE );
        }
    }

    // //////////////////////////////////////////////////////////////////////////

    transient protected int maxLevel;

    transient protected ArrayList<List<Node>> levelNodes;
    transient private int[] levelSizes = null;
    /**
     * Make and initialize levels.
     */
    protected void makeLevels(Graph graph)
    {
        // Find maximum level and node
        maxLevel = -1;
        Node maxLevelNode = null;

        for( Node n : graph.nodeList )
        {
            if( maxLevel < getLevel( n ) )
            {
                maxLevel = getLevel( n );
                maxLevelNode = n;
            }
        }

        // Make and initialize levels.
        levelNodes = new ArrayList<>( maxLevel );
        for( int j = 0; j < maxLevel; j++ )
            levelNodes.add( new ArrayList<Node>() );

        levelSizes = new int[maxLevel];

        unmarkNodes( graph );

        initialOrderNodes( graph, maxLevelNode ); // DFS order most the nodes

        for( Node n : graph.nodeList )
        {
            if( ! ( (LayoutData)n.data ).marked )
                initialOrderNodes( graph, n );
        }
    }

    public void initialOrderNodes(Graph graph, Node curr)
    {
        ( (LayoutData)curr.data ).marked = true;

        for( Edge edge : graph.getEdges( curr ) )
        {
            if( edge.to == curr ) // get in edges
            {
                Node n = edge.from;
                if( ! ( (LayoutData)n.data ).marked )
                    initialOrderNodes( graph, n );
            }
        }

        levelNodes.get( getLevel( curr ) - 1 ).add( curr );
    }

    // Do downwards barycentering on first pass, upwards on second, then average
    protected void orderNodes(Graph graph, int op, LayoutJobControl lJC)
    {
        boolean doup = true;
        boolean doin = op < 5;
        boolean doout = op > 3;

        if( doup )
            // Going upwards
            for( int i = 0; i < maxLevel; i++ )
            {
                orderLevel( graph, levelNodes.get( i ), doin, doout );
                if( lJC != null )
                {
                    lJC.done( ++operationsDone );
                    if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                        i = maxLevel;
                }
            }
        else
            // Going downwards
            for( int i = maxLevel - 1; i >= 0; i-- )
            {
                orderLevel( graph, levelNodes.get( i ), doin, doout );
                if( lJC != null )
                {
                    lJC.done( ++operationsDone );
                    if( lJC.getStatus() == JobControl.TERMINATED_BY_ERROR || lJC.getStatus() == JobControl.TERMINATED_BY_REQUEST )
                        i = -1;
                }
            }
        placeNodesInitial();
    }

    protected void orderLevel(Graph graph, List<Node> nodes, boolean doin, boolean doout)
    {
        // barycentric heuristic
        // TODO: compare to median sort

        for( Node node : nodes )
            calcBarycenter( graph, node, doin, doout, false );

        Collections.sort( nodes, new LevelComparator() );

        int levelcnt = nodes.size();
        for( int i = 0; i < levelcnt; i++ )
        {
            Node node = nodes.get( i );
            if( verticalOrientation )
                node.x = i * layerDeltaX;
            else
                node.y = i * layerDeltaY;
        }

        // transposition heuristic
        int level, w1, w2;
        for( int k = 0; k < 5; k++ )
        {
            boolean hasSwap = false;
            for( int i = 0; i < levelcnt - 1; i++ )
            {
                Node n1 = nodes.get( i );
                Node n2 = nodes.get( i + 1 );

                level = getLevel( n1 );
                w1 = calcLevelWeight( level, true, doout );
                swap( n1, n2 );
                w2 = calcLevelWeight( level, true, doout );

                if( w1 <= w2 )
                    swap( n1, n2 );
                else
                {
                    nodes.remove( n2 );
                    nodes.add( i, n2 );
                    hasSwap = true;
                    // Logger.info( log, "swap: " + n1 + " <-> " + n2);
                }
            }

            if( !hasSwap )
                break;
        }
    }

    protected void swap(Node n1, Node n2)
    {
        int t;
        if( verticalOrientation )
        {
            t = n1.x;
            n1.x = n2.x;
            n2.x = t;
        }
        else
        {
            t = n1.y;
            n1.y = n2.y;
            n2.y = t;
        }
    }

    public class LevelComparator implements Comparator<Node>
    {
        @Override
		public boolean equals(Object obj)
        {
            return this == obj;
        }

        @Override
		public int compare(Node o1, Node o2)
        {
            int r = getBarycenter( o1 ) - getBarycenter( o2 );
            if( r != 0 )
                return r;

            return 0;
        }
    }

    transient protected ArrayList<List<Edge>> levelEdges;

    protected void initLevelEdges(Graph graph)
    {
        levelEdges = new ArrayList<>( maxLevel );
        for( int i = 0; i < maxLevel; i++ )
        {
            ArrayList<Edge> edges = new ArrayList<>();
            levelEdges.add( edges );

            List<Node> nodes = levelNodes.get( i );
            for( Node node : nodes )
            {
                List<Edge> nEdges = graph.getEdges( node );
                for( Edge edge : nEdges )
                    if( edge.from == node )
                        edges.add( edge );
            }
        }
    }

    protected int calcLevelWeight(int level, boolean doIn, boolean doOut)
    {
        int weight = 0;

        level--;
        if( level > 0 && doIn )
            weight += calcEdgesWeight( levelEdges.get( level - 1 ) );

        if( doOut )
            weight += calcEdgesWeight( levelEdges.get( level ) );

        return weight;
    }

    protected int calcEdgesWeight(List<Edge> edges)
    {
        int weight = 0;
        for( int i = 0; i < edges.size(); i++ )
        {
            Edge e1 = edges.get( i );

            if( verticalOrientation )
                weight += Math.abs( e1.from.x - e1.to.x );
            else
                weight += Math.abs( e1.from.y - e1.to.y );

            // calc intersections
            for( int k = i + 1; k < edges.size(); k++ )
            {
                Edge e2 = edges.get( k );

                if( verticalOrientation )
                {
                    if( ( e1.from.x < e2.from.x && e1.to.x > e2.to.x ) || ( e2.from.x < e1.from.x && e2.to.x > e1.to.x ) )
                        weight += 1000;
                }
                else
                {
                    if( ( e1.from.y < e2.from.y && e1.to.y > e2.to.y ) || ( e2.from.y < e1.from.y && e2.to.y > e1.to.y ) )
                        weight += 1000;
                }
            }
        }

        return weight;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Place node issues
    //

    protected void placeNodesInitial()
    {
        int levelSize = 0;
        int levelOffset = 0;

        for( int i = 0; i < maxLevel; i++ )
        {
            List<Node> nodes = levelNodes.get( i );
            levelSize = placeLevelInitial( nodes, levelOffset );
            levelSizes[i] = levelSize;
            if( verticalOrientation )
            {
                levelOffset += levelSize / gridY * gridY + gridY;
                levelOffset += layerDeltaY;
            }
            else
            {
                levelOffset += levelSize / gridX * gridX + gridX;
                levelOffset += layerDeltaX;
            }
        }
    }

    protected int placeLevelInitial(List<Node> level, int levelOffset)
    {
        int size = 0;
        int offset = 0;

        for( Node node : level )
        {
            if( verticalOrientation )
            {
                node.y = levelOffset;
                node.x = offset;

                offset += snapUp( getIntralevelWidth( node ) );
                size = Math.max( size, node.height );
            }
            else
            {
                node.x = levelOffset;
                node.y = offset;

                offset += snapUp( getIntralevelWidth( node ) );
                size = Math.max( size, node.width );
            }
        }

        return size;
    }

    protected void straightenDummy(Graph graph, Node n)
    {
        List<Edge> edges = graph.getEdges( n );
        Node from = null;
        Node to = null;
        for( int i = 0; i < 2; i++ )
        {
            Edge edge = edges.get( i );
            if( edge.to == n )
                from = edge.from;
            else
                to = edge.to;
        }

        if( from == null || to == null )
        {
            log.log(Level.SEVERE, "One of the edge terminals is null" );
            return;
        }

        if( verticalOrientation )
            n.x = ( n.x + from.x + from.width / 2 + to.x + to.width / 2 ) / 3;
        else
            n.y = ( n.y + from.y + from.height / 2 + to.y + to.height / 2 ) / 3;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Create and remove a meta-root and meta-edges to all nodes.
    //

    protected Node makeMetaRoot(Graph graph)
    {
        Node metaRoot = new Node( "meta-root" );
        metaRoot.data = new LayoutData();

        for( Node n : graph.nodeList )
        {
            Edge e = new Edge( n, metaRoot );
            graph.addEdge( e );
        }

        graph.addNode( metaRoot );
        return metaRoot;
    }

    //++++++++++++++++++++++++++++++++++

    @Override
	public LayouterInfo getInfo()
    {
        LayouterInfoSupport lis = new LayouterInfoSupport( true, true, false, false, false, false );
        return lis;
    }

    @Override
	public int estimate(Graph graph, int what)
    {
        int operations = 0;
        if( graph.isConnected() )
            return estimateConnected( graph, what );
        else
        {
            List<Graph> graphs = graph.split();
            for( Graph gr : graphs )
            {
                boolean notSimpleNodeFlag = true;
                for( Node n : gr.nodeList )
                {
                    String type = n.getAttribute( "Type" );
                    if( !Util.isCompartment( n )
                            && ( type == null || ( !type.equals( "Event" ) && !type.equals( "Equation" ) && !type.equals( "Function" ) ) ) )
                        notSimpleNodeFlag = false;
                    continue;
                }
                if( !notSimpleNodeFlag )
                    operations += estimateConnected( gr, what );
            }
            return operations;
        }
    }

    public int estimateConnected(Graph graph, int what)
    {
        initLayoutData( graph );
        normalise( graph );

        ArrayList<Node> topoSortedNodes = new ArrayList<>();
        Node metaRoot = makeMetaRoot( graph );
        unmarkNodes( graph );
        topoSort( graph, metaRoot, topoSortedNodes );
        graph.removeNode( metaRoot );
        topoSortedNodes.remove( metaRoot );

        // then assign levels to nodes.
        int toposize = topoSortedNodes.size();
        maxLevel = 0;
        for( int i = 0; i < toposize; ++i )
        {
            Node v = topoSortedNodes.get( i );
            int level = 0;

            List<Edge> edges = graph.getEdges( v );
            for( Edge edge : edges )
            {
                if( edge.to == v )
                {
                    Node u = edge.from;
                    if( getLevel( u ) > level )
                        level = getLevel( u );
                }
            }

            setLevel( v, level + 1 );
            if( level + 1 > maxLevel )
                maxLevel = level + 1;
        }

        for( Edge edge : selfLoops )
        {
            graph.addEdge( edge );
        }
        // restore reversed edges
        for( int i = 0; i < graph.edgeList.size(); i++ )
        {
            Edge edge = graph.edgeList.get( i );
            if( edge.reversed )
            {
                graph.removeEdge( edge );
                edge.reverseDirection();
                graph.addEdge( edge );

                i--;
            }
        }
        return layerOrderIterationNum * maxLevel + maxNodeOptimizationNum * 10;
    }


    public static class LayoutData
    {
        public int level = 0;

        public int usageLevel = 0;

        public int barycenter = 0;

        public boolean marked = false;

        public boolean picked = false;

        public boolean dummy = false;

        public int upEdgesWeight = 0;

        public int downEdgesWeight = 0;
    }

    private Layouter pathLayouter = new HierarchicPathLayouter();
    public Layouter getPathLayouter()
    {
        return pathLayouter;
    }
    public void setPathLayouter(Layouter val)
    {
        if( val instanceof OrthogonalPathLayouter )
        {
            if( verticalOrientation )
            {
                ( (OrthogonalPathLayouter)val ).setOrientation( Orientation.BOTTOM_NAME );
            }
            else
            {
                ( (OrthogonalPathLayouter)val ).setOrientation( Orientation.RIGHT_NAME );
            }
        }
        pathLayouter = val;
    }
}
