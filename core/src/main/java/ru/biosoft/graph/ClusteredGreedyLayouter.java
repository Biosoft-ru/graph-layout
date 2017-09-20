package ru.biosoft.graph;

import java.util.ArrayList;
import java.util.Iterator;

public class ClusteredGreedyLayouter  extends GreedyLayouter
{
    /**
     * If number of nodes exceeds threshold then graph clustering
     * will be involved.
     * <p/>
     * Threshold is also used as parameter in collapse method.
     * If number of nodes less then threshold, then collapsing is terminated.
     */
    public int threshold = 50;

    /** Graph to be layoutted. */
    protected Graph graph;

    /**
     * Stack that is store graph nodes and edges that were removed by collapse method.
     */
    protected ArrayList<StackUnit> stack = new ArrayList<>();

    /**
     * Average node width. Used to calculate cluster size.
     */
    protected int averageWidth;

    /**
     * Average node height. Used to calculate cluster size.
     */
    protected int averageHeight;

    /**
     * Internal variable that indicates whether graph collapsing should be used.
     */
    protected boolean shouldCollapse;

    /** Assigns LayoutData to each node. */
    protected void initLayoutData( Graph graph )
    {
        for (Node node : graph.nodeList)
        {
            LayoutData data = new LayoutData();
            node.data = data;
            data.width = node.width;
            data.height = node.height;
            data.fixed = node.fixed;
        }

        // calculate average node size
        averageWidth = 0;
        averageHeight = 0;
        int dWidth = 0;
        int dHeight = 0;
        int nodeCount = graph.nodeCount();

        for (Node node : graph.nodeList)
        {
            averageWidth += node.width;
            averageHeight += node.height;

            dWidth += node.width * node.width;
            dHeight += node.height * node.height;
        }

        averageWidth /= nodeCount;
        averageHeight /= nodeCount;
        double sWidth = Math.sqrt( dWidth / nodeCount - averageWidth * averageWidth ) / averageWidth;
        double sHeight = Math.sqrt( dHeight / nodeCount - averageHeight * averageHeight ) / averageHeight;
        shouldCollapse = Math.max( sWidth, sHeight ) < 0.5;

        averageWidth = ( averageWidth + layerDeltaX / 2 ) / layerDeltaX * layerDeltaX;
        averageHeight = ( averageHeight + layerDeltaY / 2 ) / layerDeltaY * layerDeltaY;
    }

    /**
     * Iteratively collapse the graph by eliminating nodes with one edge.
     * 
     * For example graph: a->b->c->d->e->f will be replaced by f.
     * 
     * Removed nodes and edges are stored in the stack, so expand method can restore the graph structure.
     */
    protected void collapse()
    {
        // we use several iterations to collapse the graph
        int i;
        for( int k = 0; k < 20 && graph.nodeCount() > threshold; k++ )
        {
            boolean changed = false;
            for ( i = 0; i < graph.nodeCount(); i++ )
            {
                Node node = graph.nodeAt( i );
                int edgeCount = graph.getEdges( node ).size();
                if ( !canCollapse( node, k ) )
                    continue;

                if ( edgeCount == 1 )
                {
                    changed = true;
                    Iterator<Edge> edges = graph.getEdges( node ).iterator();
                    Edge edge = edges.next();
                    removeEdge( edge );
                    removeNode( node );
                    updateClusterSize( edge, node, 1 );
                    i--;
                    continue;
                }

                if ( edgeCount == 2 && k > 5 )
                {
                    changed = true;
                    Iterator<Edge> edges = graph.getEdges( node ).iterator();
                    Edge e1 = edges.next();
                    Edge e2 = edges.next();

                    Node from = ( e1.getFrom() == node ) ? e1.getTo() : e1.getFrom();
                    Node to = ( e2.getFrom() == node ) ? e2.getTo() : e2.getFrom();

                    removeEdge( e1 );
                    removeEdge( e2 );
                    removeNode( node );
                    addEdge( from, to );
                    updateClusterSize( e1, node, ( float ) 0.5 );
                    updateClusterSize( e2, node, ( float ) 0.5 );

                    i--;
                }
            }

            if ( !changed )
                break;
        }

        // assign new size to nodes
        for ( Node node : graph.nodeList )
        {
            float n = ( ( LayoutData ) node.data ).n;
            if ( n >= 2 )
            {
                node.width += ( ( int ) Math.sqrt( n - 1 ) ) * ( averageWidth + layerDeltaX );
                node.height += ( ( int ) Math.sqrt( n - 1 ) ) * ( averageHeight + layerDeltaY );
            }
        }
    }

    protected void removeNode( Node node )
    {
        graph.removeNode( node );
        stack.add( new StackUnit( node, true ) );
    }

    protected void removeEdge( Edge edge )
    {
        if ( edge.slaves != null )
        {
            for ( Edge slave : edge.slaves )
            {
                graph.removeEdge( slave );
                stack.add( new StackUnit( slave, true ) );
            }
        }

        graph.removeEdge( edge );
        stack.add( new StackUnit( edge, true ) );

    }

    protected void addEdge( Node from, Node to )
    {
        // ignore new selfloops
        if ( from == to )
            return;

        Edge edge = new Edge( from, to );
        graph.addEdge( edge );
        stack.add( new StackUnit( edge, false ) );
    }

    protected void updateClusterSize( Edge edge, Node collapsedNode, float weight )
    {
        Node cluster = ( edge.getFrom() == collapsedNode ) ? edge.getTo() : edge.getFrom();
        ( ( LayoutData ) cluster.data ).n += ( ( LayoutData ) collapsedNode.data ).n * weight;
    }

    protected boolean canCollapse( Node node, int i )
    {
        double t = 3 + i * 2 + 0.2 * i * i;
        float n = ( ( LayoutData ) node.data ).n;
        if ( n > t )
            return false;

        int edgeCount = graph.getEdges( node ).size();
        if ( edgeCount == 1 )
        {
            Iterator<Edge> edges = graph.getEdges( node ).iterator();
            Edge edge = edges.next();
            Node cluster = ( edge.getFrom() == node ) ? edge.getTo() : edge.getFrom();
            n += ( ( LayoutData ) cluster.data ).n;
            return n < t;
        }

        if ( edgeCount == 2 )
        {
            Iterator<Edge> edges = graph.getEdges( node ).iterator();
            Edge e1 = edges.next();
            Edge e2 = edges.next();

            if ( !e1.master || !e2.master )
                return false;

            Node from = ( e1.getFrom() == node ) ? e1.getTo() : e1.getFrom();
            Node to = ( e2.getFrom() == node ) ? e2.getTo() : e2.getFrom();

            float n1 = n / 2 + ( ( LayoutData ) from.data ).n;
            float n2 = n / 2 + ( ( LayoutData ) to.data ).n;

            return n1 < t && n2 < t;
        }

        return false;
    }

    protected void fixNodes()
    {
        // restore nodes size, align them by center and fix
        for ( Node node : graph.nodeList )
        {
            if ( node.data != null )
            {
                LayoutData data = ( LayoutData ) node.data;
                int dx = ( node.width - data.width ) / 2 / gridX * gridX;
                int dy = ( node.height - data.height ) / 2 / gridY * gridY;
                node.x += dx;
                node.y += dy;
                node.width = data.width;
                node.height = data.height;
            }

            node.fixed = true;
        }
    }

    protected void restore()
    {
        for ( int i = stack.size() - 1; i >= 0; i-- )
        {
            StackUnit unit = stack.get( i );

            if ( unit.removed )
            {
                if ( unit.node != null )
                    graph.addNode( unit.node );
                else
                    graph.addEdge( unit.edge );
            }
            else
            {
                if ( unit.node != null )
                    graph.removeNode( unit.node );
                else
                    graph.removeEdge( unit.edge );
            }
        }

        stack.clear();
    }

    /**
     * Assigns a new graph layout to the given connected graph.
     *
     * @param g - graph to be layed out.
     */
    @Override
    public void doLayout( Graph g, LayoutJobControl lJC )
    {
        // TODO: Subgraph ???
        this.graph = g;

        initLayoutData( graph );

        if( !shouldCollapse || graph.nodeCount() <= threshold )
        {
            layoutNodes( graph,  lJC);
            refineNodes( graph, 3 );
            layoutEdges( graph, lJC );
            return;
        }

        collapse();
        layoutNodes( graph, lJC );
        fixNodes();
        refineNodes( graph, 5 );

        restore();
        layoutNodes( graph, lJC );

        // clear fixed nodes
        for (Node node : graph.nodeList)
            if ( node.data != null )
                node.fixed = (( LayoutData ) node.data).fixed;

        refineNodes( graph, 5 );
        layoutEdges( graph, lJC );
    }

    // utility structures
    protected class StackUnit
    {
        boolean removed;
        Node node;
        Edge edge;

        StackUnit( Node node, boolean removed )
        {
            this.node = node;
            this.edge = null;
            this.removed = removed;
        }

        StackUnit( Edge edge, boolean removed )
        {
            this.node = null;
            this.edge = edge;
            this.removed = removed;
        }

    }
}
