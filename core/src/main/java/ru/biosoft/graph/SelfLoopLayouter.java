package ru.biosoft.graph;

/**
 * A layout algorithm that routes the selfloops (reflexive edges) of a graph. 
 * 
 * By default this layouter routes selfloops in an orthogonal fashion. 
 * It places the selfloop in the least crowded quadrant around a node.
 * 
 * This algorithm create line pathes only for self loops and ignore other edges.
 */
public class SelfLoopLayouter
{
    /** Loop edge length. Default value is 30. */
    public int loopLength = 30;

    /**
     * Lays out all selflops for the specified graph
     * 
     * Non selfloop edges are ignored.
     * 
     * @param graph - graph to be arranged.
     */
    public void doLayout(Graph graph, PathWeighter pathWeighter)
    {
        for ( Edge edge : graph.edgeList )
            if ( !edge.fixed && edge.from == edge.to )
                layoutPath ( graph, edge, pathWeighter );
    }

    /**
     * Assigns line path for selfloop edge.
     * 
     * @param graph - graph that contains the selfloop edge
     * @param edge - edge for which line path will be layed out.
     */
    public void layoutPath(Graph graph, Edge edge, PathWeighter pathWeighter)
    {
        Node node = edge.from;

        // create possible loops and select best of them
        int xf_1 = node.x + Math.min ( loopLength, node.width ) / 2;
        int xf_2 = node.x + node.width - Math.min ( loopLength, node.width ) / 2;

        int yf_1 = node.y + Math.min ( loopLength, node.height ) / 2;
        int yf_2 = node.y + node.height - Math.min ( loopLength, node.height ) / 2;

        Path[] pathes = new Path[4];
        pathes[0] = new Path ( );
        pathes[0].addPoint ( node.x, yf_1 );
        pathes[0].addPoint ( node.x - loopLength / 2, yf_1 );
        pathes[0].addPoint ( node.x - loopLength / 2, yf_1 - loopLength );
        pathes[0].addPoint ( xf_1, yf_1 - loopLength );
        pathes[0].addPoint ( xf_1, node.y );

        pathes[1] = new Path ( );
        pathes[1].addPoint ( xf_2, node.y );
        pathes[1].addPoint ( xf_2, node.y - loopLength / 2 );
        pathes[1].addPoint ( xf_2 + loopLength, node.y - loopLength / 2 );
        pathes[1].addPoint ( xf_2 + loopLength, yf_1 );
        pathes[1].addPoint ( node.x + node.width, yf_1 );

        pathes[2] = new Path ( );
        pathes[2].addPoint ( node.x + node.width, yf_2 );
        pathes[2].addPoint ( node.x + node.width + loopLength / 2, yf_2 );
        pathes[2].addPoint ( node.x + node.width + loopLength / 2, yf_2 + loopLength );
        pathes[2].addPoint ( xf_2, yf_2 + loopLength );
        pathes[2].addPoint ( xf_2, node.y + node.height );

        pathes[3] = new Path ( );
        pathes[3].addPoint ( xf_1, node.y + node.height );
        pathes[3].addPoint ( xf_1, node.y + node.height + loopLength / 2 );
        pathes[3].addPoint ( xf_1 - loopLength, node.y + node.height + loopLength / 2 );
        pathes[3].addPoint ( xf_1 - loopLength, yf_2 );
        pathes[3].addPoint ( node.x, yf_2 );

        // select best path
        Path bestPath = null;
        int bestWeight = Integer.MAX_VALUE;
        for ( int i = 0; i < 4; i++ )
        {
            int weight = pathWeighter.calcPathWeight ( graph, pathes[i], bestWeight );

            // TODO: WTF ???
            if ( i == 2 )
                bestWeight -= 100;

            if ( weight < bestWeight )
            {
                bestPath = pathes[i];
                bestWeight = weight;
            }
        }

        edge.path = bestPath;
    }

}
