package ru.biosoft.graph;

import junit.framework.TestCase;
import ru.biosoft.graph.Edge;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.Node;

import java.util.StringTokenizer;

public class TestReadGraph
        extends TestCase
{
    public void testReadGraph ( )
            throws Exception
    {
        Graph graph = new Graph ( );
        String graphText = "a->c, a->d, a->e, b->e, b->f, b->g, f->m, i->m, i->r, c->q, d->k, k->r, k->q";
        StringTokenizer tokens = new StringTokenizer ( graphText, ", " );
        while ( tokens.hasMoreTokens ( ) )
        {
            String edgeStr = tokens.nextToken ( );

            int d = edgeStr.indexOf ( '-' );
            if ( d == - 1 )
            {
                fail ( "Invalid edge: '" + edgeStr + "'." );
                continue;
            }

            String in = edgeStr.substring ( 0, d );
            String out = edgeStr.substring ( d + 2 );

            Node inNode = graph.getNode ( in );
            if ( inNode == null )
            {
                inNode = new Node ( in );
                inNode.width = 30;
                inNode.height = 20;

                graph.addNode ( inNode );
            }

            Node outNode = graph.getNode ( out );
            if ( outNode == null )
            {
                outNode = new Node ( out );
                outNode.width = 30;
                outNode.height = 20;

                graph.addNode ( outNode );
            }

            Edge edge = new Edge ( inNode, outNode );
            graph.addEdge ( edge );
        }
    }
}
