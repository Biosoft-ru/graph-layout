package ru.biosoft.graph;

import java.io.IOException;

import junit.framework.TestCase;
import ru.biosoft.graph.CompartmentCrossCostGridLayouter;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.Layouter;

/**
 * Stub class for testing performance
 */
public class TestPerfomance extends TestCase
{
    private Graph readGraph(String path) throws IOException
    {
        Graph graph = new Graph();
        String text = TestingUtils.readFileAsString( path );
        graph.fillFromText( text );
        return graph;
    }

    private void testLayouter(Layouter layouter, Graph graph)
    {
        long time = System.currentTimeMillis();
        layouter.doLayout( graph, null );
        time = System.currentTimeMillis() - time;
        System.out.println("Running time " + time + "ms");
    }

    public void test_small_CompartmentCrossCostGridLayouter() throws Exception
    {
//        Graph graph = readGraph(Directories.testData + "small.txt" );
//        testLayouter( new CompartmentCrossCostGridLayouter(), graph );
    }
}
