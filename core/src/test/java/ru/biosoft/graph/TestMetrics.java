package ru.biosoft.graph;

import junit.framework.TestCase;
import ru.biosoft.graph.ForceDirectedLayouter;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.GreedyLayouter;
import ru.biosoft.graph.HierarchicLayouter;
import ru.biosoft.graph.LayoutQualityMetrics;
import ru.biosoft.graph.Layouter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

public class TestMetrics extends TestCase
{
	private static final String[] files = new String[] {"db", "darren", "force_directed", "pubs_trivial", "northwind_trivial",
            "small_complex", "sample_dsd", "simple", "hierarchic", "moderate", "moderate2" /*, "complex" */ };

    public void testForceDirectedLayout() throws Exception
    {
        File f = new File( Directories.testOut + "forceDirectedMetrics.csv" );
        f.createNewFile();
        processLayoutMetricsTesting( f, new ForceDirectedLayouter() );
    }

    public void testHierarchicalLayout() throws IOException
    {
        File f = new File( Directories.testOut + "hierarchicalMetrics.csv" );
        f.createNewFile();
        processLayoutMetricsTesting( f, new HierarchicLayouter() );
    }

    public void testGreedyLayout() throws IOException
    {
        File f = new File( Directories.testOut + "greedyMetrics.csv" );
        f.createNewFile();
        processLayoutMetricsTesting( f, new GreedyLayouter() );
    }

    private void processLayoutMetricsTesting(File f, Layouter l) throws IOException
    {
        System.out.print("Test metrics: " + l.getClass().getName() + "\n");

    	try (FileWriter fw = new FileWriter( f ))
        {
            fw.write( "Name" + LayoutQualityMetrics.SEPARATOR + LayoutQualityMetrics.HEADER + "Time\n" );

            for( String s : files )
            {
                System.out.print("   - " + s + "... ");
            	long startTime = System.currentTimeMillis();
                try
                {
                    Graph graph = new Graph();
                    graph.fillFromText( TestingUtils.readFileAsString( Directories.testData + s + ".txt" ) );
                    l.doLayout( graph, null );

                    LayoutQualityMetrics m = LayoutQualityMetrics.getMetrics( graph );
                    String timeColumn = LayoutQualityMetrics.df.format( ( System.currentTimeMillis() - startTime ) / 1000.0 );
                    fw.write( s + LayoutQualityMetrics.SEPARATOR + m.toString() + timeColumn + "\n" );
                    System.out.println(" " + (System.currentTimeMillis()-startTime) + " ms");
                }
                catch( Throwable t )
                {
                    fw.write( s + LayoutQualityMetrics.SEPARATOR + "Exception: + " + t.getMessage() + Arrays.toString( t.getStackTrace() )
                            + "\n" );
                }
            }
        }
    }

}
