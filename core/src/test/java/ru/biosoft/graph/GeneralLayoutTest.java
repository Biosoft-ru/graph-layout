package ru.biosoft.graph;

import junit.framework.TestCase;
import ru.biosoft.graph.CompartmentCrossCostGridLayouter;
import ru.biosoft.graph.ForceDirectedLayouter;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.GreedyLayouter;
import ru.biosoft.graph.HierarchicLayouter;
import ru.biosoft.graph.LayoutQualityMetrics;
import ru.biosoft.graph.Layouter;

import java.io.IOException;

public class GeneralLayoutTest extends TestCase
{
	private static final String[] files = new String[]{"simple", "gridGraph"}; 

    public void testHierarchic() throws Exception
    {
        layout(new HierarchicLayouter());
    }

    public void testForceDirected() throws Exception
    {
        layout(new ForceDirectedLayouter());
    }
	
    public void testOrthogonal() throws Exception
    {
        layout(new GreedyLayouter());
    }

    public void testFastGrid() throws Exception
    {
    	if( TestingUtils.SKIP_LONG_TESTS )
    		return;

    	layout(new FastGridLayouter());
    }

    public void testCompartmentCrossCostGridLayout() throws Exception
    {
    	if( TestingUtils.SKIP_LONG_TESTS )
    		return;
    		
        layout(new CompartmentCrossCostGridLayouter());
    } 
    
    private void layout(Layouter l) throws Exception
    {
        System.out.print("Layout " + l.getClass().getName() + "\n");

    	TestingUtils.createTestOutDir();        
    	
        for ( String s : files )
        {
            System.out.print("   - " + s + "... ");
            try
            {
                Graph graph = new Graph();
                graph.fillFromText( TestingUtils.readFileAsString(Directories.testData + s + ".txt" ) );

                long startTime = System.currentTimeMillis();
                l.doLayout( graph , null);
                String timePresentaion = "Layout time: " + LayoutQualityMetrics.df.format( ( System.currentTimeMillis() - startTime ) / 1000.0 );

                SimpleImageGenerator.writeImageToFile( graph, Directories.testOut + s + "_" + createPostfix(l) + ".png", true, timePresentaion );
                System.out.println(" " + (System.currentTimeMillis()-startTime) + " ms");
                
                /*
                try (BufferedWriter buf = new BufferedWriter( new FileWriter(Directories.testOut + s + "_" + createPostfix(l) + ".out") ))
                {
                    LayoutQualityMetrics m = LayoutQualityMetrics.getMetrics( graph );

                    buf.write(l.getClass().getName() + "\n");
                    buf.write(timePresentaion + "\n");
                    buf.write("Quality metrics:\n   " + m.getDebugInfo() + "\n\n\n");
                    buf.write(graph.generateText(true));
                }*/
                
            }
            catch(IOException e) 
            {
            	throw new Exception("Error for layouter=" + l + ", file=" + s, e);
            }
        }
    }

    private static String createPostfix( Layouter l )
    {
        String res = l.getClass().getSimpleName();
        return res.replaceAll( "Layouter", "" );
    }
}
