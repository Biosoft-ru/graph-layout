package ru.biosoft.graph;

import java.util.List;

import junit.framework.TestCase;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.HasCyclesException;
import ru.biosoft.graph.Node;

public class GraphTest extends TestCase
{
    public void testGetRoots1() throws Throwable
    {
        Graph graph = TestingUtils.createGraph(
                new String[] {"n1", "n2", "n3"}, 
                new String[][] { {"n1", "n2"}, {"n2", "n3"}});

        List<Node> roots = graph.getRoots();
        assertEquals(1, roots.size());
        assertTrue(roots.contains(graph.getNode("n1")));
    }

    public void testGetRoots2() throws Throwable
    {
        Graph graph = TestingUtils.createGraph(
                new String[] {"n1", "n2", "n3"}, 
                new String[][] { {"n1", "n2"}, {"n3", "n2"}});

        List<Node> roots = graph.getRoots();
        assertEquals(2, roots.size());
        assertTrue(roots.contains(graph.getNode("n1")));
        assertTrue(roots.contains(graph.getNode("n3")));
    }

    public void testGetRoots3() throws Throwable
    {
        Graph graph = TestingUtils.createGraph (
                new String[] {"n1", "n2", "n3"}, 
                new String[][] { {"n1", "n2"}, {"n3", "n2"}, {"n1", "n3"}});

        List<Node> roots = graph.getRoots();
        assertEquals(1, roots.size());
        assertTrue(roots.contains(graph.getNode("n1")));
    }

    public void testGetRoots4() throws Throwable
    {
        Graph graph = TestingUtils.createGraph (
                new String[] {"n1", "n2", "n3", "n4", "n5"}, 
                new String[][] { 
                        {"n1", "n2"}, 
                        {"n1", "n3"}, 
                        {"n2", "n4"},
                        {"n3", "n4"},
                        {"n5", "n3"}});

        List<Node> roots = graph.getRoots();
        assertEquals(2, roots.size());
        assertTrue(roots.contains(graph.getNode("n1")));
        assertTrue(roots.contains(graph.getNode("n5")));
    }
    
    public void testGetRoots5() throws Throwable
    {
        Graph graph = TestingUtils.createGraph (
                new String[] {"n1", "n2", "n3", "n4"}, 
                new String[][] { {"n1", "n2"}, {"n3", "n2"}, {"n1", "n3"}, {"n3", "n4"}});

        List<Node> roots = graph.getRoots();
        assertEquals(1, roots.size());
        assertTrue(roots.contains(graph.getNode("n1")));
    }

    
    public void testGetRootsNotConnected() throws Throwable
    {
        Graph graph = TestingUtils.createGraph (
                new String[] {"n1", "n2", "n3"}, 
                new String[][] { {"n1", "n2"} });

        List<Node> roots = graph.getRoots();
        assertNull(roots);        
    }
    
    
    public void testGetRootsCycle1()
    {
        Graph graph = TestingUtils.createGraph(
                new String[] {"n1", "n2", "n3"}, 
                new String[][] { {"n1", "n2"}, {"n2", "n3"}, {"n3", "n1"}});

        HasCyclesException exception = null;
        try
        {
            graph.getRoots();            
        }
        catch( HasCyclesException e )
        {
            exception = e;
        }
        
        assertNotNull(exception);
        assertTrue(exception.getCycle().contains(graph.getNode("n1")));
        assertTrue(exception.getCycle().contains(graph.getNode("n2")));
        assertTrue(exception.getCycle().contains(graph.getNode("n3")));
    }

    public void testGetRootsCycle2()
    {
        Graph graph = TestingUtils.createGraph(
                new String[] {"n1", "n2", "n3", "n4"}, 
                new String[][] { {"n1", "n2"}, {"n2", "n3"}, {"n3", "n1"}, {"n1", "n4"}});

        HasCyclesException exception = null;
        try
        {
            graph.getRoots();            
        }
        catch( HasCyclesException e )
        {
            exception = e;
        }
        
        assertNotNull(exception);
        assertTrue(exception.getCycle().contains(graph.getNode("n1")));
        assertTrue(exception.getCycle().contains(graph.getNode("n2")));
        assertTrue(exception.getCycle().contains(graph.getNode("n3")));
    }
    
    
}
