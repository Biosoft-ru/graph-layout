package ru.biosoft.graph;

import junit.framework.TestCase;
import junit.framework.TestSuite;
import ru.biosoft.graph.Edge;
import ru.biosoft.graph.ForceDirectedLayouter;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.Layouter;
import ru.biosoft.graph.Node;
import ru.biosoft.graph.PathwayLayouter;
import ru.biosoft.graph.Util;


public class PathwayLayouterTest extends TestCase
{
    //layouter for test
    private Layouter layouter = new ForceDirectedLayouter();

    public PathwayLayouterTest(String name)
    {
        super(name);
    }

    public static junit.framework.Test suite()
    {
        TestSuite suite = new TestSuite(PathwayLayouterTest.class.getName());
        suite.addTest(new PathwayLayouterTest("test"));
        return suite;
    }

    public void test() throws Exception
    {
        Graph graph = generateGraph();
        PathwayLayouter pathwayLayouter = new PathwayLayouter(layouter);

        pathwayLayouter.prepareGraph(graph);

    	TestingUtils.createTestOutDir();        

    	int levelCount = pathwayLayouter.getLevelCount();
        for( int i = 0; i< levelCount; i++ )
        {
            Graph levelGraph = pathwayLayouter.layoutLevel(i, graph, null);
            Util.outGraph(Directories.testOut + i + "_pathwayLayoutTest.txt", levelGraph);
        }

    }

    public Graph generateGraph()
    {
        Node[] nodes = new Node[16];
        nodes[0] = new Node("extra", 0, 0, 10, 10);
        nodes[0].setAttribute("isCompartment", "true");

        nodes[1] = new Node("intra1", 2, 2, 10, 10);
        nodes[1].setAttribute("isCompartment", "true");
        nodes[1].setAttribute("compartmentName", "extra.intra");

        nodes[2] = new Node("intra2", 3, 1, 10, 10);
        nodes[2].setAttribute("isCompartment", "true");
        nodes[2].setAttribute("compartmentName", "extra.intra");

        nodes[3] = new Node("r1", 1, 0, 15, 15);
        nodes[4] = new Node("r2", 3, 1, 15, 15);
        nodes[5] = new Node("r3", 1, 0, 15, 15);
        nodes[6] = new Node("r4", 1, 0, 15, 15);
        nodes[7] = new Node("r5", 3, 1, 15, 15);
        nodes[8] = new Node("r6", 1, 0, 15, 15);

        nodes[9] = new Node("X", 1, 0, 40, 40);
        nodes[9].setAttribute("compartmentName", "extra.intra.intra1");

        nodes[10] = new Node("Y", 1, 0, 40, 40);
        nodes[10].setAttribute("compartmentName", "extra.intra.intra2");

        nodes[11] = new Node("Z", 3, 1, 40, 40);
        nodes[11].setAttribute("compartmentName", "extra.intra");

        nodes[12] = new Node("EC", 1, 0, 40, 40);
        nodes[12].setAttribute("compartmentName", "extra");

        nodes[13] = new Node("r7", 1, 0, 15, 15);
        nodes[14] = new Node("r8", 3, 1, 15, 15);

        nodes[15] = new Node("intra", 1, 0, 10, 10);
        nodes[15].setAttribute("isCompartment", "true");
        nodes[15].setAttribute("compartmentName", "extra");

        Edge[] edges = new Edge[16];
        edges[1] = new Edge(nodes[10], nodes[3]);
        edges[2] = new Edge(nodes[10], nodes[4]);

        edges[3] = new Edge(nodes[5], nodes[10]);

        edges[4] = new Edge(nodes[9], nodes[6]);
        edges[5] = new Edge(nodes[9], nodes[7]);

        edges[6] = new Edge(nodes[8], nodes[9]);

        edges[7] = new Edge(nodes[3], nodes[11]);
        edges[8] = new Edge(nodes[4], nodes[11]);

        edges[9] = new Edge(nodes[11], nodes[5]);

        edges[10] = new Edge(nodes[6], nodes[11]);
        edges[11] = new Edge(nodes[7], nodes[11]);

        edges[12] = new Edge(nodes[11], nodes[8]);

        edges[13] = new Edge(nodes[11], nodes[13]);
        edges[14] = new Edge(nodes[11], nodes[14]);
        edges[15] = new Edge(nodes[13], nodes[12]);
        edges[0] = new Edge(nodes[14], nodes[12]);

        Graph graph = new Graph();

        for( Node n : nodes )
            graph.addNode(n);


        for( Edge e : edges )
            graph.addEdge(e);

        return graph;
    }
}
