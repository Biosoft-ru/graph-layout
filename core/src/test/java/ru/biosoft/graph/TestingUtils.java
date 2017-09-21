package ru.biosoft.graph;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import ru.biosoft.graph.Edge;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.Node;

public class TestingUtils
{
	protected static boolean SKIP_LONG_TESTS = true;

    public static void createTestOutDir()
    {
    	File outDir = new File(Directories.testOut);
    	if( !outDir.exists() )
    		outDir.mkdir();
    }

    public static Graph readGraph(String path) throws IOException
    {
        Graph graph = new Graph();
        String text = TestingUtils.readFileAsString( path );
        graph.fillFromText( text );
        return graph;
    }
    
	public static String readFileAsString(String filePath) throws IOException
    {
        StringBuffer fileData = new StringBuffer( 1000 );
        char[] buf = new char[1024];
        int numRead = 0;
        try (BufferedReader reader = new BufferedReader( new FileReader( filePath ) ))
        {
            while( ( numRead = reader.read( buf ) ) != -1 )
            {
                String readData = String.valueOf( buf, 0, numRead );
                fileData.append( readData );
                buf = new char[1024];
            }
        }
        return fileData.toString();
    }

    public static Graph createGraph(String[] nodeNames, String[][] verticesNames)
    {
        Graph graph = new Graph();
        for (String nodeName : nodeNames)
        {
            graph.addNode(new Node(nodeName));
        }
        for( String[] verticesName : verticesNames )
        {
            graph.addEdge(new Edge(graph.getNode(verticesName[0]), (graph.getNode(verticesName[1]))));
        }
        return graph;
    }

}
