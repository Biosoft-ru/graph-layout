package ru.biosoft.graph;


public class LayouterInfoSupport implements LayouterInfo 
{
    protected boolean edgesLayout;
    protected boolean nodesLayout;
    protected boolean incrementalLayout;
    protected boolean compartments;
    protected boolean ports;
    protected boolean subgraphs;
    
    public LayouterInfoSupport (boolean edgesLayout, boolean nodesLayout, boolean incrementalLayout, boolean compartments, boolean ports, boolean subgraphs)
    {
        this.edgesLayout = edgesLayout;
        this.nodesLayout = nodesLayout;
        this.incrementalLayout = incrementalLayout;
        this.compartments = compartments;
        this.ports = ports;
        this.subgraphs = subgraphs;   
    }
    
    public boolean supportEdgesLayout() 		{return edgesLayout;}
    public boolean supportNodesLayout() 		{return nodesLayout;}
    public boolean supportIncrementalLayout() 	{return incrementalLayout;}
    public boolean supportCompartments() 		{return compartments;}
    public boolean supportPorts() 				{return ports;}
    public boolean supportSubgraphs() 			{return subgraphs;}
}
