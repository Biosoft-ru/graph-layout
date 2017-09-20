package ru.biosoft.graph;

public interface LayouterInfo
{
    public boolean supportEdgesLayout();
    public boolean supportNodesLayout();
    public boolean supportIncrementalLayout();
    public boolean supportCompartments();
    public boolean supportPorts();
    public boolean supportSubgraphs();
}
