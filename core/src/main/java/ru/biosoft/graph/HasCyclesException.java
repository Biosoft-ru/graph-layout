package ru.biosoft.graph;

import java.util.Set;

@SuppressWarnings ("serial")
public class HasCyclesException extends Exception
{
    private Set<Node> cycle;

    public Set<Node> getCycle()
    {
        return cycle;
    }

    public HasCyclesException(Set<Node> cycle) 
    {
        this.cycle = cycle;
    }
}
