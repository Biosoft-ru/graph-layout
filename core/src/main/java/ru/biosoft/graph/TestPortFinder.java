package ru.biosoft.graph;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

/**
 * TestPortFinder defines port constraints using node name.
 */
public class TestPortFinder implements PortFinder
{
    protected List<Edge> edges = new ArrayList<>();

    @Override
    public Point findPort(Node node, Edge edge, int x, int y)
    {
        if(!edges.contains(edge))
            edges.add(edge);
        int index = 1 + edges.indexOf(edge);

        int cx = x;
        int cy = y;

        if(node.name.endsWith("l"))
        {
            cx = node.x - 2;
            cy = node.y + 3 * index;
        }
        else if(node.name.endsWith("r"))
        {
            cx = node.x + node.width + 2;
            cy = node.y + 3 * index;
        }
        else if(node.name.endsWith("t"))
        {
            cx = node.x + 3 * index;
            cy = node.y - 2;
        }
        else if(node.name.endsWith("b"))
        {
            cx = node.x + 3 * index;
            cy = node.y + node.height + 2;
        }

        return new Point(cx, cy);
    }

}


