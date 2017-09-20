package ru.biosoft.graph;

import java.awt.Point;
import java.util.HashMap;

public class MapPortFinder implements PortFinder
{
    private final HashMap<String, Point> portMap = new HashMap<>();

    public Point findPort(Node node, Edge edge, int x, int y)
    {
        String portName = null;
        if( edge.getFrom().equals(node) )
            portName = edge.getAttribute("inputPortName");
        else if( edge.getTo().equals(node) )
            portName = edge.getAttribute("outputPortName");

        if( portName == null || !portMap.containsKey(portName) )
            return new Point(x, y);

        Point relativePoint = portMap.get(portName);

        Point result = new Point(relativePoint);
        result.translate(node.x, node.y);
        return result;
    }

    public void addPort(String nodeName, Point p)
    {
        portMap.put(nodeName, p);
    }
}
