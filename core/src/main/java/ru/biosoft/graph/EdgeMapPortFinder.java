package ru.biosoft.graph;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

public class EdgeMapPortFinder implements PortFinder
{
    private final HashMap<Object, List<Point>> portMap = new HashMap<>();

    @Override
    public Point findPort(Node node, Edge edge, int x, int y)
    {
        Object id = edge.applicationData;
        List<Point> points = portMap.get(id);
        if( points == null )
            return new Point(x, y);

        Iterator<Point> iter = points.iterator();
        Point nearestPoint = new Point(iter.next());
        nearestPoint.translate(node.x, node.y);
        double minDistance = Math.abs(nearestPoint.x - x) + Math.abs(nearestPoint.y - y);
        while( iter.hasNext() )
        {
            Point nextPoint = new Point(iter.next());
            nextPoint.translate(node.x, node.y);
            double distance = Math.abs(nextPoint.x - x) + Math.abs(nextPoint.y - y);
            if( distance < minDistance )
            {
                nearestPoint = new Point(nextPoint);
                minDistance = distance;
            }
        }
        return nearestPoint;
    }

    public void addPort(Object appData, Point p)
    {
        List<Point> points = portMap.get(appData);
        if( points == null )
        {
            points = new ArrayList<>();
            portMap.put(appData, points);
        }
        points.add(p);
    }
}
