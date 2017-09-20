package ru.biosoft.graph;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import ru.biosoft.graph.OrthogonalPathLayouter.Orientation;

public class OrientedPortFinder implements PortFinder
{
    
    private final Map<Object, Map<Orientation, List<Orientation>>> portMap = new HashMap<>();

    @Override
    public Point findPort(Node node, Edge edge, int x, int y)
    {
        Object id = edge.applicationData;
        Map<Orientation, List<Orientation>> orientationToPoints = portMap.get(id);
        if( orientationToPoints == null )
            return new Point(x, y);

        Orientation orientation = Orientation.fromString(node.getAttribute("orientation"));
        List<Orientation> points = orientationToPoints.get(orientation);

        Iterator<Orientation> iter = points.iterator();
        Point nearestPoint = getPoint(node, iter.next());
        if (!iter.hasNext())
            return nearestPoint;

        double minDistance = Math.abs(nearestPoint.x - x) + Math.abs(nearestPoint.y - y);
        while( iter.hasNext() )
        {
            Point nextPoint = getPoint(node, iter.next());
            double distance = Math.abs(nextPoint.x - x) + Math.abs(nextPoint.y - y);
            if( distance < minDistance )
            {
                nearestPoint = new Point(nextPoint);
                minDistance = distance;
            }
        }
        return nearestPoint;
    }
    

     //Node shape and location can change
    private Point getPoint(Node node, Orientation orientation)
    {
        switch (orientation)
        {
            case TOP:
                return new Point(node.x + node.width/2, node.y);
            case RIGHT:
                return new Point(node.x + node.width, node.y + node.height/2);
            case BOTTOM:
                return new Point(node.x + node.width/2, node.y + node.height);
            default:
                return new Point(node.x, node.y + node.height/2);
        }
    }
    

    public void addPort(Object appData, Orientation nodeOrientation, Orientation portOrientation)
    {
        Map<Orientation, List<Orientation>> orientationToPoints = portMap.get(appData);
        if( orientationToPoints == null )
        {
            Map<Orientation, List<Orientation>> map = new HashMap<>();
            List<Orientation> points = new ArrayList<>();
            points.add(portOrientation);
            map.put(nodeOrientation, points);
            portMap.put(appData, map);
            return;
        }
        else
        {
            List<Orientation> points = orientationToPoints.get(nodeOrientation);
            if( points == null )
            {
                points = new ArrayList<>();
                points.add(portOrientation);
                orientationToPoints.put(nodeOrientation, points);

            }
            else
            {
                points.add(portOrientation);
            }
        }
    }

}
