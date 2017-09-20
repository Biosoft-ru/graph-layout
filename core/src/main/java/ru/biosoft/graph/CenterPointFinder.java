package ru.biosoft.graph;

import java.awt.Point;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.List;

import ru.biosoft.graph.Edge;
import ru.biosoft.graph.Node;
import ru.biosoft.graph.PortFinder;

/**
 * Port finder with appropriate points in the middle of left, right, bottom and top border
 * Point coordinates are stored in the relative to node coordinates
 */
public class CenterPointFinder implements PortFinder
{
    List<Point> points;

    public CenterPointFinder(Rectangle r)
    {
        int xc = r.width / 2;
        int yc = r.height / 2;

        points = new ArrayList<>();
        points.add(new Point(0, yc));
        points.add(new Point(r.width, yc));
        points.add(new Point(xc, 0));
        points.add(new Point(xc, r.height));
    }

    @Override
    public Point findPort(Node node, Edge edge, int x, int y)
    {
        Rectangle r = node.getBounds();
        double d = Double.MAX_VALUE;

        Point point = new Point(x, y);
        Point result = new Point(points.get(0).x + r.x, points.get(0).y + r.y); //first available point
        for( Point availableP : points )
        {
            Point availablePoint = new Point(availableP.x + r.x, availableP.y + r.y);
            double d1 = point.distance(availablePoint);
            if( d1 < d )
            {
                result = availablePoint;
                d = d1;
            }
            else if( d1 == d )
            {
                Node anotherNode = edge.getFrom().equals(node) ? edge.getTo() : edge.getFrom();
                Rectangle r2 = anotherNode.getBounds();
                Point p2 = new Point((int)r2.getCenterX(), (int)r2.getCenterY());
                if( availablePoint.distance(p2) < result.distance(p2) )
                {
                    result = availablePoint;
                    d = d1;
                }
            }
        }
        return result;
    }
}
