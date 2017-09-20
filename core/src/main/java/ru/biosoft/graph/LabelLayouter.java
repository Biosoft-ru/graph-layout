package ru.biosoft.graph;

import java.awt.Rectangle;
import java.util.Iterator;
import ru.biosoft.graph.LayoutJobControl;

public class LabelLayouter extends AbstractLayouter
{
    public static final String LABEL_SIZE   = "labelSize";
    public static final String LABEL_OFFSET = "labelOffset";
    public static final String LABEL_ANGLE  = "labelAngle";

    @Override
    public void layoutEdges(Graph graph, LayoutJobControl lJC)
    {
        // layout all labels
        Iterator<Node> iter = graph.nodeIterator();
        while( iter.hasNext() )
        {
            Node node = iter.next();
            if( node.hasAttribute(LABEL_SIZE) )
            {
                String[] str = node.getAttribute(LABEL_SIZE).split(";");
                if( str.length == 2 )
                {
                    int width = Integer.parseInt(str[0]);
                    int height = Integer.parseInt(str[1]);
                    layoutLabel(graph, node, width, height);
                }
            }
        }
    }

    @Override
    void layoutNodes(Graph graph, LayoutJobControl lJC)
    {
        // nothing to do
    }

    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl lJC)
    {
        // TODO: layout labels for connected nodes
    }

    protected void layoutLabel(Graph graph, Node node, int labelWidth, int labelHeight)
    {
        double distanceRecord = 0.0;
        Position positionRecord = null;
        int penaltyRecord = Integer.MAX_VALUE;
        for( Position position : getPosiblePositions() )
        {
            double distance = Math.sqrt(Math.cos(position.angle) * Math.cos(position.angle)
                    * ( ( node.height + labelHeight ) / 2.0 + position.offset ) * ( ( node.height + labelHeight ) / 2.0 + position.offset )
                    + Math.sin(position.angle) * Math.sin(position.angle) * ( ( node.width + labelWidth ) / 2.0 + position.offset )
                    * ( ( node.width + labelWidth ) / 2.0 + position.offset ));

            int penalty = calculateLabelPenalty(graph, node, labelWidth, labelHeight, distance, position.angle, penaltyRecord);
            if( penalty < penaltyRecord )
            {
                distanceRecord = distance;
                positionRecord = position;
                penaltyRecord = penalty;
                if( penalty == 0 )
                {
                    break;
                }
            }
        }
        node.setAttribute(LABEL_ANGLE, "" + ( positionRecord.angle - Math.PI / 2.0 ));
        node.setAttribute(LABEL_OFFSET, "" + (int)distanceRecord);
    }

    protected Position[] getPosiblePositions()
    {
        Position[] result = new Position[24];
        int pos = 0;

        for( int i = 0; i < 4; i++ )
        {
            Position p = new Position();
            p.offset = 5.0;
            p.angle = 2 * Math.PI - i * Math.PI / 2.0;
            result[pos++] = p;
        }

        for( int i = 0; i < 4; i++ )
        {
            Position p = new Position();
            p.offset = 0.0;
            p.angle = 2 * Math.PI - Math.PI / 4.0 - i * Math.PI / 2.0;
            result[pos++] = p;
        }

        for( int i = 0; i < 16; i++ )
        {
            Position p = new Position();
            p.offset = 10.0;
            p.angle = 2 * Math.PI - i * Math.PI / 8.0;
            result[pos++] = p;
        }

        return result;
    }

    public static final int NODE_INTERSECT_PENALTY = 10;
    public static final int EDGE_INTERSECT_PENALTY = 1;

    protected int calculateLabelPenalty(Graph graph, Node node, int labelWidth, int labelHeight, double distance, double angle, int record)
    {
        int result = 0;

        Rectangle rect = new Rectangle();
        rect.x = node.x + ( node.width - labelWidth ) / 2 - (int) ( distance * Math.sin(angle) );
        rect.y = node.y + ( node.height - labelHeight ) / 2 + (int) ( distance * Math.cos(angle) );
        rect.width = labelWidth;
        rect.height = labelHeight;

        Iterator<Node> nodeIterator = graph.nodeIterator();
        while( nodeIterator.hasNext() )
        {
            Node n = nodeIterator.next();
            if( rect.intersects(n.getBounds()) )
            {
                result += NODE_INTERSECT_PENALTY;
                if( result >= record )
                {
                    return record + 1;
                }
            }
        }

        Iterator<Edge> edgeIterator = graph.edgeIterator();
        while( edgeIterator.hasNext() )
        {
            Edge e = edgeIterator.next();
            Path path = e.getPath();
            if( path != null )
            {
                for( int i = 0; i < path.npoints - 1; i++ )
                {
                    int xFrom = Math.min(path.xpoints[i], path.xpoints[i + 1]) - 1;
                    int yFrom = Math.min(path.ypoints[i], path.ypoints[i + 1]) - 1;
                    int width = Math.abs(path.xpoints[i] - path.xpoints[i + 1]) + 1;
                    int height = Math.abs(path.ypoints[i] - path.ypoints[i + 1]) + 1;

                    if( ( new Rectangle(xFrom, yFrom, width, height) ).intersects(rect) )
                    {
                        result += EDGE_INTERSECT_PENALTY;
                        if( result >= record )
                        {
                            return record + 1;
                        }
                    }
                }
            }
        }

        return result;
    }

    protected class Position
    {
        public double angle;
        public double offset;
    }

    @Override
    public LayouterInfo getInfo()
    {
        LayouterInfoSupport lis = new LayouterInfoSupport(true, true, false, true, false, true);
        return lis;
    }


    @Override
    public int estimate(Graph graph, int what)
    {
        return 0;
    }
}
