package ru.biosoft.graph;

import ru.biosoft.graph.Edge;
import ru.biosoft.graph.Graph;
import ru.biosoft.graph.LayoutQualityMetrics;
import ru.biosoft.graph.Node;
import ru.biosoft.graph.Path;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Iterator;

public class SimpleImageGenerator
{
    public static final int DIAGRAM_INSET = 50;
    public static final int BLOCK_TEXT_INSET = 3;

    public static final int ARROW_WIDTH = 7;
    public static final int ARROW_HEIGHT = 3;

    public static final int METRICS_HEIGHT = 200;

    public static final Font font = new Font( "Courier", Font.BOLD, 10 );

    public static final Font fontMetrics = new Font( "Arial", Font.BOLD, 48 );

    public static void writeImageToFile( Graph graph, String path, boolean includeMetrics, String extra ) throws IOException
    {
        try (FileOutputStream fos = new FileOutputStream( path ))
        {
            BufferedImage bi = drawGraph( graph, includeMetrics, extra );
            ImageIO.write( bi, "png", fos );
        }
    }

    public static BufferedImage drawGraph( Graph graph, boolean includeMetrics, String extra )
    {
        Rectangle bounds = graph.getBounds();
        int xmin = ( int ) bounds.getMinX();
        int ymin = ( int ) bounds.getMinY();
        int xmax = ( int ) bounds.getMaxX();
        int ymax = ( int ) bounds.getMaxY();

        int width = xmax - xmin + 2 * DIAGRAM_INSET;
        int heigth = ymax - ymin + 2 * DIAGRAM_INSET;

        BufferedImage bi = new BufferedImage( width, heigth, BufferedImage.TYPE_BYTE_INDEXED );
        Graphics2D g2d = bi.createGraphics();
        g2d.setBackground( new java.awt.Color( 255, 255, 255, 0 ) );
        g2d.clearRect( 0, 0, width, heigth );

        if (includeMetrics)
        {
            g2d.setColor( Color.RED );
            g2d.setFont( fontMetrics );

            LayoutQualityMetrics m = LayoutQualityMetrics.getMetrics( graph );
            g2d.drawString( m.getDebugInfo(), DIAGRAM_INSET, heigth - METRICS_HEIGHT / 2);

            g2d.drawString( extra, DIAGRAM_INSET, heigth - METRICS_HEIGHT );
        }

        g2d.translate( -xmin + DIAGRAM_INSET, -ymin + DIAGRAM_INSET );

        g2d.setColor( Color.BLACK );
        Iterator<Edge> edgeIterator = graph.edgeIterator();
        while ( edgeIterator.hasNext() )
        {
            Edge edge = edgeIterator.next();
            drawEdge( g2d, edge.getPath() );
        }

        g2d.setColor( Color.BLUE );
        g2d.setFont( font );
        Iterator<Node> nodeIterator = graph.nodeIterator();
        while ( nodeIterator.hasNext() )
        {
            Node node = nodeIterator.next();
            drawNode( g2d, node );
        }

        return bi;
    }

    private static void drawNode( Graphics g2d, Node node )
    {
        Rectangle bounds = node.getBounds();
        g2d.drawRect( ( int ) bounds.getX(), ( int ) bounds.getY(), ( int ) bounds.getWidth(), ( int ) bounds.getHeight() );

        int textStart = ( int ) bounds.getX() + BLOCK_TEXT_INSET;
        int y = ( int ) bounds.getCenterY();
        g2d.drawString( node.getName(), textStart, y );
    }

    private static void drawEdge( Graphics2D g2d, Path path )
    {
        if ( path == null || path.npoints < 2)
            return;
        g2d.setColor( Color.BLACK );
        g2d.drawPolyline( path.xpoints, path.ypoints, path.npoints );

        Path arrow = new Path();
        arrow.addPoint( -ARROW_WIDTH, ARROW_HEIGHT );
        arrow.addPoint( 0, 0 );
        arrow.addPoint( -ARROW_WIDTH, -ARROW_HEIGHT );

        AffineTransform at = new AffineTransform();
        at.translate( path.xpoints[path.npoints - 1], path.ypoints[path.npoints - 1] );
        at.rotate( path.getLastSegmentAngle() );

        Shape realArrow = at.createTransformedShape( arrow );
        g2d.draw( realArrow );
        g2d.fill( realArrow );
    }

}
