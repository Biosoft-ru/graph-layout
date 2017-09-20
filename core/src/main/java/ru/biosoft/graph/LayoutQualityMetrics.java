package ru.biosoft.graph;

import java.awt.*;
import java.text.DecimalFormat;

public class LayoutQualityMetrics
{
    public int nodeCount;

    public int edgeCount;

    public int width;

    public int height;

    /** Total number of edge bends. */
    public int bends;

    /** Total number of edge crossings. */
    public int crossings;

    /** Total number of confluences between fragments of the edge paths. */
    public int confluences;

    /** Number of edges without path. */
    // TODO: Remove after assertion that there's no such an edges
    public int withoutPath;

    /** Average edge length. */
    public double edgeLength;

    /** Ration of edge length dispersion to average edge length. */
    public double sigma;

    private LayoutQualityMetrics ( )
    {
    }

    public static LayoutQualityMetrics getMetrics ( Graph g )
    {
        LayoutQualityMetrics result = new LayoutQualityMetrics ( );
        result.estimate ( g );
        return result;
    }

    private void estimate ( Graph g )
    {
        nodeCount = g.nodeCount ( );
        edgeCount = g.edgeCount ( );

        Rectangle r = g.getBounds ( );
        width = r.width;
        height = r.height;

        for ( Object eO : g.edgeList )
        {
            Edge e = (Edge) eO;
            if ( e.getPath ( ) == null )
            {
                withoutPath++;
                continue;
            }

            Path lp = e.getPath ( );
            estimatePath ( g, lp );
            bends += ( lp.npoints - 2 );

            float len = 0;
            int dx = 0, dy = 0;
            for ( int i = 0; i < lp.npoints - 1; i++ )
            {
                dx = lp.xpoints[i] - lp.xpoints[i + 1];
                dy = lp.ypoints[i] - lp.ypoints[i + 1];

                len += Math.sqrt ( dx * dx + dy * dy );
            }

            edgeLength += len;
            sigma += len * len;
        }

        crossings /= 2;
        confluences /= 2;
        edgeLength /= g.edgeCount ( );
        sigma = Math.sqrt ( sigma / g.edgeCount ( ) - edgeLength * edgeLength ) / edgeLength;
    }

    private void estimatePath ( Graph g, Path lp )
    {
        for ( int i = 0; i < lp.npoints - 1; i++ )
        {
            int xFrom = Math.min ( lp.xpoints[i], lp.xpoints[i + 1] ) - 1;
            int yFrom = Math.min ( lp.ypoints[i], lp.ypoints[i + 1] ) - 1;
            int width = Math.abs ( lp.xpoints[i] - lp.xpoints[i + 1] ) + 1;
            int height = Math.abs ( lp.ypoints[i] - lp.ypoints[i + 1] ) + 1;

            Rectangle r = new Rectangle ( xFrom, yFrom, width, height );

            for ( Object eO : g.edgeList )
            {
                Edge e = (Edge) eO;
                // TODO: check "equals"
                if ( e.getPath ( ) != null && e.getPath ( ) != lp )
                    estimatePath ( r, e.getPath ( ) );
            }
        }
    }

    private void estimatePath ( Rectangle r, Path lp )
    {
        for ( int i = 0; i < lp.npoints - 1; i++ )
        {
            int xFrom = Math.min ( lp.xpoints[i], lp.xpoints[i + 1] ) - 1;
            int yFrom = Math.min ( lp.ypoints[i], lp.ypoints[i + 1] ) - 1;
            int width = Math.abs ( lp.xpoints[i] - lp.xpoints[i + 1] ) + 1;
            int height = Math.abs ( lp.ypoints[i] - lp.ypoints[i + 1] ) + 1;

            if ( r.intersects ( xFrom, yFrom, width, height ) )
            {
                crossings++;

                if ( width == r.width && r.width == 1 )
                    confluences++;

                if ( height == r.height && r.height == 1 )
                    confluences++;
            }
        }
    }

    public static final DecimalFormat df = new DecimalFormat ( "#.###" );

    public static final String SEPARATOR = ";";

    public static final String HEADER = "Nodes" + SEPARATOR + "Edges" + SEPARATOR + "Degree" + SEPARATOR + "Width" + SEPARATOR + "Height"
            + SEPARATOR + "Ratio" + SEPARATOR + "Bends" + SEPARATOR + "Crossings" + SEPARATOR + "Confluences" + SEPARATOR + "WithoutPath"
            + SEPARATOR + "EdgeLength" + SEPARATOR + "Sigma" + SEPARATOR;

    public String getDebugInfo()
    {
        String INGROUP_SEPARATOR = ": ";
        String GROUP_SEPARATOR = ",   ";
        StringBuffer res = new StringBuffer ( );

        res.append ( "Width" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( width );
        res.append ( GROUP_SEPARATOR );
        
        res.append ( "Height" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( height );
        res.append ( GROUP_SEPARATOR );
        
        res.append ( "Ratio" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( df.format ( ( float ) width / ( float ) height ) );
        res.append ( GROUP_SEPARATOR );

        res.append ( "Bends" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( bends );
        res.append ( GROUP_SEPARATOR );

        res.append ( "Crossings" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( crossings );
        res.append ( GROUP_SEPARATOR );

        res.append ( "Confluences" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( confluences );
        res.append ( GROUP_SEPARATOR );

        res.append ( "WithoutPath" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( withoutPath );
        res.append ( GROUP_SEPARATOR );

        res.append ( "EdgeLength" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( df.format ( edgeLength ) );
        res.append ( GROUP_SEPARATOR );

        res.append ( "Sigma" );
        res.append ( INGROUP_SEPARATOR );
        res.append ( df.format ( sigma ) );
        res.append ( GROUP_SEPARATOR );

        return res.toString ( );
    }

    @Override
    public String toString ( )
    {
        StringBuffer res = new StringBuffer ( );
        res.append ( nodeCount );
        res.append ( SEPARATOR );
        res.append ( edgeCount );
        res.append ( SEPARATOR );
        res.append ( df.format ( ( float ) edgeCount / ( float ) nodeCount ) );
        res.append ( SEPARATOR );
        res.append ( width );
        res.append ( SEPARATOR );
        res.append ( height );
        res.append ( SEPARATOR );
        res.append ( df.format ( ( float ) width / ( float ) height ) );
        res.append ( SEPARATOR );
        res.append ( bends );
        res.append ( SEPARATOR );
        res.append ( crossings );
        res.append ( SEPARATOR );
        res.append ( confluences );
        res.append ( SEPARATOR );
        res.append ( withoutPath );
        res.append ( SEPARATOR );
        res.append ( df.format ( edgeLength ) );
        res.append ( SEPARATOR );
        res.append ( df.format ( sigma ) );
        res.append ( SEPARATOR );
        return res.toString ( );
    }

}
