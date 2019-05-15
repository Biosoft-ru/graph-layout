package ru.biosoft.graph;

/**
 * Wrapper for path layouter to provide possibility to select layouter by name
 */

public class PathLayouterWrapper
{
    private static String ORTHOGONAL = "Orthogonal Layouter";
    private static String DIAGONAL = "Diagonal Layouter";
    private static String HIERARCHIC = "Hierarchic Layouter";

    private String pathLayouterName;
    private Layouter pathLayouter;

    public PathLayouterWrapper(Layouter layouter)
    {
        pathLayouter = layouter;
        pathLayouterName = getLayouterNameByClass( layouter );
    }

    public PathLayouterWrapper()
    {
    }

    public static String[] getTags()
    {
        return new String[] {ORTHOGONAL, DIAGONAL, HIERARCHIC};
    }

    public boolean hiddenOptions()
    {
        return ! ( pathLayouter instanceof OrthogonalPathLayouter );
    }

    public static Layouter getPathLayouterByName(String name)
    {
        if( name.equals( ORTHOGONAL ) )
        {
            return new OrthogonalPathLayouter();
        }
        else if( name.equals( DIAGONAL ) )
        {
            return new DiagonalPathLayouter();
        }
        else if( name.equals( HIERARCHIC ) )
        {
            return new HierarchicPathLayouter();
        }
        return null;
    }

    private String getLayouterNameByClass(Layouter layouter)
    {
        if( layouter instanceof OrthogonalPathLayouter )
        {
            return ORTHOGONAL;
        }
        else if( layouter instanceof HierarchicPathLayouter )
        {
            return HIERARCHIC;
        }
        else if( layouter instanceof DiagonalPathLayouter )
        {
            return DIAGONAL;
        }
        else
        {
            return null;
        }
    }

    public String getPathLayouterName()
    {
        return pathLayouterName;
    }

    public void setPathLayouterName(String pathLayouterName)
    {
        if( !this.pathLayouterName.equals( pathLayouterName ) )
        {
            this.pathLayouterName = pathLayouterName;
            setPathLayouterOptions( getPathLayouterByName( pathLayouterName ) );
        }
    }

    public Layouter getPathLayouter()
    {
        return pathLayouter;
    }

    public void setPathLayouterOptions(Layouter pathLayouterOptions)
    {
        this.pathLayouter = pathLayouterOptions;
    }

    public void setPathLayouter(Layouter pathLayouter)
    {
        setPathLayouterOptions( pathLayouter );
        pathLayouterName = getLayouterNameByClass( pathLayouter );
    }
}
