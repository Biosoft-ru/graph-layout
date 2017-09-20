package ru.biosoft.graph;

import java.util.ArrayList;
import java.util.List;

/**
 * <code>OrthogonalPathFinder</code> tries to find orthogonal path(s) that connects
 * two specified points. Orthogonal path is the path which consists only from
 * vertical or horizontal lines.
 *
 * The main idea of the algorithm is following:
 * if initial two points can not be connected by vertical or horizontal line,
 * then the diagonal connecting this two points is divided in two parts - vertical
 * and horizontal lines. For each part new OrthogonalPathFinder is created.
 * The newly created OrthogonalPathFinders will try to solve the simplified task -
 * connect two points located on vertical and horizontal line.
 *
 * <p>If the path does not cross any nodes, it is suggested that
 * it has found the solution. When both finders (for vertical and horizontal part)
 * have found the solution, then parent finder is also suggested to have a solution.</p>
 *
 * <p>If vertical or horizontal line finder intersects some node,
 * then we locate 2 points (for example for horizontal line it will be
 * points upper and below the intersected node), and generate new path finders
 * that will try to make new paths through this points.</p>
 *
 * <p>Finally the algoritm merges (concats) paths from subasses.</p>
 *
 * <p>The algorithm can generate not one but several possible
 * paths, further {@link OrthogonalPathLayouter} can select best of them
 * using {@link PathWeighter}.</p>
 *
 * <p>Indeed, the diagonal path (x1,y1)-(x2, y2) can be split in horizontal
 *  and vertical line by two ways:
 *  1)  (x1,y1)-(x2,y1), (x2,y1)-(x2,y2)
 *  2)  (x1,y1)-(x2,y1), (x2,y1)-(x2,y2)
 * </p>
 *
 * <p>Similarly, two paths will be generated if line intersects some node.
 * For example for horizontal line one path goes upper the intersected node
 * and second path goes below the intersected node.</p>
 *
 * @see OrthogonalPathLayouter
 */
public class OrthogonalPathFinder
{
    private final Graph graph;
    private final int x1, y1, x2, y2;
    private final int gridX, gridY;

    /**
     * Creates instance that tries to find orthogonal path(es) that
     * connects two specified points.
     *
     * @param graph - graph for which path is finding.
     * It is needed to escape graph nodes intersection by the path.
     * @param x1 - start point x coordinate
     * @param y1 - start point y coordinate
     * @param x2 - end point x coordinate
     * @param y2 - end point y coordinate
     * @param gridX - x grid step
     * @param gridY - y grid step.
     */
    public OrthogonalPathFinder(Graph graph, int x1, int y1, int x2, int y2,
                                int gridX, int gridY)
    {
        this.graph = graph;

        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;

        this.gridX = gridX;
        this.gridY = gridY;

        nextStep = true;
        solution = false;
    }

    private boolean nextStep;
    /**
     * Indicates whether path finder make next step to find the solution.
     * It is false if all paths already have a solution.
     */
    public boolean hasNextStep()
    {
        return nextStep;
    }

    private boolean solution;
    /** Indicates whether path finder have find at least one path (solution). */
    public boolean hasSolution()
    {
        return solution;
    }

    List<Path> pathes = new ArrayList<>();
    /**
     * List of different orthogonal paths that connect two specified points.
     *
     * See the algorithm description why several paths are possible.
     */
    public List<Path> getSolutions()
    {
        return pathes;
    }

    private OrthogonalPathFinder pf1 = null;
    private OrthogonalPathFinder pf2 = null;
    private OrthogonalPathFinder pf3 = null;
    private OrthogonalPathFinder pf4 = null;

    /**
     * Makes next step to find a solution.
     *
     * If vertical or horizontal line finder intersects some node,
     * then we locate 2 points (for example for horizontal line it will be
     * points upper and below the intersected node), and generate new path finders
     * that will try to make new paths through this points.
     *
     * If initial two points can not be connected by vertical or horizontal line,
     * then the diagonal connecting this two points is divided in two parts - vertical
     * and horizontal lines. For each part new OrthogonalPathFinder is created.
     * The newly created OrthogonalPathFinders will try to solve the simplified task -
     * connect two points located on vertical and horizontal line.</p>
     */
    public void nextStep()
    {
        if( !nextStep )
            return;

        if( pf1 != null )
        {
            pf1.nextStep();
            pf2.nextStep();

            if( pf1.hasSolution() && pf2.hasSolution() )
            {
                solution = true;

                List<Path> listFrom = pf1.getSolutions();
                List<Path> listTo   = pf2.getSolutions();

                for ( Path pFrom : listFrom )
                    for ( Path pTo : listTo )
                        pathes.add( Path.concat( pFrom, pTo ) );

                pf1 = null;
                pf2 = null;

                if( pf3 == null )
                    nextStep = false;
            }
        }

        if( pf3 != null )
        {
            pf3.nextStep();
            pf4.nextStep();

            if( pf3.hasSolution() && pf4.hasSolution() )
            {
                solution = true;

                List<Path> listFrom = pf3.getSolutions();
                List<Path> listTo   = pf4.getSolutions();

                for ( Path pFrom : listFrom )
                    for ( Path pTo : listTo )
                        pathes.add( Path.concat( pFrom, pTo ) );

                pf3 = null;
                pf4 = null;

                if( pf1 == null )
                    nextStep = false;
            }
        }

        if( pf1 != null || pf3 != null || !nextStep )
            return;

        // check whether it is vertical or horizontal line
        if( x1 != x2 && y1 != y2 ) // diagonal
        {
            pf1 = new OrthogonalPathFinder(graph, x1, y1, x1, y2, gridX, gridY);
            pf2 = new OrthogonalPathFinder(graph, x1, y2, x2, y2, gridX, gridY);

            pf3 = new OrthogonalPathFinder(graph, x1, y1, x2, y1, gridX, gridY);
            pf4 = new OrthogonalPathFinder(graph, x2, y1, x2, y2, gridX, gridY);

            return;
        }

        if(x1 == x2 ) // vertical line
        {
            Node node = graph.getIntersectedNode(x1-gridX, y1, x1+gridX, y2);
            if( node == null )
            {
                makeSimplePath();
                return;
            }
            else
            {
                int y = (node.y + node.height/2) /gridY*gridY;

                int xl = (node.x) /gridX*gridX - 2*gridX;
                int xr = (node.x + node.width) /gridX*gridX + 2*gridX;

                pf1 = new OrthogonalPathFinder(graph, x1, y1, xl, y, gridX, gridY);
                pf2 = new OrthogonalPathFinder(graph, xl, y, x2, y2, gridX, gridY);

                pf3 = new OrthogonalPathFinder(graph, x1, y1, xr, y, gridX, gridY);
                pf4 = new OrthogonalPathFinder(graph, xr, y, x2, y2, gridX, gridY);
            }
        }

        else // horizontal line, y1==y2
        {
            Node node = graph.getIntersectedNode(x1, y1-gridY, x2, y2+gridY);
            if( node == null )
            {
                makeSimplePath();
                return;
            }
            else
            {
                int x = (node.x + node.width/2) /gridX*gridX;

                int yt = (node.y) /gridY*gridY - 2*gridY;
                int yb = (node.y + node.height) /gridY*gridY + 2*gridY;

                pf1 = new OrthogonalPathFinder(graph, x1, y1, x, yt, gridX, gridY);
                pf2 = new OrthogonalPathFinder(graph, x, yt, x2, y2, gridX, gridY);

                pf3 = new OrthogonalPathFinder(graph, x1, y1, x, yb, gridX, gridY);
                pf4 = new OrthogonalPathFinder(graph, x, yb, x2, y2, gridX, gridY);
            }
        }
    }

    /** Utility method to make path for horizontal or vertical line. */
    protected void makeSimplePath()
    {
        Path path = new Path();
        path.addPoint(x1, y1);
        path.addPoint(x2, y2);

        pathes.add(path);

        nextStep = false;
        solution = true;
    }
}
