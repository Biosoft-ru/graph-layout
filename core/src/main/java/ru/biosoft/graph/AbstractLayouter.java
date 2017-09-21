package ru.biosoft.graph;

import java.util.logging.Level;
import java.util.logging.Logger;

public abstract class AbstractLayouter implements Layouter
{
    protected static final Logger log = Logger.getLogger(AbstractLayouter.class.getName());

    ///////////////////////////////////////////////////////////////////
    // properties
    //

    protected int operationsDone = 0;

    protected PathWeighter pathWeighter = new PathWeighter();
    public PathWeighter getPathWeighter()
    {
         return pathWeighter;
    }
    public void setPathWeighter(PathWeighter pathWeighter)
    {
        this.pathWeighter = pathWeighter;
    }

    protected SubgraphLayouter sl = new SubgraphLayouter();
    public SubgraphLayouter getSubgraphLayouter()
    {
        return sl;
    }
    public void setSubgraphLayouter(SubgraphLayouter subgraphLayouter)
    {
        sl = subgraphLayouter;
    }

    ///////////////////////////////////////////////////////////////////
    @Override
    public void doLayout(Graph graph, LayoutJobControl lJC)
    {
        long time = System.currentTimeMillis();

        if (lJC != null) lJC.begin();
        this.operationsDone = 0;
        if( graph.isConnected() )
        {
            layoutNodes(graph, lJC);
            layoutEdges(graph, lJC);
        }
        else
            sl.doLayout(this, graph, lJC);

        log.log(Level.FINE, "Layout time " + ( System.currentTimeMillis() - time ));
    }

    abstract void layoutNodes(Graph graph, LayoutJobControl lJC);

    @Override
    abstract public void layoutEdges(Graph graph, LayoutJobControl lJC);
}
