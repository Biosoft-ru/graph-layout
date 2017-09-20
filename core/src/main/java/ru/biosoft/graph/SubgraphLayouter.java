package ru.biosoft.graph;

import java.awt.Dimension;
import java.awt.Point;
import java.awt.Rectangle;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

public class SubgraphLayouter
{
    protected static final Logger log = Logger.getLogger(SubgraphLayouter.class.getName());

    /** Horisontal distance between subgraphs. */
    public int layerDeltaX = 350;
    public int getLayerDeltaX()
    {
        return layerDeltaX;
    }
    public void setLayerDeltaX(int layerDeltaX)
    {
        this.layerDeltaX = layerDeltaX;
    }

    /** Vertical distance between subgraphs. */
    public int layerDeltaY = 350;
    public int getLayerDeltaY()
    {
        return layerDeltaY;
    }
    public void setLayerDeltaY(int layerDeltaY)
    {
        this.layerDeltaY = layerDeltaY;
    }
    
    /**
     * Represents currently stacked rectangles and allows to add more rectangle to stack greedy, but efficiently 
     * @author lan
     */
    private static class RectangleStack
    {
        int width = 0;
        int height = 0;
        int gapWidth = 0;
        int gapHeight = 0;
        
        /* Already placed rectangles are surrounded by this figure:
         * 
         *    /----------- width ---------------\
         *   h|                                 |
         *   e|                                 |
         *   i|                                 |
         *   g|          /--- gapWidth ---------/
         *   h|          |
         *   t|          |gapHeight
         *    |          |
         *    \----------/
         *    
         */
        public Point addRectangle(Dimension size)
        {
            if(size.width <= 0 || size.height <= 0) return new Point(0,0);
            // Try to put new box into gap
            if(size.width<=gapWidth && size.height<=gapHeight)
            {
                Point result = new Point(width-gapWidth, height-gapHeight);
                if((gapWidth-size.width)*gapHeight > (gapHeight-size.height)*gapWidth)
                    gapWidth-=size.width;
                else
                    gapHeight-=size.height;
                return result;
            }
            // Cannot do this: remove gap and try to put it side by side
            // decide where to put new rectangle: on the right or on the bottom
            double ratio1 = ((double)width+size.width)/(Math.max(height, size.height));
            double ratio2 = ((double)height+size.height)/Math.max(width, size.width);
            if(ratio1 > ratio2)
            {   // place at the bottom
                height+=size.height;
                if(size.width >= width)
                {
                    width = size.width;
                    gapWidth = gapHeight = 0;
                } else
                {
                    if((gapHeight+size.height)*Math.min(width-size.width, gapWidth)>size.height*(width-size.width))
                    {
                        gapHeight += size.height;
                        gapWidth = Math.min(width-size.width, gapWidth);
                    } else
                    {
                        gapHeight = size.height;
                        gapWidth = width-size.width;
                    }
                }
                return new Point(0, height-size.height);
            } else
            {   // place at the right
                width+=size.width;
                if(size.height > height)
                {
                    height = size.height;
                    gapWidth = gapHeight = 0;
                } else
                {
                    if((gapWidth+size.width)*Math.min(height-size.height, gapHeight)>size.width*(height-size.height))
                    {
                        gapWidth += size.width;
                        gapHeight = Math.min(height-size.height, gapHeight);
                    } else
                    {
                        gapWidth = size.width;
                        gapHeight = height-size.height;
                    }
                }
                return new Point(width-size.width, 0);
            }
        }
    }

    /**
     * Assigns a new graph layout to the given graph.
     * 
     * First step of the algorithm is split the graph into unconnected subgraphs
     * using {@see Graph.split} method.
     * 
     * <p>
     * Then method layouts each subgraph independently and arrange initial
     * graphs by following way: - subgraphs consisting from one node will be
     * located in top row, one by one from left to right; - subgraphs consisting
     * from two or more nodes will be located in second row, one by one from
     * left to right.
     * 
     * @param initialGraph - graph to be layouted.
     */
    public void doLayout(AbstractLayouter graphLayouter, Graph initialGraph, LayoutJobControl lJC)
    {
        List<Graph> graphs = initialGraph.split();
        if (lJC != null) lJC.begin();

        // TODO: Sort layers by node count
        // arrange first layer - single nodes
        RectangleStack rectangleStack = new RectangleStack();
        //Rectangle rect = new Rectangle();
//        int n = 0;
        for( Graph graph : graphs )
        {
            if( graph.nodeCount() != 1 )
                continue;

            // process selfloops
            if( graphLayouter != null && graph.edgeCount() > 0 )
            {
                graphLayouter.layoutNodes(graph, lJC);
                graphLayouter.layoutEdges(graph, lJC);
            }

            Rectangle bounds = graph.getBounds();
            Point pos = rectangleStack.addRectangle(new Dimension(bounds.width+layerDeltaX, bounds.height+layerDeltaY));

            graph.setLocation(pos.x, pos.y + layerDeltaY);
/*            graph.setLocation(rect.x + rect.width, rect.y);

            rect = (Rectangle)rect.createUnion(graph.getBounds());
            rect.width += layerDeltaX;*/

/*            n++;
            if( n / 50 * 50 == n )
                rect = new Rectangle(0, rect.y + rect.height + layerDeltaY, 0, 0);*/
        }

        // arrange second layer - subgraphs
        for( Graph graph : graphs )
        {
            if( graph.nodeCount() == 1 )
                continue;

            if( graphLayouter != null )
                graphLayouter.layoutNodes(graph, lJC);
            
            Rectangle bounds = graph.getBounds();
            Point pos = rectangleStack.addRectangle(new Dimension(bounds.width+layerDeltaX, bounds.height+layerDeltaY));

            graph.setLocation(pos.x, pos.y + layerDeltaY);

            if( graphLayouter != null )
                graphLayouter.layoutEdges(graph, lJC);

            savePath(initialGraph, graph);
            
        }

        // self control
        int nodeCount = 0;
        int edgeCount = 0;
        for( Graph graph1 : graphs )
        {
            nodeCount += graph1.nodeCount();
            edgeCount += graph1.edgeCount();
        }

        if( nodeCount != initialGraph.nodeCount() )
            log.log(Level.SEVERE, "Error in graph split, icorrect node count: " + nodeCount + " but should be " + initialGraph.nodeCount());

        if( edgeCount != initialGraph.edgeCount() )
        	log.log(Level.SEVERE, "Error in graph split, icorrect edge count: " + edgeCount + " but should be " + initialGraph.edgeCount());
    }
    
    protected void savePath(Graph initialGraph, Graph processedGraph)
    {
        Iterator<Edge> iterLevel = processedGraph.edgeIterator();

        while( iterLevel.hasNext() )
        {
            Edge currLevel = iterLevel.next();
            if( !currLevel.master )
                continue;
            Iterator<Edge> iterInitial = initialGraph.edgeIterator();
            while( iterInitial.hasNext() )
            {
                Edge currInitial = iterInitial.next();
                if( !currInitial.master )
                    continue;
                if( currInitial.isReversed() )
                    currInitial.reverseDirection();// for hierarchicalLayouter
                if( currLevel.from.name.equals(currInitial.from.name) && currLevel.to.name.equals(currInitial.to.name) )
                {
                    currInitial.path = currLevel.path;
                    if( currInitial.slaves != null )
                    {
                        if( currLevel.slaves != null && currInitial.slaves.size() == currLevel.slaves.size() )
                        {
                            for( int j = 0; j < currInitial.slaves.size(); j++ )
                            {
                                currInitial.slaves.get(j).path = currLevel.slaves.get(j).path;
                            }
                        }
                        else
                        {
                            for( Edge slave : currInitial.slaves )
                                slave.path = new Path(currLevel.path.xpoints, currLevel.path.ypoints, currLevel.path.npoints);
                        }
                    }
                }
            }
        }
    }
    
}
