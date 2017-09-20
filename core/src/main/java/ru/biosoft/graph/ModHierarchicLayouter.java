package ru.biosoft.graph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ModHierarchicLayouter extends AbstractLayouter
{
    protected static final Logger log = Logger.getLogger(ModHierarchicLayouter.class.getName());

    protected Map<String, Object> edgesToRestore = new HashMap<>();

    public ModHierarchicLayouter()
    {
        configureForceDirectedLayouter();
    }

    // ////////////////////////////////////////////////////////////////////////
    // Properties
    //
    
    protected boolean verticalOrientation = true;
    public boolean isVerticalOrientation()
    {
        return verticalOrientation;
    }
    public void setVerticalOrientation(boolean verticalOrientation)
    {
        this.verticalOrientation = verticalOrientation;
    }

    protected boolean hoistNodes = false;
    public boolean isHoistNodes()
    {
        return hoistNodes;
    }
    public void setHoistNodes(boolean hoistNodes)
    {
        this.hoistNodes = hoistNodes;
    }

    protected int layerOrderIterationNum = 100;
    public int getLayerOrderIterationNum()
    {
        return layerOrderIterationNum;
    }
    public void setLayerOrderIterationNum(int layerOrderIterationNum)
    {
        this.layerOrderIterationNum = layerOrderIterationNum;
    }

    public static final int STRAIGHTEN_DEFAULT = 0;
    public static final int STRAIGHTEN_FORCE_DIRECTED = 1;

    protected int straightenMethod = STRAIGHTEN_DEFAULT;
    public int getStraightenMethod()
    {
        return straightenMethod;
    }
    public void setStraightenMethod(int straightenMethod)
    {
        this.straightenMethod = straightenMethod;
    }

    protected ForceDirectedLayouter forceDirectedLayouter = new ForceDirectedLayouter();
    public ForceDirectedLayouter getForceDirectedLayouter()
    {
        return forceDirectedLayouter;
    }
    public void setForceDirectedLayouter(ForceDirectedLayouter forceDirectedLayouter)
    {
        this.forceDirectedLayouter = forceDirectedLayouter;
    }

    protected int straightenIterationNum = 100;
    public int getStraightenIterationNum()
    {
        return straightenIterationNum;
    }
    public void setStraightenIterationNum(int straightenIterationNum)
    {
        this.straightenIterationNum = straightenIterationNum;
    }

    private boolean processNeighbours = false;
    public boolean isProcessNeighbours()
    {
        return processNeighbours;
    }
    public void setProcessNeighbours(boolean processNeighbours)
    {
        this.processNeighbours = processNeighbours;
    }

    protected int layerDeltaX = 30;
    public int getLayerDeltaX()
    {
        return layerDeltaX;
    }
    public void setLayerDeltaX(int layerDeltaX)
    {
        this.layerDeltaX = layerDeltaX;
    }

    protected int layerDeltaY = 30;
    public int getLayerDeltaY()
    {
        return layerDeltaY;
    }
    public void setLayerDeltaY(int layerDeltaY)
    {
        this.layerDeltaY = layerDeltaY;
    }


    protected int gridX = 10;

    protected int gridY = 10;

    protected int dummyGridX = 0;

    protected int dummyGridY = 0;

    protected int sameNameNodesWeight= 5;
    public int getsameNameNodesWeight()
    {
        return sameNameNodesWeight;
    }
    public void setsameNameNodesWeight(int sameNameNodesWeight)
    {
        this.sameNameNodesWeight = sameNameNodesWeight;
    }

    protected int dummyEdgesCoeff = 5;
    public int getdummyEdgesCoeff()
    {
        return dummyEdgesCoeff;
    }
    public void setdummyEdgesCoeff(int dummyEdgesCoeff)
    {
        this.dummyEdgesCoeff = dummyEdgesCoeff;
    }

    protected int scoreWeight = 10;
    public int getscoreWeight()
    {
        return scoreWeight;
    }
    public void setscoreWeight(int scoreWeight)
    {
        this.scoreWeight = scoreWeight;
    }

    protected int edgesCrossCoeff = 1000;
    public int getedgesCrossCoeff()
    {
        return edgesCrossCoeff;
    }

    public void setedgesCrossCoeff(int edgesCrossCoeff)
    {
        this.edgesCrossCoeff = edgesCrossCoeff;
    }


    // ////////////////////////////////////////////////////////////////////////

    @Override
    protected void layoutNodes(Graph graph, LayoutJobControl lJC)
    {
        addAlignmentDummies(graph);
        makeLevels(graph);
        addSameNodeEdges(graph);
        initLevelEdges(graph);
        placeNodesInitial();

        for( int j = 0; j < layerOrderIterationNum; j++ )
          orderNodes(graph, j);

         //straightenLayout(graph);
    }

    @Override
    public void layoutEdges(Graph graph, LayoutJobControl lJC)
    {
        // first iteration - remove all pathes
        for( Edge edge : graph.edgeList )
            edge.path = new Path();

        // second iteration - create simple pathes
        for( int i = 0; i < maxLevel; i++ )
        {
            List<ru.biosoft.graph.Node> nodes = levelNodes.get(i);
            for( ru.biosoft.graph.Node n : nodes )
            {
                List<Edge> outEdges = getOrderedEdgeSet(graph, n, false);
                int outEdgeNum = outEdges.size();
                for( int k = 0; k < outEdgeNum; k++ )
                {
                    if( outEdges.get(k) == null )
                        continue;

                    Edge edge = outEdges.get(k);
                    if( edge.master )
                    {
                        if( verticalOrientation )
                            edge.path.addPoint(n.x + n.width * ( k + 1 ) / ( outEdgeNum + 1 ), n.y + n.height);
                        else
                            edge.path.addPoint(n.x + n.width, n.y + n.height * ( k + 1 ) / ( outEdgeNum + 1 ));
                    }
                }

                List<Edge> inEdges = getOrderedEdgeSet(graph, n, true);
                int inEdgeNum = inEdges.size();
                for( int k = 0; k < inEdgeNum; k++ )
                {
                    Edge edge = inEdges.get(k);
                    if( edge == null )
                        continue;

                    if( edge.master )
                    {
                        if( verticalOrientation )
                            edge.path.addPoint(n.x + n.width * ( k + 1 ) / ( inEdgeNum + 1 ), n.y);
                        else
                            edge.path.addPoint(n.x, n.y + n.height * ( k + 1 ) / ( inEdgeNum + 1 ));
                    }
                }
            }
        }

        // third iteration - remove dummies and processing parallel edges
        removeDummies(graph);

        for( Edge edge : graph.edgeList )
        {
            if( edge.master && edge.slaves != null )
            {
                // calc the span between parallel edges
                int inEdgeNum = 0;
                List<Edge> edgeIn = graph.getEdges(edge.from);
                for( Edge e : edgeIn )
                {
                    if( e.from == edge.from )
                    {
                        inEdgeNum++;
                        if( e.slaves != null )
                            inEdgeNum += e.slaves.size();
                    }
                }

                int outEdgeNum = 0;
                List<Edge> edgeOut = graph.getEdges(edge.to);
                for( Edge e : edgeOut )
                {
                    if( e.to == edge.to )
                    {
                        outEdgeNum++;
                        if( e.slaves != null )
                            outEdgeNum += e.slaves.size();
                    }
                }

                float span = 5;
                if( verticalOrientation )
                    span = Math.min(gridX, Math.min(edge.from.width / ( inEdgeNum + 1 ), edge.to.width / ( outEdgeNum + 1 )));
                else
                    span = Math.min(gridY, Math.min(edge.from.height / ( inEdgeNum + 1 ), edge.to.height / ( outEdgeNum + 1 )));

                for( int i = 0; i < edge.slaves.size(); i++ )
                {
                    Edge slave = edge.slaves.get(i);
                    slave.path = parallelPath(edge.path, Math.round(span * ( i + 1 )));
                }
            }
        }

        // restore reversed edges
        for( int i = 0; i < graph.edgeList.size(); i++ )
        {
            Edge edge = graph.edgeList.get(i);
            if( edge.reversed )
            {
                graph.removeEdge(edge);
                edge.reverseDirection();
                graph.addEdge(edge);

                i--;
            }
        }

        if( selfLoopLayouter == null )
            selfLoopLayouter = new SelfLoopLayouter();

        // recover self loops
        for( Edge edge : selfLoops )
        {
            selfLoopLayouter.layoutPath(graph, edge, pathWeighter);
            graph.addEdge(edge);
        }
    }

    public SelfLoopLayouter selfLoopLayouter = new SelfLoopLayouter();

    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl lJC)
    {
        // TODO: separate or use DiagonalPath (currently duplicate)
        forceDirectedLayouter.layoutPath(graph, edge, lJC);
    }

    // Null element in the result means "dummy"
    protected List<Edge> getOrderedEdgeSet(Graph graph, ru.biosoft.graph.Node n, boolean in)
    {
        ArrayList<Edge> edges = new ArrayList<>();
        int level = getLevel(n) + ( in ? -2 : 0 );
        if( level >= 0 && level < maxLevel )
        {
            for( ru.biosoft.graph.Node u : levelNodes.get(level) )
            {
                Edge edge = in ? graph.getEdge(u, n) : graph.getEdge(n, u);
                if( edge != null )
                {
                    edges.add(edge);
                    if( !isDummy(n) && edge.slaves != null )
                        for( Edge slave : edge.slaves )
                            edges.add(slave);
                    if( isDummy(n) && edge.data != null )
                    {
                        int slavesCount = ( (List<?>)edge.data ).size();
                        for( int i = 0; i < slavesCount; i++ )
                            edges.add(null);
                    }
                }
            }
        }

        return edges;
    }

    protected Path parallelPath(Path originalPath, int offset)
    {
        Path path = new Path();

        int dx = verticalOrientation ? offset : 0;
        int dy = verticalOrientation ? 0 : offset;

        for( int i = 0; i < originalPath.npoints; i++ )
            path.addPoint(originalPath.xpoints[i] + dx, originalPath.ypoints[i] + dy);

        return path;
    }

    // //////////////////////////////////////////////////////////////////////////

    public void initLayoutData(Graph graph)
    {
        for( ru.biosoft.graph.Node n : graph.nodeList )
            n.data = new LayoutData();
    }

    protected void unmarkNodes(Graph graph)
    {
        for( ru.biosoft.graph.Node n : graph.nodeList )
            ( (LayoutData)n.data ).marked = false;
    }

    protected void printNodeLevels(Graph graph)
    {
        for( ru.biosoft.graph.Node node : graph.nodeList )
            System.out.print(node.name + ":" + ( (LayoutData)node.data ).level + ", ");
    }

    protected static double getScore(ru.biosoft.graph.Node node)
    {
        return ( (LayoutData)node.data ).score;
    }

    protected static void setScore(ru.biosoft.graph.Node node, double score)
    {
        ( (LayoutData)node.data ).score = score;
    }

    protected static int getLevel(ru.biosoft.graph.Node node)
    {
        return ( (LayoutData)node.data ).level;
    }

    protected static void setLevel(ru.biosoft.graph.Node node, int level)
    {
        ( (LayoutData)node.data ).level = level;
    }

    protected static int getUsageLevel(ru.biosoft.graph.Node node)
    {
        return ( (LayoutData)node.data ).usageLevel;
    }

    protected static void setUsageLevel(ru.biosoft.graph.Node node, int level)
    {
        ( (LayoutData)node.data ).usageLevel = level;
    }

    protected boolean isDummy(ru.biosoft.graph.Node node)
    {
        return ( (LayoutData)node.data ).dummy;
    }

    protected int getBarycenter(ru.biosoft.graph.Node node)
    {
        return ( (LayoutData)node.data ).barycenter;
    }

    protected int calcBarycenter(Graph graph, ru.biosoft.graph.Node node, boolean doIn, boolean doOut, boolean processSize)
    {
        int sum = 0;
        int n = 0;

        int center = 0;
        for( Edge edge : graph.getEdges(node) )
        {
            ru.biosoft.graph.Node u = null;

            if( edge.to == node && doIn )
                u = edge.from;
            else if( edge.from == node && doOut )
                u = edge.to;

            if( u != null )
            {
                if( verticalOrientation )
                    center = u.x;
                else
                    center = u.y;

                if( processSize )
                {
                    if( verticalOrientation )
                        center += u.width / 2;
                    else
                        center += u.height / 2;
                }

                sum += center;
                n++;

            }
        }

        if( processSize && processNeighbours )
        {
            // get neigthberhood nodes
            List<ru.biosoft.graph.Node> level = levelNodes.get(getLevel(node) - 1);
            int i;
            for( i = 0; i < level.size(); i++ )
            {
                if( level.get(i) == node )
                    break;
            }

            // process left node
            if( i > 0 )
            {
                ru.biosoft.graph.Node u = level.get(i - 1);
                if( verticalOrientation )
                    center = u.x + u.width / 2;
                else
                    center = u.y + u.height / 2;

                sum += center;
                n++;
            }

            // process right node node
            if( i < level.size() - 1 )
            {
                ru.biosoft.graph.Node u = level.get(i + 1);
                if( verticalOrientation )
                    center = u.x + u.width / 2;
                else
                    center = u.y + u.height / 2;

                sum += center;
                n++;
            }
        }

        if( n != 0 )
            sum = sum / n;
        else
        {
            if( verticalOrientation )
                sum = node.x + ( processSize ? node.width / 2 : 0 );
            else
                sum = node.y + ( processSize ? node.height / 2 : 0 );
        }

        ( (LayoutData)node.data ).barycenter = sum;
        return sum;
    }

    // //////////////////////////////////////////////////////////////////////////

    protected List<Edge> selfLoops;

    protected void normalise(Graph graph)
    {
        // remove self loops
        selfLoops = new ArrayList<>();

        for( int i = 0; i < graph.edgeList.size(); i++ )
        {
            Edge edge = graph.edgeList.get(i);
            if( edge.from == edge.to )
            {
                edge.path = null;
                selfLoops.add(edge);
                graph.removeEdge(edge);
                i--;
            }
        }

        // Break any cycles in the graph by reversing some edges
        unmarkNodes(graph);
        for( ru.biosoft.graph.Node node : graph.nodeList )
            if( ! ( (LayoutData)node.data ).marked )
                breakCycles(graph, node);
    }

    protected void breakCycles(Graph graph, ru.biosoft.graph.Node curr)
    {
        ( (LayoutData)curr.data ).marked = true;
        ( (LayoutData)curr.data ).picked = true;

        List<Edge> edges = graph.getEdges(curr);
        if( edges == null )
            return;

        for( int j = 0; j < edges.size(); j++ )
        {
            Edge edge = edges.get(j);
            if( edge.from == curr ) // get out edges
            {
                ru.biosoft.graph.Node n = edge.to;
                if( ( (LayoutData)n.data ).picked )
                {
                    log.log(Level.INFO, "reversed edge: " + edge + "<" + edge.slaves + ">");

                    graph.removeEdge(edge);
                    edge.reverseDirection();
                    graph.addEdge(edge);

                    j--;
                }
                else if( ! ( (LayoutData)n.data ).marked )
                    breakCycles(graph, n);
            }
        }

        ( (LayoutData)curr.data ).picked = false;
    }

    protected void assignNodeLevels(Graph graph)
    {
        // Do a topological sort on the meta-graph
        ArrayList<ru.biosoft.graph.Node> topoSortedNodes = new ArrayList<>();
        ru.biosoft.graph.Node metaRoot = makeMetaRoot(graph);
        unmarkNodes(graph);
        topoSort(graph, metaRoot, topoSortedNodes);
        graph.removeNode(metaRoot);
        topoSortedNodes.remove(metaRoot);

        // then assign levels to nodes.
        int toposize = topoSortedNodes.size();

        for( int i = 0; i < toposize; ++i )
        {

            ru.biosoft.graph.Node v = topoSortedNodes.get(i);
            setLevel(v, Integer.parseInt( ( v.getAttribute("level") ).trim()));
            setScore(v, Double.parseDouble( ( v.getAttribute("score") ).trim()));
        }
    }

    public static void topoSort(Graph graph, ru.biosoft.graph.Node curr, List<ru.biosoft.graph.Node> topoSortedNodes)
    {
        ( (LayoutData)curr.data ).marked = true;

        List<Edge> edges = graph.getEdges(curr);
        for( Edge edge : edges )
        {
            if( edge.to == curr )
            {
                ru.biosoft.graph.Node n = edge.from;
                if( ! ( (LayoutData)n.data ).marked )
                    topoSort(graph, n, topoSortedNodes);
            }
        }

        topoSortedNodes.add(curr);
    }

    protected void addSameNodeEdges(Graph graph)
    {
        boolean flag;
        for( int j = 0; j < graph.nodeCount(); j++ )
        {
            ru.biosoft.graph.Node v = graph.nodeAt(j);
            flag = false;
            for( int i = getLevel(v); i < maxLevel; i++ )
            {
                List<ru.biosoft.graph.Node> nodeList = levelNodes.get(i);
                for( ru.biosoft.graph.Node u : nodeList )
                {
                    if (v.getName().lastIndexOf("_")>-1 && u.getName().lastIndexOf("_")>-1)
                    {
                    String fromName = v.getName().substring(0,v.getName().lastIndexOf("_"));
                    String toName = u.getName().substring(0,u.getName().lastIndexOf("_"));
                    if (fromName.equalsIgnoreCase(toName))
                       {
                        Edge sameNameNodesEdge = new Edge(v, u);
                        sameNameNodesEdge.data = "dummyEdge";
                        graph.addEdge(sameNameNodesEdge);
                        flag = true;
                        break;
                       }
                    }
                }
                if (flag) break;
            }
        }


    }

    protected void addAlignmentDummies(Graph graph)
    {
        // if(graph.edgeCount()== 0)
        // {
        int maxLayer = 0;
        for( int j = 0; j < graph.nodeCount(); j++ )
        {
            int layer = Integer.parseInt(graph.nodeAt(j).getAttribute("level"));
            maxLayer = Math.max(layer, maxLayer);
        }

        ru.biosoft.graph.Node[] dummys = new ru.biosoft.graph.Node[maxLayer];

        dummys[maxLayer - 1] = new ru.biosoft.graph.Node("level" + ( maxLayer - 1 ) + ".alinement");

        if( verticalOrientation )
        {
            dummys[maxLayer - 1].width = dummyGridX;
            dummys[maxLayer - 1].height = dummyGridY;
        }
        else
        {
            dummys[maxLayer - 1].width = dummyGridX;
            dummys[maxLayer - 1].height = dummyGridY;
        }
        graph.addNode(dummys[maxLayer - 1]);
        dummys[maxLayer - 1].data = new LayoutData();
        ( (LayoutData)dummys[maxLayer - 1].data ).dummy = true;
        setLevel(dummys[maxLayer - 1], maxLayer);
        dummys[maxLayer-1].setAttribute("level", String.valueOf(maxLayer));


        for( int i = maxLayer - 2; i >= 0; i-- )
        {
            dummys[i] = new ru.biosoft.graph.Node("level" + i + ".alinement");

            if( verticalOrientation )
            {
                dummys[i].width = dummyGridX;
                dummys[i].height = dummyGridY;
            }
            else
            {
                dummys[i].width = dummyGridX;
                dummys[i].height = dummyGridY;
            }
            graph.addNode(dummys[i]);
            dummys[i].data = new LayoutData();
            ( (LayoutData)dummys[i].data ).dummy = true;
            setLevel(dummys[i], i + 1);
            dummys[i].setAttribute("level", String.valueOf(i+1));
            Edge edge = new Edge(dummys[i], dummys[i + 1]);
            graph.addEdge(edge);

        }
    }


//    protected void addLabeles(Graph graph, Object[] labelesData)
//    {
//
//        int maxLayer = 0;
//        for( int j = 0; j < graph.nodeCount(); j++ )
//        {
//            int layer = Integer.parseInt(graph.nodeAt(j).getAttribute("level"));
//            maxLayer = Math.max(layer, maxLayer);
//        }
//
//        ru.biosoft.graph.Node[] labeles = new ru.biosoft.graph.Node[maxLayer];
//
//        labeles[maxLayer - 1] = new ru.biosoft.graph.Node("level." + ( maxLayer - 1 ));
//
//        if( verticalOrientation )
//        {
//            labeles[maxLayer - 1].width = gridX;
//            labeles[maxLayer - 1].height = gridY;
//        }
//
//        else
//        {
//            labeles[maxLayer - 1].width = gridX;
//            labeles[maxLayer - 1].height = gridY;
//        }
//        graph.addNode(labeles[maxLayer - 1]);
//        labeles[maxLayer - 1].data = new LayoutData();
//        ( (LayoutData)labeles[maxLayer - 1].data ).label = true;
////        setLevel(labeles[maxLayer - 1], maxLayer);
//        labeles[maxLayer-1].setAttribute("level", String.valueOf(maxLayer));
//        labeles[maxLayer-1].setAttribute("score", String.valueOf(0));
//        labeles[maxLayer-1].applicationData = labelesData[maxLayer-1];
//
//
//        for( int i = maxLayer - 2; i >= 0; i-- )
//        {
//            labeles[i] = new ru.biosoft.graph.Node("level." + i);
//
//            if( verticalOrientation )
//            {
//                labeles[i].width = gridX;
//                labeles[i].height = gridY;
//            }
//
//            else
//            {
//                labeles[i].width = gridX;
//                labeles[i].height = gridY;
//            }
//            graph.addNode(labeles[i]);
//            labeles[i].data = new LayoutData();
//            ( (LayoutData)labeles[i].data ).label = true;
////           setLevel(labeles[i], i + 1);
//            labeles[i].setAttribute("level", String.valueOf(i+1));
//            labeles[i].setAttribute("score", String.valueOf(1));
//            labeles[i].applicationData = labelesData[i];
//
//        }
//    }

    protected void addDummies(Graph graph)
    {
        if( graph.edgeCount() != 0 )
        {
            int dummyCount = 1;
            for( int i = 0; i < graph.nodeCount(); ++i )
            {
                ru.biosoft.graph.Node to = graph.nodeList.get(i);
                if( ( (LayoutData)to.data ).dummy )
                    continue;

                List<Edge> edges = graph.getEdges(to);
                for( int j = 0; j < edges.size(); j++ )
                {
                    Edge edge = edges.get(j);
                    // MUST to clear original path to avoid invalid edge
                    // restoration at layoutEdges
                    edge.path = new Path();
                    if( edge.to == to ) // get in edges
                    {
                        if( ( (LayoutData)edge.from.data ).dummy )
                            continue;

                        if( getLevel(to) > getLevel(edge.from) + 1 )
                            j--;

                        // special treatment for parallel edges
                        List<Edge> slaves = edge.slaves;
                        List<Edge> slavesCopy = null;
                        int edgeNum = 1;
                        if( slaves != null )
                        {
                            edgeNum += slaves.size();
                            slavesCopy = new ArrayList<>( slaves );
                        }

                        // create dummy nodes
                        while( getLevel(to) > getLevel(edge.from) + 1 )
                        {
                            // create dummy node, assign the level and add it to
                            // the graph
                            ru.biosoft.graph.Node from = edge.from;
                            ru.biosoft.graph.Node dummy = new ru.biosoft.graph.Node("dummy." + dummyCount);

                            if( verticalOrientation )
                            {
                                dummy.width = dummyGridX;
                                dummy.height = dummyGridY;
                            }
                            else
                            {
                                dummy.width = dummyGridX;
                                dummy.height = dummyGridY;
                            }

                            dummyCount++;
                            graph.addNode(dummy);

                            dummy.data = new LayoutData();
                            setLevel(dummy, getLevel(from) + 1);
                            dummy.setAttribute("level", String.valueOf(getLevel(from) + 1));
                            if (edge.applicationData==null)
                            {
                            	System.out.println("Error here: "+edge.getFrom().getName()+" - "+edge.getTo().getName());
                            }
                            edgesToRestore.put(dummy.getName(), edge.applicationData);
                            ( (LayoutData)dummy.data ).dummy = true;

                            if( edgeNum > 1 )
                            {
                                if( verticalOrientation )
                                    dummy.width = dummyGridX * edgeNum;

                                else
                                    dummy.height = dummyGridY * edgeNum;
                            }

                            // create edges and add them to the graph
                            Edge in = new Edge(from, dummy);
                            in.data = slavesCopy;
                            in.reversed = edge.reversed;
                            graph.addEdge(in);

                            Edge out = new Edge(dummy, to);
                            out.data = slavesCopy;
                            out.reversed = edge.reversed;
                            graph.addEdge(out);

                            // remove edge in the graph and it slaves
                            if( slaves != null )
                                while( slaves.size() > 0 )
                                    graph.removeEdge(slaves.get(0));
                            graph.removeEdge(edge);

                            edge = out;
                        }
                    }
                }
            }
        }
    }

    protected void removeDummies(Graph graph)
    {
        for( int i = 0; i < graph.nodeCount(); ++i )
        {
            ru.biosoft.graph.Node from = graph.nodeList.get(i);
            if( isDummy(from) )
                continue;

            List<Edge> edges = graph.getEdges(from);
            int j = 0;
            while (j < edges.size())   //for( int j = 0; j < edges.size(); ++j )
            {
                Edge edge = edges.get(j++);

                // get out dummy edges
                if( edge.from == from && isDummy(edge.to) )
                {
                    Path line = new Path();
                    line.addPoint(edge.path.xpoints[0], edge.path.ypoints[0]);
                    line.addPoint(edge.path.xpoints[1], edge.path.ypoints[1]);

                    List<ru.biosoft.graph.Node> path = new ArrayList<>();
                    path.add(edge.to);

                    ru.biosoft.graph.Node to = edge.to;
                    //int pathLen = 1;
                    while( true )
                    {
                        List<Edge> dEdges = graph.getEdges(to);
                        for( int d = 0; d < 2; d++ )
                        {
                            Edge e = dEdges.get(d);
                            if( e.from == to )
                            {
                                to = e.to;
                                line.addPoint(e.path.xpoints[1], e.path.ypoints[1]);
                                break;
                            }
                        }

                        if( !isDummy(to) )
                            break;

                        path.add(to);
                        //pathLen++;
                    }

                    // restore original edge
                    Edge originalEdge = new Edge(from, to);
                    originalEdge.applicationData = edgesToRestore.get(path.get(0).getName());
                    originalEdge.reversed = edge.reversed;
                    originalEdge.path = line;
                    graph.addEdge(originalEdge);
                    originalEdge.master = true;

                    // restore parallel edges
                    List<?> slaves = (List<?>)edge.data;
                    if( slaves != null )
                        for( Object slave : slaves )
                            graph.addEdge((Edge)slave);

                    for( ru.biosoft.graph.Node aPath : path )
                        graph.removeNode(aPath);

//                    j -= pathLen;
//                    if( j < 0 )
                        j = 0;
                }
            }
        }
    }

    // //////////////////////////////////////////////////////////////////////////

    protected int maxLevel;

    protected ArrayList<List<ru.biosoft.graph.Node>> levelNodes;

    /**
     * Make and initialize levels.
     */
    protected void makeLevels(Graph graph)
    {
        // Find maximum level and node
        maxLevel = -1;
        ru.biosoft.graph.Node maxLevelNode = null;

        for( ru.biosoft.graph.Node n : graph.nodeList )
        {
            if( maxLevel < getLevel(n) )
            {
                maxLevel = getLevel(n);
                maxLevelNode = n;
            }
        }

        // Make and initialize levels.
        levelNodes = new ArrayList<>( maxLevel );
        for( int j = 0; j < maxLevel; j++ )
            levelNodes.add(new ArrayList<ru.biosoft.graph.Node>());

        unmarkNodes(graph);

        initialOrderNodes(graph, maxLevelNode); // DFS order most the nodes

        for( ru.biosoft.graph.Node n : graph.nodeList )
        {
            if( ! ( (LayoutData)n.data ).marked )
                initialOrderNodes(graph, n);
        }
    }

    public void initialOrderNodes(Graph graph, ru.biosoft.graph.Node curr)
    {
        ( (LayoutData)curr.data ).marked = true;

        for( Edge edge : graph.getEdges(curr) )
        {
            if( edge.to == curr ) // get in edges
            {
                ru.biosoft.graph.Node n = edge.from;
                if( ! ( (LayoutData)n.data ).marked )
                    initialOrderNodes(graph, n);
            }
        }

        levelNodes.get(getLevel(curr) - 1).add(curr);
    }

    // Do downwards barycentering on first pass, upwards on second, then average
    protected void orderNodes(Graph graph, int op)
    {
        boolean doup = true;
        boolean doin = op < 5;
        boolean doout = op > 3;

        if( doup )
            // Going upwards
            for( int i = 0; i < maxLevel; i++ )
                orderLevel(graph, levelNodes.get(i), doin, doout);
        else
            // Going downwards
            for( int i = maxLevel - 1; i >= 0; i-- )
                orderLevel(graph, levelNodes.get(i), doin, doout);
        placeNodesInitial();

    }

    protected void orderLevel(Graph graph, List<ru.biosoft.graph.Node > nodes, boolean doin, boolean doout)
    {
        // barycentric heuristic
        // TODO: compare to median sort

        for( ru.biosoft.graph.Node  node : nodes )
            calcBarycenter(graph, node, doin, doout, false);

        Collections.sort(nodes, levelComparator);

        int levelcnt = nodes.size();
        for( int i = 0; i < levelcnt; i++ )
        {
            ru.biosoft.graph.Node  node = nodes.get(i);
            if( verticalOrientation )
                node.x = i * layerDeltaX;
            else
                node.y = i * layerDeltaY;
        }

        // transposition heuristic
        int level;
        double w1, w2;
        for( int k = 0; k < 5; k++ )
        {
            boolean hasSwap = false;
            for( int i = 0; i < levelcnt - 1; i++ )
            {
                ru.biosoft.graph.Node n1 = nodes.get( i );
                ru.biosoft.graph.Node n2 = nodes.get( i + 1 );

                level = getLevel(n1);
                w1 = calcLevelWeight(level, true, doout);
                swap(n1, n2);
                w2 = calcLevelWeight(level, true, doout);

                if( w1 <= w2 )
                    swap(n1, n2);
                else
                {
                    nodes.remove(n2);
                    nodes.add(i, n2);
                    hasSwap = true;
                    // Logger.info( log, "swap: " + n1 + " <-> " + n2);
                }
            }

            if( !hasSwap )
                break;
        }
    }

    protected void swap(ru.biosoft.graph.Node n1, ru.biosoft.graph.Node  n2)
    {
        int t;
        if( verticalOrientation )
        {
            t = n1.x;
            n1.x = n2.x;
            n2.x = t;
        }
        else
        {
            t = n1.y;
            n1.y = n2.y;
            n2.y = t;
        }
    }

    protected LevelComparator levelComparator = new LevelComparator();

    public class LevelComparator implements Comparator<ru.biosoft.graph.Node >
    {
        @Override
        public boolean equals(Object obj)
        {
            return this == obj;
        }

        @Override
        public int compare(ru.biosoft.graph.Node  o1, ru.biosoft.graph.Node  o2)
        {
            int r = getBarycenter(o1) - getBarycenter(o2);
            if( r != 0 )
                return r;

            return 0;
        }
    }

    protected ArrayList<List<Edge>> levelEdges;

    protected void initLevelEdges(Graph graph)
    {
        levelEdges = new ArrayList<>( maxLevel );
        for( int i = 0; i < maxLevel; i++ )
        {
            ArrayList<Edge> edges = new ArrayList<>();
            levelEdges.add(edges);

            List<ru.biosoft.graph.Node > nodes = levelNodes.get(i);
            for( ru.biosoft.graph.Node  node : nodes )
            {
                List<Edge> nEdges = graph.getEdges(node);
                for( Edge edge : nEdges )
                    if( edge.from == node )
                        edges.add(edge);
            }
        }
    }

    protected double calcLevelWeight(int level, boolean doIn, boolean doOut)
    {
        double weight = 0;

        level--;
        List<ru.biosoft.graph.Node > nodes = levelNodes.get(level);
        for( ru.biosoft.graph.Node  node : nodes )
        {
           weight += node.x*getScore(node)*scoreWeight;
        }

        if( level > 0 && doIn )
            weight += calcEdgesWeight(levelEdges.get(level - 1));

        if( doOut )
            weight += calcEdgesWeight(levelEdges.get(level));

        return weight;
    }

    protected int calcEdgesWeight(List<Edge> edges)
    {
        int weight = 0;
        for( int i = 0; i < edges.size(); i++ )
        {
            Edge e1 = edges.get(i);

            if (e1.from.getName().lastIndexOf("_")>-1 && e1.to.getName().lastIndexOf("_")>-1)
            {
            String fromName = e1.from.getName().substring(0,e1.from.getName().lastIndexOf("_"));
            String toName = e1.to.getName().substring(0,e1.to.getName().lastIndexOf("_"));
            if (fromName.equalsIgnoreCase(toName)) weight += sameNameNodesWeight*Math.abs(e1.from.x - e1.to.x);
            }

            if (((LayoutData)e1.from.data).dummy == true || ((LayoutData)e1.to.data).dummy == true)
                  weight += dummyEdgesCoeff*Math.abs(e1.from.x - e1.to.x);


            if( verticalOrientation )
                weight += Math.abs(e1.from.x - e1.to.x);
            else
                weight += Math.abs(e1.from.y - e1.to.y);

            // calc intersections
            if (e1.data == null || !e1.data.equals("dummyEdge"))
            {
            for( int k = i + 1; k < edges.size(); k++ )
            {
                Edge e2 = edges.get(k);

                if( verticalOrientation )
                {
                    if( ( e1.from.x < e2.from.x && e1.to.x > e2.to.x ) || ( e2.from.x < e1.from.x && e2.to.x > e1.to.x ) )
                        weight += edgesCrossCoeff;
                }
                else
                {
                    if( ( e1.from.y < e2.from.y && e1.to.y > e2.to.y ) || ( e2.from.y < e1.from.y && e2.to.y > e1.to.y ) )
                        weight += edgesCrossCoeff;
                }
            }
            }
        }

        return weight;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Place node issues
    //

    protected void placeNodesInitial()
    {
        int levelSize = 0;
        int levelOffset = 0;

        for( int i = 0; i < maxLevel; i++ )
        {
            List<ru.biosoft.graph.Node > nodes = levelNodes.get(i);
            levelSize = placeLevelInitial(nodes, levelOffset);

            if( verticalOrientation )
            {
                levelOffset += levelSize / gridY * gridY + gridY;
                levelOffset += layerDeltaY;
            }
            else
            {
                levelOffset += levelSize / gridX * gridX + gridX;
                levelOffset += layerDeltaX;
            }
        }
    }

    protected int placeLevelInitial(List<ru.biosoft.graph.Node > level, int levelOffset)
    {
        int size = 0;
        int offset = 0;

        for( ru.biosoft.graph.Node  node : level )
        {
            if( verticalOrientation )
            {
                node.y = levelOffset;
                node.x = offset;

                offset += node.width / gridX * gridX + layerDeltaX;
                size = Math.max(size, node.height);
            }
            else
            {
                node.x = levelOffset;
                node.y = offset;

                offset += node.height / gridY * gridY + layerDeltaY;
                size = Math.max(size, node.width);
            }
        }

        return size;
    }

    // //////////////////////////////////////////////////////////////////////////

    protected void configureForceDirectedLayouter()
    {
        forceDirectedLayouter.setInitialPlacement(ForceDirectedLayouter.PLACEMENT_AS_IS);

        forceDirectedLayouter.setAttraction(0.2f);
        forceDirectedLayouter.setDistanceMethod(ForceDirectedLayouter.DISTANCE_DIAGONAL);
        forceDirectedLayouter.setEdgeLength( ( layerDeltaX + layerDeltaY ) / 3);
        forceDirectedLayouter.setRepulsion(7);
        forceDirectedLayouter.setRepulsionDistance(2);

        forceDirectedLayouter.setGravity(0);

        forceDirectedLayouter.setMaxTemperature(5);
        forceDirectedLayouter.setMinTemperature(1);
        forceDirectedLayouter.setIterationNumber(100);
    }

    protected void straightenLayout(Graph graph, LayoutJobControl lJC)
    {
        if( straightenMethod == STRAIGHTEN_FORCE_DIRECTED )
        {
            if( forceDirectedLayouter == null )
                forceDirectedLayouter = new ForceDirectedLayouter();

            forceDirectedLayouter.setHorisontalMovementAllowed(verticalOrientation);
            forceDirectedLayouter.setVerticalMovementAllowed( !verticalOrientation);

            Object[] data = new Object[graph.nodeCount()];
            List<ru.biosoft.graph.Node > nodes = graph.nodeList;
            for( int i = 0; i < graph.nodeCount(); i++ )
                data[i] = nodes.get(i).data;

            forceDirectedLayouter.layoutNodes(graph, lJC);

            for( int i = 0; i < graph.nodeCount(); i++ )
                nodes.get(i).data = data[i];

            return;
        }

        for( int i = 0; i < straightenIterationNum; i++ )
            for( int j = maxLevel - 1; j >= 0; j-- )
                straightenLevel(graph, levelNodes.get(j));
    }

    protected void straightenLevel(Graph graph, List<ru.biosoft.graph.Node > nodes)
    {
        int levelcnt = nodes.size();
        for( int i = 0; i < levelcnt; i++ )
        {
            ru.biosoft.graph.Node  node = nodes.get(i);
            calcBarycenter(graph, node, true, true, true);

            if( verticalOrientation )
            {
                int shift = node.x - ( getBarycenter(node) - node.width / 2 );
                shift = shift > 0 ? Math.min(shift, 5) : Math.max(shift, -5);
                node.x -= shift;
            }
            else
            {
                int shift = node.y - ( getBarycenter(node) - node.height / 2 );
                shift = shift > 0 ? Math.min(shift, 5) : Math.max(shift, -5);
                node.y -= shift;
            }

            if( i > 0 )
            {
                ru.biosoft.graph.Node  prev = nodes.get(i - 1);

                if( verticalOrientation )
                {
                    int overlap = prev.x + prev.width + layerDeltaX - node.x;
                    if( overlap > 0 )
                    {
                        node.x += overlap * ( i - 1 ) / i;

                        overlap = overlap / i;
                        for( int k = i - 1; k >= 0; k-- )
                        {
                            ru.biosoft.graph.Node  curr = nodes.get(k);
                            curr.x -= overlap;

                            if( k > 0 )
                            {
                                prev = nodes.get(k - 1);
                                overlap = prev.x + prev.width + layerDeltaX - curr.x;
                                if( overlap <= 0 )
                                    break;
                            }
                        }
                    }
                }
                else
                {
                    int overlap = prev.y + prev.height + layerDeltaY - node.y;
                    if( overlap > 0 )
                    {
                        node.y += overlap * ( i - 1 ) / i;

                        overlap = overlap / i;
                        for( int k = i - 1; k >= 0; k-- )
                        {
                            ru.biosoft.graph.Node  curr = nodes.get(k);
                            curr.y -= overlap;

                            if( k > 0 )
                            {
                                prev = nodes.get(k - 1);
                                overlap = prev.y + prev.height + layerDeltaY - curr.y;
                                if( overlap <= 0 )
                                    break;
                            }
                        }
                    }
                }
            }
        }
    }

    protected void straightenDummy(Graph graph, ru.biosoft.graph.Node  n)
    {
        List<Edge> edges = graph.getEdges(n);
        ru.biosoft.graph.Node  from = null;
        ru.biosoft.graph.Node  to = null;
        for( int i = 0; i < 2; i++ )
        {
            Edge edge = edges.get(i);
            if( edge.to == n )
                from = edge.from;
            else
                to = edge.to;
        }

        if( from == null || to == null )
        {
            log.log(Level.SEVERE, "One of the edge terminals is null");
            return;
        }

        if( verticalOrientation )
            n.x = ( n.x + from.x + from.width / 2 + to.x + to.width / 2 ) / 3;
        else
            n.y = ( n.y + from.y + from.height / 2 + to.y + to.height / 2 ) / 3;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Create and remove a meta-root and meta-edges to all nodes.
    //

    protected ru.biosoft.graph.Node makeMetaRoot(Graph graph)
    {
        ru.biosoft.graph.Node metaRoot = new ru.biosoft.graph.Node("meta-root");
        metaRoot.data = new LayoutData();

        for( ru.biosoft.graph.Node n : graph.nodeList )
        {
            Edge e = new Edge(n, metaRoot);
            graph.addEdge(e);
        }

        graph.addNode(metaRoot);
        return metaRoot;
    }

    @Override
    public void doLayout(Graph graph, LayoutJobControl lJC)
    {

        this.gridY = graph.nodeAt(0).height;
        this.gridX = graph.nodeAt(0).width;

                if( verticalOrientation )
            this.dummyGridY = graph.nodeAt(0).height;
        else
            this.dummyGridX = graph.nodeAt(0).width;

        initLayoutData(graph);
        normalise(graph);
//        addLabeles(graph, labelesData);
        assignNodeLevels(graph);
        addDummies(graph);

        long time = System.currentTimeMillis();
        int oneNodeGraphCount = 0;

        List<Graph> graphs = graph.split();
        Graph[] grrs = new Graph[graphs.size()];
        int[] minLayers = new int[graphs.size()];
        int[] maxLayers = new int[graphs.size()];
        int k = 0;
        for( Graph gr : graphs )
        {
            grrs[k] = gr;
            k++;
        }
        Graph tempGraph = new Graph();

        for( int i = 0; i < graphs.size(); i++ )
        {
            for( int j = 0; j < graphs.size() - 1; j++ )
            {
                if( grrs[j].nodeCount() < grrs[j + 1].nodeCount() )
                {
                    tempGraph = grrs[j];
                    grrs[j] = grrs[j + 1];
                    grrs[j + 1] = tempGraph;
                }
            }
        }

        for( int i = 0; i < graphs.size(); i++ )
        {
            int minLayer = 10000;
            int maxLayer = 0;

            for( int j = 0; j < grrs[i].nodeCount(); j++ )
            {
                ru.biosoft.graph.Node  v = grrs[i].nodeAt(j);
                if( Integer.parseInt( ( v.getAttribute("level") ).trim()) < minLayer )
                    minLayer = Integer.parseInt( ( v.getAttribute("level") ).trim());
                if( Integer.parseInt( ( v.getAttribute("level") ).trim()) > maxLayer )
                    maxLayer = Integer.parseInt( ( v.getAttribute("level") ).trim());
            }
            minLayers[i] = minLayer;
            maxLayers[i] = maxLayer;
        }

        tempGraph = new Graph();
        Graph labelGraph = new Graph();
        int tempGraphMinLayer = 1000;
        for( int i = graphs.size() - 1; i >= 0; i-- )
        {
            if( grrs[i].nodeCount() > 1 )
                break;
            else
            {
                oneNodeGraphCount++;
                if ( grrs[i].nodeAt(0).getAttribute("label").equalsIgnoreCase("true"))
                {
                    labelGraph.addNode(grrs[i].nodeAt(0));
                }
                else
                {
                tempGraph.addNode(grrs[i].nodeAt(0));
                if( minLayers[i] < tempGraphMinLayer )
                    tempGraphMinLayer = minLayers[i];
                }
            }
        }

        int xcord = 0;

        initLayoutData(labelGraph);
        normalise(labelGraph);
        assignNodeLevels(labelGraph);
        layoutNodes(labelGraph, lJC);
        labelGraph.setLocation(xcord, 0);// layerDeltaY*(minLayers[0]-1)
        xcord = labelGraph.getBounds().x + labelGraph.getBounds().width + layerDeltaX;

        if(graphs.size() - oneNodeGraphCount > 0)
        {
        layoutNodes(grrs[0], lJC);
        grrs[0].setLocation(xcord, 0);
        layoutEdges(grrs[0], lJC);
        xcord = grrs[0].getBounds().x + grrs[0].getBounds().width + layerDeltaX;
        }
        for( int i = 1; i < graphs.size() - oneNodeGraphCount; i++ )
        {
            layoutNodes(grrs[i], lJC);
            grrs[i].setLocation(xcord, 0);
            layoutEdges(grrs[i], lJC);
            xcord = grrs[i].getBounds().x + grrs[i].getBounds().width + layerDeltaX;
        }

        if( tempGraph.nodeCount() != 0 )
        {
            initLayoutData(tempGraph);
            normalise(tempGraph);
            assignNodeLevels(tempGraph);

            layoutNodes(tempGraph, lJC);
            tempGraph.setLocation(xcord, 0);
        }


        removeDummies(graph);
        log.log(Level.INFO, "Layout time " + ( System.currentTimeMillis() - time ));
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++

    @Override
    public LayouterInfo getInfo()
    {
       LayouterInfoSupport lis = new LayouterInfoSupport(true, true, false, false, false, true);
       return lis;
    }


    @Override
    public int estimate(Graph graph, int what)
    {
       return 0;
    }

    public class LayoutData
    {
        public int level = 0;

        public int usageLevel = 0;

        public int barycenter = 0;

        public double score = 0;

        public boolean marked = false;

        public boolean picked = false;

        public boolean dummy = false;
      }
}
