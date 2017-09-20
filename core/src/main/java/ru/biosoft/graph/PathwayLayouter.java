package ru.biosoft.graph;

import java.awt.Rectangle;
import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import ru.biosoft.graph.HierarchicLayouter.LayoutData;

public class PathwayLayouter extends AbstractLayouter
{
    private final Layouter layouter;

    private final HashMap<String, Set<Node>> compartmentToNodes = new HashMap<>();
    private final HashMap<Integer, Set<Node>> levelToNodesMap = new HashMap<>();
    private final Set<Node> events = new HashSet<>();
    private final Set<Node> equations = new HashSet<>();
    private final Set<Node> functions = new HashSet<>();

    private final Set<Edge> notLayoutedEdges = new HashSet<>();

    public PathwayLayouter(Layouter layouter)
    {
        super();
        this.layouter = layouter;
    }

    public Graph prepareGraph(Graph graph)
    {
        Graph graphToLayout = new Graph();
        for( Node node : graph.nodeList )
        {
            String type = node.getAttribute("Type");
            if( graph.getEdges(node) == null && type != null)
            {
                if( type.equals("Event") )
                {
                    events.add(node);
                }
                else if( type.equals("Equation")  )
                {
                    equations.add(node);
                }
                else if( type.equals("Function") )
                {
                    functions.add(node);
                }
                else
                    graphToLayout.addNode(node);
            }
            else
                graphToLayout.addNode(node);
        }
        for( Edge edge : graph.edgeList )
        {
            graphToLayout.addEdge(edge);
        }
        adjustReactions(graphToLayout);
        initMaps(graphToLayout);
        return graphToLayout;
    }

    @Override
    public void doLayout(Graph graph, LayoutJobControl jobControl)
    {
        try
        {
            if( jobControl != null )
                jobControl.begin();
            Graph graphToLayout = prepareGraph(graph);

            if( layouter.getInfo().supportCompartments() )
            {
                layouter.doLayout(graphToLayout, jobControl);
            }
            else
            {
                int levelCount = getLevelCount();
                for( int i = levelCount - 1; i >= 0; i-- )
                {
                    layoutLevel(i, graphToLayout, jobControl);
                }

                for( Edge edge : notLayoutedEdges )
                {
                    layouter.layoutPath(graphToLayout, edge, jobControl);
                }
            }

            postProcess(graphToLayout, true);
            layoutMath(graph);

            if( jobControl != null && jobControl instanceof LayoutJobControlImpl )
            {
                ( (LayoutJobControlImpl)jobControl ).resultsAreReady();
            }
            else if( jobControl != null )
                jobControl.terminate();
        }
        catch( Exception ex )
        {
            if( jobControl != null )
                jobControl.terminate();
//            ex.printStackTrace();
        }
    }

    @Override
    public void layoutNodes(Graph graph, LayoutJobControl jobControl)
    {
        layouter.doLayout(graph, jobControl);
    }

    public void layoutMath(Graph graph)
    {
        int x1 = 0;
        int x2 = 0;
        int x3 = 0;
        int y = 0;
        int yMax = 0;
        for( Node node : equations )
        {
            node.x = x1;
            node.y = y;
            y += node.height + 10;
            x2 = Math.max(x2, node.width);
        }
        yMax = y;
        y = 0;
        x2 += 10;
        for( Node node : events )
        {
            node.x = x2;
            node.y = y;
            y += node.height + 10;
            x3 = Math.max(x3, node.width);
        }
        yMax = Math.max(y, yMax);
        y = 0;
        x3 += 10;
        for( Node node : functions )
        {
            node.x = x3;
            node.y = y;
            y += node.height + 10;
        }
        yMax = Math.max(y, yMax);
        for( Node node : graph.nodeList )
        {
            if( !events.contains(node) && !equations.contains(node) && !functions.contains(node) )
                node.y += yMax;
        }
        for( Edge edge : graph.edgeList )
        {
            for( int i = 0; i < edge.path.npoints; i++ )
            {
                edge.path.ypoints[i] += yMax;
            }
        }
    }

    public int getLevelCount()
    {
        return levelToNodesMap.size();
    }

    public Graph layoutLevel(int level, Graph graph, LayoutJobControl jobControl)
    {
        Set<Node> nodeList = levelToNodesMap.get(level);
        if( nodeList == null )
            return null;

        Graph levelGraph = generateLevelGraph(nodeList, graph);

        Map<Node, Point> levelCompartments = new HashMap<>();
        for( Node node : nodeList )
        {
            if( Util.isCompartment(node) )
            {
                levelCompartments.put(node, new Point(node.getBounds().getLocation()));
            }
        }

        //do Layout
        layouter.doLayout(levelGraph, jobControl);

        if( ( layouter instanceof HierarchicLayouter ) )
        {
            makeHierarchicPath(graph, levelGraph);
        }

        //        move compartments according to their stubs new positions
        for( Node compartment : levelCompartments.keySet() )
        {
            Point oldLocation = levelCompartments.get(compartment);
            int shiftX = compartment.x - oldLocation.x;
            int shiftY = compartment.y - oldLocation.y;

            moveNodes(compartment, shiftX, shiftY, graph);
            moveEdges(compartment, shiftX, shiftY, graph);
        }
        return levelGraph;
    }

    protected Graph generateLevelGraph(Set<Node> nodeList, Graph graph)
    {
        Graph levelGraph = new Graph();

        for( Node node : nodeList )
        {
            if( compartmentToNodes.containsKey(node.name) && ( node.getAttribute("isNotResizable") == null || ! Boolean.parseBoolean(node.getAttribute("isNotResizable")) ))
            {
                node.setBounds(Util.getBounds(compartmentToNodes.get(node.name)));
            }
            levelGraph.addNode(node);
        }

        for( Edge edge : graph.edgeList )
        {
            Node from = levelGraph.getNode(edge.from.name);
            Node to = levelGraph.getNode(edge.to.name);
            if( from != null && to != null && edge.master )
            {
                Edge newEdge = graph.getEdge(from, to);
                if( newEdge == null )
                {
                    newEdge = new Edge(from, to);
                }
                levelGraph.addEdge(newEdge);
                if( edge.slaves != null )
                {
                    for( Edge slave : edge.slaves )
                        levelGraph.addEdge(slave);
                }
            }
        }

        //creating additional edges for compartments
        for( Node compartment : levelGraph.nodeList )
        {
            if( !Util.isCompartment(compartment) )
                continue;

            Set<Node> innerNodes = getInnerNodes(compartment, graph);
            for( Node node : innerNodes )
            {
                List<Edge> edges = graph.getEdges(node);
                if( edges != null )
                {
                    for( Edge edge : edges )
                    {
                        Node to = edge.to;
                        Node from = edge.from;
                        if( innerNodes.contains(to) && !innerNodes.contains(from) )
                        {
                            Node newFrom = null;
                            String curName = from.name;
                            while(newFrom == null && !curName.isEmpty() )
                            {
                                newFrom = levelGraph.getNode(curName);
                                curName = curName.indexOf('.') == -1 ? "" : curName.substring(0, curName.lastIndexOf('.'));
                            }
                            if( newFrom != null )
                            {
                                Edge newEdge = new Edge(newFrom, compartment);
                                levelGraph.addEdge(newEdge);
                                notLayoutedEdges.add(edge);
                            }
                        }
                        else if( innerNodes.contains(from) && !innerNodes.contains(to) )
                        {
                            Node newTo = null;
                            String curName = to.name;
                            while(!curName.isEmpty() && newTo == null)
                            {
                                newTo = levelGraph.getNode(curName);
                                curName = curName.indexOf('.') == -1 ? "" : curName.substring(0, curName.lastIndexOf('.'));
                            }
                            if( newTo != null )
                            {
                                Edge newEdge = new Edge(compartment, newTo);
                                levelGraph.addEdge(newEdge);
                                notLayoutedEdges.add(edge);
                            }
                        }
                    }
                }
            }
        }
        return levelGraph;
    }

    private void moveNodes(Node compartment, int shiftX, int shiftY, Graph graph)
    {
        Set<Node> nodes = compartmentToNodes.get(compartment.name);
        if( nodes == null )
            return;
        for( Node node : nodes )
        {
            node.x += shiftX;
            node.y += shiftY;

            moveNodes(node, shiftX, shiftY, graph);
            moveEdges(node, shiftX, shiftY, graph);
        }
    }

    private void moveEdges(Node compartment, int shiftX, int shiftY, Graph graph)
    {
        Set<Edge> movedEdges = new HashSet<>(); //to avoid edge double moving
        Set<Node> nodes = compartmentToNodes.get(compartment.name);
        if( nodes == null )
            return;
        for( Node node : nodes )
        {
            List<Edge> edges = graph.getEdges( node );
            if( edges != null )
            {
                for( Edge edge : edges )
                {
                    if( nodes.contains( edge.getFrom() ) && nodes.contains( edge.getTo() ) )
                    {
                        moveEdge( edge, shiftX, shiftY, movedEdges );
                        if( edge.slaves != null )
                        {
                            for( Edge e : edge.slaves )
                            {
                                moveEdge( e, shiftX, shiftY, movedEdges );
                            }
                        }
                    }
                }
            }
        }
    }

    private void moveEdge(Edge edge, int shiftX, int shiftY, Set<Edge> movedEdges)
    {
        if( movedEdges.contains( edge ) )
            return;
        //edge must be completely inside compartment

        movedEdges.add( edge );
        Path p = edge.getPath();
        for( int i = 0; i < p.npoints; i++ )
        {
            p.xpoints[i] += shiftX;
            p.ypoints[i] += shiftY;
        }
    }

    private Set<Node> getInnerNodes(Node compartment, Graph graph)
    {
        Set<Node> result = new HashSet<>();
        if( compartment == null )
            return result;
        Set<Node> nodes = compartmentToNodes.get(compartment.name);

        if( nodes != null )
        {
            for( Node node : nodes )
            {
                result.add(node);
                result.addAll(getInnerNodes(node, graph));
            }
        }
        return result;
    }

    private void postProcess(Graph graph, boolean processEdges)
    {
        int levelCount = getLevelCount();
        for( int i = levelCount - 1; i >= 0; i-- )
        {
            Set<Node> compartments = levelToNodesMap.get(i);
            if( compartments != null )
            {
                for( Node compartment : compartments )
                {
                    if( Util.isCompartment(compartment) )
                    {
                        Set<Node> nodes = compartmentToNodes.get(compartment.name);
                        if( nodes != null )
                            compartment.setBounds(Util.getBounds(nodes));
                    }
                }
            }
        }

        //shifting all nodes to the upper-left corner
        int dx = Integer.MAX_VALUE;
        int dy = Integer.MAX_VALUE;
        for( Node node : graph.nodeList )
        {
            dx = Math.min(node.x, dx);
            dy = Math.min(node.y, dy);
        }
        for( Node node : graph.nodeList )
        {
            node.x -= dx;
            node.y -= dy;
        }
        if( processEdges )
        {
            for( Edge e : graph.edgeList )
            {
                Path p = e.getPath();
                for( int i = 0; i < p.npoints; i++ )
                {
                    p.xpoints[i] -= dx;
                    p.ypoints[i] -= dy;
                }
                e.path = p;
            }
        }

    }

    private void initMaps(Graph graph)
    {
        Iterator<Node> iter = graph.nodeIterator();
        while( iter.hasNext() )
        {
            Node node = iter.next();
            Integer level = Util.getLevel(node);

            Set<Node> nodes = levelToNodesMap.get(level);
            if( nodes == null )
                nodes = new HashSet<>();
            nodes.add(node);
            levelToNodesMap.put(level, nodes);

            Node compartment = Util.getCompartment(node, graph);

            if( compartment != null )
            {
                String compartmentName = compartment.name;
                nodes = compartmentToNodes.get(compartmentName);
                if( nodes == null )
                    nodes = new HashSet<>();
                nodes.add(node);

                compartmentToNodes.put(compartmentName, nodes);
            }


        }
    }

    private void adjustReactions(Graph graph)
    {
        for( Node node : graph.nodeList )
        {
            String type = node.getAttribute("Type");
            String path = node.getAttribute("compartmentName");
            if( type != null && type.equals("Reaction") && path != null )
            {
                Set<Node> nodes = Util.getNodes(node, graph);
                nodes.remove(node);
                if( nodes.size() != 0 )
                {
                    Node[] arr = nodes.toArray(new Node[nodes.size()]);
                    path = arr[0].getAttribute("compartmentName");
                    for( int i = 1; i < arr.length; i++ )
                    {
                        String newPath = arr[i].getAttribute("compartmentName");
                        if( newPath != null )
                            path = Util.getCommonCompartment(path, newPath);
                    }
                    node.setAttribute("compartmentName", path);
                }
            }
        }
    }


    @Override
    public void layoutEdges(Graph graph, LayoutJobControl lJC)
    {
        layouter.layoutEdges(graph, lJC);
    }

    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl lJC)
    {
        layouter.layoutPath(graph, edge, lJC);
    }

    @Override
    public int estimate(Graph graph, int what)
    {
        return layouter.estimate(graph, what);
    }

    @Override
    public LayouterInfo getInfo()
    {
        return layouter.getInfo();
    }


    //hierarchic layouter issues

    private void makeHierarchicPath(Graph initialGraph, Graph levelGraph)
    {
        //workaround: Hierarchical layout can reverse edges in original graph and do not reverse them back
        Iterator<Edge> iterInitial = initialGraph.edgeIterator();
        while( iterInitial.hasNext() )
        {
            Edge currInitial = iterInitial.next();
            if(currInitial.isReversed())
            {
                currInitial.reverseDirection();
            }
        }

        Iterator<Edge> iterLevel = levelGraph.edgeIterator();
        while( iterLevel.hasNext() )
        {
            Edge currLevel = iterLevel.next();
            if( !currLevel.master )
                continue;
            iterInitial = initialGraph.edgeIterator();
            while( iterInitial.hasNext() )
            {
                Edge currInitial = iterInitial.next();
                if( !currInitial.master )
                    continue;
                if( currLevel.from.name.equals(currInitial.from.name) && currLevel.to.name.equals(currInitial.to.name) )
                {
                    Path p = currLevel.path;
                    //int pathLength = currLevel.path.npoints;
                    /*if( pathLength > 2 )
                    {
                        HashMap<Integer, List<Node>> levelMap = makeHierarchicLevelsList(levelGraph, layouter);
                        HierarchicLayouter l = (HierarchicLayouter)layouter;
                        boolean verticalOrientation = l.verticalOrientation;

                        p = correctPath(currLevel, levelMap, verticalOrientation);
                    }*/
                    //currInitial.path = new Path();
                    currInitial.path = p;
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
                                slave.path = new Path(p.xpoints, p.ypoints, p.npoints);
                        }
                    }
                }
            }
        }
    }

    private HashMap<Integer, List<Node>> makeHierarchicLevelsList(Graph graph, Layouter layouter)
    {
        HashMap<Integer, List<Node>> levelMap = new HashMap<>();
        //HierarchicLayouter l = (HierarchicLayouter)layouter;
        //boolean verticalOrientation = l.verticalOrientation;

        for( Node n : graph.nodeList )
        {
            List<Node> levelList = new ArrayList<>();
            if( n.data != null )
            {
                int level = ( (LayoutData)n.data ).level;
                if( !levelMap.containsKey(level) )
                {
                    levelList.add(n);
                    levelMap.put(level, levelList);
                }
                else
                {
                    levelMap.get(level).add(n);
                }
            }
        }

        return levelMap;
    }

    private Path correctPath(Edge pathEdge, HashMap<Integer, List<Node>> levelMap, boolean isVertical)
    {
        int startLevel = ( (LayoutData)pathEdge.from.data ).level;
        int endLevel = ( (LayoutData)pathEdge.to.data ).level;
        int increment = 1;
        if( startLevel > endLevel )
            increment = -1;
        Path p = pathEdge.path;
        int[] xP = p.xpoints;
        int[] yP = p.ypoints;
        if( !isVertical )// horizontal
        {
            for( int j = 1; j < p.npoints - 1; j++ )
            {
                yP[j] = yP[j] - (int) ( 6 * Math.random() );
            }
        }
        else
        // vertical
        {
            for( int j = 1; j < p.npoints - 1; j++ )
            {
                xP[j] = xP[j] - (int) ( 6 * Math.random() );
            }
        }
        Vector<Point> newPath = new Vector<>();
        Vector<Integer> pointTypes = new Vector<>();
        if( !isVertical )// horizontal
        {
            newPath.add(new Point(xP[0], yP[0]));
            int i = 0;
            for( i = 1; i < p.npoints; i++ )
            {
                if( startLevel + increment * i == endLevel )// for last point
                {
                    if( yP[i] == yP[i - 1] )
                    {
                        newPath.add(new Point(xP[i], yP[i]));
                    }
                    else
                    {
                        newPath.add(addBetweenPoint(new Point(xP[i - 1], yP[i - 1]), new Point(xP[i], yP[i])));
                        newPath.add(new Point(xP[i], yP[i]));
                    }
                }
                else
                // all except last
                {
                    List<Node> currLevelList = levelMap.get(startLevel + increment * i);
                    Point point = new Point(xP[i], yP[i - 1]);

                    if( !checkContains(currLevelList, point) )
                    {
                        yP[i] = yP[i - 1];
                        newPath.add(new Point(xP[i], yP[i]));
                    }
                    else
                    {
                        if( i == p.npoints - 1 || i == 1 )
                        {
                            newPath.add(addBetweenPoint(new Point(xP[i - 1], yP[i - 1]), new Point(xP[i], yP[i])));
                            newPath.add(new Point(xP[i], yP[i]));
                        }
                        else
                        {
                            i++;
                            if( i == p.npoints - 1 )
                                point = new Point(xP[i], yP[i]);
                            else
                                point = new Point(xP[i], yP[i - 1]);
                            currLevelList = levelMap.get(startLevel + increment * i);
                            if( !checkContains(currLevelList, point) )
                            {
                                yP[i] = yP[i - 1];
                                newPath.add(addBetweenPoint(new Point(xP[i - 2], yP[i - 2]), new Point(xP[i - 1], yP[i - 1])));
                                newPath.add(new Point(xP[i - 1], yP[i - 1]));
                                newPath.add(new Point(xP[i], yP[i]));
                            }
                            else
                            {
                                newPath.add(new Point(xP[i - 1], yP[i - 1]));
                                newPath.add(new Point(xP[i], yP[i]));
                            }
                        }
                    }
                }
            }

        }
        else
        // for vertical
        {
            newPath.add(new Point(xP[0], yP[0]));
            int i = 0;
            for( i = 1; i < p.npoints; i++ )
            {
                if( startLevel + increment * i == endLevel )// for last point
                {
                    if( xP[i] == xP[i - 1] )
                    {
                        newPath.add(new Point(xP[i], yP[i]));
                    }
                    else
                    {
                        newPath.add(addBetweenPoint(new Point(xP[i - 1], yP[i - 1]), new Point(xP[i], yP[i])));
                        newPath.add(new Point(xP[i], yP[i]));
                    }
                }
                else
                // all except last
                {
                    List<Node> currLevelList = levelMap.get(startLevel + increment * i);
                    Point point = new Point(xP[i - 1], yP[i]);

                    if( !checkContains(currLevelList, point) )
                    {
                        xP[i] = xP[i - 1];
                        newPath.add(new Point(xP[i], yP[i]));
                    }
                    else
                    {
                        if( i == p.npoints - 1 || i == 1 )
                        {
                            newPath.add(addBetweenPoint(new Point(xP[i - 1], yP[i - 1]), new Point(xP[i], yP[i])));
                            newPath.add(new Point(xP[i], yP[i]));
                        }
                        else
                        {
                            i++;
                            if( i == p.npoints - 1 )
                                point = new Point(xP[i], yP[i]);
                            else
                                point = new Point(xP[i - 1], yP[i]);
                            currLevelList = levelMap.get(startLevel + increment * i);
                            if( !checkContains(currLevelList, point) )
                            {
                                xP[i] = xP[i - 1];
                                newPath.add(addBetweenPoint(new Point(xP[i - 2], yP[i - 2]), new Point(xP[i - 1], yP[i - 1])));
                                newPath.add(new Point(xP[i - 1], yP[i - 1]));
                                newPath.add(new Point(xP[i], yP[i]));
                            }
                            else
                            {
                                newPath.add(new Point(xP[i - 1], yP[i - 1]));
                                newPath.add(new Point(xP[i], yP[i]));
                            }
                        }
                    }
                }
            }
        }

        pointTypes.add(Path.QUAD_TYPE);//for the first point in the path

        for( int i = 1; i < newPath.size(); i++ )
        {
            if( !isVertical )
            {
                if( newPath.get(i).y == newPath.get(i - 1).y )
                    pointTypes.add(Path.LINE_TYPE); // for horizontal
                else
                    pointTypes.add(Path.QUAD_TYPE);
            }
            else
            {
                if( newPath.get(i).x == newPath.get(i - 1).x )
                    pointTypes.add(Path.LINE_TYPE);
                else
                    pointTypes.add(Path.QUAD_TYPE);
            }
        }


        processConvexes(newPath, pointTypes, isVertical);

        p = new Path();
        for( Point point : newPath )
        {
            p.addPoint(point.x, point.y, pointTypes.get(newPath.indexOf(point)));
        }
        pathEdge.path = p;

        return p;
    }

    private void processConvexes(Vector<Point> newPath, Vector<Integer> pointTypes, boolean vertical)
    {
        for( int i = 1; i < newPath.size() - 1; i++ )
        {
            if( ( newPath.get(i).x == (int) ( ( newPath.get(i - 1).x + newPath.get(i + 1).x ) / 2d ) )
                    && ( newPath.get(i).y == (int) ( ( newPath.get(i - 1).y + newPath.get(i + 1).y ) / 2d ) ) )
            {
                if( !vertical )
                {
                    if( pointTypes.get(i - 1) == Path.LINE_TYPE )
                        newPath.get(i).y = (int) ( 8 * ( newPath.get(i - 1).y ) / 10d + 2 * ( newPath.get(i + 1).y ) / 10d );
                    else
                        newPath.get(i).y = (int) ( 2 * ( newPath.get(i - 1).y ) / 10d + 8 * ( newPath.get(i + 1).y ) / 10d );
                }
                else
                {
                    if( pointTypes.get(i + 1) == Path.QUAD_TYPE )
                        newPath.get(i).x = (int) ( 8 * ( newPath.get(i - 1).x ) / 10d + 2 * ( newPath.get(i + 1).x ) / 10d );
                    else
                        newPath.get(i).x = (int) ( 2 * ( newPath.get(i - 1).x ) / 10d + 8 * ( newPath.get(i + 1).x ) / 10d );
                }
            }

        }

    }

    private Point addBetweenPoint(Point start, Point end)
    {
        int x = (int) ( ( start.x + end.x ) / 2d );
        int y = (int) ( ( start.y + end.y ) / 2d );

        Point between = new Point(x, y);
        return between;
    }

    private boolean checkContains(List<Node> currLevelList, Point point)
    {
        for( Node n : currLevelList )
        {
            Rectangle r = n.getBounds();
            r.setLocation(r.x - 1, r.y - 1);
            r.setSize(r.width + 2, r.height + 2);
            if( r.contains(point) )
                return true;
        }
        return false;
    }

}
