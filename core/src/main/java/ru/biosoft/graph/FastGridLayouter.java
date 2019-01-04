package ru.biosoft.graph;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.NavigableSet;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;

import ru.biosoft.jobcontrol.JobControl;

public class FastGridLayouter extends AbstractLayouter
{
    //Path layouter parameters
    private Layouter pathLayouter = new OrthogonalPathLayouter();

    //Annealing parameters
    private int threadCount = 2;
    private int tMax;
    private final double tMin = 0.1; // Min annealing temperature
    private int iterations = 4; //number of iterations on each annealing step
    private double cool = 0.6; // cooling coefficient of annealing
    private double stochasticRate = 0.3;

    private boolean isStartingFromThisLayout = false;
    private boolean isEstimationDone = false;
    private boolean isScaled = false;

    //Grid parameters
    private int h; // Height of the grid
    private int w; // Width of the grid
    private final int sd = 6; // Saturation distance
    private int gridX = 70;//horizontal step for grid
    private int gridY = 60;//vertical step for grid

    private Map<String, Node> fixedNodes = null; //to restore exact size and positions

    //Weights for crosses
    private int edgeEdgeCrossCost = 300;//Penalty for edge-edge crossing
    private int edgeNodeCrossCost = 500;//Penalty for edge-node crossing
    private int nodeNodeCrossCost = 1000;//Penalty for node-node crossing
    private int strongAttraction = 4;//attraction between directly connected nodes
    private int averageAttraction = 3;//attraction between nodes connected through another
    private int weakAttraction = 0;//attraction between node connected through 2 other
    private int weakRepulsion = -1;//attraction between node connected through 3 other
    private int averageRepulsion = -1;//attraction between somehow connected nodes
    private int strongRepulsion = -3;//attraction between not connected nodes

    //Map of interacting (repulsing, attracting) between nodes
    private final Map<String, Integer> nodeInteractionMap = new HashMap<>();

    //Map of allowed points for node to be placed (inside compartments)
    private final Map<String, List<Point>> allowedPoints = new HashMap<>();

    public FastGridLayouter()
    {
        super();
    }

    @Override
	public void doLayout(Graph graph, LayoutJobControl jobControl)
    {
        if( isStartingFromThisLayout )
        {
            shift(graph);
        }

        if( !isEstimationDone )
            estimate(graph, 1);

        layoutNodes(graph, jobControl);
        layoutEdges(graph, jobControl);

        isEstimationDone = false;
    }

    @Override
	public void layoutNodes(Graph graph, LayoutJobControl jobControl)
    {
        if( !isScaled )
            storeNodes( graph );
        setAllowedPoints(graph);
        distributedAnnealing(graph, jobControl);
        restore(graph);
        Util.adjustOrientations(graph);
    }

    @Override
	public void layoutEdges(Graph graph, LayoutJobControl jobControl)
    {

        pathLayouter.layoutEdges(graph, jobControl);
    }

    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl jobControl)
    {
        pathLayouter.layoutPath(graph, edge, jobControl);
    }


    //shifting all nodes to get rid of negative coordinates
    private void shift(Graph graph)
    {
        int dx = 0;
        int dy = 0;
        for( Node node : graph.nodeList )
        {
            dx = Math.min(node.x, dx);
            dy = Math.min(node.y, dy);
        }
        for( Node node : graph.nodeList )
        {
            //            if( !node.fixed )
            node.x -= dx;
            node.y -= dy;
        }
    }

    private void checkInitialPostitions(Graph graph)
    {
        for( Node node : graph.nodeList )
        {
            Point p = new Point(node.x, node.y);
            if( !allowedPoints.get(node.name).contains(p) )
            {
                Point newP = getNearestAllowedPoint(node);
                moveNode(node, newP);
            }
        }
    }

    //distributed annealing
    private void distributedAnnealing(Graph graph, LayoutJobControl jobControl)
    {
        checkInitialPostitions(graph);
        double temperature;
        if( isStartingFromThisLayout )
        {
            Layouting thread = new Layouting(graph);
            temperature = thread.getCost();
        }
        else
            temperature = calulateMaximumTemperature(graph);

        tMax = (int)temperature;

        int cost = distributedProcess(graph);
        Graph optimal = graph.clone();
        Graph temp = graph.clone();
        int minCost = cost;

        int operations = 0;

        while( temperature > tMin )
        {
            for( int i = 0; i < iterations - 1; i++ )
            {
                if( jobControl != null )
                {
                    jobControl.done(operations++);
                    if( jobControl.getStatus() != JobControl.RUNNING )
                    {
                        temperature = tMin;
                        break;
                    }

                }
                copyNodeLayout(temp, graph);
                int tempCost = distributedProcess(graph);

                if( tempCost < minCost )
                {
                    minCost = tempCost;
                    copyNodeLayout(optimal, graph);
                }
                if( Math.random() >= Math.exp( ( cost - tempCost ) / temperature) )
                {
                    cost = tempCost;
                    copyNodeLayout(graph, temp);
                }
            }
            temperature *= cool;
        }
        copyNodeLayout(graph, optimal);
    }

    //One step in annealing
    private int distributedProcess(Graph graph)
    {
        Layouting[] threads = new Layouting[threadCount];

        int minCost = tMax;
        int winner = 0;

        for( int i = 0; i < threads.length; i++ )
        {
            Graph nextGraph = graph.clone();
            double stochasticRate = 0.3;
            permutate(nextGraph, stochasticRate);
            threads[i] = new Layouting(nextGraph);
            threads[i].start();
        }

        for( int i = 0; i < threads.length; i++ )
        {
            try
            {
                threads[i].join();
                int currentCost = threads[i].cost;
                if( currentCost < minCost )
                {
                    winner = i;
                    minCost = currentCost;
                }
            }
            catch( Exception ex )
            {
                ex.printStackTrace();
            }
        }

        copyNodeLayout(graph, threads[winner].graph);
        return minCost;
    }

    private int calulateMaximumTemperature(Graph graph)
    {
        int t = 0;
        Layouting thread;

        if( isStartingFromThisLayout )
        {
            thread = new Layouting(graph);
            t = thread.calculateCost();
            thread.interrupt();
            return t;
        }

        for( int i = 0; i < 10; i++ )
        {
            permutate(graph, 1);
            thread = new Layouting(graph);
            t += thread.calculateCost();
            thread.interrupt();
        }
        return t;
    }

    private class Layouting extends Thread
    {
        private final int[][] boundMatrix;
        private final int[][] costMatrix;
        //Node moved on previous step to pointMin
        private Node nodeMin;
        private Point pointMin;
        //Map for penalties for node moving in particular point
        Map<String, Integer[][]> deltaMap = new HashMap<>();

        private int cost;
        private final Graph graph;

        public Layouting(Graph graph)
        {
            boundMatrix = new int[w][h + 1];
            costMatrix = new int[w][h];
            this.graph = graph;
        }

        @Override
		public void run()
        {
            cost = process();
        }

        public int calculateCost()
        {
            return getCost();
        }

        //layouting nodes (finding local optimal layout)
        private int process()
        {
            int dMin = initDeltaMap();
            if( nodeMin == null || pointMin == null )
                return getCost();

            int count = 0;
            double iteartionsLimit = 1.5 * graph.nodeCount();
            while( dMin < 0 && ( count++ ) < iteartionsLimit )
            {
                Point tempPoint = pointMin;
                String tempNodeName = nodeMin.name;
                Point prevPoint = new Point(nodeMin.x, nodeMin.y);
                moveNode(nodeMin, pointMin);

                dMin = 0;
                for( Node node : graph.nodeList )
                {
                    if( node.fixed )
                        continue;
                    resetPenaltyMatrix();

                    if( !node.name.equals(nodeMin.name) )
                    {
                        Set<Node> nodes = new HashSet<>( graph.nodeList );
                        Set<Edge> notConnectedEdges = new HashSet<>( graph.edgeList );
                        Set<Node> connectedNodes = new HashSet<>( Util.getNodes( node, graph ) );
                        Set<Edge> connectedToMinEdges = new HashSet<>();
                        List<Edge> edges = graph.getEdges(nodeMin);
                        if( edges != null )
                            connectedToMinEdges.addAll(edges);

                        nodes.remove(node);
                        nodes.remove(nodeMin);
                        nodes.removeAll(Util.getCompartments(node, graph));
                        nodes.removeAll(Util.getCompartments(nodeMin, graph));

                        connectedNodes.remove(node);
                        connectedNodes.remove(nodeMin);

                        if( graph.getEdges(node) != null )
                        {
                            connectedToMinEdges.removeAll(graph.getEdges(node));
                            notConnectedEdges.removeAll(graph.getEdges(node));
                        }

                        setDistance(node, nodeMin, 1);
                        setEdgeEdge(connectedNodes, connectedToMinEdges, edgeEdgeCrossCost);
                        setNodeEdge(node, connectedToMinEdges, edgeNodeCrossCost);
                        setEdgeNode(connectedNodes, nodeMin, edgeNodeCrossCost);
                        setNodeNode(node, nodeMin, nodeNodeCrossCost);

                        if( Util.areConnected(node, nodeMin, graph) )
                        {
                            setEdgeEdge(nodeMin, notConnectedEdges, edgeEdgeCrossCost);
                            setEdgeNode(nodeMin, nodes, edgeNodeCrossCost);
                        }

                        moveNode(nodeMin, prevPoint);

                        setDistance(node, nodeMin, -1);
                        setEdgeEdge(connectedNodes, connectedToMinEdges, -edgeEdgeCrossCost);
                        setNodeEdge(node, connectedToMinEdges, -edgeNodeCrossCost);
                        setEdgeNode(connectedNodes, nodeMin, -edgeNodeCrossCost);
                        setNodeNode(node, nodeMin, -nodeNodeCrossCost);

                        if( Util.areConnected(node, nodeMin, graph) )
                        {
                            setEdgeEdge(nodeMin, notConnectedEdges, -edgeEdgeCrossCost);
                            setEdgeNode(nodeMin, nodes, -edgeNodeCrossCost);
                        }
                        moveNode(nodeMin, pointMin);
                    }

                    setPenaltyMatrix();
                    Integer[][] deltaValues = new Integer[w][h];
                    Integer[][] oldValues = deltaMap.get(node.name);

                    for( Point p : allowedPoints.get(node.name) )
                    {
                        int i = p.x;
                        int j = p.y;
                        deltaValues[i][j] = oldValues[i][j] - oldValues[node.x][node.y] + costMatrix[i][j] - costMatrix[node.x][node.y];
                        if( deltaValues[i][j] < dMin )
                        {
                            tempNodeName = node.name;
                            dMin = deltaValues[i][j];
                            tempPoint = new Point(i, j);
                        }
                    }

                    deltaMap.put(node.name, deltaValues);
                }
                pointMin = (Point)tempPoint.clone();
                nodeMin = graph.getNode(tempNodeName);
            }
            moveNode(nodeMin, pointMin);
            return getCost();
        }

        //First step in layouting algorithm
        private int initDeltaMap()
        {
            deltaMap.clear();
            int dMin = 0;
            //            int floatCost = 0;
            for( Node node : graph.nodeList )
            {
                if( node.fixed )
                    continue;
                resetPenaltyMatrix();

                Set<Edge> notConnectedEdges = new HashSet<>( graph.edgeList );
                Set<Node> connectedNodes = Util.getNodes(node, graph);
                Set<Node> nodes = new HashSet<>( graph.nodeList );

                if( graph.getEdges(node) != null )
                    notConnectedEdges.removeAll(graph.getEdges(node));

                connectedNodes.remove(node);
                nodes.remove(node);
                nodes.removeAll(Util.getCompartments(node, graph));

                setEdgeEdge(connectedNodes, notConnectedEdges, edgeEdgeCrossCost);
                setEdgeNode(connectedNodes, nodes, edgeNodeCrossCost);
                setNodeEdge(node, notConnectedEdges, edgeNodeCrossCost);
                setNodeNode(node, graph, nodeNodeCrossCost);
                setDistance(node, graph, 1);
                setPenaltyMatrix();

                Integer[][] deltaValues = new Integer[w][h];

                for( Point p : allowedPoints.get(node.name) )
                {
                    int i = p.x;
                    int j = p.y;
                    deltaValues[i][j] = costMatrix[i][j] - costMatrix[node.x][node.y];
                    if( deltaValues[i][j] < dMin )
                    {
                        nodeMin = node;
                        dMin = deltaValues[i][j];
                        pointMin = new Point(i, j);

                    }
                }
                deltaMap.put(node.name, deltaValues);
            }
            return dMin;
        }

        //Calculating cost
        private int getCost()
        {
            int cost = 0;
            int floatCost = 0;

            Graph tempGraph = new Graph();

            for( Node node : graph.nodeList )
                tempGraph.addNode(node);

            for( Edge e : graph.edgeList )
                tempGraph.addEdge(e);

            for( Node node : graph.nodeList )
            {
                if( node.fixed )
                    continue;
                resetPenaltyMatrix();

                Set<Edge> notConnectedEdges = new HashSet<>();
                Set<Node> nodes = new HashSet<>( tempGraph.nodeList );
                Set<Node> connectedNodes = Util.getNodes(node, tempGraph);

                notConnectedEdges.addAll(tempGraph.edgeList);
                List<Edge> edges = tempGraph.getEdges(node);
                if( edges != null )
                    notConnectedEdges.removeAll(edges);
                connectedNodes.remove(node);
                nodes.remove(node);
                nodes.removeAll(Util.getCompartments(node, graph));

                setEdgeEdge(connectedNodes, notConnectedEdges, edgeEdgeCrossCost);
                setEdgeNode(connectedNodes, nodes, edgeNodeCrossCost);
                setNodeEdge(node, notConnectedEdges, edgeNodeCrossCost);
                setNodeNode(node, tempGraph, nodeNodeCrossCost);
                setDistance(node, tempGraph, 1);

                setPenaltyMatrix();
                cost += costMatrix[node.x][node.y];
                floatCost += getFloatCost(node, graph.getEdges(node));
                tempGraph.removeNode(node);
            }
            return cost;// + floatCost;
        }

        public double getFloatCost(Node node, List<Edge> connectedEdges)
        {
            if( connectedEdges == null )
                return 0;
            double x1 = 0;
            double x2 = 0;
            double y1 = 0;
            double y2 = 0;

            Point p = getCenter(node);
            Point q;
            double x;
            double y;

            for( Edge edge : connectedEdges )
            {
                if( edge.to.name == node.name )
                {
                    q = getCenter(edge.from);
                    x = p.x - q.x;
                    y = p.y - q.y;
                    double length = Math.hypot(x, y);
                    x1 += x / length;
                    y1 += y / length;
                }
                else
                {
                    q = getCenter(edge.to);
                    x = q.x - p.x;
                    y = q.y - p.y;
                    double length = Math.hypot(x, y);
                    x2 += x / length;
                    y2 += y / length;
                }
            }
            return - ( x1 * x2 + y1 * y2 );
        }

        // Creates shadow zone around all nodes according to it's relation to node
        // n1
        private void setDistance(Node n1, Graph graph, int sgn)
        {
            for( Node n2 : graph.nodeList )
            {
                setDistance(n1, n2, sgn);
            }
        }

        private void setDistance(Node n1, Node n2, int sgn)
        {
            if( n2.name == n1.name )
                return;
            int weight = nodeInteractionMap.get(n1.name + n2.name);
            if( weight > 0 )
            {
                setDistanceBoundary(n2, sgn * weight);
            }
            else
            {
                setSaturationDistanceBoundary(n2, sgn * weight);
            }
        }


        //Creates boundary around node n with weight
        private void setSaturationDistanceBoundary(Node n, Integer weight)
        {
            for( int i = 0; i < w; i++ )
            {
                boundMatrix[i][0] += ( sd * weight );
            }
            int start = n.x - sd + 1;
            int end = n.x + sd - 1;

            start = Math.max(start, 0);
            end = Math.min(end, w - 1);

            for( int i = start; i <= end; i++ )
            {
                int d = sd - Math.abs(n.x - i);
                for( int j = n.y - d + 1; j <= n.y; j++ )
                {
                    boundMatrix[i][Math.max(j, 0)] -= weight;
                }
                for( int j = Math.min(n.y + 1, h); j <= Math.min(n.y + d, h); j++ )
                {
                    boundMatrix[i][j] += weight;
                }
            }
        }

        // Creates boundary around node n with weight
        private void setDistanceBoundary(Node n, int weight)
        {
            for( int i = 0; i < w; i++ )
            {
                int val = ( Math.abs(i - n.x) + n.y ) * weight;
                boundMatrix[i][0] += val;
                for( int j = 1; j <= n.y; j++ )
                {
                    boundMatrix[i][j] -= weight;
                }
                for( int j = n.y + 1; j < h; j++ )
                {
                    boundMatrix[i][j] += weight;
                }
            }
        }

        private void setNodeNode(Node n1, Graph graph, int weight)
        {
            for( Node n2 : graph.nodeList )
            {

                setNodeNode(n1, n2, weight);
            }
        }
        private void setNodeNode(Node n1, Node n2, int weight)
        {
            if( n1.equals(n2) )
                return;
            String compartment1 = n1.getAttribute("compartmentName");
            compartment1 = ( compartment1 != null ) ? compartment1.substring(compartment1.lastIndexOf(".") + 1) : "";
            String compartment2 = n1.getAttribute("compartmentName");
            compartment2 = ( compartment2 != null ) ? compartment2.substring(compartment2.lastIndexOf(".") + 1) : "";
            if( n2.name.equals(compartment1) || n1.name.equals(compartment2) )
                return;

            int width = ( n1.width + n2.width ) / 2 + 10;
            int height = ( n1.height + n2.height ) / 2 + 10;

            double x = ( n2.x + 1 ) * gridX;
            double y = ( n2.y + 1 ) * gridY;

            int iLeft = (int)Math.ceil( ( x - width ) / gridX - 1);
            int iRight = (int)Math.floor( ( x + width ) / gridX - 1);
            int jUp = (int)Math.ceil( ( y - height ) / gridY - 1);
            int jDown = (int)Math.ceil( ( y + height ) / gridY - 1);

            if( jDown == (int)Math.floor( ( y + height ) / gridY - 1) )
                jDown++;

            iLeft = Math.max(0, iLeft);
            iRight = Math.min(w - 1, iRight);
            jUp = Math.max(0, jUp);
            jDown = Math.min(h, jDown);

            for( int i = iLeft; i <= iRight; i++ )
            {
                boundMatrix[i][jUp] += weight;
                boundMatrix[i][jDown] -= weight;
            }
        }

        // Create penalty matrix for crossings between edges between n and nodes
        // from connectedNodes and otherEdges for all mappings of node n.
        // Note: connectedNodes must be connected with node n and otherEdges must
        // not.
        private void setEdgeEdge(Set<Node> connectedNodes, Set<Edge> edges, Integer weight)
        {
            for( Node node : connectedNodes )
            {
                setEdgeEdge(node, edges, weight);
            }
        }

        private void setEdgeEdge(Node connectedNode, Set<Edge> edges, Integer weight)
        {
            Point p = getCenter(connectedNode);
            for( Edge edge : edges )
            {
                Point p1 = getCenter(edge.from);
                Point p2 = getCenter(edge.to);
                setCrossingZoneBoundary(p, p1, p2, weight);
            }
        }

        private void setEdgeNode(Set<Node> connectedNodes, Set<Node> otherNodes, int weight)
        {
            for( Node node : connectedNodes )
            {
                Point p = getCenter(node);
                for( Node otherNode : otherNodes )
                {
                    setEdgeNode(p, otherNode, weight);
                }
            }
        }

        private void setEdgeNode(Set<Node> connectedNodes, Node otherNode, int weight)
        {
            for( Node node : connectedNodes )
            {
                Point p = getCenter(node);
                setEdgeNode(p, otherNode, weight);
            }
        }

        private void setEdgeNode(Node connectedNode, Set<Node> otherNode, int weight)
        {
            Point p = getCenter(connectedNode);
            for( Node node : otherNode )
            {
                setEdgeNode(p, node, weight);
            }
        }

        private void setEdgeNode(Point p, Node targetNode, int weight)
        {
            // center of targetNode
            int x = ( targetNode.x + 1 ) * gridX;
            int y = ( targetNode.y + 1 ) * gridY;
            int px = p.x;
            int py = p.y;

            Point p2;
            Point p1;

            int height = targetNode.height / 2;
            int width = targetNode.width / 2;

            if( x - width < px && px < x + width && y - height < py && py < y + height )
            {
                for( int i = 0; i < w; i++ )
                {
                    boundMatrix[i][0] += weight;
                }
                return;
            }

            if( px == x )
            {
                height *= (int)Math.signum(py - y);
                p1 = new Point(x - width, y + height);
                p2 = new Point(x + width, y + height);
            }
            else if( py == y )
            {
                width *= (int)Math.signum(px - x);
                p1 = new Point(x + width, y - height);
                p2 = new Point(x + width, y + height);
            }
            else if( ( px - x ) * ( py - y ) > 0 )
            {
                p1 = new Point(x - width, y + height);
                p2 = new Point(x + width, y - height);
            }
            else
            {
                p1 = new Point(x - width, y - height);
                p2 = new Point(x + width, y + height);
            }
            setCrossingZoneBoundary(p, p1, p2, weight);
        }


        //  Create Boundary for shadow zone surrounded by edge e and lines connecting
        // node n with edge e nodes
        public void setCrossingZoneBoundary(Point p, Point p1, Point p2, Integer weight)
        {
            if( p.equals(p1) || p.equals(p2) )
                return;

            boolean edgeIsLower = getYByX(p1, p2, p.x, true) >= p.y;
            boolean wideAngle = ( p1.x - p.x ) * ( p2.x - p.x ) <= 0;
            boolean isUpOriented = wideAngle && !edgeIsLower;
            boolean isDownOriented = wideAngle && edgeIsLower;

            for( int i = 0; i < w; i++ )
            {
                int x = ( i + 1 ) * gridX;

                NavigableSet<Double> vals = new TreeSet<>();
                double y1 = getYByX(p, p1, x, false);
                double y2 = getYByX(p, p2, x, false);
                double y3 = getYByX(p1, p2, x, true);

                int count = 0;

                if( y1 != -1 )
                {
                    vals.add(y1);
                    count++;
                }
                if( y2 != -1 )
                {
                    vals.add(y2);
                    count++;
                }
                if( y3 != -1 )
                {
                    vals.add(y3);
                    count++;
                }

                if( count == 0 )
                    continue;

                double yMax = vals.last() / gridY - 1;
                int jMax = (int)Math.ceil(yMax);


                if( isDownOriented || ( !isUpOriented && count == 1 && jMax > 0 ) )
                {
                    if( Math.floor(yMax) == Math.ceil(yMax) )
                        jMax++;
                    jMax = Math.min(Math.max(jMax, 0), h);
                    boundMatrix[i][jMax] += weight;
                    continue;
                }
                else if( isUpOriented && jMax > 0 )
                {
                    boundMatrix[i][0] += weight;
                    jMax = Math.min(jMax, h);
                    boundMatrix[i][jMax] -= weight;
                    continue;
                }


                double yMin = vals.first() / gridY - 1;
                int jMin = (int)Math.ceil(yMin);

                if( ( (int)Math.floor(yMin) == jMin ) && ( jMin != jMax ) && jMin != 0 )
                    jMin++;

                jMax = Math.min(Math.max(jMax, 0), h);
                jMin = Math.min(Math.max(jMin, 0), h);

                boundMatrix[i][jMin] += weight;
                boundMatrix[i][jMax] -= weight;
            }
        }

        private double getYByX(Point p1, Point p2, int x, boolean inner)
        {
            if( p1.x == p2.x && p1.x != x )
            {
                return -1;
            }
            if( x == p2.x )
            {
                return p2.y;
            }

            double t = ( x - p1.x );
            t /= ( p2.x - p1.x );

            if( ( inner != ( t < 1 ) ) || ( t < 0 ) )
                return -1;

            return p1.y + t * ( p2.y - p1.y );
        }



        private void setNodeEdge(Node n, Set<Edge> edges, int weight)
        {
            for( Edge edge : edges )
            {
                Point p1 = getCenter(edge.from);
                Point p2 = getCenter(edge.to);
                setCrossingZoneBoundary(n.width, n.height, p1, p2, weight);
            }
        }

        public void setCrossingZoneBoundary(int width, int height, Point p1, Point p2, int weight)
        {
            int x1 = p1.x;
            int y1 = p1.y;
            int x2 = p2.x;
            int y2 = p2.y;

            width /= 2;
            height /= 2;

            int iLeft = (int)Math.ceil( ( (double)Math.min(x1, x2) - width ) / gridX - 1);
            int iRight = (int)Math.floor( ( (double)Math.max(x1, x2) + width ) / gridX - 1);

            iLeft = Math.max(iLeft, 0);
            iRight = Math.min(iRight, w - 1);

            if( x1 == x2 || y1 == y2 )
            {
                int jUp = (int)Math.ceil( ( (double)Math.min(y1, y2) - height ) / gridY - 1);
                int jDown = (int)Math.floor( ( (double)Math.max(y1, y2) + height ) / gridY - 1);

                jUp = Math.max(jUp, 0);
                jDown = Math.min(jDown, h - 1);

                for( int i = iLeft; i <= iRight; i++ )
                {
                    boundMatrix[i][jUp] += weight;
                    boundMatrix[i][jDown + 1] -= weight;
                }
                return;
            }

            Point p1Up;
            Point p2Up;
            Point p1Down;
            Point p2Down;

            if( ( x1 - x2 ) * ( y1 - y2 ) < 0 )
            {
                p1Up = new Point(x1 - width, y1 - height);
                p2Up = new Point(x2 - width, y2 - height);
                p1Down = new Point(x1 + width, y1 + height);
                p2Down = new Point(x2 + width, y2 + height);
            }
            else
            {
                p1Up = new Point(x1 + width, y1 - height);
                p2Up = new Point(x2 + width, y2 - height);
                p1Down = new Point(x1 - width, y1 + height);
                p2Down = new Point(x2 - width, y2 + height);
            }

            for( int i = iLeft; i <= iRight; i++ )
            {
                int x = ( i + 1 ) * gridX;
                double yUp = getYByX2(p1Up, p2Up, x);
                double yDown = getYByX2(p1Down, p2Down, x);

                int jUp = (int)Math.ceil(yUp / gridY - 1);
                int jDown = (int)Math.ceil(yDown / gridY - 1);

                if( yUp == -1 )
                {
                    jUp = (int)Math.ceil((double)Math.min(y1, y2) / gridY - 1);
                }

                if( yDown == -1 )
                {
                    jDown = (int)Math.ceil((double)Math.max(y1, y2) / gridY);
                }

                if( jUp == (int) ( yUp / gridY - 1 ) )
                    jUp++;

                jUp = Math.min( Math.max( jUp, 0 ), h );
                jDown = Math.min(jDown, h);

                boundMatrix[i][jUp] += weight;
                boundMatrix[i][jDown] -= weight;
            }
        }

        // Utility functions
        private double getYByX2(Point p1, Point p2, int x)
        {
            if( p2.x == p1.x )
                return Math.min(p1.y, p2.y);
            double t = ( x - p1.x );
            t /= ( p2.x - p1.x );

            if( t < 0 || t > 1 )
                return -1;

            return p1.y + t * ( p2.y - p1.y );
        }

        private void resetPenaltyMatrix()
        {
            for( int i = 0; i < w; i++ )
            {
                for( int j = 0; j < h + 1; j++ )
                {
                    boundMatrix[i][j] = 0;
                }
            }
        }

        private void setPenaltyMatrix()
        {
            for( int i = 0; i < w; i++ )
            {
                int sum = 0;
                for( int j = 0; j < h; j++ )
                {
                    sum += boundMatrix[i][j];
                    costMatrix[i][j] = sum;
                }
            }
        }

    }
    private void moveNode(Node node, Point p)
    {
        if( node != null && p != null && !node.fixed )
        {
            node.x = p.x;
            node.y = p.y;
        }

    }


    private Set<Node> multiply(Set<Node> list, Graph graph)
    {
        Set<Node> result = new HashSet<>();

        for( Node node : list )
        {
            result.addAll(Util.getNodes(node, graph));
        }
        return result;
    }


    //Matrix of weights for connections between nodes
    private void initNodeInteractionMap(Graph graph)
    {
        nodeInteractionMap.clear();
        List<Graph> subGraphs = graph.split();

        //all nodes reset to default
        for( Node node1 : graph.nodeList )
        {
            for( Node node2 : graph.nodeList )
            {
                nodeInteractionMap.put(node1.name + node2.name, 0);
            }
        }

        //all nodes without edges are attracting to each other
        Set<Node> lonelyNodes = new HashSet<>();
        for( Node node : graph.nodeList )
        {
            if( graph.getEdges( node ).isEmpty() )
            {
                lonelyNodes.add(node);
            }
        }

        for( Node node1 : lonelyNodes )
        {
            for( Node node2 : lonelyNodes )
            {
                if( !Util.getCompartments(node1, graph).contains(node2) && !Util.getCompartments(node2, graph).contains(node1) )
                    nodeInteractionMap.put(node1.name + node2.name, strongAttraction);
            }
        }

        for( Graph subGraph : subGraphs )
        {
            if( subGraph.nodeList.size() == 1 )
                continue;
            for( Node node1 : subGraph.nodeList )
            {
                Set<Node> compartments1 = Util.getCompartments(node1, graph);

                //nodeset1 - nodes (and compartments) connected with this node directly
                Set<Node> nodeSet1 = Util.getNodes(node1, graph);
                //nodeset2 - nodes connected with this node through another
                Set<Node> nodeSet2 = multiply(nodeSet1, graph);
                //nodeset3 - nodes connected with this node through two another
                Set<Node> nodeSet3 = multiply(nodeSet2, graph);
                //nodeset4 - nodes connected with this node through three another
                Set<Node> nodeSet4 = multiply(nodeSet3, graph);
                //nodeset5 - nodes connected with this node somehow
                Set<Node> nodeSet5 = new HashSet<>( subGraph.nodeList );
                //nodeset6 - nodes not connected with this node
                Set<Node> nodeSet6 = new HashSet<>( graph.nodeList );

                nodeSet6.removeAll(nodeSet5);
                nodeSet5.removeAll(nodeSet4);
                nodeSet6.removeAll(compartments1);
                nodeSet4.removeAll(nodeSet3);
                nodeSet3.removeAll(nodeSet2);
                nodeSet2.removeAll(nodeSet1);

                for( Node node2 : nodeSet6 )
                    nodeInteractionMap.put(node1.name + node2.name, strongRepulsion);
                for( Node node2 : nodeSet5 )
                    nodeInteractionMap.put(node1.name + node2.name, averageRepulsion);
                for( Node node2 : nodeSet4 )
                    nodeInteractionMap.put(node1.name + node2.name, weakRepulsion);
                for( Node node2 : nodeSet3 )
                    nodeInteractionMap.put(node1.name + node2.name, weakAttraction);
                for( Node node2 : nodeSet2 )
                {
                    Set<Node> compartments2 = Util.getCompartments(node2, graph);
                    for( Node compartment1 : compartments1 )
                    {
                        for( Node compartment2 : compartments2 )
                        {
                            nodeInteractionMap.put(compartment1.name + compartment2.name, averageAttraction);
                        }
                    }
                    nodeInteractionMap.put(node1.name + node2.name, averageAttraction);
                }
                for( Node node2 : nodeSet1 )
                {
                    Set<Node> compartments2 = Util.getCompartments(node2, graph);
                    for( Node compartment1 : compartments1 )
                    {
                        for( Node compartment2 : compartments2 )
                        {
                            nodeInteractionMap.put(compartment1.name + compartment2.name, strongAttraction);
                        }
                    }
                    nodeInteractionMap.put(node1.name + node2.name, strongAttraction);
                }
            }
        }
    }

    private void setAllowedPoints(Graph graph)
    {
        allowedPoints.clear();
        int iLeft = 0;
        int iRight = 0;
        int jUp = 0;
        int jDown = 0;

        for( Node node : graph.nodeList )
        {
            List<Point> points = new ArrayList<>();
            Node compartment = Util.getCompartment(node, graph);
            if( compartment != null )
            {
                double width = ( -node.width + compartment.width ) / 2.0;
                double height = ( -node.height + compartment.height ) / 2.0;

                iLeft = (int)Math.ceil(compartment.x - width / gridX);
                iRight = (int)Math.floor(compartment.x + width / gridX);
                jUp = (int)Math.ceil(compartment.y - height / gridY);
                jDown = (int)Math.ceil(compartment.y + height / gridY);
                if( height % gridY == 0 )
                    jDown++;
            }
            else
            {
                double width = node.width / 2.0;
                double height = node.height / 2.0;

                iLeft = (int)Math.ceil(width / gridX - 0.5);
                iRight = (int)Math.floor(w - width / gridX - 0.5);
                jUp = (int)Math.ceil(height / gridY - 0.5);
                jDown = (int)Math.floor(h - height / gridY + 0.5);
            }

            for( int i = iLeft; i <= iRight; i++ )
            {
                for( int j = jUp; j < jDown; j++ )
                {
                    points.add(new Point(i, j));
                }
            }
            allowedPoints.put(node.name, points);
        }
    }

    private void layoutCompartments(Graph graph)
    {
        HashMap<Integer, Set<Node>> levelToCompartmentMap = new HashMap<>();
        Set<Node> compartments = new HashSet<>();

        //setting compartments to levels
        for( Node node : graph.nodeList )
        {
            if( Util.isCompartment(node) )
            {
                compartments.add(node);
                Integer i = Util.getLevel(node);

                Set<Node> nodes = levelToCompartmentMap.get(i);
                if( nodes == null )
                    nodes = new HashSet<>();
                nodes.add(node);
                levelToCompartmentMap.put(i, nodes);
            }
        }

        int levelsCount = levelToCompartmentMap.size();

        //setting compartments size
        if( !isStartingFromThisLayout )
        {
            for( int k = levelsCount - 1; k >= 0; k-- )
            {
                compartments = levelToCompartmentMap.get(k);
                if( compartments == null )
                    continue;
                for( Node compartment : compartments )
                {
                    setCompartmentSize(compartment, graph);
                    compartment.width += ( levelsCount - k ) * 10;
                    compartment.height += ( levelsCount - k ) * 10;
                    store(compartment);
                }
            }
        }
        setGridSize(graph);

        Graph compartmentsGraph = new Graph();
        //layout compartments for all levels
        for( int k = 0; k < levelsCount; k++ )
        {
            Set<Node> addCompartments = levelToCompartmentMap.get(k);
            for( Node compartment : addCompartments )
                compartmentsGraph.addNode(compartment);

            setAllowedPoints(compartmentsGraph);
            distributedAnnealing(compartmentsGraph, null);
            for( Node compartment : addCompartments )
                compartment.fixed = true;
        }
    }

    private Point getCenter(Node node)
    {
        Point result = new Point();
        result.x = ( node.x + 1 ) * gridX;
        result.y = ( node.y + 1 ) * gridY;
        return result;
    }

    private void permutate(Graph graph, double stochasticRate)
    {
        for( Node node : graph.nodeList )
        {
            if( node.fixed )
                continue;
            if( Math.random() < stochasticRate )
                moveNode(node, getRandomAllowedPoint(node));
        }
    }

    private void copyFixedNodes(Graph graph)
    {
        fixedNodes = new HashMap<>();
        for( Node node : graph.nodeList )
        {
            if( node.fixed )
                fixedNodes.put( node.name, node.clone() );
        }
    }

    private void storeNodes(Graph graph)
    {
        for( Node node : graph.nodeList )
        {
            if( !Util.isCompartment( node ) )
                store( node );
        }
        isScaled = true;
    }

    private void restore(Graph graph)
    {
        for( Node node : graph.nodeList )
        {
            restore(node);
        }
        isScaled = false;
    }

    private void store(Node node)
    {
        int oldX = node.x, oldY = node.y;
        node.x = (int)Math.ceil( ( node.x + node.width / 2.0 ) / gridX);
        node.y = (int)Math.ceil( ( node.y + node.height / 2.0 ) / gridY);
        if( node.fixed )
        {
            node.width = 2 * ( ( node.x + 1 ) * gridX - oldX );
            node.height = 2 * ( ( node.y + 1 ) * gridY - oldY );
        }
    }

    private void restore(Node node)
    {
        node.x = ( node.x + 1 ) * gridX - node.width / 2;
        node.y = ( node.y + 1 ) * gridY - node.height / 2;
        if( node.fixed )
        {
            Node originalNode = fixedNodes.get( node.getName() );
            if( originalNode != null )
            {
                node.width = originalNode.width;
                node.height = originalNode.height;
            }
        }
    }


    private void copyNodeLayout(Graph graphTo, Graph graphFrom)
    {
        for( Node nodeTo : graphTo.nodeList )
        {
            Node nodeFrom = graphFrom.getNode(nodeTo.name);
            if( nodeFrom == null )
                continue;
            nodeTo.x = nodeFrom.x;
            nodeTo.y = nodeFrom.y;
            nodeTo.width = nodeFrom.width;
            nodeTo.height = nodeFrom.height;
        }
    }

    private Point getRandomAllowedPoint(Node node)
    {
        Random rand = new Random();
        List<Point> points = allowedPoints.get(node.name);
        int i = rand.nextInt(points.size());
        return points.get(i);
    }

    private Point getNearestAllowedPoint(Node node)
    {
        List<Point> points = allowedPoints.get(node.name);
        int x = node.x;
        int y = node.y;
        if( points == null )
            return null;
        Point result = points.get(0);
        int distance = Math.abs(result.x - x) + Math.abs(result.y - y);

        for( Point p : points )
        {
            int newDistance = Math.abs(p.x - x) + Math.abs(p.y - y);
            if( newDistance < distance )
            {
                result = new Point(p.x, p.y);
                distance = newDistance;
            }
        }
        return result;
    }

    private void setGridSize(Graph graph)
    {
        int maximumWidth = 0;
        int maximumHeight = 0;
        int width = 0;
        int height = 0;
        for( Node n : graph.nodeList )
        {
            if( Util.getCompartment(n, graph) == null )
            {
                int addWidth = 2 * (int)Math.ceil((double)n.width / ( 2 * gridX ) - 0.5) + 1;
                int addHeight = 2 * (int)Math.ceil((double)n.height / ( 2 * gridY ) - 0.5) + 1;

                if( addWidth > maximumWidth )
                {
                    width += maximumWidth;
                    maximumWidth = addWidth;
                }
                else
                {
                    width += addWidth;
                }

                if( addHeight > maximumHeight )
                {
                    height += maximumHeight;
                    maximumHeight = addHeight;
                }
                else
                {
                    height += addHeight;
                }
            }
        }
        w = maximumWidth + 2 * (int)Math.round(Math.sqrt(width));
        h = maximumHeight + 3 * (int)Math.round(Math.sqrt(height));
    }

    private void setCompartmentSize(Node compartment, Graph graph)
    {
        int width = 0;
        int height = 0;
        int nodeWidth = 0;
        int nodeHeight = 0;
        for( Node node : graph.nodeList )
        {
            Node nodeCompartment = Util.getCompartment(node, graph);
            if( nodeCompartment == null || !nodeCompartment.equals(compartment) )
                continue;

            double addWidth = 2 * Math.ceil( ( (double)node.width / ( 2 * gridX ) ) - 0.5) + 1;
            double addHeight = 2 * Math.ceil( ( (double)node.height / ( 2 * gridY ) ) - 0.5) + 1;

            boolean incWidth = false;
            boolean incHeight = false;

            if( addWidth > addHeight )
            {
                incWidth = true;
            }
            else if( addWidth < addHeight )
            {
                incHeight = true;
            }
            else if( width <= height )
            {
                incWidth = true;
            }
            else
            {
                incHeight = true;
            }

            if( incWidth )
            {
                if( addWidth > 3 )
                    nodeWidth += addWidth;
                else
                    width += addWidth;
            }
            if( incHeight )
            {
                if( addHeight > 3 )
                    nodeHeight += addHeight;
                else
                    height += addHeight;
            }

            if( addWidth > nodeWidth )
                nodeWidth += addWidth;
            if( addHeight > nodeHeight )
                nodeHeight += addHeight;
        }
        width = nodeWidth + 3 * (int)Math.round(Math.sqrt(width));
        height = nodeHeight + 3 * (int)Math.round(Math.sqrt(height));

        if( width == 0 )
            width++;
        else if( width % 2 == 0 )
            width--;

        if( height == 0 )
            height++;
        else if( height % 2 == 0 )
            height--;

        compartment.width = width * gridX;
        compartment.height = height * gridY;
    }

    @Override
    public LayouterInfo getInfo()
    {
        LayouterInfoSupport lis = new LayouterInfoSupport(true, true, false, false, false, true);
        return lis;
    }


    @Override
    public int estimate(Graph graph, int what)
    {
        initNodeInteractionMap(graph);
        layoutCompartments(graph);
        setAllowedPoints(graph);
        copyFixedNodes( graph );
        storeNodes( graph );
        tMax = ( isStartingFromThisLayout() ) ? new Layouting(graph).getCost() : calulateMaximumTemperature(graph);
        double steps = Math.log(tMin / tMax);
        steps /= Math.log(cool);
        isEstimationDone = true;
        return (int)steps * iterations;
    }

    //Getters and setters
    public int getIterations()
    {
        return iterations;
    }
    public void setIterations(int iterations)
    {
        this.iterations = iterations;
    }
    public double getCool()
    {
        return cool;
    }
    public void setCool(double cool)
    {
        this.cool = cool;
    }
    public double getPerturbationRate()
    {
        return stochasticRate;
    }
    public void setPerturbationRate(double rate)
    {
        this.stochasticRate = rate;
    }
    public int getGridX()
    {
        return gridX;
    }
    public void setGridX(int gridX)
    {
        this.gridX = gridX;
    }
    public int getGridY()
    {
        return gridY;
    }
    public void setGridY(int gridY)
    {
        this.gridY = gridY;
    }
    public int getThreadCount()
    {
        return threadCount;
    }
    public void setThreadCount(int count)
    {
        this.threadCount = count;
    }
    public boolean isStartingFromThisLayout()
    {
        return isStartingFromThisLayout;
    }
    public void setStartingFromThisLayout(boolean val)
    {
        isStartingFromThisLayout = val;
    }
    public Layouter getPathLayouter()
    {
        return pathLayouter;
    }
    public void setPathLayouter(Layouter val)
    {
        pathLayouter = val;
    }
    public int getEdgeEdgeCrossCost()
    {
        return edgeEdgeCrossCost;
    }
    public void setEdgeEdgeCrossCost(int wEE)
    {
        edgeEdgeCrossCost = wEE;
    }
    public int getEdgeNodeCrossCost()
    {
        return edgeNodeCrossCost;
    }
    public void setEdgeNodeCrossCost(int wNE)
    {
        edgeNodeCrossCost = wNE;
    }
    public int getNodeNodeCrossCost()
    {
        return nodeNodeCrossCost;
    }
    public void setNodeNodeCrossCost(int wNN)
    {
        nodeNodeCrossCost = wNN;
    }
    public int getStrongAttraction()
    {
        return strongAttraction;
    }
    public void setStrongAttraction(int wD1)
    {
        strongAttraction = wD1;
    }
    public int getAverageAttraction()
    {
        return averageAttraction;
    }
    public void setAverageAttraction(int wD2)
    {
        averageAttraction = wD2;
    }
    public int getWeakAttraction()
    {
        return weakAttraction;
    }
    public void setWeakAttraction(int wD3)
    {
        weakAttraction = wD3;
    }
    public int getWeakRepulsion()
    {
        return weakRepulsion;
    }
    public void setWeakRepulsion(int wD4)
    {
        weakRepulsion = wD4;
    }
    public int getAverageRepulsion()
    {
        return averageRepulsion;
    }
    public void setAverageRepulsion(int wD5)
    {
        averageRepulsion = wD5;
    }
    public int getStrongRepulsion()
    {
        return strongRepulsion;
    }
    public void setStrongRepulsion(int wD6)
    {
        strongRepulsion = wD6;
    }
}
