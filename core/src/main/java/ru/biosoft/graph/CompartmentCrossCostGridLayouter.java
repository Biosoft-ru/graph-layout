package ru.biosoft.graph;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.Vector;

import java.util.logging.Level;

public class CompartmentCrossCostGridLayouter extends AbstractLayouter
{
    protected static final char PATH_DELIMITER = '.';

    protected int gridX = 70; // X-distance between grid points
    protected int gridY = 70; // Y-distance between grid points

    protected static int nodeWidth = 30; // node width (should be less then X-distance between grid points)
    protected static int nodeHeight = 30; // node height (should be less then Y-distance between grid points)

    protected int gridSize = 3; // border length of our square grid

    protected int gridWidth = 3;
    protected int gridHeight = 3;

    protected boolean setGridAutomatically = true;// choose grid size automatically
    protected boolean setCompSizesAutomatically = true;// choose compartment size automatically

    protected Node[][] gridLayout; // grid and nodes located on it
    protected GridPoint[][] grid; // grid

    protected double[][] deltaMatrix; // Strings of delta matrix corresponds to nodes and columns to vacant points
    protected Vector<Node> nodeVector = new Vector<>(); // delta matrix strings coordinates

    protected int edgeCrossPenalty = 50;
    protected int nodeCrossPenalty = 100;
    protected int nodeOverlapPenalty = 1000;

    protected int connectedNodesWeight = 5; // used in setWeightMatrix and edgeEdgeCrossCostFunction

    protected List<Edge> edgeList = new ArrayList<>(); // edge list of corresponding graph

    protected Map<String, Node> nodeMap = new HashMap<>(); // holds Nodes with their names as keys

    protected Map<String, Integer> nodeIndexMap = new HashMap<>(); // holds indexes of nodes in nodeVector vector

    protected int[][] weightMap; // weight between two nodes as a function of distance on graph

    protected Map<Node, List<Edge>> edgeMap = new HashMap<>(); // edge map of corresponding graph

    protected Map<String, List<GridPoint>> compartmentPointMap = new HashMap<>();

    Map<String, List<Node>> compartmentMap = new HashMap<>(); // String key - level of compartment, "0" - list of base compartments, which are not attached in any other compartment, "1" - compartments which parent compartment is in "0" list (base) etc.

    protected Graph withoutCompartmentGraph = new Graph();

    double tMax = 0.11; // Max annealing temperature
    double tMin = 0.1; // Min annealing temperature
    int ne = 4; // number of iterations on each annealing step
    double rc = 0.2; // cooling coefficient of annealing

    double probabilityThreshold = 0.3; // perturbation threshold used in neighbor function
    int saturationDist = 4; // max distance on grid (longer distances are not taken into account when repulsion is estimated)


    protected static int shiftMultiplyCoeff = 8;
    protected static int nodeShift = 2; // shift which is used in nodeOverlap procedure
    protected static int nodeOffset = 4; // offset which is used in nodeOverlap procedure

    private final Random rnd = new Random(0);

    protected static int compartmentAnnealingSteps = 3;

    protected ForceDirectedLayouter forceDirectedLayouter = new ForceDirectedLayouter();

    public CompartmentCrossCostGridLayouter()
    {
        pathLayouterWrapper = new PathLayouterWrapper( new DiagonalPathLayouter() );
    }

    protected double makeTmax(Graph graph) //max temperature identification
    {
        double maxCost = -9999;
        double minCost = 99999999;
        double cost = 0;
        for( int i = 0; i < 10; i++ )
        {
            layoutPermutation();
            cost = layoutCostFunction();
            if( cost > maxCost )
                maxCost = cost;
            if( cost < minCost )
                minCost = cost;
        }
        if (maxCost == minCost) return 0.11;
        return ( maxCost - minCost ) / 2;
    }


    protected int checkEdgeCross(Edge e1, Edge e2)
    {
        if( e1.from == e2.from || e1.from == e2.to || e1.to == e2.from || e1.to == e2.to )
            return 0;

        int e1x1, e1y1, e1x2, e1y2; // end points coordinates for edge e1
        int e2x1, e2y1, e2x2, e2y2; // end points coordinates for edge e2

        e1x1 = e1.from.x * gridX;
        e1y1 = e1.from.y * gridY;

        e1x2 = e1.to.x * gridX;
        e1y2 = e1.to.y * gridY;

        e2x1 = e2.from.x * gridX;
        e2y1 = e2.from.y * gridY;

        e2x2 = e2.to.x * gridX;
        e2y2 = e2.to.y * gridY;

        return segmentIntersect(e1x1, e1y1, e1x2, e1y2, e2x1, e2y1, e2x2, e2y2, edgeCrossPenalty);
    }


    protected int distance(int x1, int y1, int x2, int y2) // manhattan distance between two points (x1, y1) and (x2, y2)
    {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    protected int checkNodesOverlap(Node n1, Node n2)
    {
        if( checkForAttachment(n1, n2) )
            return 0;

        int offset1 = 0;
        int shift1 = 0;
        int offset2 = 0;
        int shift2= 0;

        int n1X = n1.x * gridX;
        int n1Y = n1.y * gridY;
        int n1Width = n1.width;
        int n1Height = n1.height;

        int n2X = n2.x * gridX;
        int n2Y = n2.y * gridY;
        int n2Width = n2.width;
        int n2Height = n2.height;

        if( !Util.isCompartment(n1))
        {
            n1X -= n1Width / 2d;
            n1Y -= n1Height / 2d;
            offset1 = nodeOffset;
            shift1 = nodeShift;

        }

        if( !Util.isCompartment(n2))
        {
            n2X -= n2Width / 2d;
            n2Y -= n2Height / 2d;
            offset2 = nodeOffset;
            shift2= nodeShift;
        }

        Rectangle rect1 = new Rectangle(n1X - shift1, n1Y - shift1, n1Width + offset1, n1Height + offset1);
        Rectangle rect2 = new Rectangle(n2X - shift2, n2Y - shift2, n2Width + offset2, n2Height + offset2);
        if( rect1.intersects(rect2) )
            return nodeOverlapPenalty;
        else
            return 0;
    }

    protected int checkNodeCross(Edge e, Node n)
    {
        if( e.from == n || e.to == n )
            return 0;

        if( Util.isCompartment(n) )
        {
            if( checkForAttachment(n, e.from) || checkForAttachment(n, e.to) )
                return 0;
        }
        int ex1, ey1, ex2, ey2; // end points coordinates for edge e1
        int nx1, ny1, nx2, ny2;

        ex1 = e.from.x * gridX;
        ey1 = e.from.y * gridY;

        ex2 = e.to.x * gridX;
        ey2 = e.to.y * gridY;

        nx1 = n.x * gridX - (int) ( n.width / 2d );
        ny1 = n.y * gridY - (int) ( n.height / 2d );

        nx2 = n.x * gridX - (int) ( n.width / 2d );
        ny2 = n.y * gridY - (int) ( n.height / 2d );

        if( ( ex1 - ex2 ) * ( ey1 - ey2 ) >= 0 )
        {
            ny1 += n.height;
            nx2 += n.width;
            return segmentIntersect(nx1, ny1, nx2, ny2, ex1, ey1, ex2, ey2, nodeCrossPenalty);
        }
        else
        {
            nx2 += n.width;
            ny2 += n.height;
            return segmentIntersect(nx1, ny1, nx2, ny2, ex1, ey1, ex2, ey2, nodeCrossPenalty);
        }
    }


    protected void defineCompartments(Graph graph, LayoutJobControl lJC) // initialize compartment points and nodes in corresponding compartments
    {
        this.compartmentMap = new HashMap<>(); // String key - level of compartment, "0" - list of base compartments, which are not attached in any other compartment, "1" - compartments which parent compartment is in "0" list (base) etc.
        HashMap<String, Integer> compartmentNodeNumberMap = new HashMap<>(); // contains compartment node number
        HashMap<String, Integer[]> compartmentNodeSizeMap = new HashMap<>(); // contains sum of corresponding dimensions of all compartment nodes
        HashMap<String, List<Node>> compartmentNodeMap = new HashMap<>(); // contains Nodes of corresponding compartment


        int maxAttachment = 0;

        for( Node n : graph.nodeList )
        {
        	n.fixed = false;
            List<Node> tempList = new ArrayList<>();
            if( Util.isCompartment(n)) // && !isEmptyComp(n)
            {
                if(n.getAttribute("compartmentName")==null || n.getAttribute("compartmentName").equals("root") )
                {
                    if( !compartmentMap.containsKey("0") )
                    {
                        tempList.add(n);
                        compartmentMap.put("0", tempList);
                    }
                    else
                    {
                        compartmentMap.get("0").add(n);
                    }
                    n.setAttribute("compartmentName", "root");
                }
                else
                {
                    int delimiterCount = n.getAttribute("compartmentName").length()
                            - n.getAttribute("compartmentName").replaceAll("\\.", "").length();
                    if( delimiterCount > maxAttachment )
                        maxAttachment = delimiterCount;
                    if( !compartmentMap.containsKey("" + delimiterCount) )
                    {
                        tempList.add(n);
                        compartmentMap.put("" + delimiterCount, tempList);
                    }
                    else
                    {
                        compartmentMap.get("" + delimiterCount).add(n);
                    }
                    String compName = n.getAttribute("compartmentName");
                    if (!compName.startsWith("root"))
                    n.setAttribute("compartmentName", "root."+n.getAttribute("compartmentName"));

                }
            }
            else
            {
                String compName = "root";
                if (n.getAttribute("compartmentName") == null)
                	n.setAttribute("compartmentName", compName);
                else if(!n.getAttribute("compartmentName").startsWith("root"))
                	compName += "."+n.getAttribute("compartmentName");
                else compName = n.getAttribute("compartmentName");


                n.setAttribute("compartmentName", compName);

                if( !compartmentNodeNumberMap.containsKey(compName) )
                {
                    compartmentNodeNumberMap.put(compName, 1);
                    Integer[] dim = new Integer[2];
                    dim[0] = n.width;
                    dim[1] = n.height;
                    compartmentNodeSizeMap.put(compName, dim);
                    tempList.add(n);
                    compartmentNodeMap.put(compName, tempList);
                }
                else
                {
                    int tempInt = compartmentNodeNumberMap.get(compName) + 1;
                    compartmentNodeNumberMap.put(compName, tempInt);
                    Integer[] dim = compartmentNodeSizeMap.get(compName);
                    dim[0] += n.width;
                    dim[1] += n.height;
                    compartmentNodeSizeMap.put(compName, dim);
                    compartmentNodeMap.get(compName).add(n);
                }

            }
        }


        if( setCompSizesAutomatically )
        {
            defineCompartmentsSizes(maxAttachment, compartmentNodeNumberMap, compartmentNodeSizeMap);
        }
        setGrid(compartmentNodeNumberMap, compartmentNodeSizeMap, graph);
        setCompartments();
        layoutCompartments(compartmentNodeMap, lJC);

    }

//    protected boolean isEmptyComp(Node comp) // true when comp is Node or Compartment without Nodes insight, else - false
//    {
//        String fullName = getFullName(comp.name);
//        for( String key : nodeMap.keySet() )
//        {
//            Node n = nodeMap.get(key);
//            String compName = n.getAttribute("compartmentName");
//            if( compName != null && compName.startsWith(fullName) )
//                return false;
//        }
//        return true;
//    }

    protected void defineCompartmentsSizes(int maxAttachment, HashMap<String, Integer> compartmentNodeNumberMap,
            HashMap<String, Integer[]> compartmentNodeSizeMap)
    {
        int counter = maxAttachment;
        List<Node> tempList;

        while( counter > -1 )
        {
            tempList = compartmentMap.get("" + counter);
            for( Node n : tempList )
            {
                n.width = 0;
                n.height = 0;
            }
            counter--;
        }

        counter = maxAttachment;

        while( counter > -1 )
        {
            tempList = compartmentMap.get("" + counter);
            for( Node n : tempList )
            {
                String compName = getFullName(n.name);
                boolean innerCompartment = false;

                if( n.width == 0 )
                    innerCompartment = true;


                int newWidthDimentionInc = 0;
                int newHeightDimentionInc = 0;
                int upperNodeWidth = 0;
                int upperNodeHeight = 0;
                if( innerCompartment )
                {
                    int innerNodeNumber = 0;
                    do
                    {
                        n = nodeMap.get(getNodeName(compName));
                        int nodeNumber = 1;
                        int avWidth = 0;
                        int avHeight = 0;
                        if( compartmentNodeNumberMap.containsKey(compName) )
                        {
                            nodeNumber = compartmentNodeNumberMap.get(compName);
                            avWidth = compartmentNodeSizeMap.get(compName)[0];
                            avHeight = compartmentNodeSizeMap.get(compName)[1];
                        }
                        //                        int innerNodeNumber = 0;
                        if( innerCompartment )
                        {
                            n.width = 2 * (int)Math.sqrt(nodeNumber)
                                    * (int) ( 1 + ( avWidth ) / ( gridX * nodeNumber ) + 2 * (double) ( nodeOffset + nodeShift ) / gridX )
                                    + (int) ( (double)avWidth / ( gridX * nodeNumber ) );
                            n.height = 2 * (int)Math.sqrt(nodeNumber)
                                    * (int) ( 1 + ( avHeight ) / ( gridY * nodeNumber ) + 2 * (double) ( nodeOffset + nodeShift ) / gridY )
                                    + (int) ( (double)avHeight / ( gridY * nodeNumber ) );
                            if( nodeNumber < 4 )
                            {
                                n.width = n.width + 1;
                                n.height = n.height + 1;
                            }
                            newWidthDimentionInc = n.width;
                            newHeightDimentionInc = n.height;
                            upperNodeWidth = newWidthDimentionInc;
                            upperNodeHeight = newHeightDimentionInc;
                            innerNodeNumber = nodeNumber;
                        }
                        else
                        {
                            if( n.width == 0 && n.height == 0 )
                            {
                                newWidthDimentionInc += (int) ( -2 * Math.sqrt(innerNodeNumber) + Math.sqrt(4 * innerNodeNumber + 4
                                        * Math.sqrt(innerNodeNumber) + 1 + 4 * nodeNumber) )
                                        * (int) ( 1 + ( avWidth ) / ( gridX * nodeNumber ) + (double) ( nodeOffset + nodeShift ) / gridX )
                                        + (int) ( (double)avWidth / ( gridX * nodeNumber ) );
                                newHeightDimentionInc += (int) ( -2 * Math.sqrt(innerNodeNumber) + Math.sqrt(4 * innerNodeNumber + 4
                                        * Math.sqrt(innerNodeNumber) + 1 + 4 * nodeNumber) )
                                        * (int) ( 1 + ( avHeight ) / ( gridY * nodeNumber ) + (double) ( nodeOffset + nodeShift ) / gridY )
                                        + (int) ( (double)avHeight / ( gridY * nodeNumber ) );
                                n.width += newWidthDimentionInc;
                                n.height += newHeightDimentionInc;
                            }
                            else
                            {
                                if( upperNodeWidth >= n.width || upperNodeHeight >= n.height )
                                {
                                    n.width += newWidthDimentionInc;
                                    n.height += newHeightDimentionInc;
                                }
                                else
                                {
                                    if( newWidthDimentionInc >= newHeightDimentionInc )
                                    {
                                        n.width += newWidthDimentionInc;
                                    }
                                    else
                                    {
                                        n.height += newHeightDimentionInc;
                                    }
                                }
                            }
                            upperNodeWidth = n.width;
                            upperNodeHeight = n.height;
                            innerNodeNumber += nodeNumber;
                        }

                        innerCompartment = false;
                        compName = n.getAttribute("compartmentName");
                    }
                    while( !compName.equalsIgnoreCase("root") );
                }
            }
            counter--;
        }

    }

    protected String getNodeName(String compName)
    {
        if( compName.indexOf(PATH_DELIMITER) != -1 )
        {
            return compName.replace("root.", "");//substring(compName.lastIndexOf(PATH_DELIMITER) + 1, compName.length());
        }
        else
            return compName;
    }

    protected String getFullName(String nodeName)
    {
    	return "root." + nodeName;
    }

    protected void layoutCompartments(HashMap<String, List<Node>> compartmentNodeMap,  LayoutJobControl lJC)
    {
        makeWeightMatrix(compartmentNodeMap);

        for( int i = 0; i < compartmentMap.size(); i++ )
        {
            List<Node> compList = new ArrayList<>();

            for( int j = i; j >= 0; j-- )
            {
                compList.addAll(compartmentMap.get("" + j));
            }

            Graph g = createGraph(compList, null);
            if( i != 0 )
            {
                nonBaseLevelFirstLayout(g);
            }
            else
            {
                baseLevelFirstLayout(g);
            }
            this.doCompLayout(g, lJC);
            setNodesFixed(compartmentMap.get("" + i), true);
            formatGrid(i);
            renewCompartmentPointMap();
        }
    }


    protected void makeWeightMatrix(HashMap<String, List<Node>> compartmentNodeMap)
    {
        int nodeCount = nodeVector.size();
        this.weightMap = new int[nodeCount][nodeCount];

        HashMap<String, List<Node>> totalNodeMap = new HashMap<>(); // Contains key - compartment name, values - nodes contained in this compartment ad all compartments attached in this one
        List<Node> compTotalNodeList = new ArrayList<>();
        List<Node> tempTotalNodeList = new ArrayList<>();
        for( int i = compartmentMap.size() - 1; i >= 0; i-- )
        {
            List<Node> nodeList = compartmentMap.get("" + i);
            for( Node n : nodeList )
            {
                String compName = getFullName(n.name);
                if( totalNodeMap.containsKey(compName) )
                    continue;
                do
                {
                    n = nodeMap.get(getNodeName(compName));
                    if( totalNodeMap.containsKey(compName) )
                    {
                        compTotalNodeList.addAll(totalNodeMap.get(compName));
                        tempTotalNodeList.addAll(compTotalNodeList);
                        totalNodeMap.put(compName, tempTotalNodeList);
                        tempTotalNodeList = new ArrayList<>();
                        compName = "";
                    }
                    else
                    {
                        if( compartmentNodeMap.containsKey(compName) )
                        {
                            compTotalNodeList.addAll(compartmentNodeMap.get(compName));
                        }
                        tempTotalNodeList.addAll(compTotalNodeList);
                        totalNodeMap.put(compName, tempTotalNodeList);
                        tempTotalNodeList = new ArrayList<>();
                    }
                    compName = n.getAttribute("compartmentName");
                }
                while( !compName.equalsIgnoreCase("root") );
                compTotalNodeList = new ArrayList<>();
            }
        }


        List<Node> compList = new ArrayList<>();
        List<Node> tempList = new ArrayList<>();

        for( int j = 0; j < compartmentMap.size(); j++ )
        {
            compList.addAll(compartmentMap.get("" + j));
        }
        tempList.addAll(compList);
        for( Node nC : compList )
        {
            for( Node n : tempList )
            {
                Integer nCIndex = nodeIndexMap.get( nC.name );
                Integer nIndex = nodeIndexMap.get( n.name );
                if( checkForAttachment(nC, n) )
                {
                    //if we come from estimateCompartments method, nodeIndexMap will be empty
                    if( nCIndex != null && nIndex != null )
                        this.weightMap[nCIndex][nIndex] = 0;
                    continue;
                }
                List<Node> compNodeList = totalNodeMap.get(getFullName(nC.name));
                List<Node> tempNodeList = totalNodeMap.get(getFullName(n.name));
                int weight = 1; // initially weight = 1 (but not 0)  to make compartments without edges to attract to each other

                Set<Edge> compEdgeList = new HashSet<>();
                Set<Edge> tempEdgeList = new HashSet<>();

                for( Node compartmentN : compNodeList )
                {
                    if( edgeMap.containsKey(compartmentN) )
                    {
                        compEdgeList.addAll(edgeMap.get(compartmentN));
                    }
                }

                for( Node tempN : tempNodeList )
                {
                    if( edgeMap.containsKey(tempN) )
                    {
                        tempEdgeList.addAll(edgeMap.get(tempN));
                    }
                }

                for( Edge edge : tempEdgeList )
                {
                    if( compEdgeList.contains(edge) )
                        weight++;
                }
                //if we come from estimateCompartments method, nodeIndexMap will be empty
                if( nCIndex != null && nIndex != null )
                    this.weightMap[nCIndex][nIndex] = weight;
                //                    this.weightMap.put(n.name+nC.name, weight);
            }
        }

    }


    protected boolean checkForAttachment(Node node1, Node node2) // check if one of the nodes is a parent of another (or subparent)
    {
        String name1 = node1.name;
        String name2 = node2.name;
        if( name1.length() > name2.length() )
        {
            return name1.startsWith( name2 ) && name1.charAt( name2.length() ) == PATH_DELIMITER;
        }
        else if( name1.length() < name2.length() )
        {
            return name2.startsWith( name1 ) && name2.charAt( name1.length() ) == PATH_DELIMITER;
        }
        else
            return false;
    }


    protected void setNodesFixed(List<Node> nodeList, boolean fix)
    {
        for( Node n : nodeList )
        {
            n.fixed = fix;
        }
    }

    protected void changeLayout(Graph graph)
    {
        int nodeCount = graph.nodeCount();
        Node tempNode;
        nodeVector = new Vector<>();

        GridPoint position;// = new GridPoint(0, 0);

        for( int i = 0; i < nodeCount; i++ )
        {
            tempNode = graph.nodeAt(i);
            nodeVector.addElement(tempNode);
            nodeIndexMap.put(tempNode.getName(), i);
        }

        for( int i = 0; i < nodeCount; i++ )
        {
            tempNode = graph.nodeAt(i);
            if( !tempNode.fixed )
            {
                position = getRandomVacantPoint(tempNode);
                gridLayout[position.y][position.x] = tempNode;
                tempNode.x = position.x;
                tempNode.y = position.y;
            }

        }

    }



    protected void baseLevelFirstLayout(Graph graph)
    {
        List<Node> nodeList = graph.nodeList;
        int nodeCount = graph.nodeCount();
        Node tempNode;
        nodeVector = new Vector<>();
        gridLayout = new Node[gridHeight][gridWidth];

        for( int i = 0; i < nodeCount; i++ )
        {
            tempNode = graph.nodeAt(i);
            nodeVector.addElement(tempNode);
            nodeIndexMap.put(tempNode.getName(), i);
        }

        List<Node> arrangedNodeList = new ArrayList<>();
        while( nodeList.size() > 0 )
        {
            Node maxNode = nodeList.get(0);
            int squareN1 = maxNode.width * maxNode.height;
            for( Node n2 : nodeList )
            {
                int squareN2 = n2.width * n2.height;
                if( squareN2 > squareN1 )
                    maxNode = n2;
            }
            arrangedNodeList.add(maxNode);
            nodeList.remove(maxNode);
        }

        int xPoint = 0;
        int yPoint = 0;
        for( Node n : arrangedNodeList )
        {
            if( xPoint == 0 && yPoint == 0 )
            {
                gridLayout[yPoint][xPoint] = n;
                n.x = xPoint;
                n.y = yPoint;
                xPoint += (int) ( n.width + 0.001 ) / gridX;
                yPoint += (int) ( n.height + 0.001 ) / gridY;
                continue;
            }

            if( xPoint + (int) ( n.width + 0.001 ) / gridX <= gridWidth )
            {
                gridLayout[0][xPoint] = n;
                n.x = xPoint;
                n.y = 0;
                xPoint += (int) ( n.width + 0.001 ) / gridX;
            }
            else
            {
                gridLayout[yPoint][0] = n;
                n.x = 0;
                n.y = yPoint;
                yPoint += (int) ( n.height + 0.001 ) / gridY;

            }

        }

    }

    protected void nonBaseLevelFirstLayout(Graph graph)
    {
        HashMap<String, List<Node>> compNamesMap = new HashMap<>();
        List<String> compNamesList = new ArrayList<>();

        int nodeCount = graph.nodeCount();
        Node tempNode;
        nodeVector = new Vector<>();

        List<Node> tempCompList = new ArrayList<>();

        for( Node n : graph.nodeList )
        {
            if( !n.fixed )
            {
                String compName = n.getAttribute("compartmentName");

                if (compName==null)
                    compName = "root";

                if( compNamesMap.containsKey(compName) )
                {
                    compNamesMap.get(compName).add(n);
                }
                else
                {
                    tempCompList.add(n);
                    compNamesMap.put(compName, tempCompList);
                    tempCompList = new ArrayList<>();
                    compNamesList.add(compName);
                }
            }
        }

        for( int i = 0; i < nodeCount; i++ )
        {
            tempNode = graph.nodeAt(i);
            nodeVector.addElement(tempNode);
            nodeIndexMap.put(tempNode.getName(), i);
        }

        for( String cN : compNamesList )
        {
            Node cNnode = nodeMap.get(getNodeName(cN));
            List<Node> nodeList = compNamesMap.get(cN);
            List<Node> arrangedNodeList = new ArrayList<>();
            while( nodeList.size() > 0 )
            {
                Node maxNode = nodeList.get(0);
                int squareN1 = maxNode.width * maxNode.height;
                for( Node n2 : nodeList )
                {
                    int squareN2 = n2.width * n2.height;
                    if( squareN2 > squareN1 )
                        maxNode = n2;
                }
                arrangedNodeList.add(maxNode);
                nodeList.remove(maxNode);
            }
            int xPoint = cNnode.x + 1;
            int yPoint = cNnode.y;
            for( Node n : arrangedNodeList )
            {
                if( xPoint == ( cNnode.x + 1 ) && yPoint == cNnode.y )
                {
                    gridLayout[yPoint][xPoint] = n;
                    n.x = xPoint;
                    n.y = yPoint;
                    xPoint += (int) ( n.width + 0.001 ) / gridX;
                    yPoint += (int) ( n.height + 0.001 ) / gridY;
                    continue;
                }

                if( xPoint + (int) ( n.width + 0.001 ) / gridX <= (int) ( cNnode.width + 0.001 ) / gridX + cNnode.x )
                {
                    gridLayout[cNnode.y][xPoint] = n;
                    n.x = xPoint;
                    n.y = cNnode.y;
                    xPoint += (int) ( n.width + 0.001 ) / gridX;
                }
                else
                {
                    gridLayout[yPoint][cNnode.x] = n;
                    n.x = cNnode.x;
                    n.y = yPoint;
                    yPoint += (int) ( n.height + 0.001 ) / gridY;

                }

            }
        }

    }

    protected void formatGrid(int upCompLevel)
    {
        List<Node> currentLevelCompList = compartmentMap.get("" + upCompLevel);
        for( Node n : currentLevelCompList )
        {
            for( int i = n.y; i < n.y + (int) ( n.height + 0.001 ) / gridY; i++ )
            {
                for( int j = n.x; j < n.x + (int) ( n.width + 0.001 ) / gridX; j++ )
                {
                    grid[i][j].compartmentName = getFullName(n.name);
                }

            }
        }
    }

    protected void renewCompartmentPointMap()
    {
        compartmentPointMap = new HashMap<>();
        //      List<String> compNameList = new ArrayList<String>();
        List<GridPoint> tempGP = new ArrayList<>();
        for( int i = 0; i < gridHeight; i++ )
        {
            for( int j = 0; j < gridWidth; j++ )
            {
                String compName = grid[i][j].compartmentName;
                if( !compartmentPointMap.containsKey(compName) )
                {
                    tempGP.add(grid[i][j]);
                    compartmentPointMap.put(compName, tempGP);
                    tempGP = new ArrayList<>();
                    //                   compNameList.add(compName);
                }
                else
                {
                    compartmentPointMap.get(compName).add(grid[i][j]);
                }
            }

        }

    }

    protected void setGrid(HashMap<String, Integer> compartmentNodeNumberMap, HashMap<String, Integer[]> compartmentNodeSizeMap, Graph graph)
    {
        List<Node> baseComp = compartmentMap.get("0");
        List<GridPoint> tempGP = new ArrayList<>();

        gridWidth = 0;
        gridHeight = 0;
        for( Node n : baseComp )
        {
            if( n.width >= gridWidth || n.height >= gridHeight )
            {
                gridWidth += n.width;
                gridHeight += n.height;
                continue;
            }
            if( n.width >= n.height )
            {
                gridWidth += n.width;
            }
            else
            {
                gridHeight += n.height;
            }

        }

        if( compartmentNodeNumberMap.containsKey("root") )
        {
            int nodeNumber = compartmentNodeNumberMap.get("root");

            gridWidth += 2
                    * Math.sqrt(nodeNumber)
                    * (int) ( 1 + ( compartmentNodeSizeMap.get("root")[0] ) / ( gridX * nodeNumber ) + (double) ( nodeOffset + nodeShift )
                            / gridX )
                    + (int) ((double)compartmentNodeSizeMap.get("root")[0] / ( gridX * nodeNumber ) );
            gridHeight += 2
                    * Math.sqrt(nodeNumber)
                    * (int) ( 1 + ( compartmentNodeSizeMap.get("root")[1] ) / ( gridY * nodeNumber ) + (double) ( nodeOffset + nodeShift )
                            / gridY )
                    + (int) ((double)compartmentNodeSizeMap.get("root")[1] / ( gridY * nodeNumber ) );
        }


        gridLayout = new Node[gridHeight][gridWidth];
        grid = new GridPoint[gridHeight][gridWidth];


        for( int i = 0; i < gridHeight; i++ )
        {
            for( int j = 0; j < gridWidth; j++ )
            {
                gridLayout[i][j] = null;
                grid[i][j] = new GridPoint(j, i);
                grid[i][j].compartmentName = "root";
                tempGP.add(grid[i][j]);
            }
        }

        compartmentPointMap.put("root", tempGP);

    }


    protected void makeGrid(Graph graph) // initialize grid if graph without compartments
    {
        int nodeCount = graph.nodeCount();
        if( setGridAutomatically )
            gridSize = 2 * ( (int)Math.sqrt(nodeCount) );

        int sumWidth = 0;
        int sumHeight = 0;

        for( int i = 0; i < nodeCount; i++ )
        {
            sumWidth += graph.nodeAt(i).width;
            sumHeight += graph.nodeAt(i).height;
        }

        gridWidth = gridSize * (int) ( 1 + (double) ( sumWidth ) / ( gridX * nodeCount ) + (double) ( nodeOffset + nodeShift ) / gridX );
        gridHeight = gridSize * (int) ( 1 + (double) ( sumHeight ) / ( gridY * nodeCount ) + (double) ( nodeOffset + nodeShift ) / gridY );
        gridLayout = new Node[gridHeight][gridWidth];
        grid = new GridPoint[gridHeight][gridWidth];
        Node tempNode;

        for( int i = 0; i < nodeCount; i++ )
        {
            tempNode = graph.nodeAt(i);
            nodeVector.addElement(tempNode);
            nodeIndexMap.put(tempNode.getName(), i);
        }

        List<GridPoint> tempGP = new ArrayList<>();
        int k = 0;
        for( int i = 0; i < gridHeight; i++ )
        {
            for( int j = 0; j < gridWidth; j++ )
            {
                gridLayout[i][j] = null;

                if( k < nodeCount )
                {
                    gridLayout[i][j] = nodeVector.get(k);
                    if( gridLayout[i][j].getAttribute("compartmentName") == null )
                    {
                        gridLayout[i][j].setAttribute("compartmentName", "root");
                    }
                    gridLayout[i][j].x = j;
                    gridLayout[i][j].y = i;
                    k++;
                }
                grid[i][j] = new GridPoint(j, i);
                grid[i][j].compartmentName = "root";
                tempGP.add(grid[i][j]);

            }
        }

        compartmentPointMap.put("root", tempGP);
    }

    protected void setWeightMatrix(Graph graph) // initialize weight matrix for nodes
    {
        int nodeCount = graph.nodeCount();
        this.weightMap = new int[nodeCount][nodeCount];
        Node tempNode;
        List<Edge> edgeList;
        double[][] m1 = new double[nodeCount][nodeCount];


        for( int i = 0; i < nodeCount; i++ )
        {
            m1[i][i] = 1;
            tempNode = nodeVector.get(i);
            edgeList = graph.getEdges(tempNode);
            if( edgeList != null )
            {
                for( Edge edge : edgeList )
                {
                    if( ( edge.from ).equals(tempNode) )
                        m1[i][nodeVector.indexOf(edge.to)] = 1;
                    else
                        m1[i][nodeVector.indexOf(edge.from)] = 1;
                }
            }
        }

        try
        {
            double[][] m2 = matrixMultiply(m1, m1);
            double[][] m3 = matrixMultiply(m1, m2);
            double[][] m4 = matrixMultiply(m2, m2);


            for( int i = 0; i < nodeCount; i++ )
            {
                for( int j = 0; j < nodeCount; j++ )
                {
                    if( m1[i][j] > 0 )
                        weightMap[i][j] = connectedNodesWeight;
                    else if( m1[i][j] == 0 && m2[i][j] > 0 )
                        weightMap[i][j] = 2;
                    else if( m2[i][j] == 0 && m3[i][j] > 0 )
                        weightMap[i][j] = 1;
                    else if( m3[i][j] == 0 && m4[i][j] > 0 )
                        weightMap[i][j] = 0;
                    else
                        weightMap[i][j] = -1;
                }
            }

        }
        catch( Exception e )
        {
        }


        String attractorName= "";
        if (compartmentMap.size() > 0)
        {
        	List<Node> nl = compartmentMap.get("0");
        	int minCoord = 696969;
        	for (Node n : nl)
        	{
        	  if (n.x + n.y < minCoord)
        	  {
        		  minCoord = n.x + n.y;
        		  attractorName = n.name;
        	  }
        	}
         }

        for( int i = 0; i < nodeCount; i++ )
        {
            for( int j = 0; j < nodeCount; j++ )
            {
                Node node1 = nodeVector.get( i );
                Node node2 = nodeVector.get( j );
                if( node1.name.equalsIgnoreCase( attractorName ) || node2.name.equalsIgnoreCase( attractorName ) )
                    weightMap[i][j] = 1;
                else if( Util.isCompartment( node1 ) || Util.isCompartment( node2 ) )
                    weightMap[i][j] = 0;
                else if( edgeMap.get( node1 ) == null && edgeMap.get( node2 ) == null )
                    weightMap[i][j] = 1;
            }
        }

    }


    public void layoutPermutation() // permutate given layout
    {
        for( Node node : nodeVector )
        {
            if( !node.fixed )
            {
                //GridPoint gp1 = getGridPosition(node);
                GridPoint gp2 = getRandomVacantPoint(node);
                moveNodeToPoint(gp2, node);
                //           changePointNodes(gp1, gp2);
            }
        }
    }


    protected GridPoint getGridPosition(Node node) // get position of the node on grid
    {
        GridPoint position = grid[node.y][node.x];
        return position;

    }


    protected int getDistance(Node ri, Node rj) // distance between two nodes
    {
        return Math.abs(ri.x - rj.x) + Math.abs(ri.y - rj.y);
    }



    protected double CostFunction(Node ri, Node rj) //cost between two nodes
    {
        double cost = 0;
        double weight = weightMap[nodeIndexMap.get( ri.getName() )][nodeIndexMap.get( rj.getName() )];
        if( weight >= 0 )
            cost = weight * getDistance(ri, rj);
        else
            cost = weight * Math.min(getDistance(ri, rj), saturationDist);
        return cost;
    }


    protected double edgeEdgeCrossCostFunction(Node alpha, Node betta)
    {
        double cost = 0;
        if( weightMap[nodeIndexMap.get( alpha.getName() )][nodeIndexMap.get( betta.getName() )] == connectedNodesWeight )
        {
            Edge alphaBetta = new Edge( alpha, betta );
            for( Edge e1 : edgeMap.get( alpha ) )
            {
                if( e1.from.equals( betta ) || e1.to.equals( betta ) )
                    alphaBetta = e1;
                else
                    for( Edge e2 : edgeMap.get( betta ) )
                        cost += checkEdgeCross( e1, e2 );
            }

            for( Edge e : edgeList )
                cost += checkEdgeCross( alphaBetta, e );
        }
        else
        {
            if( edgeMap.get(alpha) != null && edgeMap.get(betta) != null )
                for( Edge e1 : edgeMap.get(alpha) )
                {
                    for( Edge e2 : edgeMap.get(betta) )
                    {
                        cost += checkEdgeCross(e1, e2);
                    }
                }
        }
        return cost;
    }



    protected double nodeEdgeCrossCostFunction(Node alpha, Node betta)
    {
        double cost = 0;
        if( weightMap[nodeIndexMap.get( alpha.getName() )][nodeIndexMap.get( betta.getName() )] == connectedNodesWeight )
        {
            Edge alphaBetta = new Edge(alpha, betta);
            for( Edge e : edgeMap.get(alpha) )
            {
                if( e.from.equals(betta) || e.to.equals(betta) )
                {
                    alphaBetta = e;
                }
                else
                    cost += checkNodeCross(e, betta);
            }


            for( Edge e : edgeMap.get(betta) )
            {
                cost += checkNodeCross(e, alpha);
            }


            int nodeCount = nodeVector.size();
            for( int i = 0; i < nodeCount; i++ )
            {
                cost += checkNodeCross(alphaBetta, nodeVector.get(i));
            }
        }
        else
        {
            if( edgeMap.get(alpha) != null )
            {
                for( Edge e : edgeMap.get(alpha) )
                {
                    cost += checkNodeCross(e, betta);
                }

            }
            if( edgeMap.get(betta) != null )
            {
                for( Edge e : edgeMap.get(betta) )
                {
                    cost += checkNodeCross(e, alpha);
                }
            }
        }
        return cost;
    }


    protected double nodeCostFunction(Node node) //cost between one chosen node and all other nodes
    {
        double cost = 0;
        int nodeCount = nodeVector.size();

        for( Node rj : nodeVector )
        {
            cost += CostFunction(node, rj);
            cost += checkNodesOverlap(node, rj); //overlap
        }
        // crossings
        if( edgeMap.get(node) != null )
        {
            for( Edge e1 : edgeMap.get(node) )
            {
                for( Edge e2 : edgeList )
                {
                    cost += checkEdgeCross(e1, e2);
                }

                for( int i = 0; i < nodeCount; i++ )
                {
                    cost += checkNodeCross(e1, nodeVector.get(i));
                }

            }
        }

        for( Edge e : edgeList )
        {
            cost += checkNodeCross(e, node);
        }


        return cost;
    }


    protected double layoutCostFunction() // cost of whole layout
    {
        double cost = 0;
        int nodeCount = nodeVector.size();
        for( int i = 0; i < nodeCount - 1; i++ )
        {
            Node ri = nodeVector.get(i);
            for( int j = i + 1; j < nodeCount; j++ )
            {
                Node rj = nodeVector.get(j);
                cost += CostFunction(ri, rj);
                cost += checkNodesOverlap(ri, rj); // overlap
            }
        }
        //      crossings
        for( int i = 0; i < edgeList.size(); i++ )
            for( int j = 0; j < i; j++ )
            {
                Edge e1 = edgeList.get( i );
                Edge e2 = edgeList.get( j );
                cost += checkEdgeCross( e1, e2 );
            }

        for( int i = 0; i < nodeCount; i++ )
        {
            for( Edge e : edgeList )
            {
                cost += checkNodeCross(e, nodeVector.get(i));
            }
        }
        return cost;
    }


    protected void neighbour(String[][] stringArray) // transform gridLayout, represented as stringArray, into a similar, the coefficient of similarity is probabilityThreshold, if probabilityThreshold = 1 two layouts are equal, probabilityThreshold = 0 - layouts are completely different
    {
        gridLayout = makeLayoutFromStringArray(stringArray);
        double rand = 0;
        GridPoint position;// = new GridPoint(0, 0);
        Node tempNode;

        for( Node node : nodeVector )
        {
            if( !node.fixed )
            {
                rand = rnd.nextDouble();
                tempNode = node;
                if( rand < probabilityThreshold )
                {
                    position = getRandomVacantPoint(tempNode);
                    moveNodeToPoint(position, tempNode);
                }
            }
        }
    }


    protected Node[][] makeLayoutFromStringArray(String[][] stringArray)
    {
        int strNumber = stringArray.length;
        int colNumber = stringArray[0].length;
        Node[][] nodeMatrix = new Node[strNumber][colNumber];
        for( int i = 0; i < strNumber; i++ )
            for( int j = 0; j < colNumber; j++ )
            {
                if( stringArray[i][j] == null )
                    nodeMatrix[i][j] = null;
                else
                {
                    nodeMatrix[i][j] = nodeMap.get(stringArray[i][j]);
                    nodeMatrix[i][j].x = j;
                    nodeMatrix[i][j].y = i;
                }
            }
        return nodeMatrix;
    }


    protected void moveNodeToPoint(GridPoint gp, Node node) // moves given node to the point on grid with given coordinates
    {
        int xCord = gp.x;
        int yCord = gp.y;
        if( gridLayout[yCord][xCord] == null || gridLayout[yCord][xCord].fixed )
        {
            GridPoint position = getGridPosition(node);
            gridLayout[yCord][xCord] = node;
            node.x = xCord;
            node.y = yCord;
            gridLayout[position.y][position.x] = null;
        }
    }


    protected void changePointNodes(GridPoint gp1, GridPoint gp2)
    {
        int xCord1 = gp1.x;
        int yCord1 = gp1.y;
        int xCord2 = gp2.x;
        int yCord2 = gp2.y;
        Node tempNode = gridLayout[yCord1][xCord1];

        gridLayout[yCord1][xCord1] = gridLayout[yCord2][xCord2];
        gridLayout[yCord2][xCord2] = tempNode;

        if( gridLayout[yCord1][xCord1] != null )
        {
            gridLayout[yCord1][xCord1].x = xCord1;
            gridLayout[yCord1][xCord1].y = yCord1;
        }
        if( gridLayout[yCord2][xCord2] != null )
        {
            gridLayout[yCord2][xCord2].x = xCord2;
            gridLayout[yCord2][xCord2].y = yCord2;
        }
    }



    protected GridPoint[] getVacantPointList() // makes array of all vacant points on grid
    {
        int vacantCount = gridLayout.length * gridLayout[0].length - nodeVector.size();
        GridPoint[] vacantPositionList = new GridPoint[vacantCount];
        int vacantPointNumber = 0;
        for( int i = 0; i < gridLayout.length; i++ )
        {
            for( int j = 0; j < gridLayout[0].length; j++ )
            {
                if( gridLayout[i][j] == null )
                {
                    vacantPositionList[vacantPointNumber] = grid[i][j];
                    vacantPointNumber++;
                }
            }
        }
        return vacantPositionList;
    }


    protected GridPoint getRandomCompartmentPoint(Node node) // get random vacant point in compartment
    {
        String compartmentName = node.getAttribute("compartmentName");
        if (compartmentName==null)
            compartmentName = "root";
        List<GridPoint> compartment = compartmentPointMap.get(compartmentName);
        List<GridPoint> notNullCompartment = new ArrayList<>();


        for( GridPoint gp : compartment )
        {
            if( gp != null )
                notNullCompartment.add(gp);

        }

        int compVacantGridPointQuantity = 0;

        for( GridPoint gp : notNullCompartment )
        {
            if( gridLayout[gp.y][gp.x] == null )
                compVacantGridPointQuantity++;

        }

        GridPoint position = new GridPoint(0, 0);
        int randint = randInt(compVacantGridPointQuantity - 1);
        int count = 0;

        for( GridPoint gp : notNullCompartment )
        {
            if( gridLayout[gp.y][gp.x] == null )
            {
                if( count == randint )
                {
                    position = grid[gp.y][gp.x];
                    break;
                }
                count++;
            }

        }
        return position;
    }



    protected GridPoint getRandomVacantPoint(Node node) // get random vacant point in compartment
    {
        if( node.fixed )
            return getGridPosition(node);
        else
        {
            String compartmentName = node.getAttribute("compartmentName");
            if (compartmentName==null)
                compartmentName = "root";
            List<GridPoint> compartment = compartmentPointMap.get(compartmentName);
            int compartmentVacantPointCount = 0;

            for( GridPoint gp : compartment )
            {
                if( gp == null )
                    continue;
                if( gridLayout[gp.y][gp.x] == null && isIn(node, gp) )
                    compartmentVacantPointCount++;
            }

            if( compartmentVacantPointCount == 0 )
                return getRandomCompartmentPoint(node);

            int randint = randInt(compartmentVacantPointCount - 1);

            int count = 0;

            GridPoint position = getRandomCompartmentPoint(node);

            for( GridPoint gp : compartment )
            {
                if( gp == null )
                    continue;
                if( gridLayout[gp.y][gp.x] == null && isIn(node, gp) )
                {
                    if( count == randint )
                    {
                        position = grid[gp.y][gp.x];
                        break;
                    }
                    count++;
                }

            }
            return position;
        }
    }


    protected void initDeltaMatrix(Graph graph) // initialize delta matrix
    {
        int vacantCount = gridLayout.length * gridLayout[0].length - nodeVector.size();
        int nodeCount = graph.nodeCount();
        deltaMatrix = new double[nodeCount][vacantCount];
    }


    protected double localMin(LayoutJobControl lJC) // minimization of gridlayout cost, returns some local minimum layout
    {
        double cost;
        double dMin = 0;
        GridPoint[] vacantPointList = getVacantPointList();
        double[][] newDeltaMatrix = new double[deltaMatrix.length][deltaMatrix[0].length];

        GridPoint newPosition = new GridPoint(0, 0);
        int newPositionNumber = 0;
        Node newNode = null;

        for( Node node : nodeVector )
        {
            if( !node.fixed )
            {
                GridPoint currentNodePosition = getGridPosition(node);
                for( int vacantPointNumber = 0; vacantPointNumber < vacantPointList.length; vacantPointNumber++ )
                {
                    GridPoint tempGP = vacantPointList[vacantPointNumber];

                    if( ( node.getAttribute("compartmentName").equalsIgnoreCase(vacantPointList[vacantPointNumber].compartmentName) )
                            && isIn(node, tempGP) )
                    {
                        moveNodeToPoint(vacantPointList[vacantPointNumber], node);

                        deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = nodeCostFunction(node);

                        moveNodeToPoint(currentNodePosition, node);

                        deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] -= nodeCostFunction(node);

                        if( deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] < dMin )
                        {
                            dMin = deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber];
                            newPosition = vacantPointList[vacantPointNumber];
                            newPositionNumber = vacantPointNumber;
                            newNode = node;
                        }
                    }
                    else
                        deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = 69;
                }
            }
            else
            {
                for( int vacantPointNumber = 0; vacantPointNumber < vacantPointList.length; vacantPointNumber++ )
                {
                    deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = 69;
                }
            }
        }

        if( newNode == null )
        {
            return layoutCostFunction();
        }

        Node tempNewNode = null;
        GridPoint tempNewPosition = new GridPoint(0, 0);
        int tempNewPositionNumber = 0;


        try
        {
            while( dMin < 0 )
            {
                GridPoint oldBettaPosition = getGridPosition(newNode);
                vacantPointList[newPositionNumber] = oldBettaPosition;
                double dMinTemp = 0;
                for( Node node : nodeVector )
                {
                    if( !node.fixed )
                    {
                        for( int vacantPointNumber = 0; vacantPointNumber < vacantPointList.length; vacantPointNumber++ )
                        {
                            if( vacantPointNumber != newPositionNumber || node != newNode )
                            {
                                GridPoint tempGP = vacantPointList[vacantPointNumber];
                                if( ( node.getAttribute("compartmentName")
                                        .equalsIgnoreCase(vacantPointList[vacantPointNumber].compartmentName) )
                                        && isIn(node, tempGP) )
                                {
                                    newDeltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = deltaMatrixFastCalculate(node,
                                            newNode, vacantPointNumber, newPositionNumber, newPosition, vacantPointList); // newPosition ? newPositionNumber
                                    if( newDeltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] < dMinTemp )
                                    {
                                        dMinTemp = newDeltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber];
                                        tempNewNode = node;
                                        tempNewPosition = vacantPointList[vacantPointNumber];
                                        tempNewPositionNumber = vacantPointNumber;
                                    }
                                }
                                else
                                    newDeltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = 69;
                            }
                        }
                    }
                    else
                    {
                        for( int vacantPointNumber = 0; vacantPointNumber < vacantPointList.length; vacantPointNumber++ )
                        {
                            newDeltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = 69;
                        }
                    }
                }

                deltaMatrix[nodeIndexMap.get(newNode.getName())][newPositionNumber] = -dMin;
                moveNodeToPoint(newPosition, newNode);
                for( Node node : nodeVector )
                {
                    for( int vacantPointNumber = 0; vacantPointNumber < vacantPointList.length; vacantPointNumber++ )
                    {
                        if( vacantPointNumber != newPositionNumber || node != newNode )
                        {
                            deltaMatrix[nodeIndexMap.get(node.getName())][vacantPointNumber] = newDeltaMatrix[nodeIndexMap.get(node
                                    .getName())][vacantPointNumber];
                        }
                    }
                }

                newNode = tempNewNode;
                newPosition = tempNewPosition;
                newPositionNumber = tempNewPositionNumber;
                dMin = dMinTemp;
                if (lJC != null) {if (lJC.getStatus() == 4 || lJC.getStatus() == 5) dMin= 69;}
            }
        }
        catch( Exception e )
        {
        }
        cost = layoutCostFunction();
        return cost;
    }


    protected double deltaMatrixFastCalculate(Node alpha, Node betta, int positionPnumber, int positionQnumber, GridPoint positionQ,
            GridPoint[] vacantPointList) // calculation of delta matrix
    {
        double deltaAP = 0;
        GridPoint bettaPosition = getGridPosition(betta);

        GridPoint alphaPosition = getGridPosition(alpha);

        GridPoint positionP = vacantPointList[positionPnumber];
        if( alpha != betta && positionP != bettaPosition )
        {
            double term = CostFunction(alpha, betta);
            term += edgeEdgeCrossCostFunction(alpha, betta);
            term += nodeEdgeCrossCostFunction(alpha, betta);
            term += checkNodesOverlap(alpha, betta); //overlap

            moveNodeToPoint(positionP, alpha);
            term -= CostFunction(alpha, betta);
            term -= edgeEdgeCrossCostFunction(alpha, betta);
            term -= nodeEdgeCrossCostFunction(alpha, betta);
            term += checkNodesOverlap(alpha, betta);//overlap

            moveNodeToPoint(alphaPosition, alpha);

            moveNodeToPoint(positionQ, betta);
            term -= CostFunction(alpha, betta);
            term -= edgeEdgeCrossCostFunction(alpha, betta);
            term -= nodeEdgeCrossCostFunction(alpha, betta);
            term += checkNodesOverlap(alpha, betta);//overlap

            moveNodeToPoint(positionP, alpha);
            term += CostFunction(alpha, betta);
            term += edgeEdgeCrossCostFunction(alpha, betta);
            term += nodeEdgeCrossCostFunction(alpha, betta);
            term += checkNodesOverlap(alpha, betta);//overlap

            moveNodeToPoint(alphaPosition, alpha);
            moveNodeToPoint(bettaPosition, betta);

            deltaAP = deltaMatrix[nodeIndexMap.get(alpha.getName())][positionPnumber] + term;

        }
        else if( alpha == betta )
        {
            deltaAP = deltaMatrix[nodeIndexMap.get(betta.getName())][positionPnumber]
                    - deltaMatrix[nodeIndexMap.get(betta.getName())][positionQnumber];
        }
        else
        {
            moveNodeToPoint(positionQ, betta);
            deltaAP -= nodeCostFunction(alpha);
            moveNodeToPoint(positionP, alpha);
            deltaAP += nodeCostFunction(alpha);
            moveNodeToPoint(alphaPosition, alpha);
            moveNodeToPoint(bettaPosition, betta);
        }
        return deltaAP;
    }



    protected Node[][] gridLayout(double tMax, double tMin, int ne, double rc, boolean comp, LayoutJobControl lJC) //annealing procedure of finding some local minimum layout
    {
        double temperature = tMax;

        String[][] rMin;// = new String[gridSize][gridSize];

        double f = localMin(lJC);
        if (lJC != null) lJC.done(++operationsDone);
        double fMin = f;

        rMin = nodeMatrixClone(gridLayout);
        String[][] tempGridLayout = nodeMatrixClone(gridLayout);

        while( temperature > tMin )
        {
            for( int i = 0; i < ne; i++ )
            {

                neighbour(tempGridLayout);
                double tempF = localMin(lJC);

                double probability = rnd.nextDouble();
                if( probability < Math.exp( ( f - tempF ) / temperature) )
                {
                    f = tempF;
                    tempGridLayout = nodeMatrixClone(gridLayout);

                }

                if( tempF < fMin ) //( f < fMin )
                {
                    fMin = tempF; // (fMin = f)
                    rMin = nodeMatrixClone(gridLayout);
                }
            }
            temperature = rc * temperature;
            if (lJC != null)
            {
            	if (lJC.getStatus() == 4 || lJC.getStatus() == 5) temperature = tMin;

            	lJC.done(++operationsDone);
            }
        }

        return makeLayoutFromStringArray(rMin);
    }


    @Override
	protected void layoutNodes(Graph graph, LayoutJobControl lJC)
    {
        gridLayout = gridLayout(tMax, tMin, ne, rc, false, lJC);
    }

    protected void setCoordinates(Node[][] gridLayout)
    {
        for( int i = 0; i < gridLayout.length; i++ )
        {
            for( int j = 0; j < gridLayout[0].length; j++ )
            {
                if( gridLayout[i][j] != null )
                {

                    if( !Util.isCompartment(gridLayout[i][j]))
                    {
                        gridLayout[i][j].x = j * gridX - (int) ( gridLayout[i][j].width / 2d );
                        gridLayout[i][j].y = i * gridY - (int) ( gridLayout[i][j].height / 2d );
                    }
                    else
                    {
                        gridLayout[i][j].x = j * gridX;
                        gridLayout[i][j].y = i * gridY;
                    }
                }
            }
        }
    }


    @Override
	public void layoutEdges(Graph graph, LayoutJobControl lJC)
    {
        for( Edge edge : graph.edgeList )
            layoutPath(graph, edge, lJC);
    }


    protected int getRange(int[] point1, int[] point2) // manhattan distance between two points
    {
        return Math.abs(point1[0] - point2[0]) + Math.abs(point1[1] - point2[1]);
    }


    @Override
    public void layoutPath(Graph graph, Edge edge, LayoutJobControl lJC)
    {
        getPathLayouter().layoutPath( graph, edge, lJC );
    }

    protected void doCompLayout(Graph graph, LayoutJobControl lJC)
    {
        tMax = tMin / Math.pow(rc, compartmentAnnealingSteps);
        if( rc == 0 )
            tMax = tMin + 0.00000001;
        setWeightMatrix( graph );
        initDeltaMatrix(graph);
        layoutNodes(graph, lJC);

    }

    protected void setCompartments()
    {
        int currLevel = compartmentMap.size() - 1;
        List<Node> nodeList = new ArrayList<>();
        while( currLevel > -1 )
        {
            nodeList.addAll(compartmentMap.get("" + ( currLevel )));
            currLevel--;
        }
        for( Node n : nodeList )
        {
            n.width *= gridX;
            n.height *= gridY;
        }

    }


    @Override
	public void doLayout(Graph graph, LayoutJobControl lJC)
    {
        long time = System.currentTimeMillis();

        if (lJC != null) lJC.begin();

        this.operationsDone = 0;

        edgeList = new ArrayList<>();
        nodeMap = new HashMap<>();
        nodeIndexMap = new HashMap<>();
        edgeMap = new HashMap<>();
        compartmentPointMap = new HashMap<>();
        compartmentMap = new HashMap<>();
        nodeVector = new Vector<>();

        this.nodeMap = graph.nodeMap;
        this.edgeMap = graph.edgeMap;

        edgeList = graph.edgeList;


        //        LayoutRun lr = new LayoutRun(graph);
        //        Thread tr = new Thread(lr);
        //        tr.start();
        //        try
        //        {
        //        tr.join();
        //        }
        //        catch (Exception e)
        //        {
        //
        //        }
        if( checkForCompartments(graph) )
        {
            defineCompartments(graph, lJC);
            changeLayout(graph);
        }
        else
        {
            makeGrid(graph);
        }

        setWeightMatrix(graph);
        tMax = makeTmax(graph);
        initDeltaMatrix(graph);
        layoutNodes(graph, lJC);
        //        setCompartments();
        setCoordinates(gridLayout);

        Util.adjustOrientations(graph);
        layoutEdges(graph, lJC);

        setNodesFixed(graph.nodeList, false);

        log.log(Level.INFO, "Layout time " + ( System.currentTimeMillis() - time ));
    }

    public int randInt(int k) // random integer from [0, k]
    {
        return rnd.nextInt( k + 1 );
    }

    /**
     * Multiplying two matrices: matrix1*matrix2
     */
    public static double[][] matrixMultiply(double[][] matrix1, double[][] matrix2) throws Exception
    {
        if( matrix1[0].length != matrix2.length )
            throw new Exception("matrices dimensions doesn't match");
        double[][] result = new double[matrix1.length][matrix2[0].length];
        double[][] conMatrix2 = matrixConjugate(matrix2);
        for( int i = 0; i < matrix1.length; i++ )
        {
            for( int j = 0; j < matrix2[0].length; j++ )
            {
                result[i][j] = scalarMultiply(matrix1[i], conMatrix2[j]);
            }
        }
        return result;
    }

    public static double[][] matrixConjugate(double[][] matrix) throws Exception
    {
        double[][] conMatrix = new double[matrix[0].length][matrix.length];
        for( int i = 0; i < matrix.length; i++ )
        {
            for( int j = 0; j < matrix[0].length; j++ )
            {
                conMatrix[j][i] = matrix[i][j];
            }
        }
        return conMatrix;
    }

    /**
     * Scalar multiplying two vectors
     */
    public static double scalarMultiply(double[] vector1, double[] vector2) throws Exception
    {
        if( vector1.length != vector2.length )
            throw new Exception("vectors are with different lengths");
        double result = 0;
        for( int i = 0; i < vector1.length; i++ )
            result += vector1[i] * vector2[i];
        return result;
    }

    public static String[][] nodeMatrixClone(Node[][] nodeMatrix) // cloning of node matrix
    {
        int strNumber = nodeMatrix.length;
        int colNumber = nodeMatrix[0].length;
        String[][] clonedNameMatrix = new String[strNumber][colNumber];
        for( int i = 0; i < strNumber; i++ )
            for( int j = 0; j < colNumber; j++ )
            {
                if( nodeMatrix[i][j] == null )
                    clonedNameMatrix[i][j] = null;
                else
                    clonedNameMatrix[i][j] = nodeMatrix[i][j].getName();
            }
        return clonedNameMatrix;
    }

    protected static Graph createGraph(List<Node> nodes, List<Edge> edges)
    {
        Graph graph = new Graph();
        for( Node n : nodes )
        {
            graph.addNode(n);
        }
        if( edges != null )
        {
            for( Edge e : edges )
            {
                graph.addEdge(e);
            }
        }
        return graph;
    }

    protected void addCompartments(Graph graph)
    {
        int counter = 0;
        while( compartmentMap.containsKey("" + counter) )
        {
            List<Node> compList = compartmentMap.get("" + counter);
            for( Node n : compList )
            {
                graph.addNode(n);
            }
            counter++;
        }
    }


    protected boolean checkForCompartments(Graph graph)
    {
        for( Node n : graph.nodeList )
        {
            if( Util.isCompartment(n))
            {
                return true;
            }
        }
        return false;
    }

    // returns shift of node for appropriate layout picture
    protected static int setShift(Node n)
    {
        int shift = 0;
        int pointEntry = 0;
        String compName = n.getAttribute("compartmentName");
        if( compName.equalsIgnoreCase("") )
        {
            return shift;
        }
        else
        {
            pointEntry = compName.length() - compName.replaceAll("\\.", "").length() + 1;
            shift = pointEntry * shiftMultiplyCoeff;
        }
        return shift;
    }


    protected boolean isIn(Node node, GridPoint gp)
    {
        int leftX = 0;
        int leftY = 0;

        int rightX = gridWidth * gridX;
        int rightY = gridHeight * gridY;

        int nodeLeftX = gp.x * gridX;
        int nodeLeftY = gp.y * gridY;

        int nodeRightX = gp.x * gridX;
        int nodeRightY = gp.y * gridY;

        if( !Util.isCompartment(node))
        {
            nodeLeftX -= (int) ( node.width / 2d );
            nodeLeftY -= (int) ( node.height / 2d );

            nodeRightX += (int) ( node.width / 2d );
            nodeRightY += (int) ( node.height / 2d );
        }
        else
        {
            nodeRightX += node.width;
            nodeRightY += node.height;
        }
        if( !node.getAttribute("compartmentName").equalsIgnoreCase("root") )
        {
            Node tempNode = nodeMap.get(getNodeName(node.getAttribute("compartmentName")));
            leftX = tempNode.x * gridX;
            leftY = tempNode.y * gridY;

            rightX = tempNode.x * gridX + tempNode.width;
            rightY = tempNode.y * gridY + tempNode.height;
        }

        if( leftX <= nodeLeftX && leftY <= nodeLeftY && rightX >= nodeRightX && rightY >= nodeRightY )
            return true;

        return false;
    }


// new approach +++++++++++++++++++++++++++++++

    protected int segmentIntersect(int p1x1, int p1y1, int p1x2, int p1y2, int p2x1, int p2y1, int p2x2, int p2y2, int penalty)
    {
        if (!rectIntersect(p1x1, p1y1, p1x2, p1y2, p2x1, p2y1, p2x2, p2y2)) return 0;
        else
        {
           int s1 = area(p2x1, p2y1, p2x2, p2y2, p1x1, p1y1);
           int s2 = area(p2x1, p2y1, p2x2, p2y2, p1x2, p1y2);
           int s3 = area(p1x1, p1y1, p1x2, p1y2, p2x1, p2y1);
           int s4 = area(p1x1, p1y1, p1x2, p1y2, p2x2, p2y2);

           if (((s1>0 && s2<0) || (s1<0 && s2>0)) && ((s3<0 && s4>0) || (s3>0 && s4<0))) return penalty;
           else if (s1 == 0 && between(p2x1, p2y1, p2x2, p2y2, p1x1, p1y1)) return penalty;
           else if (s2 == 0 && between(p2x1, p2y1, p2x2, p2y2, p1x2, p1y2)) return penalty;
           else if (s3 == 0 && between(p1x1, p1y1, p1x2, p1y2, p2x1, p2y1)) return penalty;
           else if (s4 == 0 && between(p1x1, p1y1, p1x2, p1y2, p2x2, p2y2)) return penalty;
           else return 0;
        }
    }

    protected boolean rectIntersect(int p1x1, int p1y1, int p1x2, int p1y2, int p2x1, int p2y1, int p2x2, int p2y2)
    {
      int rect1X1 = Math.min(p1x1, p1x2);
      int rect1Y1 = Math.min(p1y1, p1y2);
      int rect1X2 = Math.max(p1x1, p1x2);
      int rect1Y2 = Math.max(p1y1, p1y2);

      int rect2X1 = Math.min(p2x1, p2x2);
      int rect2Y1 = Math.min(p2y1, p2y2);
      int rect2X2 = Math.max(p2x1, p2x2);
      int rect2Y2 = Math.max(p2y1, p2y2);

      if (rect1X2>=rect2X1 && rect2X2>=rect1X1 && rect1Y2>=rect2Y1 && rect2Y2>=rect1Y1) return true;

      return false;
    }

    protected boolean between(int p1x1, int p1y1, int p1x2, int p1y2, int p2x1, int p2y1)
    {
      if (Math.min(p1x1, p1x2)<=p2x1 && p2x1<=Math.max(p1x1, p1x2) && Math.min(p1y1, p1y2)<=p2y1 && p2y1<=Math.max(p1y1, p1y2)) return true;
      return false;
    }


    protected int area(int p1x1, int p1y1, int p1x2, int p1y2, int p2x1, int p2y1)
    {
        return (p1x2-p1x1)*(p2y1-p1y1)-(p1y2-p1y1)*(p2x1-p1x1);
    }

 //++++++++++++++++++++++++++++++++++++++++++++++++


    @Override
    public LayouterInfo getInfo()
    {
       LayouterInfoSupport lis = new LayouterInfoSupport(true, true, false, true, false, true);
       return lis;
    }


    @Override
    public int estimate(Graph gr, int what)
    {
    	Graph graph = createGraph(gr.nodeList, gr.edgeList);
        edgeList = new ArrayList<>();
        nodeMap = new HashMap<>();
        nodeIndexMap = new HashMap<>();
        edgeMap = new HashMap<>();
        compartmentPointMap = new HashMap<>();
        compartmentMap = new HashMap<>();
        nodeVector = new Vector<>();

        this.nodeMap = graph.nodeMap;
        this.edgeMap = graph.edgeMap;

        edgeList = graph.edgeList;

        if( checkForCompartments(graph) )
        {
            estimateCompartments(graph);
        	changeLayout(graph);
        }
        else
        {
            makeGrid(graph);
        }

        setWeightMatrix(graph);

        double maxTemp = makeTmax(graph);
        int maxAttachment = 0;
        int compNumberOfAnnealingSteps = 0;
        if( checkForCompartments(graph) )
        {
            compNumberOfAnnealingSteps++;
            for( Node n : graph.nodeList )
            {
                if( n.getAttribute("compartmentName").equals("root") )
                {

                    int delimiterCount = n.getAttribute("compartmentName").length()
                            - n.getAttribute("compartmentName").replaceAll("\\.", "").length() + 1;
                    if( delimiterCount > maxAttachment )
                        maxAttachment = delimiterCount;
                }
            }
        }

        clearRoots(graph);

        compNumberOfAnnealingSteps += maxAttachment * compartmentAnnealingSteps + maxAttachment;
        int apprNumberOfAnnealingSteps = (int) ( ( Math.log(tMin) - Math.log(maxTemp) ) / Math.log(rc) + 1 ) + 1; //(int)(tMin/(maxTemp*Math.log(rc))+1)+1;
        return apprNumberOfAnnealingSteps + compNumberOfAnnealingSteps;
    }

    protected void estimateCompartments (Graph graph)
    {
        this.compartmentMap = new HashMap<>(); // String key - level of compartment, "0" - list of base compartments, which are not attached in any other compartment, "1" - compartments which parent compartment is in "0" list (base) etc.
        HashMap<String, Integer> compartmentNodeNumberMap = new HashMap<>(); // contains compartment node number
        HashMap<String, Integer[]> compartmentNodeSizeMap = new HashMap<>(); // contains sum of corresponding dimensions of all compartment nodes
        HashMap<String, List<Node>> compartmentNodeMap = new HashMap<>(); // contains Nodes of corresponding compartment

        int maxAttachment = 0;

        for( Node n : graph.nodeList )
        {
            List<Node> tempList = new ArrayList<>();
            if( Util.isCompartment(n)) // && !isEmptyComp(n)
            {
                if( n.getAttribute("compartmentName")==null || n.getAttribute("compartmentName").equals("root") )
                {
                    if( !compartmentMap.containsKey("0") )
                    {
                        tempList.add(n);
                        compartmentMap.put("0", tempList);
                    }
                    else
                    {
                        compartmentMap.get("0").add(n);
                    }
                    n.setAttribute("compartmentName", "root");
                }
                else
                {
                    int delimiterCount = n.getAttribute("compartmentName").length()
                            - n.getAttribute("compartmentName").replaceAll("\\.", "").length();
                    if( delimiterCount > maxAttachment )
                        maxAttachment = delimiterCount;
                    if( !compartmentMap.containsKey("" + delimiterCount) )
                    {
                        tempList.add(n);
                        compartmentMap.put("" + delimiterCount, tempList);
                    }
                    else
                    {
                        compartmentMap.get("" + delimiterCount).add(n);
                    }
                    String compName = n.getAttribute("compartmentName");
                    if (!compName.startsWith("root"))
                    n.setAttribute("compartmentName", "root."+n.getAttribute("compartmentName"));
                }
                //                n.height = 0;
                //                n.width = 0;
            }
            else
            {
            	String compName = "root";
            	if (n.getAttribute("compartmentName") != null)
                compName += "."+n.getAttribute("compartmentName");

                n.setAttribute("compartmentName", compName);
                if( !compartmentNodeNumberMap.containsKey(compName) )
                {
                    compartmentNodeNumberMap.put(compName, 1);
                    Integer[] dim = new Integer[2];
                    dim[0] = n.width;
                    dim[1] = n.height;
                    compartmentNodeSizeMap.put(compName, dim);
                    tempList.add(n);
                    compartmentNodeMap.put(compName, tempList);
                }
                else
                {
                    int tempInt = compartmentNodeNumberMap.get(compName) + 1;
                    compartmentNodeNumberMap.put(compName, tempInt);
                    Integer[] dim = compartmentNodeSizeMap.get(compName);
                    dim[0] += n.width;
                    dim[1] += n.height;
                    compartmentNodeSizeMap.put(compName, dim);
                    compartmentNodeMap.get(compName).add(n);
                }

            }
        }


        if( setCompSizesAutomatically )
        {
            defineCompartmentsSizes(maxAttachment, compartmentNodeNumberMap, compartmentNodeSizeMap);
        }
        setGrid(compartmentNodeNumberMap, compartmentNodeSizeMap, graph);
        setCompartments();

        makeWeightMatrix(compartmentNodeMap);

        for( int i = 0; i < compartmentMap.size(); i++ )
        {
            List<Node> compList = new ArrayList<>();

            for( int j = i; j >= 0; j-- )
            {
                compList.addAll(compartmentMap.get("" + j));
            }

            Graph g = createGraph(compList, null);
            if( i != 0 )
            {
                nonBaseLevelFirstLayout(g);
            }
            else
            {
                baseLevelFirstLayout(g);
            }

            setNodesFixed(compartmentMap.get("" + i), true);
            formatGrid(i);
            renewCompartmentPointMap();
        }

    }

    protected void clearRoots(Graph graph)
    {
    	for (Node n : graph.nodeList)
    	{
    		String compName = n.getAttribute("compartmentName");
    		if (compName!= null && compName.startsWith("root"))
    		{
    			if (compName.equals("root")) n.setAttribute("compartmentName", null);
    			else n.setAttribute("compartmentName", compName.replaceFirst("root.",""));
    		}
    	}
    }


    //Getters and setters
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
    public int getNe()
    {
        return ne;
    }
    public void setNe(int ne)
    {
        this.ne = ne;
    }
    public double getRc()
    {
        return rc;
    }
    public void setRc(double rc)
    {
        this.rc = rc;
    }
    public double getProbabilityThreshold()
    {
        return probabilityThreshold;
    }
    public void setProbabilityThreshold(double probabilityThreshold)
    {
        this.probabilityThreshold = probabilityThreshold;
    }
    public int getSaturationDist()
    {
        return saturationDist;
    }
    public void setSaturationDist(int saturationDist)
    {
        this.saturationDist = saturationDist;
    }
}
