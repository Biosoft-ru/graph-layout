package ru.biosoft.graph;

import java.awt.geom.Point2D;
import java.util.List;

public class ForceDirectedLayouter extends AbstractLayouter
{
    public static final int PLACEMENT_AS_IS = 0;

    public static final int PLACEMENT_RANDOM = 1;

    protected int initialPlacement = PLACEMENT_RANDOM;

    protected int edgeLength = 75;

    protected float repulsion = 1;

    protected int repulsionDistance = 5;

    public float gravity = 0.06f;

    public static final int TOP_TO_BOTTOM = 0;

    public static final int BOTTOM_TO_TOP = 1;

    public static final int LEFT_TO_RIGHT = 2;

    public static final int RIGHT_TO_LEFT = 3;

    protected int orientation = TOP_TO_BOTTOM;

    public static final int DISTANCE_MINIMAL = 0;

    public static final int DISTANCE_CENTER = 1;

    public static final int DISTANCE_DIAGONAL = 2;

    protected int distanceMethod = DISTANCE_MINIMAL;

    protected float attraction = 1;

    protected float magneticIntencity = 0;

    protected int iterationNumber = 300;

    protected float minTemperature = 0;

    protected float maxTemperature = 0;



    public ForceDirectedLayouter()
    {
        super();
    }

    public ForceDirectedLayouter(int edgeLength)
    {
        super();
        this.edgeLength = edgeLength;
    }


    @Override
	void layoutNodes(Graph graph, LayoutJobControl lJC)
    {
        if( initialPlacement == PLACEMENT_RANDOM )
            randomLayout(graph);

        initLayoutData(graph);

        for( int i = 0; i < iterationNumber; i++ )
        {
            float temp = graph.nodeCount() * 3 / ( 1 + (float)Math.exp(i / 8 - 5) ) + minTemperature;

            if( maxTemperature > 0 && temp > maxTemperature )
                temp = maxTemperature;

            relax(graph, temp);
            if( lJC != null )
            {
                lJC.done(++operationsDone);
                if( lJC.getStatus() == 4 || lJC.getStatus() == 5 )
                    i = iterationNumber;
            }
        }

        // postprocess nodes - move to zero
        float x = Float.MAX_VALUE;
        float y = Float.MAX_VALUE;

        for( Node node : graph.nodeList )
        {
            if( x(node) < x )
                x = x(node);

            if( y(node) < y )
                y = y(node);
        }

        for( Node node : graph.nodeList )
        {
            if( node.fixed )
                continue;
            node.x = Math.round(x(node) - x);
            node.y = Math.round(y(node) - y);
        }

        Util.adjustOrientations(graph);
        
        // clear edges as they can affect getBounds(). They will be layouted later during layoutEdges stage
        for( Edge edge : graph.getEdges() )
        	if(!edge.fixed)
        		edge.path = new Path();
    }
    @Override
	public void layoutEdges(Graph graph, LayoutJobControl lJC)
    {
        for( Edge edge : graph.edgeList )
        	if (!edge.fixed)
        		layoutPath(graph, edge, lJC);
    }

    private Layouter dpl = new DiagonalPathLayouter();
    public Layouter getPathLayouter()
    {
        return dpl;
    }
    public void setPathLayouter(Layouter val)
    {
        dpl = val;
    }

    @Override
	public void layoutPath(Graph graph, Edge edge, LayoutJobControl lJC)
    {
        // TODO: clean after serialization fix
        if( dpl == null )
            dpl = new DiagonalPathLayouter();
        dpl.layoutPath(graph, edge, lJC);
    }

    // //////////////////////////////////////////////////////////////////////////

    protected float x(Node node)
    {
        return ( (LayoutData)node.data ).x;
    }

    protected void x(Node node, float value)
    {
        ( (LayoutData)node.data ).x = value;
    }

    protected float y(Node node)
    {
        return ( (LayoutData)node.data ).y;
    }

    protected void y(Node node, float value)
    {
        ( (LayoutData)node.data ).y = value;
    }

    protected float dx(Node node)
    {
        return ( (LayoutData)node.data ).dx;
    }

    protected void dx(Node node, float value)
    {
        ( (LayoutData)node.data ).dx = value;
    }

    protected float dy(Node node)
    {
        return ( (LayoutData)node.data ).dy;
    }

    protected void dy(Node node, float value)
    {
        ( (LayoutData)node.data ).dy = value;
    }

    // //////////////////////////////////////////////////////////////////////////

    protected void randomLayout(Graph graph)
    {
        int len = edgeLength * ( (int)Math.sqrt(graph.nodeCount()) + 2 );

        for( Node node : graph.nodeList )
        {
            if( node.fixed )
                continue;
            node.x = (int) ( Math.random() * len );
            node.y = (int) ( Math.random() * len );
        }
    }

    protected void initLayoutData(Graph graph)
    {
        for( Node n : graph.nodeList )
            n.data = new LayoutData(n);
    }

    protected void relax(Graph graph, float temp)
    {
        // calc attractive (spring) and magnetic forces
        for( Edge edge : graph.edgeList )
        {
            Point2D.Float p = distance(edge.from, edge.to);
            float L = p.x * p.x + p.y * p.y;
            float l = (float)Math.sqrt(L);
            if( L < 0.01f )
                l = 0.01f;

            // calc attractive (spring) forces
            float fa = attraction * L / edgeLength;
            float dx = fa * p.x / l;
            float dy = fa * p.y / l;

            ( (LayoutData)edge.from.data ).dx += dx;
            ( (LayoutData)edge.from.data ).dy += dy;
            ( (LayoutData)edge.to.data ).dx -= dx;
            ( (LayoutData)edge.to.data ).dy -= dy;

            // calc magnetic field
            if( this.magneticIntencity != 0 && l > 1 )
            {
                float d;
                if( orientation == LEFT_TO_RIGHT || orientation == RIGHT_TO_LEFT )
                    d = p.y / l;
                else
                    d = p.x / l;

                d = Math.abs(d);
                if( orientation == BOTTOM_TO_TOP || orientation == RIGHT_TO_LEFT )
                    d = -d;

                float fm = d * l * magneticIntencity;

                if( orientation == LEFT_TO_RIGHT || orientation == RIGHT_TO_LEFT )
                {
                    ( (LayoutData)edge.from.data ).dx -= fm;
                    ( (LayoutData)edge.to.data ).dx += fm;
                }
                else
                {
                    ( (LayoutData)edge.from.data ).dy -= fm;
                    ( (LayoutData)edge.to.data ).dy += fm;
                }
            }
        }

        // calc repulsive forces
        for( int i = 1; i < graph.nodeCount(); i++ )
        {
            Node n1 = graph.nodeList.get(i);

            for( int j = 0; j < i; j++ )
            {
                Node n2 = graph.nodeList.get(j);

                Point2D.Float p = distance(n1, n2);
                float L = p.x * p.x + p.y * p.y;
                float l = (float)Math.sqrt(L);
                if( L < 0.01f )
                {
                    l = 0.01f;
                    p.x += l;
                    p.x += l;
                }

                else if( l < repulsionDistance * edgeLength )
                {
                    float fr = repulsion * ( edgeLength * edgeLength ) / l;
                    float dx = fr * p.x / l;
                    float dy = fr * p.y / l;

                    ( (LayoutData)n1.data ).dx -= dx;
                    ( (LayoutData)n1.data ).dy -= dy;
                    ( (LayoutData)n2.data ).dx += dx;
                    ( (LayoutData)n2.data ).dy += dy;
                }
            }
        }

        // calc gravity
        // first calculate graph barycentric
        float bcx = 0;
        float bcy = 0;
        for( Node node : graph.nodeList )
        {
            bcx += x(node) + node.width / 2f;
            bcy += y(node) + node.height / 2f;
        }
        bcx /= graph.nodeCount();
        bcy /= graph.nodeCount();

        // then calc the gravity force
        for( Node node : graph.nodeList )
        {

            float dx = bcx - x(node) - node.width / 2f;
            float dy = bcy - y(node) - node.height / 2f;
            float L = dx * dx + dy * dy;
            float l = (float)Math.sqrt(L);

            int degree = 5;
            List<Edge> edges_ = graph.getEdges(node);
            if( edges_ != null )
                degree = edges_.size();

            float fg = gravity * ( 1 + degree ) * l;
            dx = fg * dx / l;
            dy = fg * dy / l;

            ( (LayoutData)node.data ).dx += fg * dx / l;
            ( (LayoutData)node.data ).dy += fg * dy / l;
        }

        // define maximum delta that depends from temperature
        for( Node node : graph.nodeList )
        {
            if( node.fixed )
                continue;

            float dx = ( (LayoutData)node.data ).dx;
            float dy = ( (LayoutData)node.data ).dy;
            float force = (float)Math.sqrt(dx * dx + dy * dy);
            if( force < 0.001 )
                continue;

            float f = Math.min(force, temp) / force;

            if( horisontalMovementAllowed )
                x(node, x(node) + dx(node) * f);
            if( verticalMovementAllowed )
                y(node, y(node) + dy(node) * f);

            dx(node, 0);
            dy(node, 0);
        }
    }

    // //////////////////////////////////////////////////////////////////////////
    // Distance calculation issues
    //

    protected Point2D.Float distance(Node from, Node to)
    {
        if( distanceMethod == DISTANCE_MINIMAL )
            return distanceMin(from, to);

        if( distanceMethod == DISTANCE_DIAGONAL )
            return distanceDiagonal(from, to);

        return distanceBetweenCenters(from, to);
    }

    /** Calculate distance between centers of two nodes. */
    protected Point2D.Float distanceBetweenCenters(Node from, Node to)
    {
        return new Point2D.Float(x(to) - x(from) + ( to.width - from.width ) / 2f, y(to) - y(from) + ( to.height - from.height ) / 2f);
    }

    /**
     * Calculates minimum distance between two nodes (bounded by corresponding
     * rectangles).
     */
    protected Point2D.Float distanceMin(Node from, Node to)
    {
        float dx = 0;
        float dy = 0;

        if( x(from) + from.width < x(to) )
            dx = x(to) - x(from) - from.width;
        else if( x(to) + to.width < x(from) )
            dx = x(to) - x(from) + to.width;

        if( y(from) + from.height < y(to) )
            dy = y(to) - y(from) - from.height;
        else if( y(to) + to.height < y(from) )
            dy = y(to) - y(from) + to.height;

        return new Point2D.Float(dx, dy);
    }

    /**
     * Calculates minimum distance between two nodes (bounded by corresponding
     * rectangles) using diagonal line between two nodes centers.
     */
    protected Point2D.Float distanceDiagonal(Node from, Node to)
    {
        Point2D.Float d = distanceMin(from, to);
        float vx = x(to) - x(from) + ( to.width - from.width ) / 2f;
        float vy = y(to) - y(from) + ( to.height - from.height ) / 2f;

        if( d.x == 0 && d.y != 0 )
            return new Point2D.Float(vx * d.y / vy, d.y);

        if( d.y == 0 && d.x != 0 )
            return new Point2D.Float(d.x, d.y * d.x / vx);

        return d;
    }



    // //////////////////////////////////////////////////////////////////////////
    // Properties
    //

    @Override
	public LayouterInfo getInfo()
    {
        LayouterInfoSupport lis = new LayouterInfoSupport(true, true, false, false, false, false);
        return lis;
    }

    @Override
	public int estimate(Graph graph, int what)
    {
        int operations = 0;
    	if (graph.isConnected()) return this.iterationNumber;
    	else
    	{
            List<Graph> graphs = graph.split();
            for (Graph gr : graphs)
            {
                boolean notSimpleNodeFlag = true;
                for (Node n : gr.nodeList)
                {
                    String type = n.getAttribute("Type");
                    if (!Util.isCompartment(n) && (type == null || (!type.equals("Event") && !type.equals("Equation") && !type.equals("Function")))) notSimpleNodeFlag = false;
                    continue;
                }
                if (!notSimpleNodeFlag)operations += this.iterationNumber;
            }
            return operations;
    	}

    }

    public int getInitialPlacement()
    {
        return initialPlacement;
    }

    public void setInitialPlacement(int initialPlacement)
    {
        this.initialPlacement = initialPlacement;
    }

    protected boolean horisontalMovementAllowed = true;

    public boolean isHorisontalMovementAllowed()
    {
        return horisontalMovementAllowed;
    }

    public void setHorisontalMovementAllowed(boolean horisontalMovementAllowed)
    {
        this.horisontalMovementAllowed = horisontalMovementAllowed;
    }

    protected boolean verticalMovementAllowed = true;

    public boolean isVerticalMovementAllowed()
    {
        return verticalMovementAllowed;
    }

    public void setVerticalMovementAllowed(boolean verticalMovementAllowed)
    {
        this.verticalMovementAllowed = verticalMovementAllowed;
    }

    public int getDistanceMethod()
    {
        return distanceMethod;
    }

    public void setDistanceMethod(int distanceMethod)
    {
        this.distanceMethod = distanceMethod;
    }

    public int getEdgeLength()
    {
        return edgeLength;
    }

    public void setEdgeLength(int edgeLength)
    {
        this.edgeLength = edgeLength;
    }

    public float getAttraction()
    {
        return attraction;
    }

    public void setAttraction(float attraction)
    {
        this.attraction = attraction;
    }

    public float getRepulsion()
    {
        return repulsion;
    }

    public void setRepulsion(float repulsion)
    {
        this.repulsion = repulsion;
    }

    public int getRepulsionDistance()
    {
        return repulsionDistance;
    }

    public void setRepulsionDistance(int repulsionDistance)
    {
        this.repulsionDistance = repulsionDistance;
    }

    public float getGravity()
    {
        return gravity;
    }

    public void setGravity(float gravity)
    {
        this.gravity = gravity;
    }

    public int getOrientation()
    {
        return orientation;
    }

    public void setOrientation(int orientation)
    {
        this.orientation = orientation;
    }

    public float getMagneticIntencity()
    {
        return magneticIntencity;
    }

    public void setMagneticIntencity(float magneticIntencity)
    {
        this.magneticIntencity = magneticIntencity;
    }

    // //////////////////////////////////////////////////////////////////////////
    // Simulation annealing parameters
    //

    public int getIterationNumber()
    {
        return iterationNumber;
    }

    public void setIterationNumber(int iterationNumber)
    {
        this.iterationNumber = iterationNumber;
    }

    public float getMinTemperature()
    {
        return minTemperature;
    }

    public void setMinTemperature(float minTemperature)
    {
        this.minTemperature = minTemperature;
    }

    public float getMaxTemperature()
    {
        return maxTemperature;
    }

    public void setMaxTemperature(float maxTemperature)
    {
        this.maxTemperature = maxTemperature;
    }

    public class LayoutData
    {
        public float x;

        public float y;

        public float dx;

        public float dy;

        public LayoutData(Node n)
        {
            x = n.x;
            y = n.y;
        }
    }
}
