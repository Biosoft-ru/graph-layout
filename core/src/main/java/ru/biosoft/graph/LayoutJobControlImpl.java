package ru.biosoft.graph;

import ru.biosoft.jobcontrol.AbstractJobControl;
import ru.biosoft.jobcontrol.JobControlException;

public class LayoutJobControlImpl extends AbstractJobControl implements LayoutJobControl
{
    private int numberOfEstimatedOperations;
    private double progressStep = 0;

    public LayoutJobControlImpl(int numberOfEstimatedOperations)
    {
        super( null );
        this.numberOfEstimatedOperations = numberOfEstimatedOperations;
        progressStep = 100d / numberOfEstimatedOperations;
    }

    public void done(int operationsDone)
    {
        if( progressStep * operationsDone < 100 )
        {
            this.setPreparedness( (int) ( progressStep * operationsDone + 0.5 ) );
            this.fireValueChanged();
        }
        else
        {
            this.setPreparedness( 100 );
            this.fireValueChanged();
        }
    }

    public boolean isTerminated()
    {
        return isTerminated;
    }

    public int getNumberOfEstimatedOperations()
    {
        return numberOfEstimatedOperations;
    }

    public void terminate()
    {
        super.terminate();
        isTerminated = true;
    }

    public void setNumberOfEstimatedOperations(int estimatedOperations)
    {
        this.numberOfEstimatedOperations = estimatedOperations;
    }

    @Override
    protected void doRun() throws JobControlException
    {
    }

}
