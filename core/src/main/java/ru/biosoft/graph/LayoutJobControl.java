package ru.biosoft.graph;

import ru.biosoft.jobcontrol.JobControl;

public interface LayoutJobControl extends JobControl
{  
   public void done(int operationsDone);

   public int getNumberOfEstimatedOperations();
   
   public void setNumberOfEstimatedOperations(int estimatedOperations);
   
   public void begin();
}
