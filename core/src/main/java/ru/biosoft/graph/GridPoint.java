package ru.biosoft.graph;

public class GridPoint 
{
    public String compartmentName;
    public int x;
    public int y;

    public GridPoint(int x, int y)
    {
       this.x = x;
       this.y = y;
       compartmentName = null;
    }
    
    public GridPoint(int x, int y, String compartmentName)
    {
       this.x = x;
       this.y = y;
       this.compartmentName = compartmentName;
   }
   
}
