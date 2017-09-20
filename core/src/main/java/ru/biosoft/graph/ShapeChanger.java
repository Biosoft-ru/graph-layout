package ru.biosoft.graph;

import java.awt.Dimension;
import java.util.HashMap;
import java.util.Map;

public class ShapeChanger
{
    private Map<String, Dimension> sizes;

    public ShapeChanger()
    {
        this.sizes = new HashMap<>();
    }

    public void setSize(String orientation, Dimension dim)
    {
        sizes.put(orientation, dim);
    }

    public void changeShape(Node node)
    {
        String orientation = node.getAttribute("orientation");
        Dimension size = sizes.get(orientation);
        if (size == null)
            return;
        
        if (size.width == node.width && size.height == node.height)
            return;
        
        node.x = node.x + (node.width - size.width ) /2;
        node.y = node.y + (node.height - size.height ) /2;
                
        node.width = size.width;
        node.height = size.height;
    }
}
