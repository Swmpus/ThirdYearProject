
public class AStarNodeHigh extends Node
{
	public double g;
	public HighLevelGridNode internalNode;
	
	public AStarNodeHigh(HighLevelGridNode internalNode, double g, double f)
	{
		super(f);
		this.internalNode = internalNode;
		this.g = g;
	}
	
	public AStarNodeHigh(HighLevelGridNode internalNode, Node parent, double g, double f)
	{
		super(parent, f);
		this.internalNode = internalNode;
		this.g = g;
	}
}
