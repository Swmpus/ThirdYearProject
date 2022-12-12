
public class AStarNode extends Node
{
	public double g;
	public int[] pos;
	
	public AStarNode(int[] pos, double g, double f)
	{
		super(f);
		this.pos = pos;
		this.g = g;
	}
	
	public AStarNode(int[] pos, Node parent, double g, double f)
	{
		super(parent, f);
		this.pos = pos;
		this.g = g;
	}
}
