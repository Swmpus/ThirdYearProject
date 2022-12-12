
public class Node
{
	public Node parent;
	public double cost;
	
	public Node(double cost) { this.parent = null; this.cost = cost; }
	
	public Node(Node parent, double cost) { this.parent = parent; this.cost = cost; }

	public int getDepth()
	{
		int depth = 1;
		Node node = parent;
		while (node != null) {
			depth += 1;
			node = node.parent;
		}
		return depth;
	}
}
