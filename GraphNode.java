import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;

public class GraphNode
{
	public static int Connectivity = 4;
	public List<Entry<GraphNode, Double>> neighbours;
	public int[] pos;
	
	public GraphNode(int[] pos)
	{
		this.pos = pos;
		neighbours = new ArrayList<Entry<GraphNode, Double>>();
	}
	
	public boolean addNeighbour(Entry<GraphNode, Double> neighbour)
	{
		if (neighbours.size() == Connectivity) {
			return false;
		}
		neighbours.add(neighbour);
		return true;
	}
}
