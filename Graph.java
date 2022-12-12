import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

public class Graph
{
	public enum MovementType
	{
		Down,
		Up,
		Left,
		Right,
		Motorway,
		Other
	}
	
	private GraphNode[][] graph;
	
	private Map<GraphNode, Double> addIfPassable(Map<GraphNode, Double> map, Shop shop, Double cost, int[] x)
	{
		if (shop.isPassable(x)) {
			map.put(graph[x[0]][x[1]], cost);
			return map;
		} else {
			return map;
		}
	}
	
	public GraphNode getAt(int[] pos)
	{
		return graph[pos[0]][pos[1]];
	}
	
	public Graph(Shop shop, MovementType[][] labels)
	{
		int height = shop.Height;
		int width = shop.Width;
		
		graph = new GraphNode[height][width];
		
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				graph[i][j] = new GraphNode(new int[] { i, j });
			}
		}
		
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				Map<GraphNode, Double> neighbours = new HashMap<GraphNode, Double>();
				switch(labels[i][j]) {
					case Down:
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i + 1, j});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j + 1});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j - 1});
					    break;
					case Up:
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i - 1, j});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j + 1});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j - 1});
					    break;
					case Left:
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j - 1});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i + 1, j});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i - 1, j});
					    break;
					case Right:
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j + 1});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i + 1, j});
					  	addIfPassable(neighbours, shop, 1.0, new int[] {i - 1, j});
					    break;
					case Motorway:
						if (j % 2 == 1) {
						  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j + 1});
						} else {
						  	addIfPassable(neighbours, shop, 1.0, new int[] {i, j - 1});
						}
						if (i % 2 == 0) {
						  	addIfPassable(neighbours, shop, 1.0, new int[] {i + 1, j});
						} else {
						  	addIfPassable(neighbours, shop, 1.0, new int[] {i - 1, j});
						}
					    break;
					default:
						for (int[] option : PathFinder.limitedMovementOptions) {
						  	addIfPassable(neighbours, shop, 1.0, new int[] { option[0] + i, option[1] + j });
						}
				}
				for (Entry<GraphNode, Double> neighbour : neighbours.entrySet()) {
					graph[i][j].addNeighbour(neighbour);
				}
			}
		}
	}
}
