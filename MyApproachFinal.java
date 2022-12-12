import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

public class MyApproachFinal
{
	private List<HighLevelGridNode> grid;
	private HighLevelGridNode restockerSpawn;
	
	public MyApproachFinal(int[] restockerEntry)
	{
		initialiseHighLevelMap();
		restockerSpawn = getGridNodeFromPos(restockerEntry);
	}
	
	public Map<Restocker, int[]> findPaths(Map<Restocker, int[]> goals, Simulation sim, HAMPCostMap costing)
	{
		Map<Restocker, int[]> out = new HashMap<Restocker, int[]>();
		Map<Integer, List<int[]>> reservationTable = new HashMap<Integer, List<int[]>>();
		
		for (Entry<Restocker, int[]> goal : goals.entrySet()) {
			if (goal.getKey().InDanger) {
				ArrayList<int[]> path = new ArrayList<int[]>();
				for (int i = 0; i < 200; i++) {
					path.add(goal.getKey().pos);
				}
				for (int i = 0; i < 200; i++) { // Reserve path
					if (0 < path.size() - 1 - i) {
						List<int[]> temp = reservationTable.get(i) == null ? new ArrayList<int[]>() : reservationTable.get(i);
						temp.add(path.get(path.size() - 1 - i));
						reservationTable.put(i, temp);
					}
				}
				out.put(goal.getKey(), goal.getKey().pos);
				continue;
			}
			
			
			
			HighLevelGridNode RestockerNode = getGridNodeFromPos(goal.getKey().pos);
			HighLevelGridNode goalNode = getGridNodeFromPos(goal.getValue());
			List<HighLevelGridNode> highPath = new ArrayList<HighLevelGridNode>();
			List<int[]> lowPath = new ArrayList<int[]>();
			
			if (goalNode != RestockerNode && !RestockerNode.getNeighbours().contains(goalNode)) {
				highPath = RestockerNode != null ? highLevel(RestockerNode, goalNode, sim) : highLevel(restockerSpawn, getGridNodeFromPos(goal.getValue()), sim);

				int[] door = getDoor(highPath.get(highPath.size() - 2), highPath.get(highPath.size() - 3), goal.getKey());
				
				List<HighLevelGridNode> highGraph = generateGraph(goal.getKey());
				
				lowPath = lowLevel(highGraph, reservationTable, goal.getKey().pos, door, sim, costing);
				
			} else {
				lowPath = lowLevel(generateGraph(goal.getKey()), reservationTable, goal.getKey().pos, goal.getValue(), sim, costing);
			}
			if (lowPath == null) {
				ArrayList<int[]> path = new ArrayList<int[]>();
				for (int i = 0; i < 200; i++) {
					path.add(goal.getKey().pos);
				}
				for (int i = 0; i < 200; i++) { // Reserve path
					if (0 < path.size() - 1 - i) {
						List<int[]> temp = reservationTable.get(i) == null ? new ArrayList<int[]>() : reservationTable.get(i);
						temp.add(path.get(path.size() - 1 - i));
						reservationTable.put(i, temp);
					}
				}
				out.put(goal.getKey(), goal.getKey().pos);
				continue;
			} else {
				for (int i = 0; i < lowPath.size(); i++) { // Reserve path
					if (0 < lowPath.size() - 1 - i) {
						List<int[]> temp = reservationTable.get(i) == null ? new ArrayList<int[]>() : reservationTable.get(i);
						temp.add(lowPath.get(lowPath.size() - 1 - i));
						reservationTable.put(i, temp);
					}
				}
			}
			
			
			if (lowPath.size() < 2) {
				out.put(goal.getKey(), lowPath.get(0));
			} else {
				out.put(goal.getKey(), lowPath.get(lowPath.size() - 2));
			}
		}
		return out;
	}
	
	public static double calcHeuristic(HighLevelGridNode neighbour, HighLevelGridNode goal)
	{
		int count = 1;
		Set<HighLevelGridNode> allNeighbours = new HashSet<HighLevelGridNode>();
		allNeighbours.addAll(neighbour.getNeighbours());
		
		while (!allNeighbours.contains(goal)) {
			count += 1;
			List<HighLevelGridNode> temp = new ArrayList<HighLevelGridNode>();
			for (HighLevelGridNode node : allNeighbours) {
				temp.addAll(node.getNeighbours());
			}
			allNeighbours.addAll(temp);
		}
		return count;
	}
	
	private List<int[]> lowLevel(List<HighLevelGridNode> graph, Map<Integer, List<int[]>> reservationTable, int[] start, int[] end, Simulation sim, HAMPCostMap costing)
	{
		if (costing != null) {
			return PathFinder.getFullPath(AStar.AStarReservedMinimalCosting(start, end, reservationTable, graph, sim, costing));
		} else {
			return PathFinder.getFullPath(AStar.AStarReservedMinimal(start, end, reservationTable, graph, sim));
		}
	}

	private HighLevelGridNode getGridNodeFromPos(int[] pos)
	{
		for (HighLevelGridNode node : grid) {
			if (node.posWithin(pos)) {
				return node;
			}
		}
		return null;
	}

	private List<HighLevelGridNode> generateGraph(Restocker key)
	{
		List<HighLevelGridNode> out = new ArrayList<HighLevelGridNode>();
		for (int i = 0; i < grid.size(); i++) {
			if (grid.get(i).posWithin(key.pos)) {
				out.add(grid.get(i));
				out.addAll(grid.get(i).getNeighbours());
				return out;
			}
		}
		out.add(restockerSpawn);
		out.addAll(restockerSpawn.getNeighbours());
		return out;
	}
	
	private int[] getDoor(HighLevelGridNode from, HighLevelGridNode to, Restocker stocker)
	{
		return from.getDoor(to, stocker.pos);
	}
	
	private void initialiseHighLevelMap()
	{
		grid = new ArrayList<HighLevelGridNode>();
		// ROW 0
		grid.add(new HighLevelGridNode(1, 1, 6, 10));
		grid.add(new HighLevelGridNode(1, 12, 6, 9));
		grid.add(new HighLevelGridNode(1, 22, 6, 9));
		grid.add(new HighLevelGridNode(1, 32, 6, 9));
		grid.add(new HighLevelGridNode(1, 42, 6, 9));
		grid.add(new HighLevelGridNode(1, 52, 6, 9));
		grid.add(new HighLevelGridNode(1, 62, 6, 16));
		// ROW 1
		grid.add(new HighLevelGridNode(8, 1, 9, 5));
		grid.add(new HighLevelGridNode(8, 12, 9, 4));
		grid.add(new HighLevelGridNode(8, 22, 9, 4));
		grid.add(new HighLevelGridNode(8, 32, 9, 4));
		grid.add(new HighLevelGridNode(8, 42, 9, 4));
		grid.add(new HighLevelGridNode(8, 52, 9, 4));
		grid.add(new HighLevelGridNode(8, 62, 9, 16));
		// ROW 2
		grid.add(new HighLevelGridNode(18, 1, 9, 10));
		grid.add(new HighLevelGridNode(18, 12, 9, 9));
		grid.add(new HighLevelGridNode(18, 22, 9, 9));
		grid.add(new HighLevelGridNode(18, 32, 9, 9));
		grid.add(new HighLevelGridNode(18, 42, 9, 9));
		grid.add(new HighLevelGridNode(18, 52, 9, 9));
		grid.add(new HighLevelGridNode(18, 62, 9, 9));
		// ROW 3
		grid.add(new HighLevelGridNode(25, 1, 9, 5));
		grid.add(new HighLevelGridNode(25, 12, 9, 4));
		grid.add(new HighLevelGridNode(25, 22, 9, 4));
		grid.add(new HighLevelGridNode(25, 32, 9, 4));
		grid.add(new HighLevelGridNode(25, 42, 9, 4));
		grid.add(new HighLevelGridNode(25, 52, 9, 4));
		grid.add(new HighLevelGridNode(25, 62, 9, 9));
		// ROW 4
		grid.add(new HighLevelGridNode(35, 1, 9, 10));
		grid.add(new HighLevelGridNode(35, 12, 9, 9));
		grid.add(new HighLevelGridNode(35, 22, 9, 9));
		grid.add(new HighLevelGridNode(35, 32, 9, 9));
		grid.add(new HighLevelGridNode(35, 42, 9, 9));
		grid.add(new HighLevelGridNode(35, 52, 9, 9));
		grid.add(new HighLevelGridNode(35, 62, 9, 9));
		// ROW 5
		grid.add(new HighLevelGridNode(42, 1, 9, 5));
		grid.add(new HighLevelGridNode(42, 12, 9, 4));
		grid.add(new HighLevelGridNode(42, 22, 9, 4));
		grid.add(new HighLevelGridNode(42, 32, 9, 4));
		grid.add(new HighLevelGridNode(42, 42, 9, 4));
		grid.add(new HighLevelGridNode(42, 52, 9, 4));
		grid.add(new HighLevelGridNode(42, 62, 9, 18));
		// ROW 6
		grid.add(new HighLevelGridNode(52, 1, 6, 10));
		grid.add(new HighLevelGridNode(52, 12, 6, 9));
		grid.add(new HighLevelGridNode(52, 22, 6, 9));
		grid.add(new HighLevelGridNode(52, 32, 6, 9));
		grid.add(new HighLevelGridNode(52, 42, 6, 9));
		grid.add(new HighLevelGridNode(52, 52, 6, 9));
		grid.add(new HighLevelGridNode(52, 62, 10, 24));
		
		for (HighLevelGridNode node : grid) {
			node.init(grid);
		}
	}
	
	private List<HighLevelGridNode> highLevel(HighLevelGridNode Start, HighLevelGridNode goal, Simulation sim)
	{
		return getHighLevelPath(AStar.AStarHigh(Start, goal, sim));
	}
	
	private List<HighLevelGridNode> getHighLevelPath(AStarNodeHigh node)
	{
		List<HighLevelGridNode> out = new ArrayList<HighLevelGridNode>();
		
		while (node != null) {
			out.add(node.internalNode);
			node = (AStarNodeHigh) node.parent;
		}
		return out;
	}
}
