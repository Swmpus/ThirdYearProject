import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class FARInspired
{	
	private static int WindowSize = 10;
	private static int Steps = 5;
	private static int internalTime = 0;
	private static Map<Restocker, List<int[]>> paths;
	private Graph graph;
	private static ExternalMovement[][] weightsExternal;
	
	private enum ExternalMovement
	{
		UpLeft,
		UpRight,
		DownLeft,
		DownRight,
		Any
	}
	
	public FARInspired()
	{
		Shop shop = Shop.getInstance();
		Graph.MovementType[][] weights = new Graph.MovementType[shop.Height][shop.Width];

		for (int i = 0; i < shop.Height; i++) {
			for (int j = 0; j < shop.Width; j++) {
				weights[i][j] = Graph.MovementType.Other;
			}
		}

		weights = generateHighways(weights);
		weights = generateAisles(weights);
		graph = new Graph(shop, weights);
	}

	public static List<int[]> getMovementOptionsAt(int[] pos)
	{
		if (weightsExternal == null) {
			Shop shop = Shop.getInstance();
			weightsExternal = new ExternalMovement[shop.Height][shop.Width];

			for (int i = 0; i < shop.Height; i++) {
				for (int j = 0; j < shop.Width; j++) {
					weightsExternal[i][j] = ExternalMovement.Any;
				}
			}
			generateHighwaysExternal();
			generateAislesExternal();
			generateSafetyExternal();
			
			return getMovementOptionsAt(pos);
		} else {
			List<int[]> options = new ArrayList<int[]>();
			if (weightsExternal[pos[0]][pos[1]] == ExternalMovement.UpLeft) {
				options.add(new int[] { -1,  0 });
				options.add(new int[] {  0, -1 });
			} else if (weightsExternal[pos[0]][pos[1]] == ExternalMovement.UpRight) {
				options.add(new int[] { -1,  0 });
				options.add(new int[] {  0,  1 });
			} else if (weightsExternal[pos[0]][pos[1]] == ExternalMovement.DownLeft) {
				options.add(new int[] {  1,  0 });
				options.add(new int[] {  0, -1 });
			} else if (weightsExternal[pos[0]][pos[1]] == ExternalMovement.DownRight) {
				options.add(new int[] {  1,  0 });
				options.add(new int[] {  0,  1 });
			} else if (weightsExternal[pos[0]][pos[1]] == ExternalMovement.Any) {
				return PathFinder.limitedMovementOptions;
			}
			return options;
		}
	}
	
	private static void generateSafetyExternal()
	{
		Shop shop = Shop.getInstance();
		for (int i = 0; i < shop.Height; i++) {
			weightsExternal[i][1] = ExternalMovement.Any;
			weightsExternal[i][shop.Width - 2] = ExternalMovement.Any;
			weightsExternal[i][shop.Width - 3] = ExternalMovement.Any;
			weightsExternal[i][shop.Width - 4] = ExternalMovement.Any;
		}
		for (int i = 0; i < shop.Width; i++) {
			weightsExternal[1][i] = ExternalMovement.Any;
			weightsExternal[shop.Height - 2][i] = ExternalMovement.Any;
		}
		
		for (int i = 0; i < shop.Height; i++) {
			for (int j = 0; j < 10; j++) {
				weightsExternal[i][j] = ExternalMovement.Any;
			}
		}
	}

	private void calculateNewWindow(Map<Restocker, int[]> goals, Simulation sim)
	{
		paths = new HashMap<Restocker, List<int[]>>();
		Map<Integer, List<int[]>> reservationTable = new HashMap<Integer, List<int[]>>();
		internalTime = Steps;
		
		for (Restocker agent : goals.keySet()) {
			if (agent.InDanger) {
				ArrayList<int[]> path = new ArrayList<int[]>();
				for (int i = 0; i < 2; i++) {
					path.add(agent.pos);
				}
				for (int i = 0; i < WindowSize; i++) { // Reserve path
					if (0 < path.size() - 1 - i) {
						List<int[]> temp = reservationTable.get(i) == null ? new ArrayList<int[]>() : reservationTable.get(i);
						temp.add(path.get(path.size() - 1 - i));
						reservationTable.put(i, temp);
					}
				}
				paths.put(agent, path);
			}
		}
		
		for (Restocker agent : goals.keySet()) {
			if (agent.InDanger) {
				continue;
			}
			List<int[]> path = PathFinder.getFullPath(AStar.FindAStarReservedGraph(agent.pos, goals.get(agent), reservationTable, graph, sim));
			
			if (path == null) {
				path = new ArrayList<int[]>();
				for (int i = 0; i < 2; i++) {
					path.add(agent.pos);
				}
			}
			for (int i = 0; i < WindowSize; i++) { // Reserve path
				if (0 < path.size() - 1 - i) {
					List<int[]> temp = reservationTable.get(i) == null ? new ArrayList<int[]>() : reservationTable.get(i);
					temp.add(path.get(path.size() - 1 - i));
					reservationTable.put(i, temp);
				}
			}
			paths.put(agent, path);
		}
	}
	
	public Map<Restocker, int[]> findWHCAStar(Map<Restocker, int[]> goals, Simulation sim)
	{
		Map<Restocker, int[]> out = new HashMap<Restocker, int[]>();
		if (internalTime == 0) {
			calculateNewWindow(goals, sim);
		}
		for (Restocker agent : goals.keySet()) {
			if (paths.get(agent) == null || paths.get(agent).size() == 1 || !paths.get(agent).get(0).equals(goals.get(agent)) || agent.InDanger) {
				calculateNewWindow(goals, sim);
				break;
			}
		}
		internalTime -= 1;
		for (Entry<Restocker, List<int[]>> entry : paths.entrySet()) {
			if (entry.getValue().size() < 3) {
				out.put(entry.getKey(), entry.getValue().get(0));
			} else {
				int[] outPos = entry.getValue().get(entry.getValue().size() - (1 + (Steps - internalTime)));
				out.put(entry.getKey(), outPos);
			}
		}
		return out;
	}
	
	private static void generateAislesExternal()
	{
		int columnOffset = 5;
		int rowOffset = 7;
		int aisleHeight = 10;
		int aisleWidth = 5;
		
		int aisleRows = 3;
		int aisleColumns = 7;
		
		for (int aisleOffsetVer = 0; aisleOffsetVer < aisleRows * (aisleHeight + rowOffset); aisleOffsetVer += aisleHeight + rowOffset) {
			for (int aisleOffsetHor = 0; aisleOffsetHor < aisleColumns * (aisleWidth + columnOffset); aisleOffsetHor += aisleWidth + columnOffset) {
				for (int i = 0; i < aisleHeight; i++) {
					for (int j = 0; j < aisleWidth; j++) {
						weightsExternal[8 + aisleOffsetVer + i][12 + aisleOffsetHor + j] = ExternalMovement.Any;
					}
				}
			}
		}
	}
	
	private static void generateHighwaysExternal()
	{
		for (int i = 1; i < 8; i++) {
			for (int j = 1; j < 61; j++) {
				if (i == 1 || j == 1 || j == 61) { //  || i == 7
					weightsExternal[i][j] = ExternalMovement.Any;
				} else if (i % 2 == 0) {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.UpRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.UpLeft;
					}
				} else {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.DownRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.DownLeft;
					}
				}
			}
		}
		for (int i = 18; i < 25; i++) {
			for (int j = 1; j < 61; j++) {
				if (j == 1 || j == 61) { // i == 18 || i == 25 || 
					weightsExternal[i][j] = ExternalMovement.Any;
				} else if (i % 2 == 0) {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.UpRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.UpLeft;
					}
				} else {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.DownRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.DownLeft;
					}
				}
			}
		}
		for (int i = 35; i < 42; i++) {
			for (int j = 1; j < 61; j++) {
				if (j == 1 || j == 61) { // i == 35 || i == 41 || 
					weightsExternal[i][j] = ExternalMovement.Any;
				} else if (i % 2 == 0) {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.UpRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.UpLeft;
					}
				} else {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.DownRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.DownLeft;
					}
				}
			}
		}
		for (int i = 52; i < 59; i++) {
			for (int j = 1; j < 61; j++) {
				if (j == 1 || j == 61) { // i == 52 || i == 59 || 
					weightsExternal[i][j] = ExternalMovement.Any;
				} else if (i % 2 == 0) {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.UpRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.UpLeft;
					}
				} else {
					if (j % 2 == 0) {
						weightsExternal[i][j] = ExternalMovement.DownRight;
					} else {
						weightsExternal[i][j] = ExternalMovement.DownLeft;
					}
				}
			}
		}
	}
	
	private Graph.MovementType[][] generateAisles(Graph.MovementType[][] weights)
	{ // Assumes vertical aisles
		int columnOffset = 5;
		int rowOffset = 7;
		int aisleHeight = 10;
		int aisleWidth = 5;
		
		int aisleRows = 3;
		int aisleColumns = 7;
		
		for (int aisleOffsetVer = 0; aisleOffsetVer < aisleRows * (aisleHeight + rowOffset); aisleOffsetVer += aisleHeight + rowOffset) {
			for (int aisleOffsetHor = 0; aisleOffsetHor < aisleColumns * (aisleWidth + columnOffset); aisleOffsetHor += aisleWidth + columnOffset) {
				for (int i = 0; i < aisleHeight; i++) {
					for (int j = 0; j < aisleWidth; j++) {
						if (j % 2 == 0) {
							weights[8 + aisleOffsetVer + i][12 + aisleOffsetHor + j] = Graph.MovementType.Down;
						} else {
							weights[8 + aisleOffsetVer + i][12 + aisleOffsetHor + j] = Graph.MovementType.Up;
						}
					}
				}
			}
		}
		return weights;
	}
	
	private Graph.MovementType[][] generateHighways(Graph.MovementType[][] weights)
	{
		for (int i = 1; i < 8; i++) {
			for (int j = 1; j < 61; j++) {
				if (i == 1 || i == 7 || j == 1 || j == 61) {
					weights[i][j] = Graph.MovementType.Other;
				} else {
					weights[i][j] = Graph.MovementType.Motorway;
				}
			}
		}
		for (int i = 18; i < 25; i++) {
			for (int j = 1; j < 61; j++) {
				if (i == 18 || i == 25 || j == 1 || j == 61) {
					weights[i][j] = Graph.MovementType.Other;
				} else {
					weights[i][j] = Graph.MovementType.Motorway;
				}
			}
		}
		for (int i = 35; i < 42; i++) {
			for (int j = 1; j < 61; j++) {
				if (i == 35 || i == 41 || j == 1 || j == 61) {
					weights[i][j] = Graph.MovementType.Other;
				} else {
					weights[i][j] = Graph.MovementType.Motorway;
				}
			}
		}
		for (int i = 52; i < 59; i++) {
			for (int j = 1; j < 61; j++) {
				if (i == 52 || i == 59 || j == 1 || j == 61) {
					weights[i][j] = Graph.MovementType.Other;
				} else {
					weights[i][j] = Graph.MovementType.Motorway;
				}
			}
		}
		return weights;
	}
}
