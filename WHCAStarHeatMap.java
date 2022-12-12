import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class WHCAStarHeatMap
{
	private static int WindowSize = 10;
	private static int Steps = 5;
	private static int internalTime = 0;
	private static Map<Restocker, List<int[]>> paths;
	
	private static void calculateNewWindow(Map<Restocker, int[]> goals, Simulation sim)
	{
		paths = new HashMap<Restocker, List<int[]>>();
		Map<Integer, List<int[]>> reservationTable = new HashMap<Integer, List<int[]>>();
		internalTime = Steps;
		
		for (Restocker agent : goals.keySet()) {
			List<int[]> path = PathFinder.getFullPath(AStar.FindAStarReservedHeat(agent.pos, goals.get(agent), reservationTable, sim));
			
			for (int i = 0; i < WindowSize; i++) {
				if (0 < path.size() - 1 - i) {
					List<int[]> temp = reservationTable.get(i) == null ? new ArrayList<int[]>() : reservationTable.get(i);
					temp.add(path.get(path.size() - 1 - i));
					reservationTable.put(i, temp);
				}
			}
			paths.put(agent, path);
		}
	}
	
	public static Map<Restocker, int[]> findPaths(Map<Restocker, int[]> goals, Simulation sim)
	{ // Currently WHCA*
		Map<Restocker, int[]> out = new HashMap<Restocker, int[]>();
		if (internalTime == 0) {
			calculateNewWindow(goals, sim);
		}
		for (Restocker agent : goals.keySet()) {
			if (paths.get(agent) == null || paths.get(agent).size() == 1 || !paths.get(agent).get(0).equals(goals.get(agent))) {
				calculateNewWindow(goals, sim);
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
}
