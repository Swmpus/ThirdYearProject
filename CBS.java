import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class CBS
{
	@SuppressWarnings("unchecked")
	public static Map<Restocker, List<int[]>> FindCBS(Map<Restocker, int[]> goals, Simulation sim, HAMPCostMap costing)
	{
		List<CBSNode> open = new ArrayList<CBSNode>();
		Map<Restocker, List<int[]>> innerResult = CBSInner(goals, new ArrayList<CBSConstraint>(), sim, costing);
		open.add(new CBSNode(	new LinkedList<CBSConstraint>(), // Zero constraints at root
								getInnerCost(innerResult), // Cost will always be > 0
								innerResult));
		
		while (!open.isEmpty()) {
			open = (List<CBSNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
			CBSNode current = open.remove(0);
			
			CBSConflict conflict = getFirstConflict(current.positions);
			if (conflict == null) {
				return current.positions;
			}
			for (Restocker agent : conflict.agents) {
				List<CBSConstraint> constraints = new ArrayList<CBSConstraint>();
				constraints.addAll(current.constraints);
				constraints.add(new CBSConstraint(conflict.time, agent, conflict.pos)); // This is not great for collisions involving more than 2 agents
				
				Map<Restocker, List<int[]>> innerResult2 = CBSInner(goals, constraints, sim, costing);
				if (innerResult2 != null) {
					open.add(new CBSNode(current, constraints, getInnerCost(innerResult2), innerResult2));
				}
			}
		}
		return null; // Failure
	}
	
	public static Map<Restocker, List<int[]>> CBSInner(Map<Restocker, int[]> goals, List<CBSConstraint> constraints, Simulation sim, HAMPCostMap costing)
	{
		Map<Restocker, List<int[]>> out = new HashMap<Restocker, List<int[]>>();
		for (Restocker agent : goals.keySet()) {
			List<CBSConstraint> agentConstraints = new ArrayList<CBSConstraint>();
			for (CBSConstraint constraint : constraints) {
				if (constraint.agent.pos[0] == agent.pos[0] && constraint.agent.pos[1] == agent.pos[1]) {
					agentConstraints.add(constraint);
				}
			}
			List<int[]> outInner;
			if (costing != null) {
				if (agent.InDanger) {
					outInner = PathFinder.getFullPath(AStar.AStarConstrainedCosting(agent.pos, agent.pos, agentConstraints, sim, costing));
				} else {
					outInner = PathFinder.getFullPath(AStar.AStarConstrainedCosting(agent.pos, goals.get(agent), agentConstraints, sim, costing));
				}
			} else {
				if (agent.InDanger) {
					outInner = PathFinder.getFullPath(AStar.AStarConstrained(agent.pos, agent.pos, agentConstraints, sim));
				} else {
					outInner = PathFinder.getFullPath(AStar.AStarConstrained(agent.pos, goals.get(agent), agentConstraints, sim));
				}
			}
			if (outInner == null) {
				return null;
			}
			out.put(agent, outInner);
		}
		return out;
	}
	
	private static CBSConflict getFirstConflict(Map<Restocker, List<int[]>> innerResult)
	{
		int longestLength = -1;
		for (Entry<Restocker, List<int[]>> entry : innerResult.entrySet()) {
			longestLength = entry.getValue().size() > longestLength ? entry.getValue().size() : longestLength;
		}
		
		for (int i = 0; i < longestLength; i++) {
			for (Entry<Restocker, List<int[]>> entryOuter : innerResult.entrySet()) {
				List<Restocker> temp = new ArrayList<Restocker>();
				for (Entry<Restocker, List<int[]>> entryInner : innerResult.entrySet()) {
					if (	entryOuter != entryInner &&
							entryInner.getValue().size() > i && entryOuter.getValue().size() > i &&
							entryOuter.getValue().get(entryOuter.getValue().size() - (i + 1))[0] == entryInner.getValue().get(entryInner.getValue().size() - (i + 1))[0] &&
							entryOuter.getValue().get(entryOuter.getValue().size() - (i + 1))[1] == entryInner.getValue().get(entryInner.getValue().size() - (i + 1))[1]) {
						if (!temp.contains(entryOuter.getKey())) {
							temp.add(entryOuter.getKey());
						}
						temp.add(entryInner.getKey());
					}
				}
				if (temp.size() > 0) {
					int[] outPos = entryOuter.getValue().get(entryOuter.getValue().size() - (i + 1));
					List<int[]> path = entryOuter.getValue();
					return new CBSConflict(i, outPos, temp);
				}
			}
		}
		return null; // No conflicts
	}
	
	private static int getInnerCost(Map<Restocker, List<int[]>> innerResult)
	{
		int cost = 0;
		for (List<int[]> path : innerResult.values()) {
			if (path == null) {
				return -1;
			}
			cost += path.size();
		}
		return cost;
	}
}
