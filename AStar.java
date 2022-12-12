import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class AStar
{
	@SuppressWarnings("unchecked")
	public static AStarNodeHigh AStarHigh(HighLevelGridNode start, HighLevelGridNode goal, Simulation sim)
	{
		List<AStarNodeHigh> closed = new ArrayList<AStarNodeHigh>();
		List<AStarNodeHigh> open = new ArrayList<AStarNodeHigh>();
		open.add(new AStarNodeHigh(start, null, 0, PathFinder.Manhattan(start.getCentre(), goal.getCentre())));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNodeHigh>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNodeHigh current = open.remove(0);
	        closed.add(current);
	        
	        if (current.internalNode == goal) {
	            return current;
	        }

	        for (HighLevelGridNode neighbour : current.internalNode.getNeighbours()) {
	        	double nextG = current.g + neighbour.calculateCost(sim);
	        	
	        	boolean replaced = false;
	        	for (AStarNodeHigh node : closed) {
	        		if (node.internalNode == neighbour) {
	        			replaced = true;
	        			break;
	        		}
	        	}
	        	if (replaced) {
	        		continue;
	        	}
	        	
	        	for (AStarNodeHigh node : open) {
	        		if (node.internalNode == neighbour) {
	        			if (nextG < node.g) {
	    	                open.remove(node);
	    	                open.add(new AStarNodeHigh(neighbour, current, nextG, nextG + MyApproachFinal.calcHeuristic(neighbour, goal)));
	    			        DataCompiler.AStarNodeGenerated();
	        			}
    	                replaced = true;
	        			break;
	        		}
	        	}
	        	if (!replaced) {
	                open.add(new AStarNodeHigh(neighbour, current, nextG, nextG + MyApproachFinal.calcHeuristic(neighbour, goal)));
			        DataCompiler.AStarNodeGenerated();
	        	}
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode AStarConstrained(int[] start, int[] goal, List<CBSConstraint> constraints, Simulation sim)
	{
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	if (!(move[0] == 0 && move[1] == 0) && !(nextPos[0] == goal[0] && nextPos[1] == goal[1]) &&
	        				(sim.restockerInDangerAt(nextPos) || !shop.isPassable(nextPos) || !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	
	        	boolean flag = false;
        		for (CBSConstraint constraint : constraints) { // Assumes that only constraints referring to the current agent have been passed in
        			if (constraint.time == current.getDepth() && constraint.pos[0] == nextPos[0] && constraint.pos[1] == nextPos[1]) {
        				flag = true;
        			}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	
	            double nextG = current.g + shop.movementCost(current.pos, move);
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (move[0] == 0 && move[1] == 0) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            } else if (other == null || nextG < other.g) {
	            	open.remove(other);
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}
	
	@SuppressWarnings("unchecked")
	public static AStarNode FindAStar(int[] start, int[] goal, Simulation sim)
	{
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }
	        
	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	if (!(nextPos[0] == goal[0] && nextPos[1] == goal[1]) && 
	        		(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
    				!shop.isPassable(nextPos) || sim.restockerInDangerAt(nextPos) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	
	            double nextG = current.g + shop.movementCost(current.pos, move);
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}
	
	@SuppressWarnings("unchecked")
	public static AStarNode FindAStarNoRestockers(int[] start, int[] goal, Simulation sim)
	{
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }
	        
	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	if (!(nextPos[0] == goal[0] && nextPos[1] == goal[1]) && 
	        		(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
    				!shop.isPassable(nextPos) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	
	            double nextG = current.g + shop.movementCost(current.pos, move);
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}
	
	@SuppressWarnings("unchecked")
	public static AStarNode FindAStarNoRestockersCosting(int[] start, int[] goal, Simulation sim, HAMPCostMap costing)
	{
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }
	        
	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	if (!(nextPos[0] == goal[0] && nextPos[1] == goal[1]) && 
	        		(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
    				!shop.isPassable(nextPos) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	
	            double nextG = current.g + shop.movementCost(current.pos, move);// + costing.getCost(nextPos);
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode FindAStarReserved(int[] start, int[] goal, Map<Integer, List<int[]>> reservationTable, Simulation sim, int windowSize)
	{
		List<int[]> restockerPos = sim.getRestockerLocations();
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        if (current.getDepth() - 1 == windowSize) {
	        	AStarNode out = AStar.FindAStarNoRestockers(start, goal, sim);
                open.add(new AStarNode(goal, current, out.g, out.g));
	        } else {
	        	for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
		        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
		        	if (!(move[0] == 0 && move[1] == 0 || nextPos[0] == goal[0] && nextPos[1] == goal[1]) &&
		        				(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
		        				!shop.isPassable(nextPos) || sim.restockerInDangerAt(nextPos) || !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null)) {
		        		continue;
		        	}
		        	
		        	boolean flag = false;
		        	if (reservationTable.get(current.getDepth()) != null) {
			        	for (int[] reserved : reservationTable.get(current.getDepth())) {
			        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
		        				flag = true;
			        		}
			        	}
			        	if (!flag && reservationTable.get(current.getDepth() - 1) != null) { // Stops restockers phasing through each other
			        		for (int[] restocker : restockerPos) {
			        			if (restocker[0] == current.pos[0] && restocker[1] == current.pos[1]) {
			        				for (int[] reserved : reservationTable.get(current.getDepth() - 1)) {
			    		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
			    	        				flag = true;
			    	        				break;
			    		        		}
			    		        	}
			        			}
			        		}
			        	}
		        	}
		        	if (flag) {
		        		continue;
		        	}
		        	
		            double nextG = current.g + shop.movementCost(current.pos, move);
		            AStarNode other = PathFinder.containsPos(open, nextPos);
		            if (move[0] == 0 && move[1] == 0) {
		            	open.remove(other);
		                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
				        DataCompiler.AStarNodeGenerated();
		            } else if (other == null || nextG < other.g) {
		                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
				        DataCompiler.AStarNodeGenerated();
		            }
		        }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode FindAStarReservedHeat(int[] start, int[] goal, Map<Integer, List<int[]>> reservationTable, Simulation sim)
	{ // Needs further testing
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	if (!(move[0] == 0 && move[1] == 0 || (nextPos[0] == goal[0] && nextPos[1] == goal[1])) &&
	        				(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
	        				!shop.isPassable(nextPos) || !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	
	        	boolean flag = false;
	        	if (reservationTable.get(current.getDepth()) != null) {
		        	for (int[] reserved : reservationTable.get(current.getDepth())) {
		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
	        				flag = true;
		        		}
		        	}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	
	        	if (PathFinder.Manhattan(nextPos, goal) < 0) {
		        	System.out.println(sim.getHeat(nextPos, HeatMap.AgentType.RESTOCKER));
	        	}
	            double nextG = current.g + shop.movementCost(current.pos, move);
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (move[0] == 0 && move[1] == 0) {
	            	open.remove(other);
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal) + sim.getHeat(nextPos, HeatMap.AgentType.RESTOCKER))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            } else if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal) + sim.getHeat(nextPos, HeatMap.AgentType.RESTOCKER))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode FindAStarReservedGraph(int[] start, int[] goal, Map<Integer, List<int[]>> reservationTable, Graph graph, Simulation sim)
	{
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
	    
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        GraphNode curGraphNode = graph.getAt(current.pos);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        for (Entry<GraphNode, Double> node : curGraphNode.neighbours) { // Configurable
	        	int[] nextPos = node.getKey().pos;
	        	if (!(nextPos[0] == goal[0] && nextPos[1] == goal[1]) && !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null) {
	        		continue;
	        	}
	        	
	        	boolean flag = false;
	        	if (reservationTable.get(current.getDepth()) != null) {
		        	for (int[] reserved : reservationTable.get(current.getDepth())) {
		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
	        				flag = true;
		        		}
		        	}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	
	            double nextG = current.g + node.getValue();
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            
	            if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode AStarReservedMinimal(int[] start, int[] goal, Map<Integer, List<int[]>> reservationTable, List<HighLevelGridNode> graph, Simulation sim)
	{
		List<int[]> restockerPos = sim.getRestockerLocations();
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	boolean flag = true;
	        	
	        	for (HighLevelGridNode node : graph) {
	        		if (node.posWithin(nextPos)) {
	        			flag = false;
	        			break;
	        		}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	if (!(move[0] == 0 && move[1] == 0 || nextPos[0] == goal[0] && nextPos[1] == goal[1]) &&
	        				(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
	        				!shop.isPassable(nextPos) || sim.restockerInDangerAt(nextPos) || !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	if (reservationTable.get(current.getDepth()) != null) {
		        	for (int[] reserved : reservationTable.get(current.getDepth())) {
		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
	        				flag = true;
	        				break;
		        		}
		        	}
		        	if (!flag && reservationTable.get(current.getDepth() - 1) != null) { // Stops restockers phasing through each other
		        		for (int[] restocker : restockerPos) {
		        			if (restocker[0] == current.pos[0] && restocker[1] == current.pos[1]) {
		        				for (int[] reserved : reservationTable.get(current.getDepth() - 1)) {
		    		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
		    	        				flag = true;
		    	        				break;
		    		        		}
		    		        	}
		        			}
		        		}
		        	}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	boolean contained = false;
	        	for (int[] option : FARInspired.getMovementOptionsAt(current.pos)) {
	        		if (option[0] == move[0] && option[1] == move[1]) {
	        			contained = true;
	        			break;
	        		}
	        	}
	            double nextG = current.g;
	        	if (contained) {
	        		nextG += 1;
	        	} else {
	        		nextG += 10;
	        	}
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (move[0] == 0 && move[1] == 0) {
	            	open.remove(other);
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Euclidean(nextPos, goal)));
			        DataCompiler.AStarNodeGenerated();
	            } else if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Euclidean(nextPos, goal)));
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}
	
	@SuppressWarnings("unchecked")
	public static AStarNode AStarReservedMinimalCosting(int[] start, int[] goal, Map<Integer, List<int[]>> reservationTable, List<HighLevelGridNode> graph, Simulation sim, HAMPCostMap costing)
	{
		List<int[]> restockerPos = sim.getRestockerLocations();
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	boolean flag = true;
	        	
	        	for (HighLevelGridNode node : graph) {
	        		if (node.posWithin(nextPos)) {
	        			flag = false;
	        			break;
	        		}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	if (!(move[0] == 0 && move[1] == 0 || nextPos[0] == goal[0] && nextPos[1] == goal[1]) &&
	        				(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
	        				!shop.isPassable(nextPos) || sim.restockerInDangerAt(nextPos) || !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	if (reservationTable.get(current.getDepth()) != null) {
		        	for (int[] reserved : reservationTable.get(current.getDepth())) {
		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
	        				flag = true;
	        				break;
		        		}
		        	}
		        	if (!flag && reservationTable.get(current.getDepth() - 1) != null) { // Stop restockers phasing through each other
		        		for (int[] restocker : restockerPos) {
		        			if (restocker[0] == current.pos[0] && restocker[1] == current.pos[1]) {
		        				for (int[] reserved : reservationTable.get(current.getDepth() - 1)) {
		    		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
		    	        				flag = true;
		    	        				break;
		    		        		}
		    		        	}
		        			}
		        		}
		        	}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	boolean contained = false;
	        	for (int[] option : FARInspired.getMovementOptionsAt(current.pos)) {
	        		if (option[0] == move[0] && option[1] == move[1]) {
	        			contained = true;
	        			break;
	        		}
	        	}
	            double nextG = current.g + costing.getCost(nextPos);
	        	if (contained) {
	        		nextG += 1;
	        	} else {
	        		nextG += 10;
	        	}
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (move[0] == 0 && move[1] == 0) {
	            	open.remove(other);
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Euclidean(nextPos, goal)));
			        DataCompiler.AStarNodeGenerated();
	            } else if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Euclidean(nextPos, goal)));
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode AStarConstrainedCosting(int[] start, int[] goal, List<CBSConstraint> constraints, Simulation sim, HAMPCostMap costing)
	{
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }

	        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
	        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
	        	if (!(move[0] == 0 && move[1] == 0 || nextPos[0] == goal[0] && nextPos[1] == goal[1]) &&
	        				(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
	        				!shop.isPassable(nextPos) || !sim.isPassable(nextPos, false)|| PathFinder.containsPos(closed, nextPos) != null)) {
	        		continue;
	        	}
	        	
	        	boolean flag = false;
        		for (CBSConstraint constraint : constraints) { // Assumes that only constraints referring to the current agent have been passed in
        			if (constraint.time == current.getDepth() && constraint.pos[0] == nextPos[0] && constraint.pos[1] == nextPos[1]) {
        				flag = true;
        			}
	        	}
	        	if (flag) {
	        		continue;
	        	}
	        	
	            double nextG = current.g + costing.getCost(nextPos);
	            AStarNode other = PathFinder.containsPos(open, nextPos);
	            if (move[0] == 0 && move[1] == 0) {
	            	open.remove(other);
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            } else if (other == null || nextG < other.g) {
	                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
			        DataCompiler.AStarNodeGenerated();
	            }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}

	@SuppressWarnings("unchecked")
	public static AStarNode FindAStarReservedCosting(int[] start, int[] goal, Map<Integer, List<int[]>> reservationTable, Simulation sim, HAMPCostMap costing, int windowSize)
	{
		List<int[]> restockerPos = sim.getRestockerLocations();
		Shop shop = Shop.getInstance();
		List<AStarNode> closed = new ArrayList<AStarNode>();
		List<AStarNode> open = new ArrayList<AStarNode>();
		open.add(new AStarNode(start, null, 0, PathFinder.Manhattan(start, goal)));
        DataCompiler.AStarNodeGenerated();
        
		while (!open.isEmpty()) {
			open = (List<AStarNode>) PathFinder.quickSortNodes(open, 0, open.size() - 1);
	        AStarNode current = open.remove(0);
	        closed.add(current);
	        
	        if (current.pos[0] == goal[0] && current.pos[1] == goal[1]) {
	            return current;
	        }
	        
	        if (current.getDepth() - 1 == windowSize) {
	        	AStarNode out = AStar.FindAStarNoRestockersCosting(start, goal, sim, costing);
                open.add(new AStarNode(goal, current, out.g, out.g));
	        } else {
		        for (int[] move : PathFinder.limitedMovementOptions) { // Configurable
		        	int[] nextPos = new int[] { current.pos[0] + move[0], current.pos[1] + move[1] };
		        	if (!(move[0] == 0 && move[1] == 0 || nextPos[0] == goal[0] && nextPos[1] == goal[1]) &&
		        				(nextPos[0] < 0 || nextPos[1] < 0 || nextPos[0] >= shop.Height || nextPos[1] >= shop.Width || 
		        				!shop.isPassable(nextPos) || sim.restockerInDangerAt(nextPos) || !sim.isPassable(nextPos, false) || PathFinder.containsPos(closed, nextPos) != null)) {
		        		continue;
		        	}
		        	
		        	boolean flag = false;
		        	if (reservationTable.get(current.getDepth()) != null) {
			        	for (int[] reserved : reservationTable.get(current.getDepth())) {
			        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
		        				flag = true;
			        		}
			        	}
			        	if (!flag && reservationTable.get(current.getDepth() - 1) != null) { // Stop restockers phasing through each other
							for (int[] restocker : restockerPos) {
			        			if (restocker[0] == current.pos[0] && restocker[1] == current.pos[1]) {
			        				for (int[] reserved : reservationTable.get(current.getDepth() - 1)) {
			    		        		if (reserved[0] == nextPos[0] && reserved[1] == nextPos[1]) {
			    	        				flag = true;
			    	        				break;
			    		        		}
			    		        	}
			        			}
			        		}
			        	}
		        	}
		        	if (flag) {
		        		continue;
		        	}
		        	
		            double nextG = current.g + costing.getCost(nextPos); // shop.movementCost(current.pos, move);
		            AStarNode other = PathFinder.containsPos(open, nextPos);
		            if (move[0] == 0 && move[1] == 0) {
		            	open.remove(other);
		                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
				        DataCompiler.AStarNodeGenerated();
		            } else if (other == null || nextG < other.g) {
		                open.add(new AStarNode(nextPos, current, nextG, nextG + PathFinder.Manhattan(nextPos, goal))); // Configurable
				        DataCompiler.AStarNodeGenerated();
		            }
		        }
	        }
	        DataCompiler.AStarNodeExpanded();
		}
		return null;
	}
}
