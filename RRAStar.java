import java.util.ArrayList;
import java.util.List;

public class RRAStar
{
	private List<AStarNode> open;
	private List<AStarNode> closed;
	
	public RRAStar(int[] start, int[] end)
	{
		AStarNode goal = new AStarNode(end, 0, PathFinder.Manhattan(end, start));
		open = new ArrayList<AStarNode>();
		closed = new ArrayList<AStarNode>();
		open.add(goal);
		resumeRRAStar(start);
	}
	
	public boolean resumeRRAStar(int[] N)
	{
		while (open.size() != 0) {
			PathFinder.quickSortNodes(open, 0, open.size() - 1);
			AStarNode current = open.remove(0);
			closed.add(current);
			if (current.pos[0] == N[0] && current.pos[1] == N[1]) {
				return true;
			}
			
			for (int[] successor : PathFinder.limitedMovementOptionsWait) {
				int[] nextpos = new int[] { current.pos[0] + successor[0], current.pos[1] + successor[1] };
				double nextG = current.g + Shop.getInstance().movementCost(current.pos, successor);
				double nextH = PathFinder.Manhattan(nextpos, N);
				AStarNode newNode = new AStarNode(nextpos, current, nextG, nextH);
				AStarNode other = PathFinder.containsPos(open, nextpos);
				if (other == null && PathFinder.containsPos(closed, nextpos) == null) {
					open.add(newNode);
				} else if (other != null && newNode.cost < other.cost) {
					open.remove(other);
					open.add(newNode);
				}
			}
		}
		return false;
	}
	
	public double abstractDist(int[] pos, int[] goal)
	{
		if (PathFinder.containsPos(closed, pos) != null) {
			return PathFinder.containsPos(closed, pos).g;
		} else if (resumeRRAStar(pos)) {
			return PathFinder.containsPos(closed, pos).g;
		} else {
			return Double.MAX_VALUE;
		}
	}
}
