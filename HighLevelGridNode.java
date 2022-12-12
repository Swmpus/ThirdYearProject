import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class HighLevelGridNode
{
	private int xOffset;
	private int yOffset;
	private int width;
	private int height;
	private Map<HighLevelGridNode, List<int[]>> doors;
	
	public HighLevelGridNode(int yOffset, int xOffset, int height, int width)
	{
		this.xOffset = xOffset;
		this.yOffset = yOffset;
		this.width = width;
		this.height = height;
		doors = new HashMap<HighLevelGridNode, List<int[]>>();
	}
	
	public boolean posWithin(int[] pos)
	{
		return pos[0] >= yOffset && pos[0] <= yOffset + height && pos[1] >= xOffset && pos[1] <= xOffset + width;
	}
	
	public void init(List<HighLevelGridNode> grid)
	{
		List<int[]> tempDoorsUp = new ArrayList<int[]>();
		List<int[]> tempDoorsDown = new ArrayList<int[]>();
		List<int[]> tempDoorsLeft = new ArrayList<int[]>();
		List<int[]> tempDoorsRight = new ArrayList<int[]>();
		int[] fullAbove = new int[] { getCentre()[0] - (height / 2) - 2, getCentre()[1] };
		int[] fullBelow = new int[] { getCentre()[0] + (height / 2) + 2, getCentre()[1] };
		int[] fullLeft = new int[] { getCentre()[0], getCentre()[1] - (width / 2) - 2 };
		int[] fullRight = new int[] { getCentre()[0], getCentre()[1] + (width / 2) + 2 };
		
		// Above / Below
		for (int x = xOffset; x <= xOffset + width; x++) {
			int[] posAbove = new int[] { yOffset - 1, x };
			int[] posTop = new int[] { yOffset, x };
			int[] posBelow = new int[] { yOffset + height, x };
			int[] posBot = new int[] { yOffset + height - 1, x };
			
			if (Shop.getInstance().isPassable(posAbove) && Shop.getInstance().isPassable(posTop)) {
				tempDoorsUp.add(posTop);
			}
			if (Shop.getInstance().isPassable(posBelow) && Shop.getInstance().isPassable(posBot)) {
				tempDoorsDown.add(posBot);
			}
		}
		
		// Left / Right
		for (int y = yOffset; y <= yOffset + height; y++) {
			int[] posLeftNext = new int[] { y, xOffset - 1 };
			int[] posLeft = new int[] { y, xOffset };
			int[] posRightNext = new int[] { y, xOffset + width };
			int[] posRight = new int[] { y, xOffset + width - 1 };
			
			if (Shop.getInstance().isPassable(posLeftNext) && Shop.getInstance().isPassable(posLeft)) {
				tempDoorsLeft.add(posLeft);
			}
			if (Shop.getInstance().isPassable(posRightNext) && Shop.getInstance().isPassable(posRight)) {
				tempDoorsRight.add(posRight);
			}
		}
		
		for (HighLevelGridNode node : grid) {
			if (node == this) {
				continue;
			}
			if (tempDoorsUp.size() > 0 && node.posWithin(fullAbove)) {
				doors.put(node, tempDoorsUp);
			} else if (tempDoorsDown.size() > 0 && node.posWithin(fullBelow)) {
				doors.put(node, tempDoorsDown);
			} else if (tempDoorsLeft.size() > 0 && node.posWithin(fullLeft)) {
				doors.put(node, tempDoorsLeft);
			} else if (tempDoorsRight.size() > 0 && node.posWithin(fullRight)) {
				doors.put(node, tempDoorsRight);
			}
		}
	}
	
	public List<HighLevelGridNode> getNeighbours()
	{
		List<HighLevelGridNode> out = new ArrayList<HighLevelGridNode>();
		for (HighLevelGridNode node : doors.keySet()) {
			out.add(node);
		}
		return out;
	}

	public int[] getDoor(HighLevelGridNode to, int[] RestockerPos)
	{
		double MinDist = Double.MAX_VALUE;
		int[] outDoor = new int[] {0, 0};
		
		for (int[] door : doors.get(to)) {
			double dist = PathFinder.Euclidean(RestockerPos, door);
			if (dist < MinDist) {
				MinDist = dist;
				outDoor = door;
			}
		}
		return outDoor;
	}

	public int[] getCentre()
	{
		return new int[] { yOffset + (height / 2), xOffset + (width / 2) };
	}

	public int[] getPos()
	{
		return new int[] { yOffset, xOffset };
	}

	public double calculateCost(Simulation sim)
	{
		int cost = 0;
		
		for (int y = yOffset; y < yOffset + height; y++) {
			for (int x = xOffset; x < xOffset + width; x++) {
				if (sim.isPassable(new int[] { y, x }, false)) { // Only count shoppers
					cost += 2;
				}
			}
		}
		return cost;
	}
}
