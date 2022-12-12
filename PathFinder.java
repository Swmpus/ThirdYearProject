import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("serial")
public class PathFinder
{
	public static List<int[]> movementOptions = new ArrayList<int[]>() {{ 
		add(new int[] {  1,  0 }); add(new int[] { -1,  0 });
		add(new int[] {  0,  1 }); add(new int[] {  0, -1 });  
		add(new int[] {  1,  1 }); add(new int[] { -1, -1 });
		add(new int[] { -1,  1 }); add(new int[] {  1, -1 }); }};
	
	public static List<int[]> limitedMovementOptions = new ArrayList<int[]>() {{ 
		add(new int[] {  1,  0 }); add(new int[] { -1,  0 });
		add(new int[] {  0,  1 }); add(new int[] {  0, -1 }); }};
		
	public static List<int[]> limitedMovementOptionsWait = new ArrayList<int[]>() {{ 
		add(new int[] {  1,  0 }); add(new int[] { -1,  0 });
		add(new int[] {  0,  1 }); add(new int[] {  0, -1 }); 
		add(new int[] {  0,  0 }); }};
	
	public static double Euclidean(int[] a, int[] b)
	{
		return Math.sqrt(Math.pow(b[0] - a[0], 2) + Math.pow(b[1] - a[1], 2));
	}
	
	public static double Manhattan(int[] a, int[] b)
	{
		return Math.abs(b[0] - a[0]) + Math.abs(b[1] - a [1]);
	}

	public static Node ShopperPath(int[] end, Shopper shopper, Simulation sim, boolean ignoreRestockers)
	{
		if (ignoreRestockers) {
			return AStar.FindAStarNoRestockers(shopper.getPos(), end, sim);
		} else {
			return AStar.FindAStar(shopper.getPos(), end, sim);
		}
	}
	
	public static List<? extends Node> quickSortNodes(List<? extends Node> array, int low, int high)
	{
	    if (high - low <= 0) {
			return array;
	    } else {
	        int p = partition(array, low, high);

	        quickSortNodes(array, low, p - 1);
	        quickSortNodes(array, p + 1, high);
	        
		    return array;
	    }
	}
	
	private static int partition(List<? extends Node> array, int low, int high)
	{
	    double pivot = array.get(high).cost;
	    int i = (low - 1);

	    for (int j = low; j <= high- 1; j++) {
	        if (array.get(j).cost < pivot) {
	            i++;
	            array = swap(array, i, j);
	        }
	    }
	    array = swap(array, i + 1, high);
	    return (i + 1);
	}

	private static <E> List<E> swap(List<E> array, int i, int j)
	{
		E temp;
		
		temp = array.get(i);
		array.set(i, array.get(j));
		array.set(j, temp);
		
		return array;
	}

	public static AStarNode containsPos(List<AStarNode> list, int[] pos)
	{
		for (AStarNode node : list) {
			if (node.pos[0] == pos[0] && node.pos[1] == pos[1]) {
				return node;
			}
		}
		return null;
	}

	public static List<int[]> getFullPath(AStarNode node)
	{
		if (node == null) {
			return null;
		}
		ArrayList<int[]> temp = new ArrayList<int[]>();
		
		temp.add(node.pos);
		while (node.parent != null) {
			node = (AStarNode) node.parent;
			temp.add(node.pos);
		}
		return temp;
	}
	
	public static int[] getFirstStep(AStarNode node)
	{
		if (node == null) {
			return null;
		} else if (node.parent == null) {
			return node.pos;
		}
		while (node.parent.parent != null) {
			node = (AStarNode) node.parent;
		}
		return node.pos;
	}
}
