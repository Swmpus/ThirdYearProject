import java.util.List;
import java.util.Map;

public class CBSNode extends Node
{
	public Map<Restocker, List<int[]>> positions;
	public List<CBSConstraint> constraints;
	
	public CBSNode(List<CBSConstraint> constraints, int cost, Map<Restocker, List<int[]>> positions)
	{
		super(cost);
		this.positions = positions;
		this.constraints = constraints;
	}
	
	public CBSNode(CBSNode parent, List<CBSConstraint> constraints, int cost, Map<Restocker, List<int[]>> positions)
	{
		super(parent, cost);
		this.positions = positions;
		this.constraints = constraints;
	}
}
