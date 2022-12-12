import java.util.List;

public class CBSConflict
{
	public int time;
	public int[] pos;
	public List<Restocker> agents;
	
	public CBSConflict(int time, int[] pos, List<Restocker> agents)
	{
		this.time = time;
		this.pos = pos;
		this.agents = agents;
	}
}
