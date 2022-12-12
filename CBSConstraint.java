
public class CBSConstraint
{
	public int time;
	public Restocker agent;
	public int[] pos;
	
	public CBSConstraint(int time, Restocker agent, int[] pos)
	{
		this.time = time;
		this.agent = agent;
		this.pos = pos;
	}
	
	@Override
	public String toString()
	{
		return "Constraint at time " + time + ", and pos (" + pos[0] + ", " + pos[1] + ") for agent: " + agent.toString();
	}
}
