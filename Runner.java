import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Runner
{
	public static void main(String[] args)
	{
		List<Algorithms> temp = new ArrayList<Algorithms>();
		temp.add(Algorithms.MyApproachFinal);
		temp.add(Algorithms.CostingMine);
		temp.add(Algorithms.WHCAS);
		temp.add(Algorithms.CostingWHCAS);
		
		for (Algorithms algorithm : temp) { // Algorithms.values()
			boolean failed = true;
			while (failed) {
				System.out.println("Running " + algorithm);
				DataCompiler.ReInit();
				Simulation sim = new Simulation(30, 20, 1000, false, algorithm); // BIG 30-20 : MEDIUM 20-13 : SMALL 10-5
				try {
					sim.run();
					failed = false;
				} catch(Exception ex) { }
			}
			try {
				DataCompiler.DumpStats(algorithm.toString());
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}
