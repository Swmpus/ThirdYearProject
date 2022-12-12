import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

interface ItemThresholdListener { void itemThresholdReached(int[] pos, int restockAmount); }

public class SystemController implements ItemThresholdListener
{
	private HeatMap heatMap;
	private List<Restocker> inactiveRestockers;
	private Map<Restocker, Item> waitingRestockers;
	private Map<Restocker, Item> activeRestockers;
	private List<Item> restockQueue;
	private Map<Restocker, int[]> nextSteps;
	private Shop shop;
	private FARInspired tester;
	private MyApproachFinal tester2;
	private Algorithms algorithm;

	public SystemController(int restockerCount, int sizeX, int sizeY, Algorithms algorithm)
	{
		this.algorithm = algorithm;
		this.heatMap = new HeatMap(sizeX, sizeY);
		this.waitingRestockers = new HashMap<Restocker, Item>();
		this.activeRestockers = new HashMap<Restocker, Item>();
		this.nextSteps = new HashMap<Restocker, int[]>();
		this.restockQueue = new ArrayList<Item>();
		
		this.inactiveRestockers = new ArrayList<Restocker>();
		for (int i = 0; i < restockerCount; i++) {
			this.inactiveRestockers.add(new Restocker());
		}
		this.shop = Shop.getInstance();
		this.shop.addListener(this);
		
		this.tester = new FARInspired();
		this.tester2 = new MyApproachFinal(new int[] { 56, 76 });
	}

	@Override
	public void itemThresholdReached(int[] pos, int restockAmount)
	{
		restockQueue.add(new Item(pos, restockAmount));
	}

	private void CBSPathfind(Simulation sim, HAMPCostMap costing)
	{
		Map<Restocker, List<int[]>> result = CBS.FindCBS(convert(), sim, costing);
		for (Entry<Restocker, List<int[]>> entry : result.entrySet()) {
			if (entry.getValue().size() >= 2) {
				nextSteps.put(entry.getKey(), entry.getValue().get(entry.getValue().size() - 2));
			} else {
				nextSteps.put(entry.getKey(), entry.getValue().get(0));
			}
		}
	}
	
	private void WHCASPathfind(Simulation sim, HAMPCostMap costing)
	{
		nextSteps = WHCAStar.findWHCAStar(convert(), sim, costing);
	}
	
	private void WHCAStarHeatMap(Simulation sim)
	{
		nextSteps = WHCAStarHeatMap.findPaths(convert(), sim);
	}
	
	private void FARInspired(Simulation sim)
	{ // Requires initialisation
		nextSteps = tester.findWHCAStar(convert(), sim);
	}

	private void MyApproachFinal(Simulation sim, HAMPCostMap costing)
	{
		nextSteps = tester2.findPaths(convert(), sim, costing);
		if (nextSteps == null) {
			nextSteps = new HashMap<Restocker, int[]>();
		}
	}
	
	public void pathfindSingleStep(Simulation sim)
	{
		for (Restocker agent : activeRestockers.keySet()) {
			if (sim.shopperNearby(agent.pos)) {
				agent.InDanger = true;
				DataCompiler.EmergencyStop();
			} else {
				agent.InDanger = false;
			}
		}
		
		switch (algorithm) {
			case CostingWHCAS:
				WHCASPathfind(sim, new HAMPCostMap(shop, sim));
				break;
			case CostingCBS:
				CBSPathfind(sim, new HAMPCostMap(shop, sim));
				break;
			case CostingMine:
				MyApproachFinal(sim, new HAMPCostMap(shop, sim));
				break;
			case CBS:
				CBSPathfind(sim, null);
				break;
			case WHCAS:
				WHCASPathfind(sim, null);
				break;
			case WHCAStarHeatMap:
				WHCAStarHeatMap(sim);
				break;
			case FARInspired:
				FARInspired(sim);
				break;
			case MyApproachFinal:
				MyApproachFinal(sim, null);
				break;
			default:
				System.out.println("Error Algorithm Doesn't Exist");
				break;
		}
	}

	private Map<Restocker, int[]> convert()
	{
		Map<Restocker, int[]> out = new HashMap<Restocker, int[]>();
		for (Entry<Restocker, Item> entry : activeRestockers.entrySet()) {
			if (entry.getValue() == null && entry.getKey().pos[0] == shop.getRestockerExit()[0] && entry.getKey().pos[1] == shop.getRestockerExit()[1]) {
				inactiveRestockers.add(entry.getKey());
				entry.getKey().returned();
				continue;
			} else if (entry.getValue() == null) {
				out.put(entry.getKey(), shop.getRestockerExit());
				continue;
			}
			
			int[] buyingLocation = shop.getBuyingLocation(shop.getMarket(entry.getValue().pos).getItem());
			
			if (entry.getKey().pos[0] == buyingLocation[0] && entry.getKey().pos[1] == buyingLocation[1]) {
				shop.restockMarket(entry.getValue().pos, entry.getValue().count);
				activeRestockers.put(entry.getKey(), null);
				out.put(entry.getKey(), shop.getRestockerExit());
			} else {
				out.put(entry.getKey(), buyingLocation);
			}
		}
		for (Restocker re : inactiveRestockers) {
			activeRestockers.remove(re);
		}
		return out;
	}
	
	public void advanceTimestep(Simulation sim)
	{
		for (Item item : restockQueue) {
			if (!inactiveRestockers.isEmpty()) {
				waitingRestockers.put(inactiveRestockers.remove(0), item);
			}
		}
		restockQueue.removeAll(waitingRestockers.values());
		
		for (Entry<Restocker, int[]> entry : nextSteps.entrySet()) {
			int[] directionVector = new int[] { entry.getValue()[0] - entry.getKey().pos[0], entry.getValue()[1] - entry.getKey().pos[1] };
			
			int[] dangerSpotClose = new int[] {entry.getValue()[0] + 2 * directionVector[0], entry.getValue()[1] + 2 * directionVector[1] };
			int[] dangerSpotA = new int[] {entry.getValue()[0] + 3 * directionVector[0], entry.getValue()[1] + 3 * directionVector[1] };
			int[] dangerSpotB = new int[] {entry.getValue()[0] + 4 * directionVector[0], entry.getValue()[1] + 4 * directionVector[1] };

			for (Restocker restocker : activeRestockers.keySet()) {
				if (restocker != entry.getKey() && dangerSpotClose[0] == entry.getKey().pos[0] && dangerSpotClose[1] == entry.getKey().pos[1]) {
					DataCompiler.RestockerCharge();
				}
			}
			
			boolean crossingProblem = false;
			for (int[] loc : sim.getShopperLocations()) {
				if ((dangerSpotA[0] == loc[0] && (dangerSpotA[1] == loc[1]) || (dangerSpotB[0] == loc[0] && (dangerSpotB[1] == loc[1])))) {
					DataCompiler.ShopperCharge();
				}
				if (entry.getKey().isCrossingAisle() && PathFinder.Euclidean(entry.getKey().pos, loc) < 10 && !crossingProblem) {
					crossingProblem = true;
				}
			}
			if (crossingProblem) {
				DataCompiler.AisleEndCrossover();
			}
			entry.getKey().moved();
			entry.getKey().pos = entry.getValue();
		}
		nextSteps.clear();
	}
	
	public boolean placeRestocker(int[] spawn)
	{
		if (waitingRestockers.isEmpty()) {
			return false;
		} else {
			Restocker temp = (Restocker) waitingRestockers.keySet().toArray()[0];
			temp.pos = spawn;
			activeRestockers.put(temp, waitingRestockers.get(waitingRestockers.keySet().toArray()[0]));
			waitingRestockers.remove((Restocker)waitingRestockers.keySet().toArray()[0]);
						
			return true;
		}
	}

	public List<int[]> getActiveRestockerLocations()
	{
		List<int[]> out = new ArrayList<int[]>();
		for (Restocker stocker : activeRestockers.keySet()) {
			out.add(stocker.pos);
		}
		return out;
	}

	public int getHeat(int[] pos, HeatMap.AgentType type)
	{
		return heatMap.getHeat(pos, type);
	}

	public void addHeat(int[] pos, HeatMap.AgentType shopper)
	{
		heatMap.addHeatAt(pos, shopper);
	}

	private class Item
	{
		public int[] pos;
		public int count;

		public Item(int[] pos, int count)
		{
			this.pos = pos;
			this.count = count;
		}
	}

	public Set<Restocker> getActiveRestockers()
	{
		return activeRestockers.keySet();
	}
}
