import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Simulation implements Runnable
{
	private Map<Shopper, int[]> shoppers;
	private Map<Shopper, List<int[]>> shopperIdealPaths;
	private SystemController controller;
	private Shop shop;
	private int timestep;
	private int maxShoppers;
	private int maxTicks;
	private boolean draw;
	
	public Simulation(int restockerCount, int customerCount, int maxTicks, boolean draw, Algorithms algorithm)
	{
		this.maxShoppers = customerCount;
		this.shop = Shop.getInstance();
		this.draw = draw;
		this.timestep = 0;
		this.maxTicks = maxTicks;
		this.controller = new SystemController(restockerCount, shop.Width, shop.Height, algorithm);
		this.shoppers = new HashMap<Shopper, int[]>();
		this.shopperIdealPaths = new HashMap<Shopper, List<int[]>>();
	}

	@Override
	public void run() {
		while (timestep < maxTicks) {
			single();
			if (draw) {
				DisplayController.getInstance().drawAll();
			}
		}
		System.out.println("DONE");
	}

	public void single()
	{
		timestep += 1;
		DataCompiler.Tick();
		if (timestep % 4 == 0 && shoppers.size() < maxShoppers) {
			shoppers.put(new Shopper(new int[] {1, Shop.getInstance().Width - 1}), new int[] {0, 0});
		}
		timeStep();
	}

	private void timeStep()
	{
		DataCompiler.Pause = true;
		placeRestockers();
		shopperPathfindSingleStep();
		DataCompiler.Pause = false;
		
		controller.pathfindSingleStep(this);
		advanceShoppers();
		controller.advanceTimestep(this);
		
		RobotsNearEachother();
		MeasureStockLevel();
	}

	private void MeasureStockLevel()
	{
		DataCompiler.StockMeasured(Shop.getInstance().measureStock());
	}

	private void RobotsNearEachother()
	{
		for (int[] restocker : controller.getActiveRestockerLocations()) {
			boolean restockerNearby = false;
			boolean shopperNearby = false;
			
			for (int[] shopperPos : shoppers.values()) {
				if (PathFinder.Euclidean(restocker, shopperPos) < 15) {
					shopperNearby = true;
					break;
				}
			}
			for (int[] restockerPos : controller.getActiveRestockerLocations()) {
				double dist = PathFinder.Euclidean(restocker, restockerPos);
				if (dist < 3 && dist > 0.1) {
					restockerNearby = true;
					break;
				}
			}
			if (restockerNearby && shopperNearby) {
				DataCompiler.RobotsCloseTogether();
			}
		}
	}

	private void placeRestockers()
	{
		for (int[] spawn : shop.getRestockerSpawns()) {
			//System.out.println(spawn[0] + ", " + spawn[1]);
			if (isPassable(spawn, true)) {
				if (!controller.placeRestocker(spawn)) {
					return;
				}
			}
		}
	}

	private void shopperPathfindSingleStep()
	{
		for (Shopper shopper : shoppers.keySet()) {
			controller.addHeat(shopper.getPos(), HeatMap.AgentType.SHOPPER);
			AStarNode out = (AStarNode) PathFinder.ShopperPath(Shop.getInstance().getBuyingLocation(shopper.getNextItem()), shopper, this, false);
			
			if (!shopperIdealPaths.containsKey(shopper) || shopperIdealPaths.get(shopper) == null) {
				shopperIdealPaths.put(shopper, PathFinder.getFullPath((AStarNode) PathFinder.ShopperPath(Shop.getInstance().getBuyingLocation(shopper.getNextItem()), shopper, this, true)));
			} else if (!shopperIdealPaths.get(shopper).contains(shopper.getPos()) && !restockerTouching(shopper.getPos()) && !shopper.isDeviating()) {
				shopper.resetDeviate();
				shopperIdealPaths.remove(shopper);
			} else if (!shopperIdealPaths.get(shopper).contains(shopper.getPos()) && shopperIdealPaths.get(shopper).size() > 2 && restockerTouching(shopper.getPos())) {
				shopper.deviating();
			} else if (shopper.isDeviating() && shopperIdealPaths.get(shopper).contains(shopper.getPos())) {
				shopper.stoppedDeviating();
				shopperIdealPaths.remove(shopper);
			}
			shoppers.put(	shopper,
							PathFinder.getFirstStep(out));
		}
	}

	private boolean restockerTouching(int[] pos)
	{
		for (int[] restocker : controller.getActiveRestockerLocations()) {
			if (PathFinder.Euclidean(pos, restocker) < 1.5) {
				return true;
			}
		}
		return false;
	}

	private void advanceShoppers()
	{
		List<Shopper> marked = new ArrayList<Shopper>();
		for (Shopper shopper : shoppers.keySet()) {
			if (shoppers.get(shopper) == null) {
				if (shopper.isStuck()) {
					marked.add(shopper);
					shopperIdealPaths.remove(shopper);
					shopper.resetDeviate();
				}
			} else if (shopper.getPos()[0] == shoppers.get(shopper)[0] && 
				shopper.getPos()[1] == shoppers.get(shopper)[1] &&
				shopper.getPos()[0] == shop.getBuyingLocation(shopper.getNextItem())[0] && 
				shopper.getPos()[1] == shop.getBuyingLocation(shopper.getNextItem())[1]) {
				
				if (shop.isMarket(shop.getMarketLocation(shopper.getNextItem()))) {
					shopper.takeItem(shop.getMarket(shop.getMarketLocation(shopper.getNextItem())));
					shopperIdealPaths.remove(shopper);
				} else {
					marked.add(shopper);
				}
				if (shopper.isDeviating() && shopperIdealPaths.containsKey(shopper) && shopperIdealPaths.get(shopper).size() > 2) {
					shopper.stoppedDeviating();
					shopperIdealPaths.remove(shopper);
				} else {
					shopper.resetDeviate();
				}
			} else {
				if (shopper.getPos()[0] == shoppers.get(shopper)[0] && shopper.getPos()[1] == shoppers.get(shopper)[1]) {
					if (shopper.isStuck()) {
						marked.add(shopper);
						shopperIdealPaths.remove(shopper);
						shopper.resetDeviate();
					}
				}
				shopper.moveTo(shoppers.get(shopper));
			}
			shoppers.put(shopper, new int[] {0, 0});
		}
		for (Shopper mark : marked) {
			shoppers.remove(mark);
			shopperIdealPaths.remove(mark);
			mark.resetDeviate();
			mark.finishedPath();
		}
	}

	public List<int[]> getShopperLocations()
	{
		List<int[]> out = new ArrayList<int[]>();
		for (Shopper shopper : shoppers.keySet()) {
			out.add(shopper.getPos());
		}
		return out;
	}

	public boolean isPassable(int[] nextPos, boolean includeRestockers)
	{
		for (Shopper shopper : shoppers.keySet()) {
			if (shopper.getPos()[0] == nextPos[0] && shopper.getPos()[1] == nextPos[1]) {
				return false;
			}
		}
		
		if (includeRestockers) {
			for (int[] pos : controller.getActiveRestockerLocations()) {
				if (PathFinder.Manhattan(pos, nextPos) < 1) {
					if (pos[0] == nextPos[0] && pos[1] == nextPos[1]) {
						return false;
					}
				}
			}
		}
		
		return true;
	}

	public List<int[]> getRestockerLocations()
	{
		return controller.getActiveRestockerLocations();
	}

	public int getHeat(int[] pos, HeatMap.AgentType type) {
		return controller.getHeat(pos, type);
	}

	public boolean shopperNearby(int[] pos)
	{
		for (Shopper shopper : shoppers.keySet()) {
			if (PathFinder.Manhattan(shopper.getPos(), pos) < 4) {
				return true;
			}
		}
		return false;
	}

	public boolean restockerInDangerAt(int[] pos)
	{
		for (Restocker restocker : controller.getActiveRestockers()) {
			if (restocker.InDanger && restocker.pos[0] == pos[0] && restocker.pos[1] == pos[1]) {
				return true;
			}
		}
		return false;
	}
	
	public boolean SpaceOccupied(int[] pos)
	{
		for (Shopper shopper : shoppers.keySet()) {
			if (shopper.getPos()[0] == pos[0] && shopper.getPos()[1] == pos[1]) {
				return true;
			}
		}
		for (int[] poss : controller.getActiveRestockerLocations()) {
			if (poss[0] == pos[0] && poss[1] == pos[1]) {
				return true;
			}
		}
		return false;
	}
}
