
public class HAMPCostMap
{ // https://homepages.laas.fr/nic/Papers/07ITRO_hri.pdf
	private double[][] hiddenZones;
	private double[][] costMerged;
	private static final double hiddenZonesWeight = 2;
	
	public HAMPCostMap(Shop shop, Simulation sim)
	{
		// visibility must be ignored because I do not model shoppers facing direction, it is assumed they are facing all directions
		
		// generate hidden positions close behind objects grid
		hiddenZones = calculateHiddenZones(shop, sim);
		
		// generate distance safety grid
		costMerged = calculateDistanceSafety(shop, sim);
		// generate visibility grid
		//visibility = calculateVisibility(shop, sim);
		// merge the grids
		// costMerged = MergeGrids();
	}
	
	private double[][] calculateDistanceSafety(Shop shop, Simulation sim)
	{
		double[][] out = new double[shop.Height][shop.Width];
		for (int i = 0; i < shop.Height; i++) {
			for (int j = 0; j < shop.Width; j++) {
				out[i][j] = 0;
			}
		}
		
		for (int[] shopper : sim.getShopperLocations()) {
			for (int i = shopper[0] - 5; i < shopper[0] + 6; i++) {
				for (int j = shopper[1] - 5; j < shopper[1] + 6; j++) {
					if (i < 0 || j < 0 || i >= shop.Height || j >= shop.Width) {
						continue;
					}
					out[i][j] += 10 / PathFinder.Manhattan(shopper, new int[] { i, j});
				}
			}
		}
		return out;
	}

	public double getCost(int[] pos)
	{
		if (hiddenZones[pos[0]][pos[1]] > 0) {
			return hiddenZonesWeight * hiddenZones[pos[0]][pos[1]];
		} else {
			return costMerged[pos[0]][pos[1]];
		}
	}
	
	private double[][] calculateHiddenZones(Shop shop, Simulation sim)
	{
		double[][] out = new double[shop.Height][shop.Width];
		for (int i = 0; i < shop.Height; i++) {
			for (int j = 0; j < shop.Width; j++) {
				out[i][j] = 0;
			}
		}
		
		for (int[] shopper : sim.getShopperLocations()) {
			out = testCircleLOS(shopper, out, shop);
		}
		return out;
	}
	
	private double[][] testCircleLOS(int[] centre, double[][] tempCostMap, Shop shop)
	{ // Draw circle using https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/
		int r = 10; // Length of aisle
	    int x = 0;
	    int y = r;
	    int d = 3 - 2 * r;
	    
	    tempCostMap = testOctants(centre, y, x, tempCostMap, shop); // Do octants at the same time so need to call testLOS eight times
	    while (y >= x)
	    {
	        x += 1;
	        
	        if (d > 0) {
	            y -= 1;
	            d = d + 4 * (x - y) + 10;
	        } else {
	            d = d + 4 * x + 6;
	        }
	        tempCostMap = testOctants(centre, y, x, tempCostMap, shop);
	    }
	    return tempCostMap;
	}
	
	private double[][] testOctants(int[] centre, int y, int x, double[][] tempCostMap, Shop shop)
	{
		tempCostMap = testLOS(centre, new int[] { centre[0] + y, centre[1] + x }, tempCostMap, shop);
		tempCostMap = testLOS(centre, new int[] { centre[0] + y, centre[1] - x }, tempCostMap, shop);
		tempCostMap = testLOS(centre, new int[] { centre[0] - y, centre[1] + x }, tempCostMap, shop);
		tempCostMap = testLOS(centre, new int[] { centre[0] - y, centre[1] - x }, tempCostMap, shop);

		tempCostMap = testLOS(centre, new int[] { centre[0] + x, centre[1] + y }, tempCostMap, shop);
		tempCostMap = testLOS(centre, new int[] { centre[0] + x, centre[1] - y }, tempCostMap, shop);
		tempCostMap = testLOS(centre, new int[] { centre[0] - x, centre[1] + y }, tempCostMap, shop);
		tempCostMap = testLOS(centre, new int[] { centre[0] - x, centre[1] - y }, tempCostMap, shop);
	    
	    return tempCostMap;
	}
	
	private double[][] testLOS(int[] from, int[] to, double[][] tempCostMap, Shop shop)
	{ // https://www.redblobgames.com/grids/line-drawing.html
	    int diagonalDistance = (int) Math.round(PathFinder.Manhattan(from, to));
	    int costCount = 0;
	    boolean costing = false;
	    
	    for (int i = 0; i <= diagonalDistance; i++) {
	        double percentage = diagonalDistance == 0 ? 0.0 : i / diagonalDistance;
	        int[] nextPos = roundPos(lerpPos(from, to, percentage));
	        
	        if (!shop.isPassable(nextPos) && !costing) {
	        	costing = true;
	        } else if (costing && costCount < 5) {
	        	costCount += 1;
	        	tempCostMap[nextPos[0]][nextPos[1]] = 10 / costCount; // maxCost / distance away
	        }
	    }
	    return tempCostMap;
	}

	private int[] roundPos(double[] pos)
	{
	    return new int[] { (int) Math.round(pos[0]), (int) Math.round(pos[1]) };
	}

	private double[] lerpPos(int[] from, int[] to, double percentage)
	{
	    return new double[] { lerp(from[0], to[0], percentage), lerp(from[1], to[1], percentage) };
	}

	private double lerp(int from, int to, double percentage)
	{
	    return from + percentage * (to - from);
	}
}
