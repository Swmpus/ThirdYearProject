
public class Market extends Square
{
	private int[] pos;
	private int itemCount;
	private int threshold;
	private int overStockAmount;
	private boolean restockersNotified;
	private String itemName;
	
	public Market(int itemcount, int threshold, int overStockAmount, String itemName, int[] pos)
	{
		super(false);
		this.itemName = itemName;
		this.pos = pos;
		this.itemCount = itemcount;
		this.threshold = threshold;
		this.overStockAmount = overStockAmount;
		restockersNotified = false;
	}
	
	public int getRestockLoad()
	{
		return threshold - itemCount + overStockAmount;
	}
	
	public boolean takeItem(int count)
	{
		if (itemCount == 0) {
			return false;
		}
		itemCount -= count;
		if (!restockersNotified && itemCount <= threshold) {
			ItemBelowThreshold();
			restockersNotified = true;
		}
		return true;
	}

	public void addItem(int count)
	{
		itemCount += count;
		if (itemCount > threshold) {
			restockersNotified = false;
		} else {
			ItemBelowThreshold();
			restockersNotified = true;
		}
	}
	
	public boolean isBelowThreshold()
	{
		return itemCount <= threshold;
	}
	
	public void ItemBelowThreshold()
	{
		Shop.getInstance().ItemBelowThreshold(pos);
	}

	public String getItem()
	{
		return itemName;
	}

	public double getStock()
	{
		return itemCount;
	}
}
