import java.util.Random;
import java.util.Set;

public class RandomHelper
{
	private static Random random;
	
	private static void initRand()
	{
		if (random == null) {
			random = new Random();
		}
	}
	
	public static int generateRandIntInclusive(int min, int max)
	{
		initRand();
		return random.nextInt((max - min) + 1) + min;
	}
	
	public static String generateItemName(Set<String> disallowed)
	{
		initRand();
		String possibleChars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		String out = "";
		do {
			out = out + possibleChars.charAt(random.nextInt(possibleChars.length()));
		} while (disallowed.contains(out));
		return out;
	}
}
