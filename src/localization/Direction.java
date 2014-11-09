package localization;

/******
 * Enum representing the current direction
 * 
 * The value of the enum represents the number of left turns from up
 * 
 * @author Scott Coooper
 */
public enum Direction {
	// 4 possible directions
	
	/** UP relative to the start */
	UP(0), 
	
	/** LEFT relative to the start */
	LEFT(1), 
	
	/** RIGHT relative to the start */
	RIGHT(3), 
	
	/** DOWN relative to the start */
	DOWN(2);	
	public final int v;					// # of left turns from up
	private Direction(int i){ v = i; }	// Direction constructor
	
	/****
	 * Return the value of this Direction as a cardinal direction (i.e. N, S, E, W)
	 * 
	 * UP=N<br>DOWN=S<br>LEFT=W<br>RIGHT=E<br>
	 * @return A string represnting this direction as a cardinal directoin
	 */
	public String asCardinal(){
		switch(this){
		case DOWN: 	return "S";
		case LEFT: 	return "W";
		case RIGHT: return "E";
		case UP: 	return "N";
		default: 	throw new RuntimeException("Shouldn't happen");
		}
	}
	
	@Override
	/*****
	 * Get the value of this direction as a single character U, D, L, R
	 * 
	 * @return A string represnting this direction
	 */
	public String toString(){
		switch(this){
		case DOWN: 	return "D";
		case LEFT: 	return "L";
		case RIGHT: return "R";
		case UP: 	return "U";
		default: 	throw new RuntimeException("Shouldn't happen");
		}
	}
}