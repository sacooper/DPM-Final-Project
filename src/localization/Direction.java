package localization;

/******
 * Enum representing the current direction
 * 
 * The value of the enum represents the number of left turns from up
 */
public enum Direction {
	UP(0), LEFT(1), RIGHT(3), DOWN(2);	// 4 possible directions
	public final int v;					// # of left turns from up
	private Direction(int i){ v = i; }	// Direction constructor
	
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