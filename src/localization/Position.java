package localization;
/********
 * Class representing a position.
 * 
 * Information include X, Y, cardinal direction,
 * and whether it is blocked. All information 
 * is absolute, relative to the origin (0,0) 
 * with UP = NORTH = positive Y direction
 * <br><br>
 * X and Y coordinates are stored as bytes to reduce
 * memory necessary, thus this class should be modified
 * in the event that (x or y) < -127 or (x or y) > 128 
 * 
 * @author Scott Cooper
 * @since v1
 * @see Localizer
 */
public class Position {
	private byte x, y;			// X and Y coordinates
	private Direction dir; 		// UP = North, DOWN = South, etc.
	private boolean isBlocked;	// Whether we are blocked

	/****
	 * Create a new absolute position with the following paramaters
	 * 
	 * @param x	X coordinate in current system
	 * @param y	Y coordinate in current system
	 * @param dir Direction relative to start
	 * @param blocked Whether this position is blocked
	 */
	public Position(byte x, byte y, Direction dir, boolean blocked) {
		this.x = x;
		this.y = y;
		this.dir = dir;
		this.isBlocked = blocked;}

	/*****
	 * Alternate constructor to all passing in paramaters as integers
	 * @param x X coordinate in current system
	 * @param y Y coordinate in current system
	 * @param dir Direction relative to start
	 * @param isBlocked Whether this position is blocked
	 */
	public Position(int x, int y, Direction dir, boolean isBlocked) {
		this((byte)x, (byte)y, dir, isBlocked);}

	/***
	 * Get the X coordinate of this position
	 * @return X coordinate of this position
	 */
	public byte getX() {
		return x;}

	/****
	 * Get the Y coordinate of this position
	 * @return Y coordinate of this position
	 */
	public byte getY() {
		return y;}

	/****
	 * Get the direction of this position
	 * @return direction of this position
	 */
	public Direction getDir() {
		return dir;}

	/****
	 * Get the number of tiles free in front of this tile. A value of 0 implies there is a tile directly in front.
	 * @return The number of open tiles in front of this position.
	 */
	public boolean isBlocked() {
		return isBlocked;}
	
	/****
	 * Rotate a direction 1 left
	 * @param d The direction to rotate
	 * @return The direction 'd' rotated 1 left<br>
	 * LEFT -> DOWN<br>
	 * DOWN -> RIGHT<br>
	 * RIGHT -> UP<br>
	 * UP -> LEFT
	 */
	public static Direction rotateLeft(Direction d) {
		switch (d) {
		case UP:
			return Direction.LEFT;
		case LEFT:
			return Direction.DOWN;
		case DOWN:
			return Direction.RIGHT;
		case RIGHT:
			return Direction.UP;
		default:
			throw new RuntimeException("Shouldn't happen");}
	}

	/****
	 * Rotate a direction 1 right
	 * @param d The direction to rotate
	 * @return The direction 'd' rotated 1 right<br>
	 * LEFT -> UP<br>
	 * DOWN -> LEFT<br>
	 * RIGHT -> DOWN<br>
	 * UP -> RIGHT
	 */
	public static Direction rotateRight(Direction d) {
		switch (d) {
		case UP:
			return Direction.RIGHT;
		case LEFT:
			return Direction.UP;
		case DOWN:
			return Direction.LEFT;
		case RIGHT:
			return Direction.DOWN;
		default:
			throw new RuntimeException("Shouldn't happen");}
	}

	/****
	 * Get the absolute X coordinate of the location of the
	 * robot relative to the start, based on the direction of start
	 * and X and Y relative to where we started
	 * 
	 * @param start The starting position to get information relative to
	 * @param r The relative position to where we started
	 * @return	The relative X coordinate based on start and r
	 */
	public static byte relativeX(Position start, Position r) {
		switch (start.getDir()) {
		case UP:
			return (byte) (start.getX() + r.getX());
		case DOWN:
			return (byte) (start.getX() - r.getX());
		case RIGHT:
			return (byte) (start.getX() + r.getY());
		case LEFT:
			return (byte) (start.getX() - r.getY()); 
		default:
			throw new RuntimeException("Shouldn't happen");}
	}

	/****
	 * Get the absolute Y coordinate of the location of the
	 * robot relative to the start, based on the direction of start
	 * and X and Y relative to where we started
	 * 
	 * @param start The starting position to get information relative to
	 * @param r The relative position to where we started
	 * @return	The relative Y coordinate based on start and r
	 */
	public static byte relativeY(Position start, Position r) {
		switch (start.getDir()) {
		case LEFT:
			return (byte) (start.getY() + r.getX());
		case RIGHT:
			return (byte) (start.getY() - r.getX());
		case UP:
			return (byte) (start.getY() + r.getY());
		case DOWN:
			return (byte) (start.getY() - r.getY());
		default:
			throw new RuntimeException("Shouldn't happen");}
	}
	
	/*****
	 * Check if two positions are equal
	 * 
	 * @param o Object to 
	 * @return true iff the Object <code>o</code> is equal this <code>this</code>
	 */
	@Override
	public boolean equals(Object o){
		if (o instanceof Position){
			Position p = (Position)o;
			return p.getDir() == this.getDir() && p.getX() == this.x && p.getY() == this.y && p.isBlocked() == this.isBlocked;
		} else return false;
		
	}
}