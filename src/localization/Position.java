package localization;
/********
 * Class representing an absolute position.
 * 
 * Information include X, Y, cardinal direction,
 * and whether it is blocked. All information 
 * is absolute, relative to the origin (0,0) 
 * with UP = NORTH = positive Y direction
 */
public class Position {
	private byte x, y;			// X and Y coordinates
	private Direction dir; 		// UP = North, DOWN = South, etc.
	private byte tilesInFront;	// Whether we are blocked

	/****
	 * Create a new absolute position with the following paramaters
	 * 
	 * @param x	X coordinate relative to origin
	 * @param y	Y coordinate relative to origin
	 * @param dir Direction relative to positive Y
	 * @param tilesInFront Number of tiles in from of this position that are open
	 */
	public Position(byte x, byte y, Direction dir, byte tilesInFront) {
		this.x = x;
		this.y = y;
		this.dir = dir;
		this.tilesInFront = tilesInFront;}

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
	public byte tilesInFront() {
		return tilesInFront;}
	
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
}