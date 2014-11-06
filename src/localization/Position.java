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
	private int x, y;			// X and Y coordinates
	private Direction dir; 		// UP = North, DOWN = South, etc.
	private boolean isBlocked;	// Whether we are blocked

	/****
	 * Create a new absolute position with the following paramaters
	 * 
	 * @param x	X coordinate relative to origin
	 * @param y	Y coordinate relative to origin
	 * @param dir Direction relative to positive Y
	 * @param isBlocked	Whether this direction is blocked
	 */
	public Position(int x, int y, Direction dir, boolean isBlocked) {
		this.x = x;
		this.y = y;
		this.dir = dir;
		this.isBlocked = isBlocked;}

	/***
	 * Get the X coordinate of this position
	 * @return X coordinate of this position
	 */
	public int getX() {
		return x;}

	/****
	 * Get the Y coordinate of this position
	 * @return Y coordinate of this position
	 */
	public int getY() {
		return y;}

	/****
	 * Get the direction of this position
	 * @return direction of this position
	 */
	public Direction getDir() {
		return dir;}

	/****
	 * Get whether this position is blocked in the specified direction
	 * @return If this position is blocked in the specified direction
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
	public static int relativeX(Position start, Position r) {
		switch (start.getDir()) {
		case UP:
			return start.getX() + r.getX();
		case DOWN:
			return start.getX() - r.getX();
		case RIGHT:
			return start.getX() + r.getY();
		case LEFT:
			return start.getX() - r.getY(); 
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
	public static int relativeY(Position start, Position r) {
		switch (start.getDir()) {
		case LEFT:
			return start.getY() + r.getX();
		case RIGHT:
			return start.getY() - r.getX();
		case UP:
			return start.getY() + r.getY();
		case DOWN:
			return start.getY() - r.getY();
		default:
			throw new RuntimeException("Shouldn't happen");}
	}
}