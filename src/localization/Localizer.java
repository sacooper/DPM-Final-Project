package localization;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;

import lejos.nxt.UltrasonicSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import main.Main;

/*******
 * Localize using a known map. Once completed, the <code>OdometeryPoseProvider</code> passed in
 * during instantiation will be corrected to a known point.
 * 
 * @author Scott Cooper
 *
 */
public class Localizer {
	private UltrasonicSensor us_scanner;
	private DifferentialPilot pilot;	// Pilot controlling movement
	private OdometryPoseProvider odo;
	private static Pose startingPose;
	private BitSet map;
	
	/****
	 * Create a new localizer. The class extends MCLPoseProvider by 
	 * interfacing the 'Main' class to acquire the necessary maps
	 * 
	 * @param pilot The DifferentialPilot used to move
	 * @param us_scanner The RangeScanner used to acquire movements
	 * @param odo The OdometryPoseProvider to correct
	 */
	public Localizer(DifferentialPilot pilot, UltrasonicSensor us_scanner, OdometryPoseProvider odo) {
		this.pilot = pilot;
		this.odo = odo;
		this.us_scanner = us_scanner;
		startingPose = null;
	}
	
//	public void _localize(){
//		map = Main.getCurrentMap();
//		
//		// STEP 1: GET TO CARDINAL DIRECTION
//		double oldRotateSpeed = pilot.getRotateSpeed();
//		pilot.setRotateSpeed(90);
//		double distance = getFilteredData();
//		boolean saw_wall = false;
//		pilot.rotate(360, true);
//		while ((distance <= (Main.TILE_WIDTH - 5) || !saw_wall) && pilot.isMoving()) {
//			distance = getFilteredData();
//			if (distance <= (Main.TILE_WIDTH -5))
//				saw_wall = true;
//		}
//		pilot.stop();
//		pilot.setRotateSpeed(oldRotateSpeed);
//		
//		// STEP 2: Travel forward over a tile, ensuring we're at the center 
//		//			of a tile with cardinal heading
//		pilot.travel(Main.TILE_WIDTH);
//	
//	}

	/***
	 * Get the starting pose or null if it has not yet been determined
	 * 
	 * @return The starting pose or null if not yet determined
	 */
	public static synchronized Pose getStartingPose() {
			return startingPose;}

	/****
	 * Set the starting pose. Note that this should only be used once during
	 * any one run.
	 * 
	 * @param startingPose The new starting pose
	 */
	public static synchronized void setStartingPose(Pose startingPose) {
			Localizer.startingPose = startingPose;}
	
	private byte tilesFree(Direction d, byte x, byte y){
		byte i = 0;
		switch(d){
		case UP:
			while ((y+i+1) < Main.NUM_TILES && !map.get(x*Main.NUM_TILES + i + y + 1))
				i++;
			break;
		case DOWN:
			while ((y-i-1) >= 0 && !map.get(x*Main.NUM_TILES  + y - i - 1))
				i++;
			break;
		case RIGHT:
			while((x+i+1) < Main.NUM_TILES && !map.get((x+i+1)*Main.NUM_TILES + y))
				i++;
			break;
		case LEFT:
			while((x-i-1) >= 0 && !map.get((x-i-1)*Main.NUM_TILES + y))
				i++;
		}
		return 4 < i ? 4 : i;
		
	}
	private ArrayList<Position> generatePossibleStates(){
		ArrayList<Position> possible = new ArrayList<Position>();
		// Initialize possible states based on map
		for (byte x = 0; x < Main.NUM_TILES; x++){
			for(byte y = 0; y < Main.NUM_TILES; y++){
				if (!map.get(x*Main.NUM_TILES + y)){
					possible.add(new Position(x, y, Direction.UP, tilesFree(Direction.UP, x, y)));
					possible.add(new Position(x, y, Direction.DOWN, tilesFree(Direction.DOWN, x, y)));
					possible.add(new Position(x, y, Direction.RIGHT, tilesFree(Direction.RIGHT, x, y)));
					possible.add(new Position(x, y, Direction.LEFT, tilesFree(Direction.LEFT, x, y)));}
			}
		}
		return possible;
	}
	
	
	/*******
	 * Get a value from the ultrasonic sensor for the current distance from the
	 * wall
	 * 
	 * @return A filtered value of the distance from the wall
	 */
	private int getFilteredData() {
		int dist;

		// do a ping
		us_scanner.ping();
		// wait for the ping to complete
		try {
			Thread.sleep(25);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		dist = us_scanner.getDistance();
		if (dist > 135)
			dist = 135;
		return dist;
	}
	
	/********
	 * Perform Localization using a known map
	 * 
	 * @return Number of observations made
	 */
	public int localize() {
		ArrayList<Position> possible = generatePossibleStates();
		// Current direction relative to where we started
		Direction current = Direction.UP;	
		
		// Current X and Y relative to where we started, # of observations
		int x = 0, y = 0, observations = 0;	
		
		while (possible.size() > 1) { // Narrow down list of states until we know where we started
			byte tiles_blocked = (byte) (getFilteredData() / (int)Main.TILE_WIDTH);
			observations++;

			// Filter out now invalid orientations
			Iterator<Position> iter = possible.iterator();
			Position me = new Position((byte)x, (byte)y, current, tiles_blocked);
			while (iter.hasNext()){
				Position s = iter.next();
				if (!valid(s, me)){
					iter.remove();}}
			
			if (possible.size() == 1) break;
			
			// TODO: Improved decision making
			if (tiles_blocked == 0) {	
				pilot.rotate(-90);
				current = Position.rotateLeft(current);
			} else {
				pilot.travel(Main.TILE_WIDTH);
				switch(current){
				case DOWN: y--; break;
				case LEFT: x--; break;
				case RIGHT: x++; break;
				case UP: y++; break;
				default: throw new RuntimeException("Shouldn't Happen");}}
			
		}

		if (possible.size() != 1)
			throw new RuntimeException("No possible states");
		
		Position startingPoint = possible.get(0);

		synchronized(odo){		// Update odometer based on known starting location
			double odo_x, odo_y;
			switch(startingPoint.getDir()){	// Need to get absolute change from starting point relative to where we are
			case DOWN:
				odo_x = -odo.getPose().getX();
				odo_y = -odo.getPose().getY();
				break;
			case LEFT:
				odo_x = -odo.getPose().getY();
				odo_y = odo.getPose().getX();
				break;
			case RIGHT:
				odo_x = odo.getPose().getY();
				odo_y = -odo.getPose().getX();
				break;
			case UP:
				odo_x = odo.getPose().getX();
				odo_y = odo.getPose().getY();
				break;
			default: throw new RuntimeException("Shouldn't Happen");}
			float new_x = (float) ((((double)startingPoint.getX()) - 0.5)*Main.TILE_WIDTH + odo_x);
			float new_y = (float) ((((double)startingPoint.getY()) - 0.5)*Main.TILE_WIDTH + odo_y);
			float new_heading = (float) (odo.getPose().getHeading() + (90f * startingPoint.getDir().v));
			odo.setPose(new Pose(new_x, new_y, new_heading));}
		return observations;	
	}



	/****
	 * Check whether a position r is possible from position s
	 * @param s The starting position to use as a 'base'
	 * @param r Where the robot is relative to where it started (current observation)
	 * @return True iff the position the position r is possible relative to position s
	 */
	private boolean valid(Position s, Position r){
		// Get real direction to check based on Position checking and 
		// where we are facing based on where we started
		Direction realDir = s.getDir();
		for (int i = 0; i < r.getDir().v; i++)
			realDir = Position.rotateLeft(realDir);
		
		// Get position
		byte x = Position.relativeX(s, r);
		byte y = Position.relativeY(s, r);
		
		if (x < 0 || x > (Main.NUM_TILES - 1) || y < 0 || y > (Main.NUM_TILES - 1) || map.get(x*Main.NUM_TILES + y)) return false;
		
		return r.tilesInFront() == tilesFree(realDir, x, y);
		
//		switch(realDir){
//		case UP:
//			if (r.isBlocked())
//				return (y == (Main.NUM_TILES - 1) || map[x][y+1]);
//			else
//				return (y < (Main.NUM_TILES - 1) && !map[x][y+1]);
//		case DOWN:
//			if (r.isBlocked())
//				return (y == 0 || map[x][y-1]);
//			else
//				return (y > 0 && !map[x][y-1]);
//		case LEFT:
//			if (r.isBlocked())
//				return (x == 0 || map[x-1][y]);
//			else
//				return (x > 0 && !map[x-1][y]);
//		case RIGHT:
//			if (r.isBlocked())
//				return (x == (Main.NUM_TILES - 1) || map[x+1][y]);
//			else
//				return (x < (Main.NUM_TILES - 1) && !map[x+1][y]);}
//		return true;
	}
	
}