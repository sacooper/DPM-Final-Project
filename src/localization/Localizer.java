package localization;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;

import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import main.Main;
import navigation.OdometryCorrection;

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
	private static Position startingPoint;
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
		this.map = Main.getCurrentMap();
		startingPoint = null;
	}

	/***
	 * Get the starting pose or null if it has not yet been determined
	 * 
	 * @return The starting pose or null if not yet determined
	 */
	public static synchronized Position getStartingPosition() {
			return startingPoint;}
	
	/****
	 * Check whether a position is blocked 
	 * 
	 * @param d The direction to check in
	 * @param x The X coordinate of the position to check
	 * @param y The Y coordinate of the position to check
	 * @return True iff the position represented by d, x, y is blocked
	 */
	private boolean isBlocked(Direction d, byte x, byte y){
		switch(d){
		case UP: return (y + 1) == Main.NUM_TILES || map.get(x+Main.NUM_TILES + y + 1);
		case DOWN:
			return y == 0 || map.get(x+Main.NUM_TILES + y - 1);
		case RIGHT:
			return ((x+1) == Main.NUM_TILES || map.get((x+1)*Main.NUM_TILES + y));
		case LEFT:
			return (x == 0 || map.get((x-1)*Main.NUM_TILES + y));
		default:
			throw new RuntimeException("Invalid direction");
		}
		
	}
	
	/****
	 * Generate the list of all possible states. It is assumed that
	 * the map has already be set.
	 * 
	 * @return An arraylist of all possible starting positions
	 */
	private ArrayList<Position> generatePossibleStates(){
		ArrayList<Position> possible = new ArrayList<Position>();
		// Initialize possible states based on map
		for (byte x = 0; x < Main.NUM_TILES; x++){
			for(byte y = 0; y < Main.NUM_TILES; y++){
				if (!map.get(x*Main.NUM_TILES + y)){
					possible.add(new Position(x, y, Direction.UP, isBlocked(Direction.UP, x, y)));
					possible.add(new Position(x, y, Direction.DOWN, isBlocked(Direction.DOWN, x, y)));
					possible.add(new Position(x, y, Direction.RIGHT, isBlocked(Direction.RIGHT, x, y)));
					possible.add(new Position(x, y, Direction.LEFT, isBlocked(Direction.LEFT, x, y)));}
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
		return dist > 50 ? 50 : dist;
	}
	/********
	 * Perform Localization using a known map
	 * 
	 * @return Number of observations made
	 * @since v4
	 */
	public int localize(){
		ArrayList<Position> seen = new ArrayList<Position>();		// All of the places we've been
		ArrayList<Position> possible = generatePossibleStates(); 	// All possible starting points left
		
		// Current direction relative to where we started
		Direction current = Direction.UP;	
		// Current X and Y relative to where we started, # of observations
		int x = 0, y = 0, observations = 0;	
		
		boolean secondPass = false;
		
		while (possible.size() > 1) { // Narrow down list of states until we know where we started
			
			// Check if we've been where we are before
			if (contains(seen, new Position(x, y, null, false))){
				boolean isBlocked = getBlockedStatus();
				
				if (isBlocked || (contains(seen, forward(new Position(x, y, current, false))) && !secondPass)){
					pilot.rotate(-90);
					current = Position.rotateRight(current);
					isBlocked = getBlockedStatus();
					if (isBlocked || (contains(seen, forward(new Position(x, y, current, false))) && !secondPass)){
						pilot.rotate(180);
						current = Position.rotateLeft(Position.rotateLeft(current));
						isBlocked = getBlockedStatus();
						if (isBlocked && !secondPass){
							secondPass = true;
							pilot.rotate(-90);
							current = Position.rotateRight(current);
							continue;
						} else if (isBlocked && secondPass){
							throw new RuntimeException("can't localize");
						}
					}
				}
				pilot.travel(Main.TILE_WIDTH);
				switch(current){
					case DOWN: y--; break;
					case LEFT: x--; break;
					case RIGHT: x++; break;
					case UP: y++; break;
					default: throw new RuntimeException("Shouldn't Happen");
				}
				if (possible.size() != 1) correct();
			}
			secondPass = false;
			
			boolean  leftBlocked, rightBlocked,
				isBlocked =getBlockedStatus();
			
			observations++;
			Position me = new Position(x, y, current, isBlocked);
			Iterator<Position> iter = possible.iterator();
			
			while (iter.hasNext()){
				if(!valid(iter.next(), me))
					iter.remove();
			}
			
			if (possible.size() == 1) break;
			else if (possible.size() < 1) localize();
			
			/********************************************/
			
			pilot.rotate(-90);
			current = Position.rotateRight(current);
			rightBlocked = getBlockedStatus();
			
			observations++;
			me = new Position(x, y, current, rightBlocked);
			iter = possible.iterator();
			
			while (iter.hasNext()){
				if(!valid(iter.next(), me))
					iter.remove();
			}
			
			if (possible.size() == 1) break;
			else if (possible.size() < 1) localize();
			
			
			/********************************************/
			
			pilot.rotate(180);
			current = Position.rotateLeft(Position.rotateLeft(current));
			leftBlocked = getBlockedStatus();
			
			observations++;
			me = new Position(x, y, current, leftBlocked);
			iter = possible.iterator();
			while (iter.hasNext()){
				if(!valid(iter.next(), me))
					iter.remove();
			}
			
			if (possible.size() == 1) break;
			else if (possible.size() < 1) localize();
			
			/********************************************/
			
			if (!leftBlocked && !contains(seen, forward(new Position(x, y, current, false))));
			else if (!isBlocked && !contains(seen, forward(new Position(x, y, Position.rotateRight(current), false)))){
				pilot.rotate(-90);
				current = Position.rotateRight(current);
			} else if (!rightBlocked && !contains(seen, forward(new Position(x, y, Position.rotateRight(Position.rotateRight(current)), false)))){
				pilot.rotate(-180);
				current = Position.rotateRight(Position.rotateRight(current));
			} else {
				pilot.rotate(90);
				current = Position.rotateLeft(current);
			}

			seen.add(new Position(x, y, null, false));
			pilot.travel(Main.TILE_WIDTH);
			switch(current){
				case DOWN: y--; break;
				case LEFT: x--; break;
				case RIGHT: x++; break;
				case UP: y++; break;
				default: throw new RuntimeException("Shouldn't Happen");
			}
			if (possible.size() != 1) correct();
		}

		if (possible.size() != 1) localize();		// restart if error
		
		startingPoint = possible.get(0);
		
		float real_x = Position.relativeX(startingPoint, new Position(x, y, current, false));
		float real_y = Position.relativeY(startingPoint, new Position(x, y, current, false));
		Direction real = startingPoint.getDir();
		for (int i = 0; i < current.v; i++)
			real = Position.rotateLeft(real);
		
		float heading = 0;
		switch (real){
			case UP: heading = 90f; break;
			case DOWN: heading = -90f; break;
			case RIGHT: heading = 0f; break;
			case LEFT: heading = 180f; break;
			default: throw new RuntimeException("Invalid direction");
		}
		
		odo.setPose(new Pose(real_x * Main.TILE_WIDTH - Main.TILE_WIDTH /2f, real_y * Main.TILE_WIDTH - Main.TILE_WIDTH /2f, heading));
		
		Sound.beep();
		return observations;	
	}
	
	/****
	 * Method to return whether the robot is currently blocked in the forward direction.
	 * Factored out to allow for changing of threshold in a central location.
	 * 
	 * @since v4
	 * 
	 * @see #getFilteredData()
	 * @return true iff the robot is currently blocked in the forward direction.
	 */
	private boolean getBlockedStatus() {
		for (int i = 0; i < 4; i++){
			boolean ping1 = getFilteredData() < Main.TILE_WIDTH * 2f/3f;
			boolean ping2 = getFilteredData() < Main.TILE_WIDTH * 2f/3f;
			
			if (ping1 == ping2)
				return ping1;
		}
		
		return getFilteredData() < Main.TILE_WIDTH * 2f / 3f;
	}

	/********
	 * Perform Localization using a known map
	 * 
	 * @return Number of observations made
	 * @deprecated
	 * @since v0
	 */
	public int localize_old() {
		ArrayList<Position> seen = new ArrayList<Position>();
		
		ArrayList<Position> possible = generatePossibleStates();
		// Current direction relative to where we started
		Direction current = Direction.UP;	
		// Current X and Y relative to where we started, # of observations
		int x = 0, y = 0, observations = 0;	
		
		while (possible.size() > 1) { // Narrow down list of states until we know where we started
			seen.add(new Position(x, y, null, false));
			boolean isBlocked = true;
			for (byte i = 0; i < 4 && possible.size() > 1; i++){
				isBlocked = getFilteredData() < (Main.TILE_WIDTH);
				observations++;
				Position me = new Position(x, y, current, isBlocked);
				Iterator<Position> iter = possible.iterator();
				while (iter.hasNext()){
					if(!valid(iter.next(), me))
						iter.remove();
				}
				if (i != 3){
					pilot.rotate(-90, false);
					current = Position.rotateRight(current);}
			}
			
		
			if (possible.size() > 1){
				boolean foundNewSpot = false;
				
				while (!foundNewSpot){
					int checkCount = 0;
					while (isBlocked && !contains(seen, forward(new Position(x, y, current, false))) && ++checkCount < 4){
						pilot.rotate(-90, false);
						current = Position.rotateRight(current);
						isBlocked = getFilteredData() < Main.TILE_WIDTH;
					}
				
					pilot.travel(Main.TILE_WIDTH);
					switch(current){
					case DOWN: y--; break;
					case LEFT: x--; break;
					case RIGHT: x++; break;
					case UP: y++; break;
					default: throw new RuntimeException("Shouldn't Happen");
					}
					
					if (!contains(seen, new Position(x, y, null, false)))
						foundNewSpot = true;
					
					if (possible.size() != 1) correct();
				}
			lejos.util.Delay.msDelay(300);
			}	
		}

		if (possible.size() != 1)
			throw new RuntimeException("No possible states");
		
		startingPoint = possible.get(0);
//		Display.pause();
//		LCD.clear();
//		LCD.drawString(startingPoint.getX() + ", " + startingPoint.getY(), 0, 0);
//		LCD.drawString(startingPoint.getDir().asCardinal(), 0, 1);
//		Button.waitForAnyPress();
//		Display.resume();
		
		float real_x = Position.relativeX(startingPoint, new Position(x, y, current, false));
		float real_y = Position.relativeY(startingPoint, new Position(x, y, current, false));
		Direction real = startingPoint.getDir();
		for (int i = 0; i < current.v; i++)
			real = Position.rotateLeft(real);
		
		float heading = 0;
		switch (real){
			case UP: heading = 90f; break;
			case DOWN: heading = -90f; break;
			case RIGHT: heading = 0f; break;
			case LEFT: heading = 180f; break;
			default: throw new RuntimeException("Invalid direction");
		}
		
		odo.setPose(new Pose(real_x * Main.TILE_WIDTH - Main.TILE_WIDTH /2f, real_y * Main.TILE_WIDTH - Main.TILE_WIDTH /2f, heading));
		
		Sound.beep();
		return observations;	
	}



	/****
	 * Check whether a position r is possible from position s
	 * @param s The starting position to use as a 'base'
	 * @param r Where the robot is relative to where it started (current observation)
	 * @see Position
	 * @return True iff the position the position r is possible relative to position s
	 */
	private boolean valid(Position s, Position r){
		// Get real direction to check based on Position checking and 
		// where we are facing based on where we started
		Direction correctedDir = s.getDir();
		for (int i = 0; i < r.getDir().v; i++)
			correctedDir = Position.rotateLeft(correctedDir);
		
		// Get position
		byte x = Position.relativeX(s, r);
		byte y = Position.relativeY(s, r);
		
		if (x < 0 || x > (Main.NUM_TILES - 1) || y < 0 || y > (Main.NUM_TILES - 1) || map.get(x*Main.NUM_TILES + y)) return false;
		
		
		switch(correctedDir){
		case UP:
			if (r.isBlocked())
				return (y == (Main.NUM_TILES - 1) || map.get(x*Main.NUM_TILES + y + 1));
			else
				return (y < (Main.NUM_TILES - 1) && !map.get(x*Main.NUM_TILES + y + 1));
		case DOWN:
			if (r.isBlocked())
				return (y == 0 || map.get(x*Main.NUM_TILES + y - 1));
			else
				return (y > 0 && !map.get(x*Main.NUM_TILES + y - 1));
		case LEFT:
			if (r.isBlocked())
				return (x == 0 || map.get((x-1)*Main.NUM_TILES + y));
			else
				return (x > 0 && !map.get((x-1)*Main.NUM_TILES + y));
		case RIGHT:
			if (r.isBlocked())
				return (x == (Main.NUM_TILES - 1) || map.get((x+1)*Main.NUM_TILES + y));
			else
				return (x < (Main.NUM_TILES - 1) && !map.get((x+1)*Main.NUM_TILES + y));}
		return true;
	}
	
	/****
	 * Get the position 1 forward from the position p. No error checking on the
	 * validity of the position is performed. 
	 * @see Position
	 * @param p The starting position
	 * @return The position 1 forward from p
	 */
	private static Position forward(Position p){
		switch(p.getDir()){
		case DOWN: return new Position(p.getX(), p.getY() - 1, null, false);
		case LEFT: return new Position(p.getX() - 1, p.getY(), null, false);
		case RIGHT: return new Position(p.getX() + 1, p.getY(), null, false);
		case UP: return new Position(p.getX(), p.getY() + 1, null, false);
		default: throw new RuntimeException("Invalid direction in Localizer.forward()");
		}
	}
	
	/***
	 * Check whether the Position p is containined in the ArrayList seen. A
	 * position is considered contained within seen iff the X and Y coordinates
	 * are equal
	 * @param seen The arraylist of positions to check from
	 * @param p The position to check for
	 * @return true iff the p is contained with seen, otherwise false
	 */
	private static boolean contains(ArrayList<Position> seen, Position p){
		for (Position q : seen)
			if (q.getX() == p.getX() && q.getY() == p.getY())
				return true;
		return false;
	}
	
	/*****
	 * Correct using the values provided from the OdometryCorrection class.
	 * 
	 * @since v4
	 * @see OdometryCorrection
	 */
	private void correct(){
		Pose current = odo.getPose();
		double ang = OdometryCorrection.lastHeadingCorrection();
		double dist = OdometryCorrection.lastDistanceCorrection();
		
		
		if (dist > 0){
			pilot.rotate(ang);
			pilot.travel(dist);
		} else {
			pilot.rotate(-ang);
			pilot.travel(dist);
			pilot.rotate(2*ang);
		}
		odo.setPose(current);}
	
	
}