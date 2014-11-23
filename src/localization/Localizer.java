package localization;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import main.Display;
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
	 */
	public int localize() {
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
	
	private static Position forward(Position p){
		switch(p.getDir()){
		case DOWN: return new Position(p.getX(), p.getY() - 1, null, false);
		case LEFT: return new Position(p.getX() - 1, p.getY(), null, false);
		case RIGHT: return new Position(p.getX() + 1, p.getY(), null, false);
		case UP: return new Position(p.getX(), p.getY() + 1, null, false);
		default: throw new RuntimeException("Invalid direction in Localizer.forward()");
		}
	}
	
	private static boolean contains(ArrayList<Position> seen, Position p){
		for (Position q : seen)
			if (q.getX() == p.getX() && q.getY() == p.getY())
				return true;
		return false;
	}
	
}