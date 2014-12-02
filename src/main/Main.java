package main;

import java.util.BitSet;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import localization.Localizer;
import navigation.MovementController;
import navigation.OdometryCorrection;
import blocks.Arm;
import blocks.BlockRescuer;

/*****
 * Main class containing all constants. Primary control of the robot occurs here.
 * 
 * @author Scott Cooper
 *
 */
public class Main {

	public static final NXTRegulatedMotor 
		MOTOR_LEFT = Motor.A,
		ARM = Motor.B,
		MOTOR_RIGHT = Motor.C;
	
	public static final UltrasonicSensor ULTRASONIC = new UltrasonicSensor(SensorPort.S2);
	public static final ColorSensor COLORSENSOR_LEFT = new ColorSensor(SensorPort.S1),
									COLORSENSOR_RIGHT = new ColorSensor(SensorPort.S3);
	
	private static DifferentialPilot pilot;
	private static Navigator nav;
	private static MovementController moveController;
	private static BlockRescuer blockRescuer;
	private static OdometryPoseProvider odo;
	private static Localizer localizer;
	private static Display display;
	private static OdometryCorrection odoCorrection;
	private static Arm arm;
	private static long start, time; 
	
	public static final float	 
		LEFT_WHEEL_D = 4.155f,
		RIGHT_WHEEL_D = 4.1645f,
		WHEEL_BASE = 17.95f,		
		TILE_WIDTH = 30.48f;
	
	public static final int
		NUM_TILES = 12;		// XXX
	
	// List of maps for use in competition
	// usage: maps[map_number][x][y]
	private static final BitSet[] maps;

	private static final int NUM_MAPS = 6;	// XXX
	private static int mapNumber = 0;
	private static Waypoint dropoff = null;
	
	/***
	 * Private constructor to prevent external instantiation
	 */
	private Main(){};
	
	public static void main(String[] args) {
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		pilot.setAcceleration(15);
		pilot.setTravelSpeed(20);
		pilot.setRotateSpeed(90);
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		
		// Instantiate a new OdometryCorrection and disable it
		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
		
		display = new Display(odo);
		
		// Instantiate a new Localizer
		localizer = new Localizer(pilot, ULTRASONIC, odo);
		
		// Instantiate a new Navigator to control movement
		nav = new Navigator(pilot, odo);
		
		// Instantiate a new MovementController for travelling to waypoints
		moveController = new MovementController(nav);

		// Instantiate a new Arm for controlling claw movement
		ARM.setAcceleration(200);
		ARM.setSpeed(360);
		arm = new Arm(ARM, Arm.ArmState.RAISED);
		
		// Instantiate a new blockRescuer
		blockRescuer = new BlockRescuer(pilot, nav, ULTRASONIC, arm);
		
		setup();		
		LCD.clear();
		
		
		for (int x = 0; x < Main.NUM_TILES; x++){
			for (int y = 0; y < Main.NUM_TILES; y++){
				if (getCurrentMap().get(x * Main.NUM_TILES + y))
					LCD.drawChar('X', x, Main.NUM_TILES - 1 - y);
			}
		}
		
		
		while (Button.waitForAnyPress() != Button.ID_ENTER);
		start = System.currentTimeMillis();
		time = -1;
		
		display.start();
		odoCorrection.start();
		OdometryCorrection.enable();
		
		Display.setCurrentAction(Display.Action.LOCALIZING);
		localizer.localize();
		
		OdometryCorrection.enable();
		long x = System.currentTimeMillis();
		
		while (shouldContinue()){
			Display.setCurrentAction(Display.Action.MOVING);
			moveController.travelToTile(1, 2, -90);
			
			new Thread(new Runnable(){
				public void run(){
					unblockPickupArea();
					moveController.regenerate();
				}
			}).start();
			
			pilot.travel(odo.getPose().getY() - Main.TILE_WIDTH);	// Move to top of current tile
			
			Display.setCurrentAction(Display.Action.BLOCK_ACTION);
			blockRescuer.rescueBlock();
			
			Display.setCurrentAction(Display.Action.MOVING);
			moveController.travelToTile((int)dropoff.getX(), (int)dropoff.getY(), (float)dropoff.getHeading());
	
			pilot.travel(Main.TILE_WIDTH/4f);
			Display.setCurrentAction(Display.Action.BLOCK_ACTION);
			
			new Thread(new Runnable(){
				public void run(){
					blockPickupArea();
					moveController.regenerate();
				}
			}).start();
			
			arm.drop();
			if (time == -1)
				time = 2 * (System.currentTimeMillis() - x);
		}
		System.exit(0);
	}

	private static boolean shouldContinue() {
		return time == -1 ? true : (7*60 + 30)*1000 + start - System.currentTimeMillis() > time; 
	}

	/****
	 * Get the number of the current map
	 * 
	 * @return the current map number
	 */
	public static int getMapNumber() {
		return mapNumber;}
	
	/****
	 * Get the current map as a 2-D array of booleans where TRUE represents that tile
	 * (map[x][y]) being blocked)
	 * 
	 * @return A 2-D array representing the current map.
	 */
	public static BitSet getCurrentMap(){
		return maps[getMapNumber()];
	}
	
	static {		
		// Add button listener to escape button to allow for exiting at any time
		Button.ESCAPE.addButtonListener(
				new ButtonListener(){
					@Override
					public void buttonPressed(Button b) {
						if (b.getId() == Button.ID_ESCAPE) System.exit(0);}
		
					@Override
					public void buttonReleased(Button b) {}});
		
		maps = new BitSet[Main.NUM_MAPS];
		
		for (int i = 0; i < Main.NUM_MAPS; i++){
			for (int x = 0; x < Main.NUM_TILES; x++){
				for (int y = 0; y < Main.NUM_TILES; y++){
					maps[i] = new BitSet(Main.NUM_TILES * Main.NUM_TILES);
					maps[i].clear();}}}
	
	/**********************************************=
		// The follow are for beta demonstrations
		
		// setBlock(map #, x, y, isBlocked)
		
		// Initialization of map 0
		setBlock(0, 0, 5, true);
		setBlock(0, 1, 7, true);
		setBlock(0, 2, 4, true);
		setBlock(0, 2, 6, true);
		setBlock(0, 2, 7, true);
		setBlock(0, 3, 5, true);
		setBlock(0, 4, 1, true);
		setBlock(0, 4, 2, true);
		setBlock(0, 4, 3, true);
		setBlock(0, 6, 2, true);
		setBlock(0, 6, 5, true);
		setBlock(0, 7, 0, true);
		setBlock(0, 7, 2, true);
		setBlock(0, 7, 3, true);
		setBlock(0, 7, 6, true);
		
		// Initialization of map 1
		setBlock(1, 0, 5, true);
		setBlock(1, 1, 6, true);
		setBlock(1, 2, 0, true);
		setBlock(1, 2, 3, true);
		setBlock(1, 2, 4, true);
		setBlock(1, 3, 1, true);
		setBlock(1, 3, 7, true);
		setBlock(1, 4, 4, true);
		setBlock(1, 4, 6, true);
		setBlock(1, 4, 7, true);
		setBlock(1, 5, 0, true);
		setBlock(1, 7, 0, true);
		setBlock(1, 7, 1, true);
		setBlock(1, 7, 6, true);
		setBlock(1, 7, 7, true);
		
		// Initialization of map 2
		setBlock(2, 0, 7, true);
		setBlock(2, 2, 3, true);
		setBlock(2, 2, 6, true);
		setBlock(2, 3, 2, true);
		setBlock(2, 3, 3, true);
		setBlock(2, 3, 4, true);
		setBlock(2, 3, 6, true);
		setBlock(2, 4, 0, true);
		setBlock(2, 4, 7, true);
		setBlock(2, 5, 0, true);
		setBlock(2, 5, 5, true);
		setBlock(2, 6, 4, true);
		setBlock(2, 7, 0, true);
		setBlock(2, 7, 4, true);
		setBlock(2, 7, 6, true);
		
	**********************************************/
		
	/**************************************************
	 * The follow maps are for the FINAL DEMONSTRATION
	 ************************************************/
		
		
		// Map 1
		setBlock(0, 0, 6, true); //
		setBlock(0, 0, 9, true); //
		setBlock(0, 1, 3, true); //
		setBlock(0, 2, 2, true); //
		setBlock(0, 2, 11, true); //
		setBlock(0, 3, 5, true); //
		setBlock(0, 3, 10, true); //
		setBlock(0, 4, 4, true); //
		setBlock(0, 4, 7, true); //
		setBlock(0, 5, 0, true); //
		setBlock(0, 5, 2, true); //
		setBlock(0, 6, 3, true); //
		setBlock(0, 7, 7, true); //
		setBlock(0, 8, 0, true); //
		setBlock(0, 8, 1, true); //
		setBlock(0, 8, 6, true); //
		setBlock(0, 9, 4, true); //
		setBlock(0, 9, 7, true); //
		setBlock(0, 9, 9, true); //
		setBlock(0, 10, 2, true);// 
		setBlock(0, 10, 6, true);//
		setBlock(0, 11, 1, true);//
		
		// Map 2
		setBlock(1, 0, 4, true); //
		setBlock(1, 0, 5, true); //
		setBlock(1, 3, 2, true); //
		setBlock(1, 3, 5, true); //
		setBlock(1, 3, 6, true); //
		setBlock(1, 3, 9, true); //
		setBlock(1, 4, 5, true); //
		setBlock(1, 4, 9, true); //
		setBlock(1, 4, 10, true); //
		setBlock(1, 5, 5, true); //
		setBlock(1, 6, 8, true); //
		setBlock(1, 6, 9, true); //
		setBlock(1, 7, 0, true); //
		setBlock(1, 8, 4, true); //
		setBlock(1, 8, 5, true); //
		setBlock(1, 8, 10, true); //
		setBlock(1, 9, 1, true); //
		setBlock(1, 9, 11, true); //
		setBlock(1, 10, 7, true); //
		setBlock(1, 11, 2, true); //
		setBlock(1, 11, 5, true); //
		setBlock(1, 11, 7, true); //

		// Map 3
		setBlock(2, 0, 3, true);
		setBlock(2, 0, 8, true);
		setBlock(2, 2, 3, true);
		setBlock(2, 2, 6, true);
		setBlock(2, 3, 2, true);
		setBlock(2, 3, 4, true);
		setBlock(2, 3, 10, true);
		setBlock(2, 4, 1, true);
		setBlock(2, 4, 5, true);
		setBlock(2, 4, 6, true);
		setBlock(2, 4, 7, true);
		setBlock(2, 6, 6, true);
		setBlock(2, 6, 8, true);
		setBlock(2, 7, 0, true);
		setBlock(2, 7, 11, true);
		setBlock(2, 9, 6, true);
		setBlock(2, 10, 0, true);
		setBlock(2, 10, 3, true);
		setBlock(2, 10, 4, true);
		setBlock(2, 10, 9, true);
		setBlock(2, 11, 2, true);
		setBlock(2, 11, 11, true);

		// Map 4
		setBlock(3, 0, 2, true);
		setBlock(3, 0, 3, true);
		setBlock(3, 0, 4, true);
		setBlock(3, 0, 8, true);
		setBlock(3, 1, 4, true);
		setBlock(3, 2, 0, true);
		setBlock(3, 2, 5, true);
		setBlock(3, 2, 10, true);
		setBlock(3, 3, 2, true);
		setBlock(3, 3, 9, true);
		setBlock(3, 3, 11, true);
		setBlock(3, 4, 3, true);
		setBlock(3, 5, 10, true);
		setBlock(3, 6, 4, true);
		setBlock(3, 8, 4, true);
		setBlock(3, 8, 8, true);
		setBlock(3, 9, 0, true);
		setBlock(3, 9, 4, true);
		setBlock(3, 9, 6, true);
		setBlock(3, 11, 1, true);
		setBlock(3, 11, 5, true);
		setBlock(3, 11, 10, true);
		
		// Map 5
		setBlock(4, 0, 3, true);
		setBlock(4, 0, 5, true);
		setBlock(4, 0, 8, true);
		setBlock(4, 3, 1, true);
		setBlock(4, 3, 3, true);
		setBlock(4, 3, 10, true);
		setBlock(4, 4, 3, true);
		setBlock(4, 4, 6, true);
		setBlock(4, 4, 7, true);
		setBlock(4, 4, 10, true);
		setBlock(4, 5, 8, true);
		setBlock(4, 6, 10, true);
		setBlock(4, 7, 6, true);
		setBlock(4, 7, 7, true);
		setBlock(4, 8, 1, true);
		setBlock(4, 8, 5, true);
		setBlock(4, 8, 10, true);
		setBlock(4, 9, 2, true);
		setBlock(4, 9, 6, true);
		setBlock(4, 10, 10, true);
		setBlock(4, 11, 2, true);
		setBlock(4, 11, 9, true);
		
		// Map 6
		setBlock(5, 0, 10, true); //
		setBlock(5, 1, 5, true); //
		setBlock(5, 2, 3, true); //
		setBlock(5, 3, 2, true); //
		setBlock(5, 3, 10, true); //
		setBlock(5, 4, 2, true); //
		setBlock(5, 4, 7, true); //
		setBlock(5, 5, 2, true); //
		setBlock(5, 5, 4, true); //
		setBlock(5, 5, 6, true); // it was setBlock(5, 5, 7, true); 
		setBlock(5, 5, 11, true); //
		setBlock(5, 6, 5, true);  //
		setBlock(5, 6, 9, true); //
		setBlock(5, 7, 6, true); //
		setBlock(5, 7, 8, true); //
		setBlock(5, 8, 2, true); //
		setBlock(5, 8, 10, true); //
		setBlock(5, 9, 0, true); //
		setBlock(5, 9, 4, true); //
		setBlock(5, 9, 8, true); //
		setBlock(5, 10, 5, true); //
		setBlock(5, 10, 10, true); //
		
		/*************************************/
		
	}

	/*****
	 * Set the status of a coordinate for a given map
	 * 
	 * @param map The map number of this point
	 * @param x The x coordinate of this point
	 * @param y The y coordinate of this point
	 * @param v The value at this point. True implies blocked, false implies not blocked.
	 */
	private static void setBlock(int map, int x, int y, boolean v){
		if (maps==null) throw new RuntimeException("Maps not initialized");
		if (map >= maps.length) throw new RuntimeException("Invalid map number");
		
		maps[map].set(x * Main.NUM_TILES + y, v);}
	
	public static DifferentialPilot getPilot() {
		return pilot;}
	
	public static void setup(){
		int x =0, y = 0, option;
		do {
			LCD.clear();
			LCD.drawString("Map: " + (mapNumber + 1), 0, 0);
			option = Button.waitForAnyPress();
			switch (option){
			case Button.ID_LEFT: mapNumber = (mapNumber + (Main.NUM_MAPS - 1)) % Main.NUM_MAPS; break;
			case Button.ID_RIGHT: mapNumber = (mapNumber + 1) % Main.NUM_MAPS; break;
			default: break;
			}
			
		} while (option != Button.ID_ENTER);
		
		blockPickupArea();
		new Thread(new Runnable(){
			public void run(){
				moveController.regenerate();
			}
		}).start();
		
		do {
			LCD.clear();
			LCD.drawString("X: " + x, 0, 0);
			option = Button.waitForAnyPress();
			switch (option){ 
			case Button.ID_LEFT: x = (x + (Main.NUM_TILES - 1)) % Main.NUM_TILES; break;
			case Button.ID_RIGHT: x = (x + 1) % Main.NUM_TILES; break;
			default: break;
			}
			
		} while (option != Button.ID_ENTER);
		
		do {
			LCD.clear();
			LCD.drawString("Y: " + y, 0, 0);
			option = Button.waitForAnyPress();
			switch (option){
			case Button.ID_LEFT: y = (y + (Main.NUM_TILES - 1)) % Main.NUM_TILES; break;
			case Button.ID_RIGHT: y = (y + 1) % Main.NUM_TILES; break;
			default: break;
			}
			
		} while (option != Button.ID_ENTER);
		
//		int ang = 0;
		
		if (x-1 >= 0 && !maps[mapNumber].get((x-1)*Main.NUM_TILES + y))
			dropoff = new Waypoint(x-1, y, 0); // ang = 0;
		else if (x + 1 < Main.NUM_TILES && !maps[mapNumber].get((x+1)*Main.NUM_TILES + y))
			dropoff = new Waypoint(x+1, y, 180); // ang = 180;
		else if (y - 1 >= 0 && !maps[mapNumber].get(x*Main.NUM_TILES + y - 1))
			dropoff = new Waypoint(x, y-1, 90); // ang = 90;
		else
			dropoff = new Waypoint(x, y+1, -90); // ang = -90;
		
//		dropoff = new Waypoint(x, y, ang);
		
	}
	
	/***
	 * Mark the pickup area as blocked. Necessary to prevent any path 
	 * from moving through the pickup area.
	 */
	private static void blockPickupArea() {
		setBlock(Main.getMapNumber(), 0, 0, true);
		setBlock(Main.getMapNumber(), 0, 1, true);
		setBlock(Main.getMapNumber(), 1, 0, true);
		setBlock(Main.getMapNumber(), 1, 1, true);
	}
	
	/***
	 * Mark the pickup area as unblocked. Necessary to allow
	 * pathfinding from within the pickup area.
	 */
	private static void unblockPickupArea() {
		setBlock(Main.getMapNumber(), 0, 0, false);
		setBlock(Main.getMapNumber(), 0, 1, false);
		setBlock(Main.getMapNumber(), 1, 0, false);
		setBlock(Main.getMapNumber(), 1, 1, false);
	}
}
