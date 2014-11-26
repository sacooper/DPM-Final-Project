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
import lejos.robotics.navigation.Pose;
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
	private static long start;		// TODO: INITIALIZE AND USE
	
	public static final float	 
		LEFT_WHEEL_D = 4.155f,
		RIGHT_WHEEL_D = 4.155f,
//		WHEEL_BASE = 17.525f,
		WHEEL_BASE = 18.05f,		
		TILE_WIDTH = 30.48f;
	
	public static final int
		NUM_TILES = 4;							// FIXME
	
	// List of maps for use in competition
	// usage: maps[map_number][x][y]
	private static final BitSet[] maps;

	private static final int NUM_MAPS = 6;		// FIXME
	private static int mapNumber = 0;			// FIXME
	private static Waypoint dropoff = null;
	
	/***
	 * Private constructor to prevent external instantiation
	 */
	private Main(){};
	
//	public static void arm_lock_test(String[] args){
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		pilot.setAcceleration(500);
//		pilot.setTravelSpeed(22);
//		pilot.setRotateSpeed(90);
//		ARM.stop(true);
//		Button.waitForAnyPress();
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//		pilot.travel(Main.TILE_WIDTH);
//		pilot.travel(-Main.TILE_WIDTH);
//	}
//	
//	public static void block_test(String[] args){
//		// Instantiate a new DifferentialPilot to control movement
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		pilot.setAcceleration(500);
//		pilot.setTravelSpeed(22);
//		pilot.setRotateSpeed(90);
//		// Instantiate a new OdometryPoseProvider, of maintaining current pose
//		odo = new OdometryPoseProvider(pilot);
//		odo.setPose(new Pose(0, 0, -90));
//		nav = new Navigator(pilot, odo);
//		arm = new Arm(ARM, ArmState.RAISED);
//		blockRescuer = new BlockRescuer(pilot, ULTRASONIC, arm);
//		Button.waitForAnyPress();
//		blockRescuer.rescueBlock();
//		pilot.travel(Main.TILE_WIDTH);
//		arm.drop();		
//	}
//	
//	public static void localizer_test(String[] args){
//		// Instantiate a new DifferentialPilot to control movement
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		pilot.setAcceleration(2000);
//		pilot.setTravelSpeed(22);
//		pilot.setRotateSpeed(90);
//		// Instantiate a new OdometryPoseProvider, of maintaining current pose
//		odo = new OdometryPoseProvider(pilot);
//		display = new Display(odo);
//		display.start();
//		
//		localizer = new Localizer(pilot, ULTRASONIC, odo);
//		arm = new Arm(ARM);
//		blockRescuer = new BlockRescuer(pilot, ULTRASONIC, arm);
//		Button.waitForAnyPress();
//		
//		localizer.localize();
//		
//		Button.waitForAnyPress();
//		
//	}
//	
//	
//	public static void odo_test_2(String[] args){
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		
//		pilot.setAcceleration(500);
//		pilot.setTravelSpeed(22);
//		pilot.setRotateSpeed(90);
//		// Instantiate a new OdometryPoseProvider, of maintaining current pose
//		odo = new OdometryPoseProvider(pilot);
//		
//		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
//		
//		odoCorrection.start();
//	
//		display = new Display(odo);
//		display.start();
//		
//		odo.setPose(new Pose(-15, -15, 0));
//		
//		nav = new Navigator(pilot, odo);
//		Button.waitForAnyPress();
//
//		OdometryCorrection.enable();
//		Delay.msDelay(1000);
//		
//		Path p = new Path();
//		p.add(new Waypoint(Main.TILE_WIDTH*1.5, -Main.TILE_WIDTH/2f));
//		p.add(new Waypoint(Main.TILE_WIDTH*1.5, Main.TILE_WIDTH*1.5));
//		p.add(new Waypoint(-Main.TILE_WIDTH/2f, Main.TILE_WIDTH*1.5));
//		p.add(new Waypoint(-Main.TILE_WIDTH/2f, -Main.TILE_WIDTH/2f));
//		
//		for (Waypoint w : p){
//			nav.goTo(w);
//			Button.waitForAnyPress();
//		}
//		
//		
//		
//	}
//
//	public static void odo_correction_test(String[] args){
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		DifferentialPilot pilot2 = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		
//		pilot.setAcceleration(500);
//		pilot.setTravelSpeed(22);
//		pilot.setRotateSpeed(90);
//		pilot2.setRotateSpeed(90);
//		// Instantiate a new OdometryPoseProvider, of maintaining current pose
//		odo = new OdometryPoseProvider(pilot);
//		
//		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
//		
//		odoCorrection.start();
//	
//		display = new Display(odo);
//		display.start();
//		
//		Button.waitForAnyPress();
//		
//		pilot2.rotate(20);
//
//		OdometryCorrection.enable();
//		Delay.msDelay(1000);
//		pilot.travel(Main.TILE_WIDTH*1.5f);
//		
//		
//		
//	}
//	
//	public static void nav_test(String[] args){
//		Button.waitForAnyPress();
//		// Instantiate a new DifferentialPilot to control movement
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		pilot.setAcceleration(2000);
//		pilot.setTravelSpeed(22);
//		pilot.setRotateSpeed(90);
//		// Instantiate a new OdometryPoseProvider, of maintaining current pose
//		odo = new OdometryPoseProvider(pilot);
//		
//		// Instantate a new OdometryCorrection and disable it
//		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
//		odo.setPose(new Pose(-15, -15, 90));
//		nav = new Navigator(pilot, odo);
//		moveController = new MovementController(nav);
//		odoCorrection.start();
//		moveController.travelToWaypoint(new Waypoint(Main.TILE_WIDTH, 2.5*Main.TILE_WIDTH, 0));
//		
//	}
	
	
	public static void main(String[] args){
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		pilot.setAcceleration(500);
		pilot.setTravelSpeed(22);
		pilot.setRotateSpeed(90);
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		
		// Instantate a new OdometryCorrection and disable it
		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
		
		display = new Display(odo);
		
		// Instantiate a new Localizer
		localizer = new Localizer(pilot, ULTRASONIC, odo);
		
		// Instantiate a new Navigator to control movement
		nav = new Navigator(pilot, odo);
		
		// Instantiate a new MovementController for travelling to waypoints
		moveController = new MovementController(nav);

		// Instantiate a new Arm for controlling claw movement
		arm = new Arm(ARM, Arm.ArmState.RAISED);
		
		// Instantiate a new blockRescuer
		blockRescuer = new BlockRescuer(pilot, nav, ULTRASONIC, arm);
		odoCorrection.start();
		
		
		display.start();
		
		Button.waitForAnyPress();
		
//		odo.setPose(new Pose(Main.TILE_WIDTH*1.5f, 0, -90));
//		blockRescuer.rescueBlock();
		odo.setPose(new Pose(Main.TILE_WIDTH*-.5f, Main.TILE_WIDTH*-.5f, 90));
		OdometryCorrection.enable();
		moveController.travelToTile(3, 3);
		moveController.travelToTile(3, 0);
		moveController.travelToTile(0, 0);
		moveController.travelToTile(3, 3);
	}
	
	
	public static void _main(String[] args) {
		setup();
		
		LCD.clear();
		
		for (int x = 0; x < Main.NUM_TILES; x++){
			for (int y = 0; y < Main.NUM_TILES; y++){
				if (getCurrentMap().get(x * Main.NUM_TILES + y))
					LCD.drawChar('X', x, Main.NUM_TILES - 1 - y);
			}
		}
		
		
		Button.waitForAnyPress();
		
		
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		pilot.setAcceleration(500);
		pilot.setTravelSpeed(22);
		pilot.setRotateSpeed(90);
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		
		// Instantate a new OdometryCorrection and disable it
		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
		
		display = new Display(odo);
		
		// Instantiate a new Localizer
		localizer = new Localizer(pilot, ULTRASONIC, odo);
		
		// Instantiate a new Navigator to control movement
		nav = new Navigator(pilot, odo);
		
		// Instantiate a new MovementController for travelling to waypoints
		moveController = new MovementController(nav);

		// Instantiate a new Arm for controlling claw movement
		arm = new Arm(ARM, Arm.ArmState.RAISED);
		
		// Instantiate a new blockRescuer
		blockRescuer = new BlockRescuer(pilot, nav, ULTRASONIC, arm);
		odoCorrection.start();
		display.start();
		
		Display.setCurrentAction(Display.Action.LOCALIZING);
		localizer.localize();
		
		OdometryCorrection.enable();
		Display.setCurrentAction(Display.Action.MOVING);
//		moveController.travelToWaypoint(new Waypoint(Main.TILE_WIDTH, 2.5*Main.TILE_WIDTH, 0));
		
		blockPickupArea();
		moveController.regenerate();
		moveController.travelToTile(1, 2, -90);
		unblockPickupArea();
		moveController.regenerate();
		pilot.travel(odo.getPose().getY() - Main.TILE_WIDTH);	// Move to top of current tile
		
		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
		blockRescuer.rescueBlock();
		
		Display.setCurrentAction(Display.Action.MOVING);
		moveController.travelToTile((int)dropoff.getX(), (int)dropoff.getY(), (float)dropoff.getHeading());

		pilot.travel(-Main.TILE_WIDTH/2f);
		                  
		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
		arm.drop();
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
	
	/**********************************************/
		// The following is from Lab 5 :
		maps[0].set(0*Main.NUM_TILES + 3, true);	// (0,3)
		maps[0].set(1*Main.NUM_TILES + 0, true);	// (1,0)
		maps[0].set(2*Main.NUM_TILES + 2, true);	// (2,2)
		maps[0].set(3*Main.NUM_TILES + 2, true);	// (3,2)
		//////////////////////////////
	/************************************************/
	/**********************************************
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
		
		
		// Initialization of map 3
		
		// Initialization of map 4
		
		// Initialization of map 5
	 **********************************************/
	}

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
			LCD.drawString("Map: " + mapNumber, 0, 0);
			option = Button.waitForAnyPress();
			switch (option){
			case Button.ID_LEFT: mapNumber = (mapNumber + (Main.NUM_MAPS - 1)) % Main.NUM_MAPS; break;
			case Button.ID_RIGHT: mapNumber = (mapNumber + 1) % Main.NUM_MAPS; break;
			default: break;
			}
			
		} while (option != Button.ID_ENTER);
		
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
		
		int ang = 0;
		
		if (x-1 >= 0 && !maps[mapNumber].get((x-1)*Main.NUM_TILES + y))
			ang = 0;
		else if (x + 1 < Main.NUM_TILES && !maps[mapNumber].get((x+1)*Main.NUM_TILES + y))
			ang = 180;
		else if (y - 1 >= 0 && !maps[mapNumber].get(x*Main.NUM_TILES + y - 1))
			ang = 90;
		else
			ang = -90;
		
		dropoff = new Waypoint(x, y, ang);
		
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
