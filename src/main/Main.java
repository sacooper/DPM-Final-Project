package main;

import java.util.BitSet;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;
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
 * Main class contianing all constants. Primary control of the robot occurs here.
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
	
	public static final float	 
		LEFT_WHEEL_D = 4.155f,
		RIGHT_WHEEL_D = 4.155f,
		WHEEL_BASE = 17.525f,
		TILE_WIDTH = 30.48f;
	
	// TODO: Set to 12 for real maps
	public static final int
		NUM_TILES = 4;
	
	// List of maps for use in competition
	// usage: maps[map_number][x][y]
	private static final BitSet[] maps;

	private static final int NUM_MAPS = 6;
	private static Waypoint pickup = new Waypoint(0, 0, 0);
	private static Waypoint dropoff = new Waypoint(0, 0, 45);
	
	
	private Main(){};
	
	public static void main(String[] args){
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		pilot.setAcceleration(2000);
		pilot.setTravelSpeed(22);
		pilot.setRotateSpeed(70);
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		display = new Display(odo);
		display.start();
		
		localizer = new Localizer(pilot, ULTRASONIC, odo);
		arm = new Arm(ARM);
		blockRescuer = new BlockRescuer(pilot, odo, ULTRASONIC, arm);
		Button.waitForAnyPress();
		
//		blockRescuer.rescueBlock();
		
		localizer.localize();
		
		Button.waitForAnyPress();
		
	}
	
	
	
	public static void odo_test(String[] args){


		MOTOR_LEFT.setAcceleration(2000);
		MOTOR_RIGHT.setAcceleration(2000);
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		pilot.setAcceleration(2000);
		pilot.setTravelSpeed(24);
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		
		// Instantate a new OdometryCorrection and disable it
		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
		OdometryCorrection.disable();
		odoCorrection.start();
		
		display = new Display(odo);
		display.start();
		// Instantiate a new Localizer
		localizer = new Localizer(pilot, ULTRASONIC, odo);
		
		// Instantiate a new Navigator to control movement
		nav = new Navigator(pilot, odo);
		
		// Instantiate a new MovementController for travelling to waypoints
		moveController = new MovementController(nav);
		
		Button.waitForAnyPress();
		
		OdometryCorrection.enable();

		odo.setPose(new Pose(-15, -15, 90));
		
		pilot.travel(Main.TILE_WIDTH, false);
		
	}
	
	public static void _main(String[] args) {
		MOTOR_LEFT.setAcceleration(1000);
		MOTOR_RIGHT.setAcceleration(1000);
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		
		// Instantate a new OdometryCorrection and disable it
		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
		OdometryCorrection.disable();
		odoCorrection.start();
		
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
		blockRescuer = new BlockRescuer(pilot, odo, ULTRASONIC, arm);
		
		// moveController.travelToWaypoint(new Waypoint(75, 75, 0));
		
		display.start();
		
		Display.setCurrentAction(Display.Action.LOCALIZING);
		localizer.localize();
		
		Display.setCurrentAction(Display.Action.MOVING);
		moveController.travelToWaypoint(pickup);
		
		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
		blockRescuer.rescueBlock();
		
		Display.setCurrentAction(Display.Action.MOVING);
		moveController.travelToWaypoint(dropoff);
		
		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
		arm.dropBlock();
		arm.raiseArm();

	}

	/****
	 * Get the number of the current map
	 * 
	 * @return the current map number
	 */
	public static int getMapNumber() {
		return 5;
	}
	
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
		
		for (int i = 0; i < 6; i++){
			for (int x = 0; x < Main.NUM_TILES; x++){
				for (int y = 0; y < Main.NUM_TILES; y++){
					maps[i] = new BitSet(Main.NUM_TILES * Main.NUM_TILES);
					maps[i].clear();}}}
		
		
		// TODO: Initialization of maps
		
		// The following is from Lab 5 :
		maps[5].set(0*Main.NUM_TILES + 3, true);	// (0,3)
		maps[5].set(1*Main.NUM_TILES + 0, true);	// (1,0)
		maps[5].set(2*Main.NUM_TILES + 2, true);	// (2,2)
		maps[5].set(3*Main.NUM_TILES + 2, true);	// (3,2)
		////////////////////////////////

	/**********************************************	
		// The follow are for beta demonstrations
		// Initialization of map 0
		maps[0].set(7 * Main.NUM_MAPS + 0); // (7,0)
		maps[0].set(4*Main.NUM_TILES + 1);	// (4,1)
		maps[0].set(4*Main.NUM_TILES + 2);	// (4,2)
		maps[0].set(4*Main.NUM_TILES + 3);	// (4,3)
		maps[0].set(6*Main.NUM_TILES + 2);	// (6,2)
		maps[0].set(7*Main.NUM_TILES + 2);	// (7,2)
		maps[0].set(7*Main.NUM_TILES + 3);	// (7,3)
		maps[0].set(0*Main.NUM_TILES + 5);	// (0,5)
		maps[0].set(2*Main.NUM_TILES + 4);	// (2,4)
		maps[0].set(3*Main.NUM_TILES + 5);	// (3,5)
		maps[0].set(2*Main.NUM_TILES + 6);	// (2,6)
		maps[0].set(2*Main.NUM_TILES + 7);	// (2,7)
		maps[0].set(1*Main.NUM_TILES + 7);	// (1,7)
		maps[0].set(6*Main.NUM_TILES + 5);	// (6,5)
		maps[0].set(7*Main.NUM_TILES + 6);	// (7,6)
		
		// Initialization of map 1
		
		// Initialization of map 2
		
		// Initialization of map 3
		
		// Initialization of map 4
		
		// Initialization of map 5
	 **********************************************/
	}

	public static DifferentialPilot getPilot() {
		return pilot;
	}
	
}
