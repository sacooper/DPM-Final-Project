package main;

import java.util.ArrayList;

import lejos.geom.Line;
import lejos.geom.Rectangle;
import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import localization.Localizer;
import localization.OdometryCorrection;
import navigation.MovementController;
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
	
	// TODO: Update wheel paramaters based on design
	public static final float	 
		LEFT_WHEEL_D = 4.2065f,
		RIGHT_WHEEL_D = 4.2065f,
		WHEEL_BASE = 17.65f,
		TILE_WIDTH = 30.48f;
	
	// TODO: Set to 12 for real maps
	public static final int
		NUM_TILES = 4;
	
	// List of maps for use in competition
	// usage: maps[map_number][x][y]
	private static final boolean[][][] maps;

	private static final int NUM_MAPS = 6;
	
	// Current LineMap. Map is used in both Localization and pathfinding in MovementController
	private static LineMap currentMap = null;

	private static Waypoint pickup = new Waypoint(0, 0, 0);

	private static Waypoint dropoff = new Waypoint(0, 0, 45);
	
//	public static final Object POSE_LOCK = odo;
	
	private Main(){};
	
	public static void main(String[] args){
//

		MOTOR_LEFT.setAcceleration(2000);
		MOTOR_RIGHT.setAcceleration(2000);
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
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

		odo.setPose(new Pose(-15, -20, 90));
		
		pilot.travel(Main.TILE_WIDTH, false);
		
	}
	
//	public static void main_primary(String[] args) {
//		MOTOR_LEFT.setAcceleration(1000);
//		MOTOR_RIGHT.setAcceleration(1000);
//		// Instantiate a new DifferentialPilot to control movement
//		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
//		
//		// Instantiate a new OdometryPoseProvider, of maintaining current pose
//		odo = new OdometryPoseProvider(pilot);
//		
//		// Instantate a new OdometryCorrection and disable it
//		odoCorrection = new OdometryCorrection(odo, COLORSENSOR_LEFT, COLORSENSOR_RIGHT);
//		OdometryCorrection.disable();
//		odoCorrection.start();
//		
//		display = new Display(odo);
//		
//		// Instantiate a new Localizer
//		localizer = new Localizer(pilot, ULTRASONIC, odo);
//		
//		// Instantiate a new Navigator to control movement
//		nav = new Navigator(pilot, odo);
//		
//		// Instantiate a new MovementController for travelling to waypoints
//		moveController = new MovementController(nav);
//
//		// Instantiate a new blockRescuer
//		blockRescuer = new BlockRescuer(pilot, ULTRASONIC, ARM);
//		
//		// moveController.travelToWaypoint(new Waypoint(75, 75, 0));
//		
//		display.start();
//		
//		Display.setCurrentAction(Display.Action.LOCALIZING);
//		localizer.localize();
//		
//		Display.setCurrentAction(Display.Action.MOVING);
//		moveController.travelToWaypoint(pickup);
//		
//		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
//		blockRescuer.rescueBlock();
//		
//		Display.setCurrentAction(Display.Action.MOVING);
//		moveController.travelToWaypoint(dropoff);
//		
//		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
//		blockRescuer.dropBlock();
//		blockRescuer.raiseArm();
//
//	}

	public static DifferentialPilot getPilot() {
		return pilot;
	}

	public static int getMapNumber() {
		return 0;
	}
	
	public static boolean[][] getBoolMap(){
		return maps[getMapNumber()];
	}
	/****
	 * Get the LineMap for the Localizer
	 * based on a given map. The number should be in the range
	 * [0,5], representing one of the 6 maps.
	 * @return A RangeMap representing the given map
	 */
	public static LineMap getMap() {
		if (currentMap == null){
			ArrayList<Line> lines = new ArrayList<Line>();
			int mapNumber = Main.getMapNumber();
			if (mapNumber >= 0 && mapNumber < 6){
				for (int x = 0; x < Main.NUM_TILES; x++){
					for (int y = 0; y < Main.NUM_TILES; y++){
						if (maps[mapNumber][x][y]){	// Check if this tile is blocked
							float left_x = ((x - 1) * Main.TILE_WIDTH);
							float right_x = (x * Main.TILE_WIDTH);
							float bottom_y = ((y - 1) * Main.TILE_WIDTH);
							float top_y = (y * Main.TILE_WIDTH);
							
							Line bottom = new Line(left_x, bottom_y, right_x, bottom_y);
							Line top = new Line(right_x, top_y, left_x, top_y);
							Line left = new Line(left_x, top_y, left_x, bottom_y);
							Line right = new Line(right_x, bottom_y, right_x, top_y);
							
							lines.add(bottom);
							lines.add(right);
							lines.add(top); 
							lines.add(left);
							
						}
					}
				}

				Line[] lineArray = new Line[lines.size()];
				lineArray = lines.toArray(lineArray);
				
				currentMap = new LineMap(
						lineArray,
						new Rectangle(
								(float)(Main.TILE_WIDTH * -1), // top left x
//								(float)(Main.TILE_WIDTH * (Main.NUM_TILES - 1)), // top left y
								(float)(Main.TILE_WIDTH * (- 1)), // top left y
								(float)(Main.TILE_WIDTH * Main.NUM_TILES), 	 // height
								(float)(Main.TILE_WIDTH * Main.NUM_TILES))); // width
			} else throw new RuntimeException("Invalid Map Number");
		}
		
		return currentMap;
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
		
		maps = new boolean[Main.NUM_MAPS][Main.NUM_TILES][Main.NUM_TILES];
		
		for (int i = 0; i < 6; i++){
			for (int x = 0; x < Main.NUM_TILES; x++){
				for (int y = 0; y < Main.NUM_TILES; y++){
					maps[i][x][y] = false;}}}
		
		
		// TODO: Initialization of maps
		
		// The following is from Lab 5 :
		maps[0][0][3] = true;		// <
		maps[0][1][0] = true;		// <
		maps[0][2][2] = true;		// <
		maps[0][3][2] = true;	 	// <
		////////////////////////////////

		// Initialization of map 1
		
		// Initialization of map 2
		
		// Initialization of map 3
		
		// Initialization of map 4
		
		// Initialization of map 5
	}
	
}