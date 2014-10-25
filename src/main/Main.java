package main;

import java.util.ArrayList;

import lejos.geom.Line;
import lejos.geom.Rectangle;
import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.FixedRangeScanner;
import lejos.robotics.RangeScanner;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import localization.Localizer;
import navigation.MovementController;
import blocks.BlockRescuer;

public class Main {

	public static final NXTRegulatedMotor 
		MOTOR_LEFT = Motor.A,
		ARM = Motor.B,
		MOTOR_RIGHT = Motor.C;
	
	public static final UltrasonicSensor ULTRASONIC = new UltrasonicSensor(SensorPort.S2);
	
	private static DifferentialPilot pilot;
	private static Navigator nav;
	private static RangeScanner scanner;
	private static MovementController moveController;
	private static BlockRescuer blockRescuer;
	private static OdometryPoseProvider odo;
	private static Localizer localizer;
	
	// TODO: Update wheel paramaters based on design
	public static final float	 
		LEFT_WHEEL_D = 4.28f,
		RIGHT_WHEEL_D = 4.28f,
		WHEEL_BASE = 17.75f,
		TILE_WIDTH = 30.48f;
	
	// TODO: Set to 12 for real maps
	public static final int
		NUM_TILES = 4;
	
	// List of maps for use in competition
	// usage: maps[map_number][x][y]
	private static final boolean[][][] maps;
	
	// Current LineMap. Map is used in both Localization and pathfinding in MovementController
	private static LineMap currentMap = null;

	private static Waypoint pickup = new Waypoint(0, 0, 0);

	private static Waypoint dropoff = new Waypoint(0, 0, 45);
	
	public static final Object POSE_LOCK = new Object();
	
	private Main(){};
	
	public static void main(String[] args) {
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		
		// Instantiate a new RangeScanner, for use in localization
		scanner = new FixedRangeScanner(pilot, ULTRASONIC);
		
		// Instantiate a new OdometryPoseProvider, of maintaining current pose
		odo = new OdometryPoseProvider(pilot);
		
		// Instantiate a new Localizer
		localizer = new Localizer(pilot, scanner, odo);
		
		// Instantiate a new Navigator to control movement
		nav = new Navigator(pilot, odo);
		
		// Instantiate a new MovementController for travelling to waypoints
		moveController = new MovementController(nav);

		// Instantiate a new blockRescuer
		blockRescuer = new BlockRescuer(pilot, ULTRASONIC, ARM);
		
		// moveController.travelToWaypoint(new Waypoint(75, 75, 0));
		
		Display.setCurrentAction(Display.Action.LOCALIZING);
		localizer.localize();
		
		Display.setCurrentAction(Display.Action.MOVING);
		moveController.travelToWaypoint(pickup);
		
		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
		blockRescuer.rescueBlock();
		
		Display.setCurrentAction(Display.Action.MOVING);
		moveController.travelToWaypoint(dropoff);
		
		Display.setCurrentAction(Display.Action.BLOCK_ACTION);
		blockRescuer.dropBlock();
		blockRescuer.resetArm();

	}

	public static DifferentialPilot getPilot() {
		return pilot;
	}

	public static int getMapNumber() {
		return 0;
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
							float right_x = ((x + 0.5f) * Main.TILE_WIDTH);
							float left_x = ((x - 0.5f) * Main.TILE_WIDTH);
							float bottom_y = ((y -0.5f) * Main.TILE_WIDTH);
							float top_y = ((y + 0.5f) * Main.TILE_WIDTH);
							
							Line bottom = new Line(left_x, right_x, bottom_y, bottom_y);
							Line top = new Line(left_x, right_x, top_y, top_y);
							Line left = new Line(left_x, left_x, bottom_y, top_y);
							Line right = new Line(right_x, right_x, bottom_y, top_y);
							
							lines.add(bottom);
							lines.add(top);
							lines.add(left);
							lines.add(right);
						}
					}
				}

				Line[] lineArray = new Line[lines.size()];
				lineArray = lines.toArray(lineArray);
				
				currentMap = new LineMap(
						lineArray,
						new Rectangle(
								(float)(Main.TILE_WIDTH * -0.5), // top left x
								(float)(Main.TILE_WIDTH * (Main.NUM_TILES - 0.5)), // top right y
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
		
		maps = new boolean[6][Main.NUM_TILES][Main.NUM_TILES];
		
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
