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
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import localization.FullPoseProvider;
import localization.Localizer;

public class Main {

	public static final NXTRegulatedMotor 
		MOTOR_LEFT = Motor.A,
		ARM = Motor.B,
		MOTOR_RIGHT = Motor.C;
	
	private static final UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
	
	private static DifferentialPilot pilot;
	private static Navigator nav;
	private static RangeScanner scanner;
	private static FullPoseProvider poseProvider;
	
	// TODO: UPDATE WHEEL_BASE VALUES BASED ON DESIGN
	public static final double	 
		LEFT_WHEEL_D = 6.28,
		RIGHT_WHEEL_D = 6.28,
		WHEEL_BASE = 15.8,
		TILE_WIDTH = 30.48;
	
	// TODO: SET TO 12 FOR FINAL COMPETITION
	public static final int
		NUM_TILES = 4;
	
	// List of maps for use in competition
	// usage: maps[map_number][x][y]
	private static final boolean[][][] maps;
	
	// Current LineMap. Map is used in both Localization and pathfinding in MovementController
	private static LineMap currentMap = null;
	
	public static void main(String[] args) {
		// Instantiate a new DifferentialPilot to control movement
		pilot = new DifferentialPilot(LEFT_WHEEL_D, RIGHT_WHEEL_D, WHEEL_BASE, MOTOR_LEFT, MOTOR_RIGHT, false);
		
		// Instantiate a new RangeScanner, for use in localization
		scanner = new FixedRangeScanner(pilot, us);
		
		// Create a full pose provider, containging both an odometer and a localizer
		poseProvider = new FullPoseProvider(pilot, scanner, false);
		
		// Create a new navigator
		nav = new Navigator(pilot, poseProvider);
		
		int option = Button.waitForAnyPress();
		if (option == Button.ID_ESCAPE) System.exit(0);
		
		// Add button listener to escape button to allow for exiting at any time
		Button.ESCAPE.addButtonListener(
				new ButtonListener(){
					@Override
					public void buttonPressed(Button b) {
						if (b.getId() == Button.ID_ESCAPE) System.exit(0);}
		
					@Override
					public void buttonReleased(Button b) {}});
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
							float right_x = (float) ((x + 0.5) * Main.TILE_WIDTH);
							float left_x = (float) ((x - 0.5) * Main.TILE_WIDTH);
							float bottom_y = (float) ((y -0.5) * Main.TILE_WIDTH);
							float top_y = (float) ((y + 0.5) * Main.TILE_WIDTH);
							
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
				
				currentMap = new LineMap(
						(Line[])lines.toArray(), 
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
		maps = new boolean[6][Main.NUM_TILES][Main.NUM_TILES];
		
		for (int i = 0; i < 6; i++){
			for (int x = 0; x < Main.NUM_TILES; x++){
				for (int y = 0; y < Main.NUM_TILES; y++){
					maps[i][x][y] = false;}}}
		
		// TODO: Initialization of map 0
		
		// TODO: Initialization of map 1
		
		// TODO: Initialization of map 2
		
		// TODO: Initialization of map 3
		
		// TODO: Initialization of map 4
		
		// TODO: Initialization of map 5
	}
	
}
