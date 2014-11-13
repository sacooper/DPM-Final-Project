package blocks;

import lejos.nxt.LCD;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import main.Main;
import navigation.OdometryCorrection;

/*****
 * Block rescuer class. All movement of the arm, and finding and 'rescuing' of block occurs here.
 * 
 * @author Scott Cooper
 *
 */
public class BlockRescuer {	
	private DifferentialPilot pilot;
	private final double WIDTH = Main.TILE_WIDTH, HEIGHT = 2 * Main.TILE_WIDTH;
	private final int THRESHOLD = 2;
	private UltrasonicSensor us;
	private OdometryPoseProvider odo;
	private Arm arm;
	
	/*****
	 * Instantiate a new BlockRescuer with the following paramaters
	 * 
	 * @param pilot The <code>DifferentialPilot</code> controlling movement
	 * @param us The <code>UltrasonicSensor</code> to use for detecting a block
	 * @param arm The <code>Arm</code> that controls the claw
	 */
	public BlockRescuer(DifferentialPilot pilot, OdometryPoseProvider odo, UltrasonicSensor us, Arm arm){
		this.pilot = pilot;
		this.us = us;
		this.odo = odo;
		this.arm = arm;
	}
	
	/***
	 * Rescue a block. It is assumed that the robot is currently
	 * at the waypoint specified for the dropoff zone.
	 */
	public void rescueBlock(){
		double old_r = pilot.getRotateSpeed(), old_t = pilot.getTravelSpeed();

		OdometryCorrection.disable();
		us.continuous();
		pilot.setTravelSpeed(7);
		pilot.setRotateSpeed(35);
		// Stage 1: Find block
		int dist = searchForBlock();
		// Stage 2: Pick up block
		arm.lowerArm();
		pilot.travel(dist);
		arm.raiseArm();
		OdometryCorrection.enable();
		pilot.setRotateSpeed(old_r);
		pilot.setTravelSpeed(old_t);
	}
	
	/***
	 * Search for a block, and once found, orient ourselves to pick it up.
	 * 
	 * @return The distance from the block
	 */
	private int searchForBlock(){
		// TODO: FIND OPTIMAL ROTATE SPEED
		pilot.rotate(-odo.getPose().getHeading());
		pilot.travel(WIDTH/2f);
		pilot.rotate(90);
		int dist = 0;
		int count = 0;
		boolean foundBlock = false;
		while (!foundBlock && count < HEIGHT/(Main.TILE_WIDTH / 3f)){
			pilot.rotate(45);
			int lastDistance = getFilteredData();
			int current = lastDistance;
			pilot.rotate(-90, true);
			while(pilot.isMoving()){
				current = getFilteredData();
				LCD.clear();
				LCD.drawInt(lastDistance - current, 0, 0);
				if (lastDistance - current > THRESHOLD){
					// FOUND BLOCK
					pilot.stop();
					foundBlock = true;
					dist = current;
					break;
				}
				lastDistance = current;
			}
			if (foundBlock) break;
			count++;
			pilot.rotate(45);
			pilot.travel(Main.TILE_WIDTH/3f);
		}
		pilot.rotate(-30);
		pilot.travel(-10);
		return dist+10;
	}

	/*******
	 * Get a value from the ultrasonic sensor for the current distance from the
	 * wall
	 * 
	 * @return A filtered value of the distance from the wall
	 */
	private int getFilteredData() {
		int dist;
		// there will be a delay here
		dist = us.getDistance();
		
		return (int) Math.min(dist, Main.TILE_WIDTH);
	}
}
