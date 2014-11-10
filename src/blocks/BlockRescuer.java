package blocks;

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
	private final int THRESHOLD = 3;
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
		OdometryCorrection.disable();
		// Stage 1: Find block
		int dist = searchForBlock();
		// Stage 2: Pick up block
		arm.lowerArm();
		pilot.travel(dist);
		arm.raiseArm();
		OdometryCorrection.enable();
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
		while (!foundBlock && count < HEIGHT/(Main.TILE_WIDTH / 2)){
			pilot.rotate(45);
			int lastDistance = getFilteredData();
			int current = lastDistance;
			pilot.rotate(-90, true);
			while(pilot.isMoving()){
				current = getFilteredData();
				if (lastDistance - current > THRESHOLD){
					// FOUND BLOCK
					pilot.quickStop();
					foundBlock = true;
					dist = current;
					break;
				}
				lastDistance = current;
			}
			if (foundBlock) break;
			count++;
			pilot.rotate(45);
			pilot.travel(Main.TILE_WIDTH/2f);
		}
		return dist;
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
		us.ping();
		// wait for the ping to complete
		try {
			Thread.sleep(25);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		dist = us.getDistance();
		if (dist > Main.TILE_WIDTH/2)
			dist = (int) (Main.TILE_WIDTH/2);
		return dist;
	}
}
