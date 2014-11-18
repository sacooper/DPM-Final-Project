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
	private final int THRESHOLD = 2, SWEEP = 35;
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
//		OdometryCorrection.enable();
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
		int dist = 0;
		int count = 0;
		boolean foundBlock = false;
		byte tryCount = 0;
		while (!foundBlock){
			if (tryCount < 2){
				while (!foundBlock && count < 4) {
					pilot.rotate(SWEEP);
					int lastDistance = getFilteredData();
					int current = lastDistance;
					pilot.rotate(-2 * SWEEP, true);
					while(pilot.isMoving()){
						current = getFilteredData();
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
					pilot.rotate(SWEEP);
					pilot.travel(Main.TILE_WIDTH/3f);
				}
			}
			if (foundBlock)	break;
			
			tryCount++;
			if (tryCount < 2){
				pilot.travel(-count * Main.TILE_WIDTH);
				pilot.rotate(-90);
				pilot.travel(Main.TILE_WIDTH);
				pilot.rotate(90);
				count = 0;}
			else{
				pilot.travel(-count * Main.TILE_WIDTH);
				pilot.rotate(90);
				pilot.travel(Main.TILE_WIDTH/2f);
				pilot.rotate(-90);
				pilot.travel(Main.TILE_WIDTH*1.5f);
				pilot.travel(-10);
				return 5;
			}
			
		}
		pilot.rotate(-15);
		pilot.travel(-15);
		return dist+7;
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
		
		return (int) Math.min(dist, Main.TILE_WIDTH*.75);
	}
}
