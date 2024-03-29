package blocks;

import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import main.Main;
import navigation.OdometryCorrection;

/*****
 * Block rescuer class. All movement of the arm, and finding and 'rescuing' of block occurs here.
 * 
 * @author Scott Cooper
 * @since v0
 */
public class BlockRescuer {	
	private DifferentialPilot pilot;
	private final int THRESHOLD = 2, SWEEP = 35;
	private UltrasonicSensor us;
	private Arm arm;
	private Navigator nav;
	
	/*****
	 * Instantiate a new BlockRescuer with the following paramaters
	 * 
	 * @param pilot The <code>DifferentialPilot</code> controlling movement
	 * @param us The <code>UltrasonicSensor</code> to use for detecting a block
	 * @param arm The <code>Arm</code> that controls the claw
	 */
	public BlockRescuer(DifferentialPilot pilot, Navigator nav, UltrasonicSensor us, Arm arm){
		this.pilot = pilot;
		this.us = us;
		this.arm = arm;
		this.nav = nav;
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
		
		pilot.arc(-Main.WHEEL_BASE/2f, -30);
		pilot.arc(Main.WHEEL_BASE/2f, 60);
		pilot.arc(-Main.WHEEL_BASE/2f, -30);
		pilot.travel(6);
		
		arm.raise_with_rev();
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
		int dist = 0;
		int count = 0;
		boolean foundBlock = false;
		byte tryCount = 0;
		int ang = 0;
		while (!foundBlock){
			if (tryCount < 2){
				while (!foundBlock && count < 6) {
					
					if (getFilteredData() < 6){
						foundBlock = true;
						ang = 35;
						dist = 6;
						break;
					}
					
					pilot.rotate(SWEEP);
					int lastDistance = getFilteredData();
					int current = lastDistance;
					ang = 0;
					while(ang < SWEEP*2){
						pilot.rotate(-5);
						ang += 5;
						current = getFilteredData();
						if (lastDistance - current > THRESHOLD || current < 6){
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
					pilot.travel(Main.TILE_WIDTH/4f);
				}
			}
			if (foundBlock)	break;
			
			tryCount++;
			if (tryCount < 2){
				pilot.travel(-count * Main.TILE_WIDTH/4f);
				pilot.rotate(-90);
				pilot.travel(Main.TILE_WIDTH);
				pilot.rotate(90);
				count = 0;}
			else{
				pilot.travel(-count * Main.TILE_WIDTH/4f);
				pilot.rotate(90);
				pilot.travel(Main.TILE_WIDTH/2f);
				pilot.rotate(-90);
				return (int) (Main.TILE_WIDTH*1.5f);
			}
		}
		
		if (ang < 25)	// 0, 5, 10, 15, 20
			nav.rotateTo(-80);
		else if (ang < 50 ) // 20, 25, 30, 35, 40, 45
			nav.rotateTo(-90);
		else	// 50, 55, 60, 65, 70
			nav.rotateTo(-100);
		pilot.travel(-24);
		return dist + 15;
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
