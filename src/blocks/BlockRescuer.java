package blocks;

import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Waypoint;

/*****
 * Block rescuer class. All movement of the arm, and finding and 'rescuing' of block occurs here.
 * 
 * @author Scott Cooper
 *
 */
public class BlockRescuer {	
	private DifferentialPilot pilot;
	private UltrasonicSensor us;
	private Arm arm;
	
	/*****
	 * Instantiate a new BlockRescuer with the following paramaters
	 * 
	 * @param pilot The <code>DifferentialPilot</code> controlling movement
	 * @param us The <code>UltrasonicSensor</code> to use for detecting a block
	 * @param arm The <code>Arm</code> that controls the claw
	 */
	public BlockRescuer(DifferentialPilot pilot, UltrasonicSensor us, Arm arm){
		this.pilot = pilot;
		this.us = us;
		this.arm = arm;
	}
	
	/***
	 * Rescue a block. It is assumed that the robot is currently
	 * at the waypoint specified for the dropoff zone.
	 */
	public void rescueBlock(){
		// Stage 1: Find block
		Waypoint blockDest = searchForBlock();
		
		double dx, dy;
		
		// Stage 2: Move to block and position ourselves to pick it up
		
		// Stage 3: Pick up block
		arm.lowerArm();
		pilot.travel(10);
		arm.raiseArm();
	}
	
	/***
	 * Search for a block, and return the waypoint of the
	 * block as seen by the robot.
	 * @return A Waypiont representing the block.
	 */
	private Waypoint searchForBlock(){
		return new Waypoint(0, 0);
	}

}
