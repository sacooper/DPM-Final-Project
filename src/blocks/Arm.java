package blocks;

import lejos.nxt.NXTRegulatedMotor;
import main.Main;

/*****
 * Class represnting the arm of the robot, controlling raising and lower, and dropping
 * of blocks.
 * 
 * @author Scott Cooper
 * @since v1
 *
 */
public class Arm {

	/****
	 * Enum representing the current state of the arm, preventing attempts
	 * to lower the arm when it is already lowered, or raised when already raised, 
	 * which may cause damage to the arm.
	 * 
	 * @since v1
	 * @author Scott Cooper
	 */
	public static enum ArmState{
		/**Arm is currently lowered, ready to pick up a block*/
		LOWERED, 
		
		/**Arm is currently raised*/
		RAISED}
	
	private NXTRegulatedMotor arm;		// Motor controlling the arm
	private ArmState armState;			// Current state of the arm
	
	/****
	 * Create a new arm in the raised position
	 * 
	 * @param arm The <code>NXTRegulatedMotor</code> controlling the arm
	 */
	public Arm(NXTRegulatedMotor arm){
		this(arm, ArmState.RAISED);
	}

	/****
	 * Create a new arm in the specified position
	 * 
	 * @param arm The <code>NXTRegulatedMotor</code> controlling the arm
	 * @param armState The current state of the arm
	 */
	public Arm(NXTRegulatedMotor arm, ArmState armState){
		this.arm = arm;
		this.armState = armState;
		this.arm.stop(true);
		this.arm.setSpeed(300);
	}
	
	/****
	 * Raise the arm
	 */
	public void raiseArm(){
		switch(this.armState){
		case RAISED:
			return;
		case LOWERED:
			arm.rotate(500);
			break;
		}
		this.armState = ArmState.RAISED;
	}
	
	/****
	 * Lower the arm
	 */
	public void lowerArm(){
		switch(this.armState){
		case RAISED:
			arm.rotate(-500);
			break;
		case LOWERED:
			return;
		}
		this.armState = ArmState.LOWERED;
	}

	/***
	 * Drop the block. Once the arm has been lowered, the
	 * robot will reverse, preventing picking up the block
	 * while raising the arm. If the arm is not
	 * currently raised, a RuntimeException is thrown.
	 */
	public void drop() {
		switch(this.armState){
		case RAISED:
			lowerArm();
			Main.getPilot().travel(-Main.TILE_WIDTH/2f);
			raiseArm();
			break;
		case LOWERED:
			throw new RuntimeException("Can't drop block");
		}
		this.armState = ArmState.RAISED;
	}
	
	/***
	 * Raise the arm while reversing. The robot will first
	 * grip the claw. Once a sufficient grip has been achieved,
	 * the robot will reverse back 10cm to prevent
	 * hitting objects while raising the block (i.e. the wall or other blocks)
	 * If the arm is not currently lowered, a RuntimeException is thrown
	 */
	public void raise_with_rev(){
		switch (this.armState){
		case RAISED: throw new RuntimeException("Can't raise arm");
		case LOWERED:
			arm.rotate(200);
			Main.getPilot().travel(-10);
			arm.rotate(300);
			break;
		}
		armState = ArmState.RAISED;
	}
}
