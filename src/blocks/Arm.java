package blocks;

import lejos.nxt.NXTRegulatedMotor;
import main.Main;

/*****
 * Class represnting the arm of the robot, controlling raising and lower, and dropping
 * of blocks.
 * 
 * @author Scott Cooper
 *
 */
public class Arm {

	/****
	 * Current state of the arm
	 * 
	 * @author Scott Cooper
	 */
	public static enum ArmState{
		/**Arm is currently lowered, ready to pick up a block*/
		LOWERED, 
		
		/**Arm is currently raised*/
		RAISED}
	
	private NXTRegulatedMotor arm;
	private ArmState armState;
	
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

	public void drop() {
		lowerArm();
		Main.getPilot().travel(-Main.TILE_WIDTH/2f);
		raiseArm();
	}
	
	public void raise_with_rev(){
		switch (this.armState){
		case RAISED: throw new RuntimeException("Can't raise arm");
		case LOWERED:
			arm.rotate(200);
			Main.getPilot().travel(-10);
			arm.rotate(300);
		}
		armState = ArmState.RAISED;
	}
}
