package blocks;

import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Waypoint;

public class BlockRescuer {
	private static enum ArmState{LOWERED, RAISED, DROPPED}
	
	private DifferentialPilot pilot;
	private UltrasonicSensor us;
	private final NXTRegulatedMotor arm;
	private ArmState armState;
	
	public BlockRescuer(DifferentialPilot pilot, UltrasonicSensor us, NXTRegulatedMotor arm){
		this.pilot = pilot;
		this.us = us;
		this.arm = arm;
		this.armState = ArmState.RAISED;
	}
	
	public void rescueBlock(){
		// Stage 1: Find block
		Waypoint blockDest = searchForBlock();
		
		double dx, dy;
		
		// Stage 2: Move to block and position ourselves to pick it up
		
		// Stage 3: Pick up block
		this.lowerArm();
		pilot.travel(10);
		this.raiseArm();
	}
	
	private Waypoint searchForBlock(){
		return new Waypoint(0, 0);
	}
	
	public boolean dropBlock(){
		// TODO: Check constants for dropBlock()
		switch(this.armState){
		case RAISED:
			arm.rotate(-220);
			this.armState = ArmState.DROPPED;
			return true;
		default:
			return false;
		}
	}
	
	public void raiseArm(){
		// TODO: Check constants for raiseArm()
		switch(this.armState){
		case RAISED:
			return;
		case DROPPED:
			arm.rotate(-260);
			break;
		case LOWERED:
			arm.rotate(-400);
			break;
		}
		this.armState = ArmState.RAISED;
	}
	
	public void lowerArm(){
		// TODO: Check constants for lowerArm()
		switch(this.armState){
		case RAISED:
			arm.rotate(400);
			break;
		case DROPPED:
			arm.rotate(260);
			break;
		case LOWERED:
			return;
		}
		this.armState = ArmState.RAISED;
	}
}
