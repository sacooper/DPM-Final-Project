package localization;

import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.robotics.Color;
import lejos.robotics.localization.OdometryPoseProvider;
import main.Main;

/*******************
 * Team 2
 * 
 * @author Scott Cooper <br>
 *         A class to correct whatever odometer is passed into it based on the
 *         known distance between grid lines. When the color sensor detects a
 *         line, a correction is made, based on the current and previous angles
 *         of the robot.
 */
public class OdometryCorrection extends Thread {

	// /**********
	// * Class to contain the correction information */
	// private static final class Correction{
	// /* Enum to contain if we are increasing or decreasing the correction */
	// public static enum Change{PLUS, MINUS};
	//
	// // An option of the last value we had
	// private Option<Double> last;
	//
	// // What type of change we last had
	// private Option<Change> change;
	//
	// /**
	// * Instantiate a new correction with values of "none"
	// * for the last value and in which direction we're correcting
	// */
	// public Correction(){
	// last = Option.none();
	// change = Option.none();}
	//
	// // Getters and setters for the type of correction and last value (may be
	// "none")
	// public Option<Change> getChange() {return change;}
	//
	// public void setChange(Change change) {this.change.toSome(change);}
	//
	// public Option<Double> getLast(){return this.last;}
	//
	// public void setLast(Double last){this.last.toSome(last);}}

	// Correction period and how long to wait before looking for another line
	private static final long CORRECTION_PERIOD = 10, WAIT = 500;

	// The threshold of the difference in values that constitutes seeing a lien
	private static final int THRESHOLD = 11;

	/*
	 * The difference in the last value and the current value after a switch
	 * from Correction.Change.PLUS to Correctoin.Change.MINUS (or vice-versa).
	 * This is required because the rotation point is NOT at the color sensor,
	 * thus seeing a line doesn't necessarily mean that the robot is at the same
	 * spot as when it saw the line going the other direction (in fact, it
	 * DEFINATELLY will not be)
	 */
	private static final double CHANGE = 4.5;

	// Member variables for the correction
	private OdometryPoseProvider odo;
	private ColorSensor cs_left, cs_right;

	// Whether or not we've disabled the correction (disabled when turning)
	private boolean disabled;

	/*******
	 * Instantiate a new OdometryCorrection with the odometer it is to correct
	 * 
	 * @param odo
	 *            Odometer to correct
	 */
	public OdometryCorrection(OdometryPoseProvider odo, ColorSensor cs_left,
			ColorSensor cs_right) {
		this.cs_left = cs_left;
		this.cs_right = cs_right;
		this.odo = odo;
	}

	// run method (required for Thread)
	public void run() {
		/*
		 * The following variables are declard here to prevent re-allocaation
		 * each iteration of while(true){...}
		 */
		long correctionStart; // When we started the current correction period

		int newColor_left, newColor_right, lastColor_left = -1, lastColor_right = -1;

		boolean sawLine_left = false, sawLine_right = false; // Whether or not we saw a line

		double x_l, y_l, x_r, y_r, t_l, t_r;

		// The ColorSensor best saw lines with this color
		cs_left.setFloodlight(Color.GREEN);
		cs_right.setFloodlight(Color.GREEN);

		while (true) {
			correctionStart = System.currentTimeMillis();
			newColor_left = cs_left.getNormalizedLightValue();
			newColor_right = cs_right.getNormalizedLightValue();
			sawLine_left = false;

			if (lastColor_left != -1 && lastColor_left - newColor_left > THRESHOLD && !disabled && !sawLine_left) {
				sawLine_left = true;

				synchronized (Main.POSE_LOCK) {
					x_l = odo.getPose().getX();
					y_l = odo.getPose().getY();
					t_l = odo.getPose().getHeading();
				}
			}
			lastColor_left = newColor_left;

			if (lastColor_right != -1 && lastColor_right - newColor_right > THRESHOLD && !disabled && !sawLine_right){
				sawLine_right = true;
				
				synchronized(Main.POSE_LOCK){
					x_r = odo.getPose().getX();
					y_r = odo.getPose().getY();
					t_r = odo.getPose().getHeading();
				}
			}
			lastColor_right = newColor_right;
			
			if (sawLine_left && sawLine_right){
				sawLine_left = false;
				sawLine_right = false;
				
				// perform correction...
				
			}
			
			
			long diff = System.currentTimeMillis() - correctionStart;
			if (diff < CORRECTION_PERIOD) {
				try {
					if (sawLine_left)
						Thread.sleep(WAIT - diff);
					else
						Thread.sleep(CORRECTION_PERIOD - diff);
				} catch (InterruptedException e) {
				}
			}
		}
	}

	/***
	 * Enable OdometryCorrection
	 */
	public void enable() {
		disabled = false;
	}

	/****
	 * Disable OdometryCorrection
	 */
	public void disable() {
		disabled = true;
	}
}