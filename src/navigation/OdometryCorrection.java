package navigation;

import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.robotics.Color;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
import main.Display;
import main.Main;

/**
 * The <code>OdometryCorrection</code> class corrects the odometry whenever one of the two 
 * color sensors of the robot crosses a grid line. 
 * 
 * It has a private method which determines which grid line is closest to a given coordinate.
 * 
 * An instance of this class holds an instance of the <code>OdometryPoseProvider</code> class which it updates when
 * a grid line is crossed.
 * 
 * @see OdometeryPoseProvider
 * 
 * @author Scott Cooper
 *
 */

public class OdometryCorrection extends Thread {
	private static final byte CORRECTION_PERIOD = 5;
	private static boolean enabled;
	private final static double X_OFFSET = 4, // 7.3
								Y_OFFSET = 7, // 7.3
								THRESHOLD = 25;

	private OdometryPoseProvider odometer;
	private ColorSensor leftCS, rightCS;
	

	/**
	 * The constructor of the <code>OdometryCorrection</code> initializes instances of the class <code>OdometeryPoseProvider</code>.
	 * 
	 * @param odometer 	The <code>OdometeryPoseProvider</code> that is used for the robot
	 * @param leftCS 	The left <code>ColorSensor</code> that is used to check grid lines
	 * @param rightCS 	The right <code>ColorSensor</code>that is used to check grid lines
	 */
	public OdometryCorrection(OdometryPoseProvider odometer, ColorSensor leftCS, ColorSensor rightCS) {		
		this.odometer = odometer;
		this.leftCS = leftCS;
		this.rightCS = rightCS;
		enabled = false;
	}


	/** If the <code>boolean isTurning</code> from the <code>Navigation</code> is false,
	 * check if either of the two <code>ColorSensors</code> crosses a grid line. If one does, 
	 * determine which grid line is closest and update the <code>Odometer</code> accordingly.
	 * 
	 * {@inheritDoc}
	 */
	public void run() {
		// Variables
		double tempAngle = 0, XError, YError;
		long correctionStart, correctionEnd;

		// 	Calculate the hypotenuse, as well as the angle offset for both the left and right
		//	ColorSensors.
		final double hypotenuse = Math.sqrt(X_OFFSET*X_OFFSET + Y_OFFSET*Y_OFFSET);
		final double leftOffset = - Math.atan(Y_OFFSET/X_OFFSET);
		final double rightOffset = -leftOffset;

		leftCS.setFloodlight(Color.GREEN);
		rightCS.setFloodlight(Color.GREEN);
		
		int lastColorLeft = -1, lastColorRight = -1;
		int ticks = 0;
		boolean leftFirst = false;
		boolean sawLeft = false, sawRight = false;
		
		//	This while loop is used to check if either of the ColorSensors crosses a grid line.
		// 	If one does, it updates the odometer. It only does this when the robot is not turning.

		while (true) {
			correctionStart = System.currentTimeMillis();
			int newColorLeft = leftCS.getNormalizedLightValue(), 
				newColorRight = rightCS.getNormalizedLightValue();
			
			if (lastColorLeft == -1) lastColorLeft = newColorLeft;
			if (lastColorRight == -1) lastColorRight = newColorRight;
			
			//	The odometry correction only runs if enabled
			if(enabled){
				//	If the light value read by the ColorSensor is below the ambient light
				//	by a percentage, the ColorSensor has crossed a grid line.
				Pose p = odometer.getPose();
				ticks++;
				if (lastColorLeft - newColorLeft > THRESHOLD) {
	
					Sound.beep();
//					System.exit(0);
					//	The temporary angle is the angle that the robot is currently at plus
					//	the left offset angle.
					tempAngle = (p.getHeading())*Math.PI/180 + leftOffset -Math.PI/2;

					//	If the angle is over 2*PI, it is corrected.
					if (tempAngle > 2*Math.PI)
						tempAngle -= 2*Math.PI;

					//	The XError and YError are set as the difference between the closest grid 
					//	line according to the odometer and the measured position of the grid line.
					XError = Math.abs(getLine(p.getX() + hypotenuse * Math.sin(tempAngle)) - (p.getX() + hypotenuse * Math.sin(tempAngle)));
					YError = Math.abs(getLine(p.getY() + hypotenuse * Math.cos(tempAngle)) - (p.getY() + hypotenuse * Math.cos(tempAngle)));

					//	The minimum between the X and Y error is found. If the X error is the 
					//	smaller, the X position of the odometer is updated. If the Y is smaller,
					//	the Y position of the odometer is updated.
					if(Math.min(XError,  YError) == XError)		
						odometer.setPose(new Pose((float) (getLine(p.getX() + hypotenuse * Math.sin(tempAngle)) - hypotenuse * Math.sin(tempAngle)), p.getY(), p.getHeading()));			
					else
						odometer.setPose(new Pose(p.getX(), (float) (getLine(p.getY() + hypotenuse * Math.cos(tempAngle)) - hypotenuse * Math.cos(tempAngle)), p.getHeading()));
					
					if (!sawRight ){
						ticks = 0;
						leftFirst = true;
					}
					sawLeft = true;
				}
				//	The following if statement is nearly identical to the one above. The right 
				//	ColorSensor is polled instead of the left one.
				if (lastColorRight - newColorRight > THRESHOLD) {
					Sound.beep();
//					System.exit(1);
					//	The rightOffset is used to calculate the tempAngle instead of the leftOffset.
					tempAngle = (p.getHeading())*Math.PI/180 + rightOffset - Math.PI/2;

					if (tempAngle > 2*Math.PI)
						tempAngle -= 2*Math.PI;

					XError = Math.abs(getLine(p.getX() + hypotenuse * Math.sin(tempAngle)) - (p.getX() + hypotenuse * Math.sin(tempAngle)));
					YError = Math.abs(getLine(p.getY() + hypotenuse * Math.cos(tempAngle)) - (p.getY() + hypotenuse * Math.cos(tempAngle)));

					if(Math.min(XError,  YError) == XError)				
						odometer.setPose(new Pose((float) (getLine(p.getX() + hypotenuse * Math.sin(tempAngle)) - hypotenuse * Math.sin(tempAngle)), p.getY(), p.getHeading()));					
					else
						odometer.setPose(new Pose(p.getX(), (float) (getLine(p.getY() + hypotenuse * Math.cos(tempAngle)) - hypotenuse * Math.cos(tempAngle)), p.getHeading()));					
					
					if ( !sawLeft){
						ticks = 0;
						leftFirst = false;
					}
					sawRight = true;
					
				}
				
				
				if (sawRight && sawLeft){
					sawRight = false;
					sawLeft = false;
					Sound.beep();
					odometer.setPose(new Pose(odometer.getPose().getX(), odometer.getPose().getY(), (float) (p.getHeading() + (leftFirst ? -ticks : ticks)*Main.getPilot().getTravelSpeed()/46f)));
				}

			}
			
			lastColorLeft = newColorLeft;
			lastColorRight = newColorRight;
			
			// this ensures the odometry correction occurs only once every period
//			correctionEnd = System.currentTimeMillis();
//			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
//				try {
//					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
//				} catch (InterruptedException e) {}
//			}
		}
	}
	
	/****
	 * Find the closest gridline to a coordinate
	 * @param coordinate coordinate to find gridline closest too
	 * @return the coordinate of the closest gridline
	 */
	private static double getLine(double coordinate) {
		return Math.round(coordinate / Main.TILE_WIDTH) * Main.TILE_WIDTH;
	}

	/****
	 * Enable OdometryCorrection globally
	 */
	public static void enable(){enabled = true;}
	
	/****
	 * Disable OdometeryCorrection globally
	 */
	public static void disable(){enabled = false;}


}