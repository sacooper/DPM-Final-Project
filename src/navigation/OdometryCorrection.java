package navigation;

import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.robotics.Color;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.util.Delay;
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
	private static boolean enabled;
	private final static double X_OFFSET = 3.4, // 7.3
								Y_OFFSET = 3.25, // 7.3
								THRESHOLD = 11;

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
		double tempAngle = 0;

		// 	Calculate the hypotenuse, as well as the angle offset for both the left and right
		//	ColorSensors.
		final double hypotenuse = Math.sqrt(X_OFFSET*X_OFFSET + Y_OFFSET*Y_OFFSET);
		final double offset = Math.atan(Y_OFFSET/X_OFFSET);

		leftCS.setFloodlight(Color.RED);
		rightCS.setFloodlight(Color.RED);
		
		int lastColorLeft = -1, lastColorRight = -1;
		boolean leftFirst = false;
		boolean sawLeft = false, sawRight = false;
		Pose lastPose = null;
		
		//	This while loop is used to check if either of the ColorSensors crosses a grid line.
		// 	If one does, it updates the odometer. It only does this when the robot is not turning.

		while (true) {
			int newColorLeft = leftCS.getNormalizedLightValue(), 
				newColorRight = rightCS.getNormalizedLightValue();
			
			if (lastColorLeft == -1) lastColorLeft = newColorLeft;
			if (lastColorRight == -1) lastColorRight = newColorRight;
			
			//	The odometry correction only runs if enabled
			if(enabled){
				//	If the light value read by the ColorSensor is below the ambient light
				//	by a percentage, the ColorSensor has crossed a grid line.
				Pose p = odometer.getPose();
				if (lastColorLeft - newColorLeft > THRESHOLD) {
					if (!sawRight ){
						lastPose = new Pose(p.getX(), p.getY(), p.getHeading());
						leftFirst = true;
					}
					sawLeft = true;
				}
				//	The following if statement is nearly identical to the one above. The right 
				//	ColorSensor is polled instead of the left one.
				if (lastColorRight - newColorRight > THRESHOLD) {
					if ( !sawLeft){
						lastPose = new Pose(p.getX(), p.getY(), p.getHeading());
						leftFirst = false;
					}
					sawRight = true;
					
				}
				
				
				if (sawRight && sawLeft){
					sawRight = false;
					sawLeft = false;
					double correction = Math.toDegrees(Math.atan(lastPose.distanceTo(p.getLocation()) / (X_OFFSET * 2))) ; 
					double heading = p.getHeading() + (leftFirst ? -correction : correction);
					
					LCD.drawString(lastPose.distanceTo(p.getLocation()) + "", 0, 4);
					if (leftFirst){
						tempAngle = heading * Math.PI / 180f - offset + Math.PI/2;
					} else {
						tempAngle = heading * Math.PI / 180f + offset + Math.PI/2;
					}
					
					float new_x, new_y;
					
					heading = (heading + 360) % 360;
					new_y = p.getY();
					new_x = p.getX();
					
					if (heading < 45 || heading >= 315){
						// Positive x
						new_x = (float) (getLine(p.getX() + Math.abs(hypotenuse * Math.sin(tempAngle))) - Math.abs(hypotenuse * Math.sin(tempAngle)));
					} else if (heading >= 45 && heading < 135){
						// Positive y
						new_y = (float) (getLine(p.getY() + Math.abs(hypotenuse * Math.cos(tempAngle))) - Math.abs(hypotenuse * Math.cos(tempAngle)));
					} else if (heading >= 135 && heading < 225){
						// Negative X
						new_x = (float) (getLine(p.getX() - Math.abs(hypotenuse * Math.sin(tempAngle))) + Math.abs(hypotenuse * Math.sin(tempAngle)));
					} else if (heading >= 225 && heading < 315){
						// Negative Y
						new_y = (float) (getLine(p.getY() + Math.abs(hypotenuse * Math.cos(tempAngle))) - Math.abs(hypotenuse * Math.cos(tempAngle)));
					}
					odometer.setPose(new Pose(new_x, new_y, (float) heading));
					
					
					Delay.msDelay(1000);
				}

			}
			
			lastColorLeft = newColorLeft;
			lastColorRight = newColorRight;
			
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
	public static void enable(){
		enabled = true;
	}
	
	/****
	 * Disable OdometeryCorrection globally
	 */
	public static void disable(){enabled = false;}


}