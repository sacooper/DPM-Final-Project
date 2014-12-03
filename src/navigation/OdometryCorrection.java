package navigation;

import lejos.nxt.ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
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
 * @since v1
 */

public class OdometryCorrection extends Thread {
	private static double lastHeadingCorrection;		// Value of the last heading correction
	
	private static boolean enabled;						// Whether odometry correction is enabled
	private final static double X_OFFSET = 3, 			// X distance of ultrasonic sensors from center
								Y_OFFSET = 3.25,		// Y distance of ultrasonic sensors from center
								THRESHOLD = 11;			// Threshold for line detection

	private static OdometryPoseProvider odometer;		// odometer to correct
	private static Pose last;							// The last position we saw a line at (by both CS)
	
	private ColorSensor leftCS, rightCS;				// Left and right color sensors
	

	/**
	 * The constructor of the <code>OdometryCorrection</code>.
	 * 
	 * @param odometer 	The <code>OdometeryPoseProvider</code> that is used for the robot
	 * @param leftCS 	The left <code>ColorSensor</code> that is used to check grid lines
	 * @param rightCS 	The right <code>ColorSensor</code>that is used to check grid lines
	 */
	public OdometryCorrection(OdometryPoseProvider odometer, ColorSensor leftCS, ColorSensor rightCS) {		
		OdometryCorrection.odometer = odometer;
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
		leftCS.setFloodlight(Color.RED);
		rightCS.setFloodlight(Color.RED);
		
		int lastColorLeft = -1, lastColorRight = -1, newColorLeft = -1, newColorRight = -1;
		boolean leftFirst = false;
		boolean sawLeft = false, sawRight = false;
		Pose lastPose = null;
		
		//	This while loop is used to check if either of the ColorSensors crosses a grid line.
		// 	If one does, it updates the odometer. It only does this when the robot is not turning.
		lastColorLeft = leftCS.getNormalizedLightValue();
		lastColorRight = rightCS.getNormalizedLightValue();
		Pose p;
		
		while (true) {
			newColorLeft = leftCS.getNormalizedLightValue(); 
			newColorRight = rightCS.getNormalizedLightValue();
			
			//	The odometry correction only runs if enabled
			if(enabled){
				p = odometer.getPose();
				
				// Check if we detected a line on the left side
				if (lastColorLeft - newColorLeft > THRESHOLD) {
					if (!sawRight ){
						lastPose = new Pose(p.getX(), p.getY(), p.getHeading());
						leftFirst = true;
					}
					sawLeft = true;
				}
				//	Check if we detected a line on the right side
				if (lastColorRight - newColorRight > THRESHOLD) {
					if (!sawLeft){
						lastPose = new Pose(p.getX(), p.getY(), p.getHeading());
						leftFirst = false;
					}
					sawRight = true;
					
				}
				
				// Once we've detected a line on both sides, calculate heading correction and save position
				if (sawRight && sawLeft){
					sawRight = false;
					sawLeft = false;
					OdometryCorrection.last = new Pose(p.getX(), p.getY(), p.getHeading());
					lastHeadingCorrection = (leftFirst ? 1 : -1 ) * Math.abs(Math.toDegrees(Math.atan(lastPose.distanceTo(p.getLocation()) / (X_OFFSET * 2))));
				}

			}
			
			// Set previous light values to current
			lastColorLeft = newColorLeft;
			lastColorRight = newColorRight;
			
		}
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
	
	/****
	 * Get the amount the heading was off the last time a line was crossed.
	 * Following a call to this method, the last heading is set to 0 to 
	 * prevent subsequent corrections if a line is missed.
	 * 
	 * @return The amount the heading needs to be corrected in degrees
	 */
	public static double lastHeadingCorrection(){
		double temp = lastHeadingCorrection;
		lastHeadingCorrection = 0;
		return temp;}
	
	/****
	 * Get the amount the distance was off the last time a line was crossed.
	 * Following a call to this method, the last distance correction
	 * is set to 0 to prevent subsequent corrections if a line is missed.
	 * 
	 * @return The amount the distance needs to be corrected
	 */
	public static double lastDistanceCorrection(){
		if (last == null) return 0;
		double off = last.distanceTo(odometer.getPose().getLocation()) - Main.TILE_WIDTH/2f - Y_OFFSET;
		last = null;
		return (off > Main.TILE_WIDTH/3f) ? 0 : -off;
	}


}