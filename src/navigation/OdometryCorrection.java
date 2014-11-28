package navigation;

import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
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
 *
 */

public class OdometryCorrection extends Thread {
	private static double lastHeading, lastDist;
	
	private static boolean enabled;
	private final static double X_OFFSET = 3.4, 
								Y_OFFSET = 3.25,
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
					if (!sawLeft){
						lastPose = new Pose(p.getX(), p.getY(), p.getHeading());
						leftFirst = false;
					}
					sawRight = true;
					
				}
				
				
				if (sawRight && sawLeft){
					sawRight = false;
					sawLeft = false;
					double correction = Math.abs(Math.toDegrees(Math.atan(lastPose.distanceTo(p.getLocation()) / (X_OFFSET * 2)))) ;

					LCD.clear(3);
					lastHeading = leftFirst ? correction : -correction;
					LCD.drawString(lastHeading + "", 0, 3);
					double heading = Math.round(p.getHeading()/90f)*90f + (leftFirst ? -correction : correction);

					tempAngle = Math.PI - offset - Math.abs(lastHeading);
					
					float new_x, new_y;
					
					heading = (heading + 360) % 360;
					new_y = p.getY();
					new_x = p.getX();
					
					if (heading < 45 || heading >= 315){
						// Positive x
						new_x = (float) (getLine(p.getX() + Math.abs(hypotenuse * Math.cos(tempAngle))) - Math.abs(hypotenuse * Math.cos(tempAngle)));
						lastDist = (new_x > p.getX() ? -1 : 1) * (Math.abs(new_x - p.getX()));
					} else if (heading >= 45 && heading < 135){
						// Positive y
						new_y = (float) (getLine(p.getY() + Math.abs(hypotenuse * Math.cos(tempAngle))) - Math.abs(hypotenuse * Math.cos(tempAngle)));
						lastDist = (new_y > p.getY() ? -1 : 1) * (Math.abs(new_y - p.getY()));
					} else if (heading >= 135 && heading < 225){
						// Negative X
						new_x = (float) (getLine(p.getX() - Math.abs(hypotenuse * Math.cos(tempAngle))) + Math.abs(hypotenuse * Math.cos(tempAngle)));
						lastDist = (new_x > p.getX() ? 1 : -1) * (Math.abs(new_x - p.getX()));
					} else if (heading >= 225 && heading < 315){
						// Negative Y
						new_y = (float) (getLine(p.getY() - Math.abs(hypotenuse * Math.cos(tempAngle))) + Math.abs(hypotenuse * Math.cos(tempAngle)));
						lastDist = (new_y > p.getY() ? 1 : -1) * (Math.abs(new_y - p.getY()));
					}
					
					lastDist = Math.abs(lastDist) > 8 ? 0 : lastDist;							
					LCD.clear(4);
					LCD.drawString(lastDist + "", 0, 4);
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
	
	/****
	 * Get the amount the heading was off the last time a line was crossed.
	 * Following a call to this method, the last heading is set to 0 to 
	 * prevent subsequent corrections if a line is missed.
	 * 
	 * @return The amount the heading needs to be corrected
	 */
	public static double lastHeadingCorrection(){
		double temp = lastHeading;
		lastHeading = 0;
		return temp;}
	
	/****
	 * Get the amount the distance was off the last time a line was crossed.
	 * Following a call to this method, the last distance correction
	 * is set to 0 to prevent subsequent corrections if a line is missed.
	 * 
	 * @return The amount the distance needs to be corrected
	 */
	public static double lastDistanceCorrection(){
		double temp = lastDist;
		lastDist = 0;
		return temp;}
	
	


}