package main;

import lejos.nxt.LCD;
import lejos.robotics.localization.PoseProvider;
import localization.Localizer;
import localization.Position;

/*****
 * Display to print current heading of the robot. 
 * 
 * @author Scott Cooper
 *
 */
public class Display extends Thread {
	
	/*****
	 * Enum representing the current action the robot is performing
	 * 
	 * @author Scott Cooper
	 */
	public static enum Action { 
		/**** The robot is currently localizing */
		LOCALIZING, 
		/** The robot is currently moving */
		MOVING, 
		/** The robot is currently search for / picking up / dropping a block */
		BLOCK_ACTION;
		
		@Override
		/****
		 * Get the action currently being performed as a string
		 */
		public String toString(){
			switch(this) {
			case LOCALIZING: return "LOCALIZING";
			case MOVING: return "MOVING";
			case BLOCK_ACTION: return "BLOCK ACTION";
			default: return "?????";
		}
		}
	}
	
	private static boolean clear, paused;
	private static final int DELAY = 1000;
	private static Action currentAction = null;
	private PoseProvider poseProvider;
	
	/****
	 * Create a new display, which updates using the provided PoseProvider
	 * 
	 * @param poseProvider The pose provider to use in updating the display
	 */
	public Display(PoseProvider poseProvider){
		this.poseProvider = poseProvider;
		clear = true;
		paused = false;
	}
	
	@Override
	public void run(){

		float x, y, h;
		String posStr = "";
		while(true){
			if (!paused){
				if (clear) LCD.clear();
				x = poseProvider.getPose().getX();
				y = poseProvider.getPose().getY();
				h = poseProvider.getPose().getHeading();
				
				LCD.drawString("X: " + Display.formattedDoubleToString(x, 2) + posStr, 0, 0);
				LCD.drawString("Y: " + Display.formattedDoubleToString(y, 2) + posStr, 0, 1);
				LCD.drawString("H: " + Display.formattedDoubleToString(h, 2) + posStr, 0, 2);
				LCD.drawString(currentAction == null ? "" : currentAction.toString(), 0, 3);
				LCD.drawString("Start: " + startingPointAsString(), 0, 4);
			}
			try{
				Thread.sleep(DELAY);
			}catch(Exception e){}
		}
	}
	
	/****
	 * Return the double 'x' formatted and rounded to 'places' places
	 * 
	 * @param x The double to round
	 * @param places The number of places to round to
	 * @return A formatted string representing 'x' rounded to 'places' places
	 */
	public static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;
		
		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";
		
		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0)
				t = -t;
			
			while (t > 0) {
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}
			
			result += stack;
		}
		
		// put the decimal, if needed
		if (places > 0) {
			result += ".";
		
			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long)x);
			}
		}
		
		return result;
	}
	
	/***
	 * Set the current action to display
	 * 
	 * @param action The current action being performed
	 */
	public static synchronized void setCurrentAction(Action action){
		currentAction = action;}

	
	/****
	 * Get the starting point as a string, or an empty string if
	 * it has not been determined yet.
	 * 
	 * @return A string representing the starting position
	 */
	public static String startingPointAsString(){
		Position current;
		current = Localizer.getStartingPosition();
		if (current == null) return "";
		
		return current.getX() + " " + current.getY() + " " + current.getDir().asCardinal();
	}
	
	/***
	 * Print a location to the LCD screen 
	 * 
	 * @param x X coordinate to display
	 * @param y Y Coordinate to display
	 * @param h Heading to display
	 */
	public static void printLocation(float x, float y, float h){
		LCD.clear();
		LCD.drawString("X: " + Display.formattedDoubleToString(x, 2), 0, 0);
		LCD.drawString("Y: " + Display.formattedDoubleToString(y, 2), 0, 1);
		LCD.drawString("H: " + Display.formattedDoubleToString(h, 2), 0, 2);
	}
	
	/** Enable clearing of the LCD screen (default) */
	public static void enableClear(){clear = true;}
	
	/** Disable clearing of the LCD screen */
	public static void disableClear(){clear = false;}
	
	/** Pause printing to the LCD screen. The LCD screen will not be cleared */
	public static void pause(){paused = true;}
	
	/** Resume printing to the LCD screen. The screen will be cleared based
	 * on the most recent change. */
	public static void resume(){paused=false;}
}
