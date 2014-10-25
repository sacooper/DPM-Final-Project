package localization;

import lejos.robotics.RangeScanner;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import main.Main;

public class FullPoseProvider implements PoseProvider {
	
	private boolean positionKnown;
	private OdometryPoseProvider odo;
	private Localizer localizer;
	private Pose start;
	
	public FullPoseProvider(DifferentialPilot pilot, RangeScanner rs, boolean positionKnown){
		odo = new OdometryPoseProvider(pilot);
		localizer = new Localizer(pilot, rs);
		this.positionKnown = positionKnown;
	}
	
	public FullPoseProvider(DifferentialPilot pilot, RangeScanner rs){
		this(pilot, rs, false);}
	
	/****
	 * Get the localizer created by this FullPoseProvider
	 * @return The localizer create by this FullPoseProvider
	 */
	public Localizer getLocalizer(){
		synchronized(Main.POSE_LOCK){
			return localizer;}}
	
	/****
	 * Get the OdometeryPoseProvider created by this FullPoseProvider
	 * @return The OdometeryPoseProvider created by this FullPoseProvider
	 */
	public OdometryPoseProvider getOdo(){
		synchronized(Main.POSE_LOCK){
			return odo;}}
	
	/****
	 * Returns the known pose based on the odometer or 
	 * the estimated pose based on the localizer
	 * 
	 * @return The current pose
	 */
	@Override
	public Pose getPose() {
		synchronized(Main.POSE_LOCK){
			if (positionKnown){
				return odo.getPose();
			} else return localizer.getPose();}
	}


	/***
	 * Set the current pose
	 * 
	 * @param aPose The pose to set. Both the OdometeryPoseProvide and Loclizer will be updated to this value
	 */
	@Override
	public void setPose(Pose aPose) {
		synchronized (Main.POSE_LOCK){
			odo.setPose(aPose);
			localizer.setPose(aPose);}
	}
	
	/***
	 * Set the  position known flag, indicating whether to use
	 * odometery pose
	 * @param positionKnown Whether the current pose is based on a known position. 
	 * False results in use of the localization PoseProvider. True results in use
	 * of the odometery PoseProvider.
	 */
	public void setPositionKnown(boolean positionKnown){
		synchronized(Main.POSE_LOCK){
			this.positionKnown = positionKnown;}}
	
	/****
	 * Get whether the position is known.
	 * @return Whether the position is known, and thus whether we are using
	 * the odometery PoseProvider (true) or Localizer
	 */
	public boolean getPositionKnown(){ 
		synchronized(Main.POSE_LOCK){
			return this.positionKnown; }}
	
	public void localize(){
		
	}

}
