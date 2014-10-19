package localization;

import lejos.robotics.RangeScanner;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;

public class FullPoseProvider implements PoseProvider {
	
	private boolean positionKnown;
	private OdometryPoseProvider odo;
	private Localizer localizer;
	
	public FullPoseProvider(DifferentialPilot pilot, RangeScanner rs, boolean positionKnown){
		odo = new OdometryPoseProvider(pilot);
		localizer = new Localizer(pilot, rs);
		this.positionKnown = positionKnown;
	}
	
	/****
	 * Get the localizer created by this FullPoseProvider
	 * @return The localizer create by this FullPoseProvider
	 */
	public Localizer getLocalizer(){return localizer;}
	
	/****
	 * Get the OdometeryPoseProvider created by this FullPoseProvider
	 * @return The OdometeryPoseProvider created by this FullPoseProvider
	 */
	public OdometryPoseProvider getOdo(){return odo;}
	
	/****
	 * Returns the known pose based on the odometer or 
	 * the estimated pose based on the localizer
	 * 
	 * @return The current pose
	 */
	@Override
	public Pose getPose() {
		if (positionKnown){
			return odo.getPose();
		} else return localizer.getPose();
	}


	/***
	 * Set the current pose
	 * 
	 * @param aPose The pose to set. Both the OdometeryPoseProvide and Loclizer will be updated to this value
	 */
	@Override
	public void setPose(Pose aPose) {
		odo.setPose(aPose);
		localizer.setPose(aPose);
	}
	
	public void setPositionKnown(boolean positionKnown){
		this.positionKnown = positionKnown;}
	
	public boolean getPositionKnown(){ return this.positionKnown; }

}
