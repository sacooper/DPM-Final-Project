package localization;

import lejos.robotics.RangeReadings;
import lejos.robotics.RangeScanner;
import lejos.robotics.localization.MCLPoseProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import main.Main;

public class Localizer extends MCLPoseProvider {
	// Number of particles to use in MCLPoseProvider
	private static final int PARTICLES = 200, BORDER = 10;

	// Angles to get readings at
	private static final float[] ANGLES = { -45f, 0f, 45f };

	private RangeReadings readings; 	// Readings acquired from last update
	private DifferentialPilot pilot;	// Pilot controlling movement
	private OdometryPoseProvider odo;
	
	/****
	 * Create a new localizer. The class extends MCLPoseProvider by 
	 * interfacing the 'Main' class to acquire the necessary maps
	 * 
	 * @param pilot The DifferentialPilot used to move
	 * @param scanner The RangeScanner used to acquire movements
	 * @param odo The OdometryPoseProvider to correct
	 */
	public Localizer(DifferentialPilot pilot, RangeScanner scanner, OdometryPoseProvider odo) {
		super(pilot, scanner, Main.getMap(), PARTICLES, BORDER);
		this.getScanner().setAngles(ANGLES);
		this.pilot = pilot;
		this.readings = new RangeReadings(ANGLES.length);
		this.odo = odo;
	}

	/****
	 * Update the readings
	 * 
	 * @return true iff we were able to update readings successfully
	 */
	private boolean updateReadings() {
		boolean incomplete = true;
		int count = 0;
		float cumAngle = 0;
		do {
			readings = this.getScanner().getRangeValues();
			incomplete = readings.incomplete();

			if (!incomplete)
				break;

			float randAngle = -180 + 360 * (float) Math.random();
			pilot.rotate(randAngle);
			cumAngle += randAngle;
			count++;
		} while (incomplete && count < 20);
		pilot.rotate(-cumAngle);
		return count < 5;

	}

	/***
	 * Update the particles. In the event that the update
	 * based on the reading is unsuccessful, the particle
	 * set it reset
	 * 
	 * @return true iff the update was successful
	 */
	private boolean updateParticles() {
		if (!this.updateReadings()) {
			return false;
		}

		boolean updateOK = this.update(readings);

		if (!updateOK) { // either improbable readings or resample failed
			this.generateParticles();
		}

		return updateOK;

	}

	/**
	 * Check if estimated pose is accurate enough
	 * 
	 * @param mcl The MCLPoseProvider to check the readings of
	 * @return true iff the current state provides a good idea of location
	 */
	private static boolean goodEstimate(MCLPoseProvider mcl) {
		float sx = mcl.getSigmaX();
		float sy = mcl.getSigmaY();
		float xr = mcl.getXRange();
		float yr = mcl.getYRange();
		return sx < 5 && sy < 5 && xr < 40 && yr < 40;
	}

	/****
	 * Localize using the current map. 
	 */
	public void localize() {
		synchronized (Main.POSE_LOCK) {
			updateParticles();
			while (this.isBusy());
			
			boolean goodEst = false;
			do {
				if (!Localizer.goodEstimate(this)) {
					move();
					while (this.isBusy());
					updateParticles();
				}
			} while (!goodEst);
		}
	}

	/***
	 * Perform a movement to obtain a better of our current pose.
	 */
	private void move() {
		// TODO: Improved Movement Decion (better than random move)
		randomMove();
	}

	private void randomMove() {
		float angle = -180 + (float) Math.random() * 360;
		while (this.isBusy())
			;
		pilot.rotate(angle);
		float distance = (float) (Main.TILE_WIDTH * (float) Math.random());
		// Get forward range
		float forwardRange = this.getScanner().getRangeFinder().getRange();
		// Don't move forward if we are near a wall
		if (forwardRange > 180)
			forwardRange = 30;
		if (forwardRange < 20)
			distance = forwardRange - 30;
		if (distance > forwardRange - 20)
			distance = forwardRange - 30;
		
		pilot.travel(distance);
	}
}
