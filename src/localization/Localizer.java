package localization;

import lejos.robotics.RangeReadings;
import lejos.robotics.RangeScanner;
import lejos.robotics.localization.MCLPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import main.Main;

public class Localizer extends MCLPoseProvider {
	private static final int PARTICLES = 200, BORDER = 0;

	private static final float[] ANGLES = { -45f, 0f, 45f };

	private RangeReadings readings;
	private DifferentialPilot pilot;
	public Localizer(DifferentialPilot pilot, RangeScanner scanner) {
		super(pilot, scanner, Main.getMap(), PARTICLES, BORDER);
		this.getScanner().setAngles(ANGLES);
		this.pilot = pilot;
		this.readings = new RangeReadings(ANGLES.length);
	}

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

	private boolean updateParticles() {
		if (!this.updateReadings()) {
			return false;
		}
		
		boolean updateOK = false;

		updateOK = this.update(readings);

		if (!updateOK) { // either improbable readings or resample failed
			this.generateParticles();  }
		
		return updateOK;

	}

	  /**
	   * Check if estimated pose is accurate enough
	   */
	  boolean goodEstimate(Pose pose) {
	    float sx = this.getSigmaX();
	    float sy = this.getSigmaY();
	    float xr = this.getXRange();
	    float yr = this.getYRange();
	    return sx <5 && sy < 5 && xr < 40 && yr < 40 ;
	  }
	  
	public void localize() {
		updateParticles();
		while (this.isBusy());

		Pose pose = this.getPose();
		boolean goodEst = false;
		do {
			if (!goodEstimate(pose)) {
				move();
				while (this.isBusy());
				updateParticles();
			}
		} while (!goodEst);
	}

	// Perform a movement 
	private void move() {
		
		
	}

}
