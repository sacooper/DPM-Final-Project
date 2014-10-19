package navigation;

import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.DijkstraPathFinder;
import main.Main;

public class MovementController {
	private Navigator nav;
	private DijkstraPathFinder pathFinder;
	
	public MovementController(Navigator nav){
		this.nav = nav;
		pathFinder = new DijkstraPathFinder(Main.getMap());
	}
	
	public void travelToWaypoint(Waypoint w) throws DestinationUnreachableException {
		nav.clearPath();
		nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), w));
	}
}
