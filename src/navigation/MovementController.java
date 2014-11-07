package navigation;

import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.AstarSearchAlgorithm;
import lejos.robotics.pathfinding.FourWayGridMesh;
import lejos.robotics.pathfinding.NodePathFinder;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.PathFinder;
import lejos.robotics.pathfinding.ShortestPathFinder;
import main.Display;
import main.Main;


/**
 * Movement controller that finds a path using a given map and travels there.
 * 
 * @author Scott Cooper
 */
public class MovementController{
	private static final int GRIDSPACE = 25, CLEARANCE = 15;
	private Navigator nav;	// Navigator to control movement
	private PathFinder pathFinder;	// Path finder
	
	/***
	 * Create a new movement Controller
	 * 
	 * @param nav The navigator to use for movement and pose-providing
	 */
	public MovementController(Navigator nav){
		this.nav = nav;
		// Use a regular grid of node points. Grid space = 20. Clearance = 15:
		FourWayGridMesh grid = new FourWayGridMesh(Main.getMap(), GRIDSPACE, CLEARANCE);
		
		// Use A* search:
		AstarSearchAlgorithm alg = new AstarSearchAlgorithm();
		
		// Give the A* search alg and grid to the PathFinder:
		pathFinder = new NodePathFinder(alg, grid);
	}
	
	/***
	 * Find the shortest path to the Waypoint w and begin traveling to it
	 * 
	 * @param w The destination waypoint
	 * @throws DestinationUnreachableException Thrown by PathFinder if there is
	 * 			not a route to the waypoint 'w'
	 */
	public void travelToWaypoint(Waypoint w) {
		nav.stop();
		nav.clearPath();
		Path p;
		try {
			p = pathFinder.findRoute(nav.getPoseProvider().getPose(), w);
		} catch (DestinationUnreachableException e) {
			String x = Display.formattedDoubleToString(w.getX(), 1),
				   y = Display.formattedDoubleToString(w.getX(), 1),
				   t = Display.formattedDoubleToString(w.getHeading(), 1);
			throw new RuntimeException("No path: (" + x + ", " + y + ", " + t + ")");
		}
		nav.followPath(p);
		nav.waitForStop();
	}
}
