package navigation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.AstarSearchAlgorithm;
import lejos.robotics.pathfinding.NavigationMesh;
import lejos.robotics.pathfinding.Node;
import lejos.robotics.pathfinding.NodePathFinder;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.PathFinder;
import main.Display;
import main.Main;


/**
 * Movement controller that finds a path using a given map and travels there.
 * 
 * @author Scott Cooper
 */
public class MovementController{
	private static class Grid implements NavigationMesh {
		private static boolean isValid(Node n){
			return n.x >= 0 && n.x < Main.NUM_TILES && n.y >= 0 && n.y < Main.NUM_TILES;}
		
		private ArrayList<Node> set;
		private boolean[][] map;
		
		public Grid(boolean[][] map){
			this.map = map;
			regenerate();
		}
		
		@Override
		public int addNode(Node node, int neighbors) {
			set.add(node);
			int n_count = 0;
			int[] n = {-1, 0, 1};
			for (int x : n){
				for (int y : n){
					if (Math.abs(x) + Math.abs(y) == 1)
						for (Node n2 : set)
							if (n2.x == node.x + x && n2.y == node.y + y){
								connect(node, n2);
								n_count++;}
				}
			}
			
			return n_count;
		}

		@Override
		public boolean removeNode(Node node) {
			if (!set.contains(node))
				return false;
			
			set.remove(node);
			return true;
			
			
		}

		@Override
		public boolean connect(Node node1, Node node2) {
			if (isValid(node1) && isValid(node2)){
				node1.addNeighbor(node2);
				node2.addNeighbor(node1);
				return true;
			} else return false;

		}

		@Override
		public boolean disconnect(Node node1, Node node2) {
			if (node1.getNeighbors().contains(node2) && node1.getNeighbors().contains(node1)){
				node1.removeNeighbor(node2);
				node2.removeNeighbor(node1);
				return true;
			} else return false;
		}

		@Override
		public Collection<Node> getMesh() {
			return set;
		}

		@Override
		public void regenerate() {
			set = new ArrayList<Node>();
			
			for (int x = 0; x < map.length; x++){
				for (int y = 0; y < map[x].length; y++){
					if (!map[x][y]){
						addNode(new Node(x, y), 5);
					}
						
				}
			}
		}
		
	}
	
	private Navigator nav;			// Navigator to control movement
	private PathFinder pathFinder;	// Path finder
	
	/***
	 * Create a new movement Controller
	 * 
	 * @param nav The navigator to use for movement and pose-providing
	 */
	public MovementController(Navigator nav){
		this.nav = nav;

		Grid grid = new Grid(Main.getBoolMap());
		pathFinder = new NodePathFinder(new AstarSearchAlgorithm(), grid);
	}
	
	private static float coordAsTile(double c){
		return (int)((c + Main.TILE_WIDTH) / Main.TILE_WIDTH);
	}
	
	/***
	 * Find the shortest path to the Waypoint w and begin traveling to it
	 * 
	 * @param w The destination waypoint
	 */
	public void travelToWaypoint(Waypoint w) {
		nav.stop();
		nav.clearPath();
		Path p;
		
		Pose me = nav.getPoseProvider().getPose();
		Pose tile = new Pose(coordAsTile(me.getX()), coordAsTile(me.getY()), me.getHeading());
		Waypoint dest = new Waypoint(coordAsTile(w.getX()), coordAsTile(w.getY()));
		try {
			p = pathFinder.findRoute(tile, dest);
		} catch (DestinationUnreachableException e) {
			String x = Display.formattedDoubleToString(w.getX(), 1),
				   y = Display.formattedDoubleToString(w.getY(), 1),
				   t = Display.formattedDoubleToString(w.getHeading(), 1);
			throw new RuntimeException(x + ", " + y + ", " + t);
		}

		for (Waypoint way : p){
			way.x = way.x * Main.TILE_WIDTH - Main.TILE_WIDTH/2f;
			way.y = way.y * Main.TILE_WIDTH - Main.TILE_WIDTH/2f;
		}
		nav.singleStep(false);
		nav.followPath(p);
		nav.waitForStop();
		nav.goTo(w);
		nav.waitForStop();
		nav.rotateTo(w.getHeading());
		nav.waitForStop();
	}
}
