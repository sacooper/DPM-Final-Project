package navigation;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Collection;

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
	
	/*****
	 * Implementation of a NavigationMesh using tiles as the nodes.
	 * <br>
	 * A node can only be connect to tiles it is touching (the 4 surrounding tiles),
	 * ensuring that all paths found only require movement in the X and Y directions,
	 * therefore all movement on a large scale can be corrected usng <code>OdometryCorrection</code>
	 * <br>
	 * This is to be utilized by the <code>NodePathFinder</code> using the A* algorithm
	 * provided via the leJOS library
	 * 
	 * @author Scott Cooper
	 */
	private static class Grid implements NavigationMesh {
		
		/****
		 * Check if a <code>Node</code> is within in the bounds of the current map.
		 * @param n The node to check the validity of
		 * @return True iff the node 'n' is valid on the current map
		 */
		private static boolean isValid(Node n){
			return n.x >= 0 && n.x < Main.NUM_TILES && n.y >= 0 && n.y < Main.NUM_TILES;}
		
		// Set of nodes in this NavigationMesh
		private ArrayList<Node> set;
		
		/**
		 * Instantiate a new Grid using the provided map
		 * 
		 * @param map The map to base the grid off of
		 */
		public Grid(BitSet map){
			regenerate();
		}
		
		/***
		 * Add a node to this <code>NavigationMesh</code>. Note that 
		 * the value of neighbors is not used because of the inherint
		 * maximum of 4
		 * 
		 * {@inheritDoc}
		 */
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

		/****
		 * Remove a node from the given mesh.
		 * 
		 * {@inheritDoc}
		 */
		@Override
		public boolean removeNode(Node node) {
			if (!set.contains(node))
				return false;
			
			set.remove(node);
			return true;
			
			
		}

		/***
		 * Connect node1 and node2. 
		 * 
		 * {@inheritDoc}
		 */
		@Override
		public boolean connect(Node node1, Node node2) {
			if (isValid(node1) && isValid(node2)){
				node1.addNeighbor(node2);
				node2.addNeighbor(node1);
				return true;
			} else return false;

		}

		/****
		 * Disconnect node1 and node2.
		 * 
		 * {@inheritDoc}
		 */
		@Override
		public boolean disconnect(Node node1, Node node2) {
			if (node1.getNeighbors().contains(node2) && node1.getNeighbors().contains(node1)){
				node1.removeNeighbor(node2);
				node2.removeNeighbor(node1);
				return true;
			} else return false;
		}

		/****
		 * Get the current mesh.
		 * 
		 * {@inheritDoc}
		 */
		@Override
		public Collection<Node> getMesh() {
			return set;
		}

		/*****
		 * Regenerate the set of nodes using the map provided
		 * during instantiation.
		 * 
		 * {@inheritDoc}
		 */
		@Override
		public void regenerate() {
			BitSet map = Main.getCurrentMap();
			set = new ArrayList<Node>();
			
			for (int x = 0; x < Main.NUM_TILES; x++){
				for (int y = 0; y < Main.NUM_TILES; y++){
					if (map.get(x*Main.NUM_TILES + y)){
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

		Grid grid = new Grid(Main.getCurrentMap());
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
