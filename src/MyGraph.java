import java.util.ArrayList;
import java.util.Set;

import javax.swing.JFrame;

import org.jgrapht.ext.JGraphXAdapter;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import com.mxgraph.layout.mxCircleLayout;
import com.mxgraph.layout.mxIGraphLayout;
import com.mxgraph.swing.mxGraphComponent;

import javafx.util.Pair;

/**
 * this class represents an undirected, weighted graph.
 *
 */
public class MyGraph {

	private SimpleWeightedGraph<Point, DefaultWeightedEdge> graph;
	private ArrayList<Point> pointsList;
	private ArrayList<DefaultWeightedEdge> edgesList;
	private DefaultWeightedEdge tempEdge;
	private Point lastVertex = null;
	private Point currentLastVertex = null;

	/**
	 * this method is a constructor.
	 */
	public MyGraph() {
		graph = new SimpleWeightedGraph<Point, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		pointsList = new ArrayList<Point>();
		edgesList = new ArrayList<DefaultWeightedEdge>();
		tempEdge = new DefaultWeightedEdge();
	}

	/**
	 * this method gets a pair of point and integer, and adds the point as a vertex to the graph, creates an edge between the new vertex and the last one in the graph.
	 * additionally, it adds the integer as a weight to the created edge. 
	 * @param pair
	 */
	public void addVertexAndEdge(Pair<Point, Integer> pair) {
		lastVertex = getLastElement(graph.vertexSet());
		graph.addVertex(pair.getKey());
		pointsList.add(pair.getKey());
		currentLastVertex = getLastElement(graph.vertexSet());
		if(pointsList.size()>1) {
			if(lastVertex!=null) {
				tempEdge = graph.addEdge(lastVertex, currentLastVertex);
				graph.setEdgeWeight(tempEdge, pair.getValue());
				edgesList.add(tempEdge);

			}

		}
	}

	/**
	 * this method draw a graph.
	 */
	public void drawGraph() {
		JFrame new_window = new JFrame();
		new_window.setSize(500,500);
		new_window.setTitle("Graph Viewer");
		JGraphXAdapter<Point, DefaultWeightedEdge> graphAdapter = new JGraphXAdapter<Point, DefaultWeightedEdge>(graph);

		mxIGraphLayout layout = new mxCircleLayout(graphAdapter);
		layout.execute(graphAdapter.getDefaultParent());

		new_window.add(new mxGraphComponent(graphAdapter));

		new_window.pack();
		new_window.setLocationByPlatform(true);
		new_window.setVisible(true);
	}

	/**
	 * this method prints the graph.
	 */
	public void printGraph() {
		Set<Point> vertexSet = graph.vertexSet();
		for(Point x : vertexSet) {
			System.out.println("point: "+x);
		}
		Set<DefaultWeightedEdge> edgesSet = graph.edgeSet();
		for(DefaultWeightedEdge e : edgesSet) {
			System.out.println("edge: "+e);

		}
		

	}

	/**
	 * this method get the last vertex of a given vertices set.
	 * @param set
	 * @return
	 */
	public Point getLastElement(Set<Point> set) {
    	Point last = null;
    	if(set.size() > 0) {
    		for(Point x : set) {
    			last = x;
    		}
    	}
        return last;
    }

	/**
	 * this method gets the last point of the points array.
	 * @return
	 */
	public Point getLastPoint() {
		return pointsList.get(pointsList.size()-1);
	}
	
	/**
	 * this method gets the one before the last vertex.
	 * @return
	 */
	public Point getBeforeLastPoint() {
		return pointsList.get(pointsList.size()-2);
	}
	
	/**
	 * this method remove the last point from the points array.
	 */
	public void removeLastPoint() {
		if(!pointsList.isEmpty()) {
			pointsList.remove(pointsList.size()-1);
		}
	}
	
	/**
	 * this method gets the last edge of the graph and returns it.
	 * @return
	 */
	public DefaultWeightedEdge getLastEdge() {
		DefaultWeightedEdge answer = null;
		Set<DefaultWeightedEdge> set = graph.edgeSet();
		for(DefaultWeightedEdge e : set) {
			answer = e;
		}
		return answer;
	}
	
	/**
	 * this method removes the last vertex of the graph.
	 */
	public void removeLastVertex() {
		Point lastVertex = getLastElement(graph.vertexSet());
		if(lastVertex != null) {
			graph.removeVertex(lastVertex);
		}
	}
	
	/**
	 * this method removes the last edge of the graph.
	 */
	public void removeLastEdge() {
		DefaultWeightedEdge lastEdge = getLastEdge();
		if(lastEdge != null) {
			graph.removeEdge(getLastEdge());
		}
	}
	
	/**
	 * this method returns the weight of the last edge of the graph.
	 * @return
	 */
	public int getLastEdgeWeight() {
		DefaultWeightedEdge lastEdge = getLastEdge();
		if(lastEdge != null) {
			return (int)graph.getEdgeWeight(lastEdge);
		}
		return 0;
	}

	/**
	 * this method returns true if the graph is empty. otherwise, returns false.
	 * @return
	 */
	public boolean isEmpty() {
		return graph.vertexSet().isEmpty();
	}
}
