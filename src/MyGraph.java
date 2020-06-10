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

public class MyGraph {

	private SimpleWeightedGraph<Point, DefaultWeightedEdge> graph;
	private ArrayList<Point> pointsList;
	private ArrayList<DefaultWeightedEdge> edgesList;
	private DefaultWeightedEdge tempEdge;
	private Point lastVertex = null;
	private Point currentLastVertex = null;

	public MyGraph() {
		graph = new SimpleWeightedGraph<Point, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		pointsList = new ArrayList<Point>();
		edgesList = new ArrayList<DefaultWeightedEdge>();
		tempEdge = new DefaultWeightedEdge();
	}

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
	
	public Point getLastElement(Set<Point> set) {
    	Point last = null;
    	if(set.size() > 0) {
    		for(Point x : set) {
    			last = x;
    		}
    	}
        return last;
    }

	public Point getLastPoint() {
		return pointsList.get(pointsList.size()-1);
	}
	
	public void removeLastVertex() {
		graph.removeVertex(getLastElement(graph.vertexSet()));
	}
}
