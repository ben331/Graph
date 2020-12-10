package datastructure;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Stack;

import org.junit.jupiter.api.Test;

class GraphBTest {

	private GraphB<Integer, String> graph;
	
	void setup1() {
		graph = new GraphB<>(GraphB.SIMPLE_GRAPH);
		
		graph.add(1, "Uno");
		graph.add(2, "Dos");
		graph.getNodes().get(0).addAdjacent(1, 1);
		graph.getNodes().get(1).addAdjacent(0, 1);
	}
	
	void setup2() {
		graph = new GraphB<>(GraphB.SIMPLE_GRAPH);
		
		graph.add(1, "Uno");
		graph.add(2, "Dos");
		graph.add(3, "Tres");
		graph.getNodes().get(0).addAdjacent(1, 1);
		graph.getNodes().get(0).addAdjacent(2, 3);
		graph.getNodes().get(1).addAdjacent(0, 1);
		graph.getNodes().get(1).addAdjacent(2, 2);
		graph.getNodes().get(2).addAdjacent(0, 3);
		graph.getNodes().get(2).addAdjacent(1, 2);
		
	}
	
	void setup3() {
		graph = new GraphB<>(GraphB.SIMPLE_GRAPH);
		
		graph.add(1, "Uno");
		graph.add(2, "Dos");
		graph.add(3, "Tres");
		graph.add(4, "Cuatro");
		graph.add(5, "Cinco");
		graph.getNodes().get(0).addAdjacent(1, 1);
		graph.getNodes().get(1).addAdjacent(0, 1);
		graph.getNodes().get(1).addAdjacent(2, 3);
		graph.getNodes().get(2).addAdjacent(0, 2);
		graph.getNodes().get(2).addAdjacent(3, 1);
		graph.getNodes().get(3).addAdjacent(2, 1);
		graph.getNodes().get(2).addAdjacent(4, 1);
		graph.getNodes().get(4).addAdjacent(0, 1);
		graph.getNodes().get(3).addAdjacent(4, 3);
		graph.getNodes().get(4).addAdjacent(3, 3);
	}
	
	@Test
	void testGetters() {
		setup1();
		
		//Nodes
		assertEquals(2, graph.getNodes().size());
		assertEquals(1, graph.getNodes().get(0).getKey());
		assertEquals(2, graph.getNodes().get(1).getKey());
		
		//Adayacents
		assertEquals(2, graph.getNodes().get(graph.getNodes().get(0).getNeiborgIndex(0)).getKey());
		assertEquals(1, graph.getNodes().get(graph.getNodes().get(1).getNeiborgIndex(0)).getKey());
	}
	
	@Test
	void testAdd() {
		setup1();
		
		//Add
		graph.add(3, "Tres");
		graph.getNodes().get(2).addAdjacent(1, 1);
		graph.getNodes().get(2).addAdjacent(1, 3);
		
		//Test
		assertEquals(3, graph.getNodes().size());
		
		//Nodes
		assertEquals(3, graph.getNodes().size());
		assertEquals(1, graph.getNodes().get(0).getKey());
		assertEquals(2, graph.getNodes().get(1).getKey());
		
		//Adayacents
		assertEquals(2, graph.getNodes().get(graph.getNodes().get(0).getNeiborgIndex(0)).getKey());
		assertEquals(1, graph.getNodes().get(graph.getNodes().get(1).getNeiborgIndex(0)).getKey());
	}
	
	@Test
	void testRemoveVertex() {
		setup2();
		graph.removeVertex(3);
		
		//Nodes
		assertEquals(2, graph.getNodes().size());
		assertEquals(1, graph.getNodes().get(0).getKey());
		assertEquals(2, graph.getNodes().get(1).getKey());
		
		//Adayacents
		assertEquals(2, graph.getNodes().get(graph.getNodes().get(0).getNeiborgIndex(0)).getKey());
		assertEquals(1, graph.getNodes().get(graph.getNodes().get(1).getNeiborgIndex(0)).getKey());
	}
	
	@Test
	void testRemoveEdge() {
		
		setup2();
		graph.removeEdge(2, 3);
		
		assertEquals(3, graph.getNodes().size());
		
		//Nodes
		assertEquals(1, graph.getNodes().get(0).getKey());
		assertEquals(2, graph.getNodes().get(1).getKey());
		assertEquals(3, graph.getNodes().get(2).getKey());
		
		//Adayacents
		assertEquals(2, graph.getNodes().get(graph.getNodes().get(0).getNeiborgIndex(0)).getKey());
		assertEquals(3, graph.getNodes().get(graph.getNodes().get(0).getNeiborgIndex(1)).getKey());
		assertEquals(1, graph.getNodes().get(graph.getNodes().get(2).getNeiborgIndex(0)).getKey());
		
	}
	
	@Test
	void testBFS() {
		setup3();
		
		ArrayList<Node<Integer,String>> route = graph.BFS();
		
		for(int i=0; i<5; i++) {
			assertEquals(i+1, route.get(i).getKey());
		}
	}
	
	@Test
	void testDFS() {
		setup3();
		
		ArrayList<Node<Integer,String>> route = graph.DFS();
		
		for(int i=0; i<5; i++) {
			assertEquals(i+1, route.get(i).getKey());
		}
	}
	
	@Test
	void testDijkstra() {
		setup3();
		
		Double[] prevs = graph.dijkstra(4).get(1);
		
		Stack<Integer> route = graph.buildRoute(prevs, 5);
		
		assertEquals(4, route.pop());
		assertEquals(3, route.pop());
		assertEquals(5, route.pop());
	}
	
	@Test
	void testPrim() {
		setup3();
		
		GraphB<Integer, String> minExpansion = graph.prim();
		
		for(int i=0; i<graph.getNodes().size(); i++) {
			assertEquals(graph.getNodes().get(i).getKey() , minExpansion.getNodes().get(i).getKey(), "line"+i);
		}
		
		Node<Integer,String> current = graph.getNodes().get(0);
		assertEquals(2 , current.getNeiborgIndex(0));
		
		current = graph.getNodes().get(1);
		assertEquals(1 , current.getNeiborgIndex(0));
		assertEquals(3 , current.getNeiborgIndex(1));
		
		current = graph.getNodes().get(2);
		assertEquals(2 , current.getNeiborgIndex(0));
		assertEquals(4 , current.getNeiborgIndex(1));
		assertEquals(5 , current.getNeiborgIndex(2));
		
		current = graph.getNodes().get(3);
		assertEquals(3 , current.getNeiborgIndex(0));
		
		current = graph.getNodes().get(4);
		assertEquals(3 , current.getNeiborgIndex(0));
	}
	
	@Test
	void testKruskal() {
		setup3();
		
		GraphB<Integer, String> minExpansion = graph.kruskal();
		
		for(int i=0; i<graph.getNodes().size(); i++) {
			assertEquals(graph.getNodes().get(i).getKey() , minExpansion.getNodes().get(i).getKey(), "line"+i);
		}
		
		Node<Integer,String> current = graph.getNodes().get(0);
		assertEquals(2 , current.getNeiborgIndex(0));
		
		current = graph.getNodes().get(1);
		assertEquals(1 , current.getNeiborgIndex(0));
		assertEquals(3 , current.getNeiborgIndex(1));
		
		current = graph.getNodes().get(2);
		assertEquals(2 , current.getNeiborgIndex(0));
		assertEquals(4 , current.getNeiborgIndex(1));
		assertEquals(5 , current.getNeiborgIndex(2));
		
		current = graph.getNodes().get(3);
		assertEquals(3 , current.getNeiborgIndex(0));
		
		current = graph.getNodes().get(4);
		assertEquals(3 , current.getNeiborgIndex(0));
	}
	
}