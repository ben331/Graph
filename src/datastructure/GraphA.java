package datastructure;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class GraphA <K extends Comparable<K>,V> implements IGraph<K,V> {
	
	private ArrayList<Node<K,V>> nodes;

	private ArrayList<ArrayList<Edge>> horizontal;

	public GraphA() {

		horizontal = new ArrayList<>();
		nodes = new ArrayList<>();
	}

	public void addNode(K key, V value, int pos) {
		
		ArrayList<Edge> adjacency = null;
		Node<K, V> newNode = new Node<K, V>(key, value, pos, adjacency);
		nodes.add(newNode);
		
		ArrayList<Edge> newVertical = new ArrayList<>();
		Edge edgePeronal = new Edge(nodes.size(), nodes.size(), "");
		Edge edgeNull = null;
		
		for (int i = 0; i < nodes.size(); i++) {
			
			if (i == nodes.size() -1) {
				newVertical.add(edgePeronal);
			}else {
				newVertical.add(edgeNull);
			}
			
		}
		
		if (!horizontal.isEmpty()) {
			for (int i = 0; i < nodes.size(); i++) {
				
				horizontal.get(i).add(edgeNull);		
			}
		}
		
		horizontal.add(newVertical);
		
	}
	
	public void bindTwoEdge(K key1, K key2, double weight) {
		int edNum1 = positionSearch(key1);
		int edNum2 = positionSearch(key2);
		
		if(!(edNum1 == -1 || edNum2 ==-1)) {
			Edge edge1 = new Edge(edNum2, edNum1, weight, "");
			Edge edge2 = new Edge(edNum1, edNum2, weight, "");
			
			horizontal.get(edNum1).set(edNum2, edge1);
			horizontal.get(edNum2).set(edNum1, edge2);
		}
	}

	@Override
	public ArrayList<Node<K, V>> BFS() {
		
		ArrayList<Node<K, V>> listReturn = new ArrayList<>();
		Queue<Integer> numeros = new LinkedList<>();
		ArrayList<Integer> numerosPasados = new ArrayList<>();
		
		
		numeros.add(0);
		numerosPasados.add(-1);
		
		do {
			int numCurrent = numeros.remove();
			
			if (!numerosPasados.contains(numCurrent)) {
				
				for (int j = 0; j < nodes.size(); j++) {
					
					if ( numCurrent != j) {
						
						Edge edgeHelper = horizontal.get(numCurrent).get(j);
						
						if (edgeHelper != null) {
							
							numeros.add(edgeHelper.getOrigin());	
						}	
					}
				}
				
				numerosPasados.add(numCurrent);
				listReturn.add(nodes.get(numCurrent));
			}
			
		}while(!numeros.isEmpty());

		
		return listReturn;
	}

	@Override
	public ArrayList<Node<K, V>> DFS() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double[][] floydWarshall() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ArrayList<Double[]> dijkstra(K principalKey) {
		// TODO Auto-generated method stub
		return null;
	}


	@Override
	public GraphA<K,V> prim() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void add(K key, V value) {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void removeVertex(K key) {
		// TODO Auto-generated method stub
		
	}

	public void removeEdge(K keyVertex, K keyAdyacent) {
		// TODO Auto-generated method stub
	}

	@Override
	public IGraph<K, V> kruskal() {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public V search(K key) {

		V valorReturn = null;

		for (int i = 0; i < nodes.size() && valorReturn != null ; i++) {
			if (nodes.get(i).getKey().equals(key)) {
				valorReturn = nodes.get(i).getValue();
			}
		}
		
		return valorReturn;
	}
	
	public int positionSearch(K key) {
		
		int valorReturn = -1;
		
		for (int i = 0; i < nodes.size() && valorReturn != -1; i++) {
			if (nodes.get(i).getKey().equals(key)) {
				valorReturn = i;
			}
		}
		
		return valorReturn;
	}
	
}