package datastructure;

import java.util.ArrayList;

public interface IGraph<K extends Comparable<K>,V> {
	
	public ArrayList<Node<K,V>> BFS();
	public ArrayList<Node<K,V>> DFS();
	public double[][] floydWarshall();
	public ArrayList<Double[]> dijkstra(K principalKey);
	public IGraph<K,V> prim();
	public IGraph<K,V> kruskal();
	public void add(K key, V value);
	public void removeVertex(K key);
	public void removeEdge(K keyVertex, K keyAdyacent);
	public V search(K key);
	
}
