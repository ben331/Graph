package datastructure;

public interface IUnionFind<K extends Comparable<K>,V> {
	public void makeset(K key, V value);
	public void union(Set<K,V> set1, Set<K,V> set2) ;
	public Set<K,V> find(K key);
	
}
