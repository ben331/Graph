package datastructure;

import java.util.ArrayList;

public class UnionFind<K extends Comparable<K>,V> implements IUnionFind<K,V>{
	
	private ArrayList<Set<K,V>> sets;

	public ArrayList<Set<K, V>> getSets() {
		return sets;
	}


	public UnionFind() {
		sets = new ArrayList<>();
	}
	
	public void makeset(K key, V value) {
		sets.add(new Set<K,V>(key, value));
	}
	
	public void union(Set<K,V> set1, Set<K,V> set2) {
		
		set1.getLastElement().setNextElement(set2.getFirstElement());
		set2.getRepresentative().setRepresentative(set1.getRepresentative());
		sets.remove(set2);
	}
	
	public Set<K,V> find(K key) {
		Set<K,V> set= null;
		for(int i=0; i<sets.size() && set==null; i++) {
			Element<K,V> current=sets.get(i).getFirstElement();
			while(current!=null && set==null) {
				if(current.getKey().compareTo(key)==0) {
					set = sets.get(i);
				}
			}
		}
		return set;
	}
}
