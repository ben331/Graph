package datastructure;

import java.io.Serializable;

public class Tuple<K extends Comparable<K>,V,P extends Comparable<P>> implements Serializable{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -1441522304459643339L;
	private K key;
	private V value;
	private P priority;
	
	public Tuple(K key, V value, P priority) {
		this.key = key;
		this.value = value;
		this.setPriority(priority);
	}
	public K getKey() {
		return key;
	}
	public V getValue() {
		return value;
	}
	
	public void setDelete() {
		key = null;
		value = null;
	}
	public P getPriority() {
		return priority;
	}
	public void setPriority(P priority) {
		this.priority = priority;
	}
}
