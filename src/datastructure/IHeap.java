package datastructure;

public interface IHeap<K extends Comparable<K>,V, P extends Comparable<P>> {
	
	public void heapify(int i);
	public void insert(K key, V value, P priority) throws IndexOutOfBoundsException;
	public V extractMax();
	public void increaseKey(K key);
}
