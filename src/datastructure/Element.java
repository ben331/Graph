package datastructure;

public class Element<K extends Comparable<K>,V> {
	
	private K key;
	private V value;
	
	private Element<K,V> nextElement;
	
	private Element<K,V> representative;

	public Element(K key, V value, Element<K,V> representative) {
		super();
		this.key = key;
		this.value = value;
		this.representative = representative;
	}

	public V getValue() {
		return value;
	}

	public void setValue(V value) {
		this.value = value;
	}

	public Element<K,V> getNextElement() {
		return nextElement;
	}

	public void setNextElement(Element<K,V> nextElement) {
		this.nextElement = nextElement;
	}

	public Element<K,V> find() {
		Element<K,V> r = representative;
		while(r.representative!=r) {
			r = r.representative;
		}
		return r;
	}

	public void setRepresentative(Element<K,V> representative) {
		this.representative = representative;
	}

	public K getKey() {
		return key;
	}

	public void setKey(K key) {
		this.key = key;
	}
}
