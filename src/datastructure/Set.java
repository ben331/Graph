package datastructure;

public class Set<K extends Comparable<K>, V> implements Comparable<Set<K,V>> {
	
	private Element<K,V> firstElement;
	
	private Element<K,V> lastElement;
	
	private Element<K,V> representative;

	public Set(K key, V value) {
		Element<K,V> element = new Element<>(key, value, null);
		representative = firstElement = lastElement = element;
		element.setRepresentative(element);
	}

	public Element<K,V> getFirstElement() {
		return firstElement;
	}

	public Element<K,V> getLastElement() {
		return lastElement;
	}

	public void setLastElement(Element<K,V> lastElement) {
		this.lastElement = lastElement;
	}

	public Element<K,V> getRepresentative() {
		return representative;
	}

	public void setRepresentative(Element<K,V> representative) {
		this.representative = representative;
	}
	
	@Override
	public int compareTo(Set<K,V> set2) {
		return representative.getKey().compareTo(set2.getRepresentative().getKey());
	}
	
}
