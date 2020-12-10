package datastructure;

public class Edge {
	
	private int origin;
	private int index;
	private double weight;
	private String indexS;
	
	
	public Edge(int origin, int index, String indexS) {
		super();
		this.origin = origin;
		this.index = index;
		this.weight = 1;
		this.indexS=indexS;
	}
	
	public Edge(int origin, int index, double weight, String indexS) {
		super();
		this.origin = origin;
		this.index = index;
		this.weight = weight;
		this.indexS=indexS;
	}
	
	public int getIndex() {
		return index;
	}
	public double getWeight() {
		return weight;
	}
	
	public String toString() {
		return index+"";
	}

	public int getOrigin() {
		return origin;
	}

	public String getIndexS() {
		return indexS;
	}

	public void setIndexS(String indexS) {
		this.indexS = indexS;
	}
}
