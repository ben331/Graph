package datastructure;

public class Priority implements Comparable<Priority>{
	
	private double weight;
	private int pos;
	
	public Priority(double weight) {
		super();
		this.setWeight(weight);
	}
	
	public Priority(double weight, int pos) {
		super();
		this.setWeight(weight);
	}

	@Override
	public int compareTo(Priority p) {
		if(weight<p.weight) {
			return 1;
		}else if(weight>p.weight) {
			return -1;
		}else {
			if(pos<p.pos) {
				return 1;
			}else{
				return -1;
			}
		}
	}

	public double getWeight() {
		return weight;
	}

	public void setWeight(double weight) {
		this.weight = weight;
	}

	public int getPos() {
		return pos;
	}
}
