package datastructure;

import java.util.ArrayList;

public class Heap<K extends Comparable<K>, V, P extends Comparable<P>> implements IHeap<K,V,P>{
	
	private Tuple<? extends Comparable<?>,?,? extends Comparable<?>>[] array;
	
	private int size;

	public Heap(int maxLength) {
		size =0; 
		array = new Tuple<?,?,?>[maxLength];
	}
	
	public int getLength() {
		return size;
	}
	
	public boolean isEmpty() {
		return size==0;
	}
	
	@SuppressWarnings("unchecked")
	public void heapify(int i) {
		int largest = i;
		P priorityLargest = (P) array[i].getPriority();
		Tuple<K,V,P> aux;
		int l = i*2;
		int r = l+1;
		
		if(l<size) {
			P priorityLeft=(P) array[l].getPriority();
			if(priorityLeft.compareTo(priorityLargest)>0) {
				largest=l;
				priorityLargest=priorityLeft;
			}
		}if(r<size) {
			P priorityRight=(P) array[r].getPriority();
			if(priorityRight.compareTo(priorityLargest)>0) {
				largest=r;
				priorityLargest=priorityRight;
			}
		}if(largest!=i) {
			aux = (Tuple<K, V, P>) array[largest];
			array[largest]=array[i];
			array[i]=aux;
			
			heapify(largest);
		}	
	}
	
	@SuppressWarnings("unchecked")
	private void increaseKey(int i) {
		Tuple<K,V,P> aux;
		int indexFather =(int) Math.floor(i/2);
		
		P priorityFather = (P) array[indexFather].getPriority();
		P priorityCurrent = (P) array[i].getPriority();
		
		while(i>0 && priorityFather.compareTo(priorityCurrent)<0) {
			aux = (Tuple<K, V, P>) array[indexFather];  //Exchange
			array[indexFather]=array[i];
			array[i]=aux;
			
			i=indexFather;	//increase counter and variables
			indexFather = (int) Math.floor(i/2);
			priorityFather = (P) array[indexFather].getPriority();
			priorityCurrent = (P) array[i].getPriority();
		}
	}

	@Override
	public void insert(K key, V value, P priority) throws IndexOutOfBoundsException{
		if(array.length>size) {
			Tuple<K,V,P> element = new Tuple<>(key, value, priority);
			array[size]=element;
			if(size>0) {
				increaseKey(size);
			}
			size++;
		}else {
			throw new IndexOutOfBoundsException("Heap is full");
		}
	}

	@SuppressWarnings("unchecked")
	@Override
	public V extractMax() {
		if(size==0) {
			throw new IndexOutOfBoundsException("Heap is empty");
		}else {
			V max = (V) array[0].getValue();
			array[0]= array[size -1];
			size--;
			heapify(0);
			return max;
		}
	}
	
	@SuppressWarnings("unchecked")
	public ArrayList<V> toArrayList(){
		ArrayList<V> list = new ArrayList<>();
		for(int i=0; i<size; i++) {
			list.add((V) array[i].getValue());
		}
		return list;
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public void increaseKey(K key) {
		for(int i=0; i<size; i++) {
			if(((K)array[i].getKey()).compareTo(key)==0) {
				increaseKey(i);
				break;
			}
		}
	}
}
