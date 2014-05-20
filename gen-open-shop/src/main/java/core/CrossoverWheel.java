package core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

public class CrossoverWheel {

	/* 
	 * Key: Right side of range
	 * Value: Chromosome parent
	 * */
	private Map<Integer, Chromosome> wheel;
	private int size;
	
	private Random random = new Random();
	
	
	public CrossoverWheel(List<Chromosome> data) {
		int size = 0;
		wheel = new TreeMap<Integer,Chromosome>();
		for (Chromosome chromosome: data) {
			size+=chromosome.getValue();
			wheel.put(size,chromosome);
		}	
		size++;
	}
	
	
	/*TODO: test! */
	public Chromosome getParent() {
		int key = random.nextInt(size);
		int index = Collections.binarySearch(new ArrayList<Integer>(wheel.keySet()), key,null);
		return wheel.get(index);
	}
	
}
