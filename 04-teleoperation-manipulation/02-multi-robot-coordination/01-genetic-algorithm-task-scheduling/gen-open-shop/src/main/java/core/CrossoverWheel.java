package core;

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
        size = 0;
		wheel = new TreeMap<Integer,Chromosome>();
		for (Chromosome chromosome: data) {
			size+=chromosome.getValue();
			wheel.put(size,chromosome);
		}	
	}

    public Map<Integer, Chromosome> getWheel() {
        return wheel;
    }

    public int getSize() {
        return size;
    }

	public Chromosome getParent() {
		int key = random.nextInt(size)+1;
		for (Integer sector: wheel.keySet()) {
            if (sector >= key) {
                return wheel.get(sector);
            }
        }
        return null;
	}
	
}
