package core;

import problem.ScheduleManager;

import java.util.List;

public class Chromosome {
	
	private List<Integer> genom;	
	private Integer value = null;
	private Integer hybridValue = null;
	
	public Chromosome(List<Integer> genom) {
		this.genom = genom;
	}
	
	public int getLength() {
		return genom.size();
	}

	public int getGen(int index) {
		return genom.get(index);
	}
	
	/* TODO: test */
	public void swapGenes(int g1, int g2) {
		int buf = genom.get(g1);
		genom.set(g1, genom.get(g2));
		genom.set(g2, buf);
	}
	
	public int getValue() {
		if (value == null) {
			value = ScheduleManager.makespan(this);
		}
		return value;
	}
}
