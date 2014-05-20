package logger;

import java.util.ArrayList;
import java.util.List;

import core.Chromosome;
import core.Population;

public class SelectionStep {
	
	Population population;
	List<Chromosome> deleted;
	
	public SelectionStep() {
		deleted = new ArrayList<Chromosome>();
	}
	
	public void logSuccess(List<Chromosome> success) {
		population = new Population(success);
	}
	
	public void logFailure(Chromosome c) {
		deleted.add(c);
	}

}
