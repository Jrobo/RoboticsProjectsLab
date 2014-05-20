package logger;

import core.Population;

public class PopulationStep extends Step {
	
	private Population population;
	
	public PopulationStep(Population population) {
		this.population = population.clone();
	}
	
}
