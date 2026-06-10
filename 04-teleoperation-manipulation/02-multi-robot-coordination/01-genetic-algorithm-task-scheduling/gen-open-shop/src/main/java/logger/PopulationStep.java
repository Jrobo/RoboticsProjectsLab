package logger;

import core.Population;

public class PopulationStep extends Step {

	private Population population;

	public PopulationStep(Population population) {
		this.population = population.clone();
		type = Type.POPULATION;
	}

	public Population getPopulation() {
		return population;
	}

	public void setPopulation(Population population) {
		this.population = population;
	}

}
