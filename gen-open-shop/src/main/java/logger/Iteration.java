package logger;

import java.util.List;

import core.Chromosome;
import core.Population;

public class Iteration {
	private PopulationStep population;
	private CrossoverStep crossover;
	private MutationStep mutation;
	private SelectionStep selection;

	public Iteration() {
	}

	public PopulationStep getPopulation() {
		return population;
	}

	public CrossoverStep getCrossover() {
		return crossover;
	}

	public MutationStep getMutation() {
		return mutation;
	}

	public SelectionStep getSelection() {
		return selection;
	}

	public void addPopulation(Population population) {
		this.population = new PopulationStep(population);
	}

	public void addCrossoverParents(Chromosome p1, Chromosome p2) {
		if (crossover == null) {
			crossover = new CrossoverStep();
		}
		crossover.addParents(p1, p2);
	}

	public void addCrossoverChilds(Chromosome c1, Chromosome c2) {
		if (crossover == null) {
			crossover = new CrossoverStep();
		}
		crossover.addChilds(c1, c2);
	}

	public void addCrossoverAlone(Chromosome a) {
		if (crossover == null) {
			crossover = new CrossoverStep();
		}
		crossover.addAlone(a);
	}

	public void addMutation(Chromosome c, int pos) {
		if (mutation == null) {
			mutation = new MutationStep();
		}
		mutation.addMutation(c, pos);
	}

	public void logSelectionSuccess(List<Chromosome> success) {
		if (selection == null)
			selection = new SelectionStep();
		selection.logSuccess(success);
	}

	public void logSelectionFailure(Chromosome c) {
		if (selection == null)
			selection = new SelectionStep();
		selection.logFailure(c);
	}

}
