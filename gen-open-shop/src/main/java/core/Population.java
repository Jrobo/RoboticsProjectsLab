package core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import logger.StepLogger;

public class Population implements Cloneable {

	private int size;
	private List<Chromosome> individuals;

	private int oneLength;

	private double mutaionProbability = 0.3d;

	private Random random = new Random();

	public Population(int size, int oneLength) {
		this.size = size;
		this.oneLength = oneLength;
		individuals = generatePopulation(size, oneLength);
		StepLogger.start();
		StepLogger.logPopulationStep(this);
	}

	public Population(List<Chromosome> individuals) {
		this.individuals = new ArrayList<Chromosome>(individuals);
		this.size = individuals.size();
		if (this.size > 0) {
			this.oneLength = individuals.get(0).getLength();
		} else {
			this.oneLength = 0;
		}
	}

	public int getOneLength() {
		return oneLength;
	}

	public double getMutaionProbability() {
		return mutaionProbability;
	}

	public List<Chromosome> getIndividuals() {
		return individuals;
	}

	public void setMutaionProbability(double mutaionProbability) {
		this.mutaionProbability = mutaionProbability;
	}

	private List<Chromosome> generatePopulation(int size, int n) {
		List<Chromosome> result = new ArrayList<Chromosome>();
		List<Integer> genes = new ArrayList<Integer>();
		for (int i = 0; i < n; i++) {
			genes.add(i);
		}
		for (int i = 0; i < size; i++) {
			Collections.shuffle(genes);
			result.add(new Chromosome(new ArrayList<Integer>(genes)));
		}
		return result;
	}

	public void iterate() {
		List<Chromosome> newPopulation = uniformCrossover();
		swapMutation(newPopulation);
		eliteSelection(newPopulation);
		StepLogger.next();
		StepLogger.logPopulationStep(this);
	}

	private List<Chromosome> uniformCrossover() {
		CrossoverWheel wheel = new CrossoverWheel(individuals);
		List<Chromosome> newPopulation = new ArrayList<Chromosome>();
		for (int i = 0; i < size / 2; i++) {
			newPopulation.addAll(crossover(wheel.getParent(),
					wheel.getParent(), getMask(oneLength)));
		}
		if (newPopulation.size() < size) {
			Chromosome alone = wheel.getParent();
			newPopulation.add(alone);
			StepLogger.logCrossoverAlone(alone);

		}
		return newPopulation;
	}

	private int getMask(int length) {
		int mask = 0;
		for (int i = 0; i < length; i++) {
			mask *= 2;
			mask += random.nextInt(2);
		}
		return mask;
	}

	private List<Chromosome> crossover(Chromosome p1, Chromosome p2, int mask) {
		StepLogger.logCrossoverParents(p1, p2);
		List<Chromosome> result = new ArrayList<Chromosome>();
		if (p1.equals(p2)) {
			result.add(p1);
			result.add(p2);
			StepLogger.logCrossoverChilds(p1, p2);
			return result;
		}

		List<Integer> c1 = new ArrayList<Integer>();
		List<Integer> c2 = new ArrayList<Integer>();
		for (int i = 0; i < p1.getLength(); i++) {
			if (mask % 2 == 1) {
				c1.add(p2.getGen(i));
				c2.add(p1.getGen(i));
			} else {
				c1.add(p1.getGen(i));
				c2.add(p2.getGen(i));
			}
			mask /= 2;
		}
		Set<Integer> gens1 = new HashSet<Integer>();
		Set<Integer> gens2 = new HashSet<Integer>();
		for (int i = 0; i < p1.getLength(); i++) {
			if (gens1.contains(c1.get(i))) {
				int item = (c1.get(i) + 1) % p1.getLength();
				while (gens1.contains(item)) {
					item++;
					item %= p1.getLength();
				}
				c1.set(i,item);
				gens1.add(item);
			} else {
				gens1.add(c1.get(i));
			}
			if (gens2.contains(c1.get(i))) {
				int item = (c2.get(i) + 1) % p1.getLength();
				while (gens2.contains(item)) {
					item++;
					item %= p2.getLength();
				}
				c2.set(i,item);
				gens2.add(item);
			} else {
				gens2.add(c2.get(i));
			}
		}
		result.add(new Chromosome(c1));
		result.add(new Chromosome(c2));
		StepLogger.logCrossoverChilds(new Chromosome(c1), new Chromosome(c2));
		return result;
	}

	private void swapMutation(List<Chromosome> population) {
		for (Chromosome chromosome : population) {
			if (random.nextDouble() < mutaionProbability) {
				int index = random.nextInt(oneLength);
				StepLogger.logMutation(chromosome, index);
				int g1 = index == 0 ? oneLength - 1 : index - 1;
				int g2 = index == oneLength - 1 ? 0 : index + 1;
				chromosome.swapGenes(g1, g2);
			}
		}
	}

	private void eliteSelection(List<Chromosome> newPopulation) {
		individuals.addAll(newPopulation);
		Collections.sort(individuals, new Comparator<Chromosome>() {

			@Override
			public int compare(Chromosome arg0, Chromosome arg1) {
				return arg0.getValue() - arg1.getValue();
			}
		});
		while (individuals.size() > size) {
			StepLogger.logSelectionFailed(individuals.get(size));
			individuals.remove(size);
		}
		StepLogger.logSelectionPassed(individuals);
	}

	public Population clone() {
		return new Population(individuals);
	}

}
