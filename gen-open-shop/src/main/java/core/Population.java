package core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

public class Population {

	private int size;
	private List<Chromosome> individuals;

	private int oneLength;

	private double mutaionProbability = 0.3d;

	private Random random = new Random();

	public Population(int size, int oneLength) {
		this.size = size;
		this.oneLength = oneLength;
		individuals = generatePopulation(size, oneLength);
	}

	public int getOneLength() {
		return oneLength;
	}

	public double getMutaionProbability() {
		return mutaionProbability;
	}

	public void setMutaionProbability(double mutaionProbability) {
		this.mutaionProbability = mutaionProbability;
	}

	private List<Chromosome> generatePopulation(int size, int n) {
		List<Chromosome> result = new ArrayList<>();
		List<Integer> genes = new ArrayList<>();
		for (int i = 0; i < n; i++) {
			genes.add(i);
		}
		for (int i = 0; i < size; i++) {
			Collections.shuffle(genes);
			result.add(new Chromosome(new ArrayList<>(genes)));
		}
		return result;
	}
	
	public void iterate() {
		List<Chromosome> newPopulation = uniformCrossover();
		swapMutation(newPopulation);
		eliteSelection(newPopulation);
	}

	private List<Chromosome> uniformCrossover() {
		CrossoverWheel wheel = new CrossoverWheel(individuals);
		List<Chromosome> newPopulation = new ArrayList<>();
		for (int i = 0; i < size / 2; i++) {
			newPopulation.addAll(crossover(wheel.getParent(),
					wheel.getParent(), getMask(oneLength)));
		}
		if (newPopulation.size() < size) {
			newPopulation.add(wheel.getParent());
		}
		return newPopulation;
	}

	/* TODO: test! */
	private int getMask(int length) {
		int mask = 0;
		for (int i = 0; i < length; i++) {
			mask *= 2;
			mask += random.nextInt(2);
		}
		return mask;
	}

	private List<Chromosome> crossover(Chromosome p1, Chromosome p2, int mask) {
		List<Chromosome> result = new ArrayList<>();
		if (p1.equals(p2)) {
			result.add(p1);
			result.add(p2);
			return result;
		}

		List<Integer> c1 = new ArrayList<>();
		List<Integer> c2 = new ArrayList<>();
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
		result.add(new Chromosome(c1));
		result.add(new Chromosome(c2));
		return result;
	}

	private void swapMutation(List<Chromosome> population) {
		for (Chromosome chromosome : population) {
			if (random.nextDouble() < mutaionProbability) {
				int index = random.nextInt(oneLength);
				int g1 = index == 0 ? oneLength - 1 : index - 1;
				int g2 = index == oneLength - 1 ? 0 : index + 1;
				chromosome.swapGenes(g1, g2);
			}
		}
	}

	/* TODO: test*/
	private void eliteSelection(List<Chromosome> newPopulation) {
		individuals.addAll(newPopulation);
		Collections.sort(individuals, new Comparator<Chromosome>() {

			@Override
			public int compare(Chromosome arg0, Chromosome arg1) {
				return arg0.getValue() - arg1.getValue();
			}
		});
		while (individuals.size() > size) {
			individuals.remove(size);
		}
	}

}
