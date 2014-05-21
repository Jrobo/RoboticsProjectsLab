package logger;

import java.util.ArrayList;
import java.util.List;

import org.javatuples.Pair;

import core.Chromosome;

public class CrossoverStep extends Step {

	List<Pair<Chromosome, Chromosome>> parents;
	List<Pair<Chromosome, Chromosome>> childs;

	Chromosome alone;

	public CrossoverStep() {
		parents = new ArrayList<Pair<Chromosome, Chromosome>>();
		childs = new ArrayList<Pair<Chromosome, Chromosome>>();
		type = Type.CROSSOVER;

	}

	public List<Pair<Chromosome, Chromosome>> getParents() {
		return parents;
	}

	public List<Pair<Chromosome, Chromosome>> getChilds() {
		return childs;
	}

	public Chromosome getAlone() {
		return alone;
	}

	public void addParents(Chromosome p1, Chromosome p2) {
		parents.add(Pair.with(p1.clone(), p2.clone()));
	}

	public void addChilds(Chromosome c1, Chromosome c2) {
		childs.add(Pair.with(c1.clone(), c2.clone()));
	}

	public void addAlone(Chromosome a) {
		alone = a.clone();
	}
}
