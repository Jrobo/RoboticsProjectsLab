package logger;

import java.util.ArrayList;
import java.util.List;

import org.javatuples.Pair;

import core.Chromosome;

public class MutationStep extends Step {
	
	List<Pair<Chromosome, Integer>> mutation;
	
	public MutationStep() {
		mutation = new ArrayList<Pair<Chromosome,Integer>>();
		type = Type.MUTATION;
	}
	
	public void addMutation(Chromosome c, int pos) {
		mutation.add(Pair.with(c.clone(), pos));
	}
}
