package tests;

import core.Chromosome;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */

public class ChromosomeTest {

    private static final Integer[] TEST_ARRAY = {1, 2, 3, 4, 5};
    private static final Integer[] SWAP_POSITION_2 = {1, 4, 3, 2, 5};

    Chromosome chromosome1;

    @Before
    public void init() {
        chromosome1 = new Chromosome(Arrays.asList(1,2,3,4,5));
    }

    @Test
    public void testSwap() {
        Assert.assertArrayEquals(TEST_ARRAY, chromosome1.getGenom().toArray());

        chromosome1.swapGenes(1,3);
        Assert.assertArrayEquals(SWAP_POSITION_2, chromosome1.getGenom().toArray());
    }

}

