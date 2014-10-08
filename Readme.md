# Quadruped walking explorations

## Requires:

	* Mosek academic license: <http://mosek.com/resources/academic-license>
	* Gurobi academic license: <http://www.gurobi.com/products/licensing-and-pricing/academic-licensing>

## Setup:

	git submodule update --init --recursive
	make

## Running the solver

	cd matlab
	sol = testQuadrupedNonGaited();
	playbackLittleDog(sol);