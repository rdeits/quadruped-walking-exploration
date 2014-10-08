# Quadruped walking explorations

## Requires:

* Mosek academic license: <http://mosek.com/resources/academic-license>
* Gurobi academic license: <http://www.gurobi.com/products/licensing-and-pricing/academic-licensing>
* tbxmanager for matlab <http://tbxmanager.com/>
* yalmip for matlab
	* you can install yalmip with `tbxmanager install yalmip` from within matlab

## Setup:

	git submodule update --init --recursive
	make

## Running the solver

	cd matlab
	matlab

in the matlab console:

	sol = testQuadrupedNonGaited();
	playbackLittleDog(sol);