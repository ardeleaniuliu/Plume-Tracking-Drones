# Plume-Tracking-Drones
Repo with bits of code developed during PhD.
If using this code, consider citing Iuliu Ardelean PhD Thesis or https://doi.org/10.2514/1.C037299, as appropriate.

The code includes:
- get_robot: scalable model of rotary-wing electric aerial robots based on https://doi.org/10.2514/1.C037299
- get_cost: Momentum Theory implementation that predicts the power consumption of any given aerial robot.
- astar_path_planning: path planning algorithm that can compute the shortest distance or the least energy path in any given windy urban domain, for any given aerial robot. Algorithm outputs optimum path coordinates as well as the optimum ground speed ("3D+speed" path planning).
- vanilla_LBM: Lattice Boltzmann Method CFD solver for 2D domains for quick prototyping.
- get_uc_domain: pollution model that can compute pollution distribution for any given source location in any given windy domain.
- TCP snippets: for MATLAB-MATLAB communication. MATLAB-UNITY to be added in the future.
