% =========================================================================
% FILENAME: sfc_simulation_parameters.m
% DESCRIPTION:
%   Generates a complete set of simulation parameters for the SFC gas
%   turbine starter project. It includes:
%   1. Raw input data in SI units, following the project dictionary.
%   2. A complete per-unit system definition based on machine ratings.
%   3. Derived SI unit parameters (R, L, etc.) for direct use in
%      simulation blocks (e.g., Simulink/Simscape Electrical).
%
%   The final output is a single struct 'sfc_params' saved to a .mat file.
% =========================================================================
%clear; clc;

equivalent_grid_side_resistance = 0.0163;
equivalent_grid_side_inductance = 0.4941e-3;

equivalent_machine_side_resistance = 0.0066; 
equivalent_machine_side_inductance = 0.2508e-3;

dc_link_resistance = 0.0503;
dc_link_inductance = 8e-3;