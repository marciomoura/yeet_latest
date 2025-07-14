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

% Initialize the main parameter struct
sfc_params = struct();

disp('--- Setting up SFC Simulation Parameters ---');

%% ========================================================================
%  1. RAW INPUT DATA (SI UNITS)
%  Storing all provided data directly into the struct using dictionary names.
% =========================================================================
disp('Step 1: Loading raw input data...');

% --- Grid (Source) Data ---
sfc_params.grid.voltage_line_rms_rated = 10.5e3; % [V] Nominal line-to-line voltage
sfc_params.grid.frequency_rated = 50;            % [Hz] Line frequency
sfc_params.grid.short_circuit_power_min = 300e6; % [VA] Minimum short-circuit power

% --- Transformer Data ---
sfc_params.transformer.power_rated = 2950e3;        % [VA] Rated power (S_T)
sfc_params.transformer.voltage_primary_rated = 10.5e3;  % [V] Rated primary voltage
sfc_params.transformer.voltage_secondary_rated = 2090;  % [V] Rated secondary no-load voltage (Uv0)
sfc_params.transformer.impedance_short_circuit_percent = 9.5; % [%] Short-circuit impedance (exT)
sfc_params.transformer.xr_ratio = 10; % Assumed X/R ratio for stability

% --- DC Link Data ---
sfc_params.dc_link.inductance = 8e-3;   % [H] DC reactor inductance (Ld)
sfc_params.dc_link.current_rated = 1044; % [A] Rated DC current (Id_rated)

% --- Synchronous Machine Data ---
sfc_params.machine_converter.power_rated = 2950e3;    % [VA] Rated apparent power (S_rated,M). Assumed equal to transformer rating.
sfc_params.machine_converter.voltage_rated = 1980;      % [V] Rated line-to-line voltage (V_rated,LL,M)
sfc_params.machine_converter.frequency_rated = 11;      % [Hz] Rated electrical frequency (f_rated,M)
sfc_params.machine_converter.xr_ratio = 10; % Assumed X/R ratio for stability

% --- Converter-Specific Reactances (from SFC Design Data) ---
% These are given in per-unit on a different base, will be converted later.
sfc_params.grid_converter.reactance_commutation_pu_design = 0.073;   % [pu] Total CLS commutation reactance from design sheet
sfc_params.machine_converter.reactance_commutation_pu_design = 0.0593; % [pu] Total CMS commutation reactance from design sheet

%% ========================================================================
%  2. PER-UNIT SYSTEM DEFINITION (Aligned with Documentation)
%  The system is defined around the synchronous machine's ratings.
% =========================================================================
disp('Step 2: Defining the machine-based per-unit system...');

% --- AC Side (Machine) Base Quantities ---
S_base = sfc_params.machine_converter.power_rated;
V_base = sfc_params.machine_converter.voltage_rated;
f_base = sfc_params.machine_converter.frequency_rated;
omega_base = 2 * pi * f_base;

sfc_params.per_unit.base_power = S_base;
sfc_params.per_unit.base_voltage_ac = V_base;
sfc_params.per_unit.base_frequency = f_base;
sfc_params.per_unit.base_current_ac = S_base / (sqrt(3) * V_base);
sfc_params.per_unit.base_impedance_ac = V_base^2 / S_base;
sfc_params.per_unit.base_inductance_ac = sfc_params.per_unit.base_impedance_ac / omega_base;
sfc_params.per_unit.base_flux_ac = sfc_params.per_unit.base_voltage_ac / (sqrt(3) * omega_base); % RMS phase flux linkage

% --- DC Link Base Quantities ---
% Derived to ensure power invariance (P_base_AC = P_base_DC)
sfc_params.per_unit.base_voltage_dc = (3 * sqrt(2) / pi) * V_base;
sfc_params.per_unit.base_current_dc = S_base / sfc_params.per_unit.base_voltage_dc;
sfc_params.per_unit.base_impedance_dc = sfc_params.per_unit.base_voltage_dc / sfc_params.per_unit.base_current_dc;

%% ========================================================================
%  3. DERIVED SI PARAMETERS FOR SIMULATION BLOCKS
%  Calculating physical R, L values needed for model parameterization.
% =========================================================================
disp('Step 3: Calculating derived SI parameters for simulation...');

% --- Grid Source Parameters (Thevenin Equivalent) ---
% The grid impedance must be calculated based on the grid-side voltage and system power base.
V_base_grid = sfc_params.grid.voltage_line_rms_rated;
Z_base_grid = V_base_grid^2 / S_base;
Z_grid_pu = S_base / sfc_params.grid.short_circuit_power_min;
Z_grid_ohm = Z_grid_pu * Z_base_grid;
% Assume a grid X/R ratio of 10 for stability, and Z_grid is mostly reactive
X_grid_ohm = Z_grid_ohm;
R_grid_ohm = X_grid_ohm / 10;
sfc_params.grid.resistance_thevenin_ohm = R_grid_ohm;
sfc_params.grid.inductance_thevenin_H = X_grid_ohm / (2 * pi * sfc_params.grid.frequency_rated);

% --- Transformer Parameters ---
% The transformer leakage is given in % on its own ratings.
% Since S_base = S_transformer, no power base conversion is needed.
sfc_params.transformer.reactance_leakage_pu = sfc_params.transformer.impedance_short_circuit_percent / 100;
% Calculate winding resistance from assumed X/R ratio for stability.
sfc_params.transformer.resistance_winding_pu = sfc_params.transformer.reactance_leakage_pu / sfc_params.transformer.xr_ratio;

% --- DC Link Resistance ---
% Estimate a small resistance for the DC link inductor for stability
sfc_params.dc_link.resistance_ohm = 0.1 * sfc_params.per_unit.base_impedance_dc;

% --- Total Line-Side Commutation Impedance ---
% This is the equivalent impedance "seen" by the grid-side converter terminals.
% It is the sum of the grid Thevenin impedance (referred to secondary) and the
% transformer leakage impedance.
V_base_secondary = sfc_params.transformer.voltage_secondary_rated;
Z_base_secondary = V_base_secondary^2 / S_base;

% Calculate transformer impedance in SI units (Ohm) referred to secondary
R_transformer_ohm = sfc_params.transformer.resistance_winding_pu * Z_base_secondary;
X_transformer_ohm = sfc_params.transformer.reactance_leakage_pu * Z_base_secondary;

% Refer grid impedance to secondary side
R_grid_referred_ohm = sfc_params.grid.resistance_thevenin_ohm * (V_base_secondary / V_base_grid)^2;
X_grid_referred_ohm = sfc_params.grid.inductance_thevenin_H * (2 * pi * sfc_params.grid.frequency_rated) * (V_base_secondary / V_base_grid)^2;

% Total equivalent commutation impedance is the sum of the components
R_comm_grid_converter_ohm = R_grid_referred_ohm + R_transformer_ohm;
X_comm_grid_converter_ohm = X_grid_referred_ohm + X_transformer_ohm;

% Store the final SI values for the simulation model
sfc_params.grid_converter.resistance_commutation_ohm = R_comm_grid_converter_ohm;
sfc_params.grid_converter.inductance_commutation_H = 0.5 * X_comm_grid_converter_ohm / (2 * pi * sfc_params.grid.frequency_rated);

% For reference, calculate the total PU commutation reactance
sfc_params.grid_converter.reactance_commutation_pu = 0.5 * X_comm_grid_converter_ohm / Z_base_secondary;


% --- Machine Commutation Reactance (for simplified machine model) ---
% The machine is a voltage source behind this impedance.
% The design value is given on a different base, so we use it directly but
% calculate the SI value based on our machine-defined base impedance.
Z_base_machine = sfc_params.per_unit.base_impedance_ac;
X_comm_machine_pu = sfc_params.machine_converter.reactance_commutation_pu_design;
% Calculate the physical reactance in Ohms AT THE MACHINE'S RATED FREQUENCY
X_comm_machine_ohm = X_comm_machine_pu * Z_base_machine;
% From this, calculate the physical inductance, which is a constant value.
sfc_params.machine_converter.inductance_commutation_H = 0.2* X_comm_machine_ohm / (2 * pi * sfc_params.machine_converter.frequency_rated);
% Calculate resistance from X/R ratio for stability
sfc_params.machine_converter.resistance_commutation_ohm = 0.2* X_comm_machine_ohm / sfc_params.machine_converter.xr_ratio;

%% ========================================================================
%  4. SAVE DATA AND DISPLAY SUMMARY
% =========================================================================
% Save the complete struct to a .mat file for loading into Simulink
save('sfc_simulation_parameters.mat', 'sfc_params');

disp('--------------------------------------------------');
disp('SFC Simulation Parameters successfully generated.');
disp('Data saved to: sfc_simulation_parameters.mat');
disp('--------------------------------------------------');
fprintf('\nKey SI and Per-Unit Parameters for Simulation Blocks:\n');
fprintf('  Grid Thevenin Inductance:      %.4f mH\n', sfc_params.grid.inductance_thevenin_H * 1e3);
fprintf('  Grid Thevenin Resistance:      %.4f Ohm\n', sfc_params.grid.resistance_thevenin_ohm);
fprintf('  Transformer Winding Resistance:  %.4f mOhm (SI, referred to secondary)\n', R_transformer_ohm * 1e3);
fprintf('  Grid Converter Comm. Inductance: %.4f mH\n', sfc_params.grid_converter.inductance_commutation_H * 1e3);
fprintf('  Grid Converter Comm. Resistance: %.4f mOhm\n', sfc_params.grid_converter.resistance_commutation_ohm * 1e3);
fprintf('  DC Link Inductor:              %.1f mH\n', sfc_params.dc_link.inductance * 1e3);
fprintf('  DC Link Resistance:            %.4f Ohm\n', sfc_params.dc_link.resistance_ohm);
fprintf('  Machine Comm. Inductance:      %.4f mH (SI), %.4f pu\n', sfc_params.machine_converter.inductance_commutation_H * 1e3, sfc_params.machine_converter.reactance_commutation_pu_design);
fprintf('  Machine Comm. Resistance:      %.4f Ohm\n', sfc_params.machine_converter.resistance_commutation_ohm);
disp('==================================================');