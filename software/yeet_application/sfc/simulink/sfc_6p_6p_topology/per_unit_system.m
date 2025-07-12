% =========================================================================
% FILENAME: define_lci_pu_system.m
% DESCRIPTION:
%   Defines the complete per-unit (pu) system for the LCI gas turbine
%   starter project based on provided component data.
%   The script calculates base values for each system section and converts
%   key parameters to per-unit. The results are stored in a struct.
% =========================================================================
%clear; clc;

disp('--- Defining LCI Per-Unit System ---');

%% 1. System-Wide Base Quantities
% The system base power is anchored to the transformer's rating.
S_base = 2950e3;    % [VA] Rated power of the transformer (ST)
f_base_L = 50;      % [Hz] Base frequency for the line side (fL)

%% 2. High Voltage (HV) Side - Line & Transformer Primary
disp('Calculating HV Side (Primary) Bases...');
bases.HV.S_base = S_base;
bases.HV.U_base = 10.5e3;   % [V] Nominal line voltage (UL)
bases.HV.I_base = S_base / (sqrt(3) * bases.HV.U_base); % [A]
bases.HV.Z_base = bases.HV.U_base^2 / S_base;          % [Ohm]

% Per-unit grid impedance (based on minimum short-circuit power)
SCC_min = 300e6; % [VA]
bases.HV.Z_grid_pu = S_base / SCC_min;

%% 3. Low Voltage (LV) Side - Transformer Secondary & Converter Line Side (CLS)
disp('Calculating LV Side (Secondary / CLS) Bases...');
bases.LV.S_base = S_base;
bases.LV.U_base = 2090;     % [V] Rated converter no-load voltage (Uv0)
bases.LV.I_base = S_base / (sqrt(3) * bases.LV.U_base); % [A]
bases.LV.Z_base = bases.LV.U_base^2 / S_base;          % [Ohm]

% Per-unit transformer impedance (given as 9.5% on its own base)
X_T_pu_on_own_base = 0.095; % [pu] From exT = 9.5%
% Since S_base = ST, no conversion is needed.
bases.LV.X_T_pu = X_T_pu_on_own_base;

% --- Total Line-Side Commutation Reactance ---
% This is the most important value for the line-side converter simulation.
% It is the sum of the grid impedance (referred to LV side) and the transformer impedance.
% NOTE: The grid impedance is already in pu on S_base, so it's the same value.
X_comm_CLS_pu_calculated = bases.HV.Z_grid_pu + bases.LV.X_T_pu;

% The LCI Design Data provides a specific value for this.
% FOR SIMULATION, USE THE EXPLICITLY PROVIDED VALUE as it is the final
% design parameter, accounting for all factors.
bases.LV.X_comm_pu = 0.073; % [pu] From "Converter Line Side (CLS)" data

% We add a note about the discrepancy for engineering reference.
bases.LV.note = ['Calculated X_comm_pu = ', num2str(X_comm_CLS_pu_calculated), ...
                 '. Using the specified design value of 0.073 pu.'];
disp(['NOTE: ', bases.LV.note]);

%% 4. DC Link
disp('Calculating DC Link Bases...');
% Base DC voltage is the ideal no-load voltage from the rectifier
bases.DC.U_base = (3 * sqrt(2) / pi) * bases.LV.U_base; % [V]
bases.DC.I_base = 1044; % [A] Using the rated DC current (Id) is standard practice
bases.DC.Z_base = bases.DC.U_base / bases.DC.I_base;   % [Ohm]

% Per-unit DC link inductor
Ld_physical = 8e-3; % [H]
% The base inductance is derived from base Z and line frequency
bases.DC.L_base = bases.DC.Z_base / (2 * pi * f_base_L); % [H]
bases.DC.Ld_pu = Ld_physical / bases.DC.L_base;

%% 5. Machine Side (CMS) - Simplified Model
% As requested, the machine is modeled as an ideal voltage source behind a
% commutation reactance.
disp('Calculating Machine Side (CMS) Bases...');
bases.CMS.S_base = S_base;
bases.CMS.f_base = 11;      % [Hz] Base frequency for the machine side (fM_base)
bases.CMS.U_base = 1980;    % [V] Machine service voltage (UM)
bases.CMS.I_base = S_base / (sqrt(3) * bases.CMS.U_base); % [A]
bases.CMS.Z_base = bases.CMS.U_base^2 / S_base;          % [Ohm] at any frequency

% Per-unit machine commutation reactance
% The data provides this directly on the correct converter base.
bases.CMS.X_comm_pu = 0.0593; % [pu] From "Converter Machine Side (CMS)" data
% For reference, calculate the physical reactance at the base frequency
bases.CMS.X_comm_ohm = bases.CMS.X_comm_pu * bases.CMS.Z_base; % [Ohm] at 11 Hz

%% 6. Final Summary and Output
disp('--- Per-Unit System Definition Complete ---');
fprintf('\n');
disp('==================== SUMMARY OF PU BASES ====================');
fprintf('System Base Power (S_base): %.2f kVA\n\n', S_base/1e3);

disp('--- HV Side (Primary) ---');
fprintf('  U_base_HV: %.2f kV\n', bases.HV.U_base/1e3);
fprintf('  I_base_HV: %.2f A\n', bases.HV.I_base);
fprintf('  Z_base_HV: %.2f Ohm\n', bases.HV.Z_base);
fprintf('  Z_grid_pu: %.4f pu\n\n', bases.HV.Z_grid_pu);

disp('--- LV Side (CLS) ---');
fprintf('  U_base_LV: %.0f V\n', bases.LV.U_base);
fprintf('  I_base_LV: %.2f A\n', bases.LV.I_base);
fprintf('  Z_base_LV: %.3f Ohm\n', bases.LV.Z_base);
fprintf('  X_comm_pu: %.4f pu (Design Value for Simulation)\n\n', bases.LV.X_comm_pu);

disp('--- DC Link ---');
fprintf('  U_base_DC: %.0f V\n', bases.DC.U_base);
fprintf('  I_base_DC: %.0f A\n', bases.DC.I_base);
fprintf('  Ld_pu:     %.4f pu\n\n', bases.DC.Ld_pu);

disp('--- Machine Side (CMS) ---');
fprintf('  f_base_CMS: %.0f Hz\n', bases.CMS.f_base);
fprintf('  U_base_CMS: %.0f V\n', bases.CMS.U_base);
fprintf('  I_base_CMS: %.2f A\n', bases.CMS.I_base);
fprintf('  Z_base_CMS: %.3f Ohm\n', bases.CMS.Z_base);
fprintf('  X_comm_pu:  %.4f pu (Design Value for Simulation)\n', bases.CMS.X_comm_pu);
disp('=============================================================');

% Save the 'bases' struct to a .mat file for use in Simulink
save('lci_pu_system_bases.mat', 'bases');
disp('PU system bases saved to lci_pu_system_bases.mat');