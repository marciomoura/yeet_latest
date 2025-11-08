% MATLAB script to create parameters mat-file
% Converted from sfc_6p_6p_topology_machine_parameters.json

sim_sample_time_s = 12.5e-6;

sim_thyr_snub_resistance_ohm = 66;
sim_thyr_snub_capacitance_f = 1e-6;

sim_source_base_voltage_rms = 2090;
sim_source_phase_a_deg = 0;
sim_source_phase_b_deg = -120;
sim_source_phase_c_deg = 120;

sim_input_transformer_nominal_power_va = 3362e3;
sim_input_transformer_nominal_frequency_hz = 50;

sim_input_transformer_hv_phase_to_phase_rms_voltage_v = 6600;
sim_input_transformer_hv_resistance_pu = 0.0018;
sim_input_transformer_hv_inductance_pu = 0.0629;

sim_input_transformer_lv_phase_to_phase_rms_voltage_v = 2200;
sim_input_transformer_lv_resistance_pu = 0.0024;
sim_input_transformer_lv_inductance_pu = 0.0839;

sim_input_transformer_magnetization_resistance_pu = 500;
sim_input_transformer_magnetization_inductance_pu = 500;

sim_dc_link_resistance_ohm = 10e-3;
sim_dc_link_inductance_h = 8e-3;

sim_machine_nominal_power_va = 2950e3;
sim_machine_nominal_phase_to_phase_voltage_v = 1980;
sim_machine_nominal_frequency_hz = 50;

sim_machine_stator_resistance_ohm = 0.0047;
sim_machine_stator_leakage_inductance_h = 0.00059;
sim_machine_magnetizing_inductance_d_h = 0.00808;
sim_machine_magnetizing_inductance_q_h = 0.00808;
sim_machine_canay_inductance_h = -0.00035;

sim_machine_damper_resistance_d_ohm = 0.136;
sim_machine_damper_leakage_inductance_d_h = 0.0015;
sim_machine_damper_resistance_q_ohm = 0.136;
sim_machine_damper_leakage_inductance_q_h = 0.0015;

sim_machine_field_resistance_ohm = 0.0014;
sim_machine_field_leakage_inductance_h = 0.00167;

sim_machine_inertia_coefficient = 670;
sim_machine_pole_pairs = 1;

sim_machine_initial_speed_deviation_percent = -99.999999;
sim_machine_initial_electrical_angle_deg = 0;
sim_machine_initial_current_magnitude_a_pu = 0;
sim_machine_initial_current_magnitude_b_pu = 0;
sim_machine_initial_current_magnitude_c_pu = 0;
sim_machine_initial_phase_angle_a_deg = 0;
sim_machine_initial_phase_angle_b_deg = -120;
sim_machine_initial_phase_angle_c_deg = 120;
sim_machine_initial_field_voltage = 0;

% Save all variables to mat-file
save('sfc_6p_6p_topology_machine_parameters.mat');
