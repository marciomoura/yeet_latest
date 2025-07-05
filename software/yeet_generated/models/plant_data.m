global par;

Ts = 0.0001;

par.base.S = 555e6;                          % Rated 3-phase power (VA)
par.base.Vline_rms = 24e3;                   % Rms line (phase-to-phase) voltage
par.base.V = sqrt(2/3) * par.base.Vline_rms; % Peak phase voltage
par.base.I = (2/3) * par.base.S/par.base.V;  % Peak phase current                   

par.base.f = 60;                            % Frequency (Hz)
par.base.p = 1;                             % Pair of poles
par.base.w = 2 * pi * par.base.f;           % Frequency (rad/s)
par.base.f_mech = par.base.f / par.base.p;  % Mechanical frequency (hz)
par.base.w_mech = 2 * pi * par.base.f_mech; % Mechanical frequency (rad/s)

par.base.Z = par.base.V / par.base.I;       % Base impedance (Ohm)
par.base.L = par.base.Z / par.base.w;       % Base inductance (H)
par.base.Psi = par.base.V / par.base.w;     % Base flux (V*s)

par.system.H = 3.5;                         % Inertia constant MWs/MVA
par.system.Kd = 0.5;                        % Damping constant / friction

par.line.pu.Xt = 0.1;
par.line.pu.X = 0.1;
par.line.pu.R = par.line.pu.X/20;

par.line.si.R = par.line.pu.R * par.base.Z;
par.line.si.L = par.line.pu.X * par.base.L;
par.line.si.Lt = par.line.pu.Xt * par.base.L;

% Machine 1
par.machine.pu.Ra = 0.0030;
par.machine.pu.Rfd = 0.000805721899402720;
par.machine.pu.R1d = 0.108313780715318;
par.machine.pu.R1q = 0.348452867829983; 
par.machine.pu.Xfd = 0.472500000000000;
par.machine.pu.Xad = 1.35000000000000;
par.machine.pu.Xaq = 0.850000000000000;
par.machine.pu.Xl = 0.15;
par.machine.pu.X1d = 0.466666666666667;
par.machine.pu.X1q = 0.463636363636364;
par.machine.pu.Xd = par.machine.pu.Xl + par.machine.pu.Xad;
par.machine.pu.Xq = par.machine.pu.Xl + par.machine.pu.Xaq;
par.machine.pu.Xd_d = 0.5;
par.machine.pu.Xd_dd = 0.35;
par.machine.pu.Xq_dd = 0.45;
par.machine.pu.Td0_d = 6;
par.machine.pu.Td0_dd = 0.02;
par.machine.pu.Tq0_dd = 0.01;

% Machine 2
par.machine.pu.Ra = 0.0030;
par.machine.pu.Rfd = 0.000537147932935147;
par.machine.pu.R1d = 0.0433255122861271;
par.machine.pu.R1q = 0.0387169853144426; 
par.machine.pu.Xfd = 0.472500000000000;
par.machine.pu.Xad = 1.35000000000000;
par.machine.pu.Xaq = 0.850000000000000;
par.machine.pu.Xl = 0.15;
par.machine.pu.X1d = 0.466666666666667;
par.machine.pu.X1q = 0.463636363636364;
par.machine.pu.Xd = par.machine.pu.Xl + par.machine.pu.Xad;
par.machine.pu.Xq = par.machine.pu.Xl + par.machine.pu.Xaq;
par.machine.pu.Xd_d = 0.5;
par.machine.pu.Xd_dd = 0.35;
par.machine.pu.Xq_dd = 0.45;
par.machine.pu.Td0_d = 9;
par.machine.pu.Td0_dd = 0.05;
par.machine.pu.Tq0_dd = 0.09;

% Simulation setpoint
par.setpoint.pu.grid_voltage = 0.995;
par.setpoint.si.grid_voltage = sqrt(3/2) * par.base.V * par.setpoint.pu.grid_voltage;

% Used in the speed estimation algorithm
par.machine.pu.Ld = par.machine.pu.Xd/par.base.w;
par.machine.pu.Lq = par.machine.pu.Xq/par.base.w;
par.machine.pu.L0 = (par.machine.pu.Ld+par.machine.pu.Lq)/2;
par.machine.pu.L1 = (par.machine.pu.Ld-par.machine.pu.Lq)/2;
