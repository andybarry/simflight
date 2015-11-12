trajlib_path = 'traj-archive/oct8-from-data-obstacle-ready.mat';

disp(['Loading ' trajlib_path '...']);
load(trajlib_path);


logs = [];
logs = [ logs FlightLog('2015-09-26', 1, '14', lib)];
logs = [ logs FlightLog('2015-09-26', 1, '15', lib)];
logs = [ logs FlightLog('2015-09-26', 1, '16', lib)];
logs = [ logs FlightLog('2015-09-26', 1, '18', lib)];

logs = [ logs FlightLog('2015-10-08', 3, '10', lib)];
logs = [ logs FlightLog('2015-10-08', 3, '12', lib)];
logs = [ logs FlightLog('2015-10-08', 3, '10', lib)];

logs = [ logs FlightLog('2015-10-10', 3, '02', lib)];
logs = [ logs FlightLog('2015-10-10', 3, '07', lib)];
logs = [ logs FlightLog('2015-10-10', 3, '08', lib)];

logs = [ logs FlightLog('2015-10-11', 3, '05', lib)];
logs = [ logs FlightLog('2015-10-11', 3, '11', lib)];
logs = [ logs FlightLog('2015-10-11', 3, '13', lib)];


disp('done loading all logs.');