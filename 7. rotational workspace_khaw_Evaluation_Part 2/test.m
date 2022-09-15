%figures to keep
figs2keep = [1 3 4 10];

% Uncomment the following to 
% include ALL windows, including those with hidden handles (e.g. GUIs)
% all_figs = findall(0, 'type', 'figure');

all_figs = findobj(0, 'type', 'figure');
delete(setdiff(all_figs, figs2keep));