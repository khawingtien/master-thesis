%figures to keep
figs2keep = [11, 13];

% Uncomment the following to 
% include ALL windows, including those with hidden handles (e.g. GUIs)
% all_figs = findall(0, 'type', 'figure');

all_figs = findobj(0, 'type', 'figure');
delete(setdiff(all_figs, figs2keep));