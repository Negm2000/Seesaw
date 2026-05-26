function add_annotations_test_model(mdl, title_text, vsat_note, inputs_pos, log_pos)
%ADD_ANNOTATIONS_TEST_MODEL  Add clean title + area annotations to a test model.
%
% Inputs:
%   mdl         model name (loaded)
%   title_text  string for top title
%   vsat_note   string for the saturation callout
%   inputs_pos  [L T R B] area rectangle for "Reference Protocol Inputs"
%   log_pos     [L T R B] area rectangle for "QUARC Logging"
%
% Wipes existing annotations / area blocks first so this is idempotent.

% Wipe annotations
ah = find_system(mdl,'FindAll','on','Type','annotation');
for i = 1:numel(ah); delete(ah(i)); end
% Wipe pre-existing area blocks we manage by name
for name = {'AREA_INPUTS','AREA_LOG'}
    bp = [mdl '/' name{1}];
    if getSimulinkBlockHandle(bp) ~= -1; delete_block(bp); end
end

% Title (plain note)
t = Simulink.Annotation([mdl '/TITLE']);
t.Text     = title_text;
t.FontSize = 20; t.FontWeight = 'bold';
t.Position = [inputs_pos(1), inputs_pos(2) - 80];

% V_sat callout
v = Simulink.Annotation([mdl '/VSAT_NOTE']);
v.Text     = vsat_note;
v.FontSize = 20; v.FontWeight = 'bold';
v.Position = [inputs_pos(1) + 700, inputs_pos(2) - 80];

% Area: Reference Inputs
add_block('built-in/Area', [mdl '/AREA_INPUTS'], ...
    'Position',        inputs_pos, ...
    'Text',            'Reference Protocol Inputs', ...
    'FontSize',        20, ...
    'FontWeight',      'bold', ...
    'BackgroundColor', '[0.85 0.92 1.0]', ...
    'ForegroundColor', '[0.1 0.3 0.6]');

% Area: QUARC Logging
add_block('built-in/Area', [mdl '/AREA_LOG'], ...
    'Position',        log_pos, ...
    'Text',            'QUARC Logging (To Host File)', ...
    'FontSize',        20, ...
    'FontWeight',      'bold', ...
    'BackgroundColor', '[0.90 1.00 0.90]', ...
    'ForegroundColor', '[0.10 0.50 0.20]');
end
