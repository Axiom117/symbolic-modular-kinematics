classdef ExecutionConfig
%EXECUTIONCONFIG  L3 execution configuration: load, validate, and partition variables.
%   A value class that loads an L3 execution-config YAML file, validates it
%   against the JSON Schema and the symbolRegistry, and provides methods to
%   partition variables into known/unknown sets for FK or IK solving.
%
%   For closed_loop mode, closure_cuts may be:
%     - Explicitly declared in the YAML (required for L3 world系 loops, e.g. M-REx).
%     - Auto-derived from EdgeGraph's closed_mate edges when omitted
%       (for L2 internal loops where DSL already declares closed: true).
%   Likewise, world_binding may be:
%     - Explicitly declared in the YAML (required for L3 world系 loops).
%     - Omitted for L2 internal loops (ground handled by IR root nodes).
%
%   Usage:
%       e = ir.Expander(dslYaml);
%       cfg = ir.ExecutionConfig('execution-config.yaml', e.SymbolRegistry, e.EdgeGraph_);
%       % cfg.Mode         — 'open_loop' or 'closed_loop'
%       % cfg.EndFrame     — target frame ref (e.g. 'pipette.tip_origin')
%       % cfg.ClosureCuts  — explicit or auto-derived cut pairs
%       % cfg.getSolvingDirection()   — 'FK' or 'IK'
%       % [knownV, unknownV] = cfg.partitionVariables();
%
%   See also: +ir/Expander, +solver/KinematicModel, +ir/EdgeGraph

    % ---- public read-only properties ----
    properties (SetAccess = private)
        Mode          (1,:) char            % 'open_loop' or 'closed_loop'
        EndFrame      (1,:) char            % target frame ref (instance-qualified)
        KnownList     (:,1) cell            % cell array of known variable refs
        UnknownList   (:,1) cell            % cell array of unknown variable refs
        WorldBindings (:,1) struct          % struct array: ground, T (4×4 double); empty for open_loop
        ActuatedJoints (:,1) cell           % cell array of actuated joint refs; empty for closed_loop
        ClosureCuts   (:,1) struct          % struct array: near, far, components; empty for open_loop
        ClosureSource (1,:) char            % 'explicit' (from YAML), 'auto' (from closed_mate edges), or '' (open_loop)
        Tolerances    (1,1) struct          % struct: translation_mm, rotation_rad
        ConfigPath    (1,:) char            % absolute path to the loaded config file
    end

    % ---- private properties ----
    properties (Access = private)
        SymbolRegistry_ (:,1) struct        % copy of the symbolRegistry for validation
    end

    % ---- public methods ----
    methods

        %% Constructor: load execution-config, validate schema + cross-refs.
        %   obj = ExecutionConfig(CONFIGYAML, SYMBOLREGISTRY)
        %   obj = ExecutionConfig(CONFIGYAML, SYMBOLREGISTRY, EDGEGRAPH)
        %     CONFIGYAML      : path to L3 execution-config YAML file
        %     SYMBOLREGISTRY  : struct array from Expander.SymbolRegistry
        %     EDGEGRAPH       : (optional) ir.EdgeGraph — needed to auto-derive
        %                       closure_cuts from closed_mate edges when omitted
        %                       in the YAML (L2 internal loops).
        function obj = ExecutionConfig(configYaml, symbolRegistry, edgeGraph)
            arguments
                configYaml      (1,:) char
                symbolRegistry  (:,1) struct
                edgeGraph       = []  % optional; only needed for auto-derivation
            end

            % ---- resolve path ----
            here = fileparts(fileparts(mfilename('fullpath')));
            configYaml = core.PathUtils.resolve(configYaml, here);
            assert(exist(configYaml, 'file') > 0, ...
                'ir:ExecutionConfig:fileNotFound', ...
                'Execution config file not found: %s', configYaml);
            obj.ConfigPath = configYaml;

            % ---- load YAML ----
            cfg = core.readYaml(configYaml);

            % ---- cross-validate required fields ----
            assert(isfield(cfg, 'mode'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "mode" field.');
            assert(isfield(cfg, 'endFrame'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "endFrame" field.');
            assert(isfield(cfg, 'known'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "known" field.');
            assert(isfield(cfg, 'unknown'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "unknown" field.');

            % ---- store mode and endFrame ----
            obj.Mode = cfg.mode;
            obj.EndFrame = cfg.endFrame;

            % ---- validate mode ----
            validModes = {'open_loop', 'closed_loop'};
            assert(ismember(obj.Mode, validModes), ...
                'ir:ExecutionConfig:invalidMode', ...
                'mode must be "open_loop" or "closed_loop", got "%s".', obj.Mode);

            % ---- store known/unknown lists ----
            obj.KnownList = core.CommonUtils.asList(cfg.known);
            obj.UnknownList = core.CommonUtils.asList(cfg.unknown);

            % ---- store symbolRegistry for validation ----
            obj.SymbolRegistry_ = symbolRegistry;

            % ---- validate mode-conditional fields ----
            switch obj.Mode
                case 'open_loop'
                    % actuated_joints required, closure_cuts + world_binding forbidden
                    assert(isfield(cfg, 'actuated_joints'), ...
                        'ir:ExecutionConfig:missingField', ...
                        'open_loop mode requires "actuated_joints" field.');
                    obj.ActuatedJoints = core.CommonUtils.asList(cfg.actuated_joints);
                    obj.WorldBindings = struct('ground', {}, 'T', {});
                    obj.ClosureCuts = struct('near', {}, 'far', {}, 'components', {});

                    assert(~isfield(cfg, 'closure_cuts'), ...
                        'ir:ExecutionConfig:fieldConflict', ...
                        'open_loop mode forbids "closure_cuts".');
                    assert(~isfield(cfg, 'world_binding'), ...
                        'ir:ExecutionConfig:fieldConflict', ...
                        'open_loop mode forbids "world_binding".');

                case 'closed_loop'
                    obj.ActuatedJoints = {};
                    obj.ClosureSource = '';

                    assert(~isfield(cfg, 'actuated_joints'), ...
                        'ir:ExecutionConfig:fieldConflict', ...
                        'closed_loop mode forbids "actuated_joints".');

                    % --- closure_cuts: explicit YAML, auto-derived, or error ---
                    if isfield(cfg, 'closure_cuts')
                        % explicit cuts from YAML (L3 world系 loops)
                        cutsRaw = core.CommonUtils.asList(cfg.closure_cuts);
                        obj.ClosureCuts = struct('near', {}, 'far', {}, 'components', {});
                        for i = 1:numel(cutsRaw)
                            cut = cutsRaw{i};
                            comps = core.CommonUtils.field(cut, 'components', ...
                                {'tx','ty','tz','rx','ry','rz'});
                            obj.ClosureCuts(end+1) = struct( ... %#ok<AGROW>
                                'near', cut.near, ...
                                'far', cut.far, ...
                                'components', {comps});
                        end
                        obj.ClosureSource = 'explicit';
                    elseif ~isempty(edgeGraph)
                        % auto-derive from closed_mate edges (L2 internal loops)
                        allMates = edgeGraph.findMates();
                        closedMask = strcmp({allMates.kind}, 'closed_mate');
                        closedMates = allMates(closedMask);

                        assert(~isempty(closedMates), ...
                            'ir:ExecutionConfig:noClosureCuts', ...
                            ['closed_loop mode: no closure_cuts in YAML and ' ...
                             'no closed_mate edges in EdgeGraph. ' ...
                             'Either declare closure_cuts explicitly (L3 loop) ' ...
                             'or ensure DSL contains closed: true connections (L2 loop).']);

                        obj.ClosureCuts = struct('near', {}, 'far', {}, 'components', {});
                        for i = 1:numel(closedMates)
                            % closed_mate edges are directional: socket → plug.
                            % The socket side (from) is on the spanning tree;
                            % the plug side (to) is the chord end.
                            % Both are reachable via FK (graph is connected through
                            % bidirectional mate edges).
                            obj.ClosureCuts(end+1) = struct( ... %#ok<AGROW>
                                'near', closedMates(i).from, ...   % socket (tree side)
                                'far',  closedMates(i).to, ...     % plug  (chord side)
                                'components', {{'tx','ty','tz','rx','ry','rz'}});
                        end
                        obj.ClosureSource = 'auto';
                    else
                        error('ir:ExecutionConfig:noClosureCuts', ...
                            ['closed_loop mode: closure_cuts not declared in YAML ' ...
                             'and no EdgeGraph provided for auto-derivation. ' ...
                             'Pass EdgeGraph as the third argument, or declare ' ...
                             'closure_cuts explicitly.']);
                    end

                    % --- world_binding: explicit YAML, or empty (L2 internal loops) ---
                    if isfield(cfg, 'world_binding')
                        wbRaw = core.CommonUtils.asList(cfg.world_binding);
                        obj.WorldBindings = struct('ground', {}, 'T', {});
                        for i = 1:numel(wbRaw)
                            wb = wbRaw{i};
                            assert(isfield(wb, 'ground') && isfield(wb, 'T'), ...
                                'ir:ExecutionConfig:invalidWorldBinding', ...
                                'world_binding entry %d must contain "ground" and "T".', i);
                            obj.WorldBindings(end+1) = struct( ... %#ok<AGROW>
                                'ground', wb.ground, ...
                                'T', wb.T);
                        end
                    else
                        % L2 internal loop: ground handled by IR root nodes,
                        % no explicit world_binding needed.
                        obj.WorldBindings = struct('ground', {}, 'T', {});
                    end
            end

            % ---- tolerances (optional, with defaults) ----
            obj.Tolerances = struct( ...
                'translation_mm', 0.001, ...
                'rotation_rad', 0.001);
            if isfield(cfg, 'tolerances')
                tol = cfg.tolerances;
                if isfield(tol, 'translation_mm')
                    obj.Tolerances.translation_mm = tol.translation_mm;
                end
                if isfield(tol, 'rotation_rad')
                    obj.Tolerances.rotation_rad = tol.rotation_rad;
                end
            end

            % ---- cross-validate against symbolRegistry ----
            obj.validateAgainstRegistry();
        end

        %% partitionVariables  Split registry into known and unknown sets.
        %   [knownVars, unknownVars] = obj.partitionVariables()
        %     knownVars   : struct array — subset of symbolRegistry marked as known
        %     unknownVars : struct array — subset of symbolRegistry marked as unknown
        function [knownVars, unknownVars] = partitionVariables(obj)
            knownVars = obj.lookupByNameList(obj.KnownList);
            unknownVars = obj.lookupByNameList(obj.UnknownList);
        end

        %% getSolvingDirection  Determine FK or IK based on known/unknown partition.
        %   dir = obj.getSolvingDirection()
        %     returns 'FK' if endFrame is in unknown (joint vals known → compute pose)
        %     returns 'IK' if endFrame is in known (target pose known → solve joint vals)
        function dir = getSolvingDirection(obj)
            if ismember(obj.EndFrame, obj.UnknownList)
                dir = 'FK';
            elseif ismember(obj.EndFrame, obj.KnownList)
                dir = 'IK';
            else
                error('ir:ExecutionConfig:endFrameNotPartitioned', ...
                    'endFrame "%s" must appear in either known or unknown list.', ...
                    obj.EndFrame);
            end
        end

        %% getKnownJointVars  Return known joint variables with their sym handles.
        %   kjv = obj.getKnownJointVars()
        %     returns struct array: name, symHandle — for joint-type known vars
        function kjv = getKnownJointVars(obj)
            [knownVars, ~] = obj.partitionVariables();
            jointMask = strcmp({knownVars.type}, 'joint');
            kjv = knownVars(jointMask);
        end

        %% getUnknownJointVars  Return unknown joint variables with their sym handles.
        %   ujv = obj.getUnknownJointVars()
        %     returns struct array: name, symHandle — for joint-type unknown vars
        function ujv = getUnknownJointVars(obj)
            [~, unknownVars] = obj.partitionVariables();
            jointMask = strcmp({unknownVars.type}, 'joint');
            ujv = unknownVars(jointMask);
        end

    end

    % ---- private methods ----
    methods (Access = private)

        %% validateAgainstRegistry  Cross-validate all refs against symbolRegistry.
        function validateAgainstRegistry(obj)
            regNames = {obj.SymbolRegistry_.name};

            % -- endFrame must exist in registry --
            assert(ismember(obj.EndFrame, regNames), ...
                'ir:ExecutionConfig:refNotFound', ...
                'endFrame "%s" not found in symbolRegistry.', obj.EndFrame);

            % -- all known/unknown refs must exist --
            allRefs = [obj.KnownList(:); obj.UnknownList(:)];
            for i = 1:numel(allRefs)
                ref = allRefs{i};
                assert(ismember(ref, regNames), ...
                    'ir:ExecutionConfig:refNotFound', ...
                    'Variable ref "%s" (in known/unknown) not found in symbolRegistry.', ref);
            end

            % -- known + unknown must cover all joint variables exactly --
            jointInReg = regNames(strcmp({obj.SymbolRegistry_.type}, 'joint'));
            partitionedJoints = intersect([obj.KnownList(:); obj.UnknownList(:)], jointInReg);
            missingJoints = setdiff(jointInReg, partitionedJoints);
            extraJoints = setdiff(partitionedJoints, jointInReg);
            assert(isempty(missingJoints), ...
                'ir:ExecutionConfig:incompletePartition', ...
                'Joint variable(s) not covered by known/unknown: %s', ...
                strjoin(missingJoints, ', '));
            assert(isempty(extraJoints), ...
                'ir:ExecutionConfig:extraPartition', ...
                'Non-joint ref(s) in known/unknown: %s', ...
                strjoin(extraJoints, ', '));

            % -- no overlap between known and unknown --
            overlap = intersect(obj.KnownList, obj.UnknownList);
            assert(isempty(overlap), ...
                'ir:ExecutionConfig:overlap', ...
                'Variable(s) appear in both known and unknown: %s', ...
                strjoin(overlap, ', '));

            % -- open_loop: all actuated_joints must be joint-type in registry --
            if ~isempty(obj.ActuatedJoints)
                for i = 1:numel(obj.ActuatedJoints)
                    ref = obj.ActuatedJoints{i};
                    assert(ismember(ref, regNames), ...
                        'ir:ExecutionConfig:refNotFound', ...
                        'actuated_joint "%s" not found in symbolRegistry.', ref);
                    idx = find(strcmp(regNames, ref), 1);
                    assert(strcmp(obj.SymbolRegistry_(idx).type, 'joint'), ...
                        'ir:ExecutionConfig:typeMismatch', ...
                        'actuated_joint "%s" is type "%s", expected "joint".', ...
                        ref, obj.SymbolRegistry_(idx).type);
                end
            end

            % -- closed_loop: closure_cuts near/far must exist --
            if ~isempty(obj.ClosureCuts)
                for i = 1:numel(obj.ClosureCuts)
                    cut = obj.ClosureCuts(i);
                    assert(ismember(cut.near, regNames), ...
                        'ir:ExecutionConfig:refNotFound', ...
                        'closure_cut near "%s" not found in symbolRegistry.', cut.near);
                    assert(ismember(cut.far, regNames), ...
                        'ir:ExecutionConfig:refNotFound', ...
                        'closure_cut far "%s" not found in symbolRegistry.', cut.far);
                end
            end
        end

        %% lookupByNameList  Find registry entries matching a list of names.
        function entries = lookupByNameList(obj, nameList)
            entries = struct('name', {}, 'type', {}, 'symHandle', {}, ...
                'scope', {}, 'module_type', {}, 'instance', {});
            regNames = {obj.SymbolRegistry_.name};
            for i = 1:numel(nameList)
                idx = find(strcmp(regNames, nameList{i}), 1);
                if ~isempty(idx)
                    entries(end+1) = obj.SymbolRegistry_(idx); %#ok<AGROW>
                end
            end
        end

    end

end
