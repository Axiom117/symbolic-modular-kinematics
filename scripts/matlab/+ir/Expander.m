classdef Expander < handle
%EXPANDER  IR expander: DSL + module library + parameter config → IR graph.
%   A handle-class that encapsulates the full DSL→IR expansion pipeline:
%   load mechanism DSL, resolve module library, inject geometric/joint
%   parameters, expand every instance's internal frame graph into the
%   shared EdgeGraph, process port-to-port connections (mate edges),
%   register ground nodes, and propagate global poses via FK.
%
%   EXPANDER is the extraction target of A.3.0: all logic previously
%   embedded in +viz/mechanism.m's setup section and local functions
%   (localExpandInstance, localParsePort, localLookupFrame) lives here.
%   The visualization layer (+viz/mechanism) now only reads the public
%   properties and renders.
%
%   Usage:
%       e = ir.Expander('../../specs/dsl/examples/open-chain-2r/robot_description.yaml', ...
%                        '../../specs/dsl/examples/open-chain-2r/joint_config.yaml');
%       % e.MechName, e.Instances, e.ConnectionInfo, e.Poses, e.LibDir
%
%   See also: +ir/EdgeGraph, +viz/mechanism, +core/PoseGraph

    % ---- public read-only properties ----
    properties (SetAccess = private)
        MechName       (1,:) char            % mechanism name from DSL
        Instances      (:,1) struct          % struct array: name, type, md, bodies, frames, joints
        ConnectionInfo (:,1) struct          % struct array: socketNode, plugNode, closed, label
        Poses                              % containers.Map: frame name → 4×4 homogeneous transform
        LibDir         (1,:) char            % absolute path to the module library directory
    end

    % ---- private properties ----
    properties (Access = private)
        EdgeGraph_                          % ir.EdgeGraph handle (internal accumulator)
        DefCache_                           % containers.Map: module_type → parsed module def
    end

    % ---- public methods ----
    methods

        %% Constructor: run the full DSL→IR expansion pipeline.
        %   obj = Expander(DSLYAML, CONFIGYAML)
        %     DSLYAML    : path to mechanism-assembly DSL file
        %     CONFIGYAML : (optional) path to per-instance joint variable config
        function obj = Expander(dslYaml, configYaml)
            if nargin < 1 || isempty(dslYaml)
                error('ir:Expander:usage', ...
                    'Usage: ir.Expander(dslYaml[, configYaml])');
            end
            if nargin < 2; configYaml = ''; end

            % ---- resolve paths ----
            % 'here' = scripts/matlab/ (two levels up from +ir/)
            here = fileparts(fileparts(mfilename('fullpath')));

            dslYaml = core.PathUtils.resolve(dslYaml, here);
            assert(exist(dslYaml, 'file') > 0, 'DSL file not found: %s', dslYaml);

            % ---- load & validate DSL ----
            dsl = core.readYaml(dslYaml);
            assert(isfield(dsl, 'mechanism'), 'Not a mechanism assembly: %s', dslYaml);

            ver = core.CommonUtils.field(dsl, 'dsl_version', 0);
            assert(isequal(ver, 0), 'Unsupported dsl_version %s (expected 0).', num2str(ver));

            obj.MechName = dsl.mechanism;
            dslDir = fileparts(dslYaml);

            libRel = core.CommonUtils.field(dsl, 'module_library', '../../modules/');
            obj.LibDir = core.PathUtils.resolve(libRel, dslDir);
            assert(exist(obj.LibDir, 'dir') > 0, 'Module library not found: %s', libRel);

            % ---- load parameter configs ----
            % geometric parameters: <libDir>/config/dimensions.yaml (keyed by module_type)
            dimCfg = struct();
            dimCfgPath = fullfile(obj.LibDir, 'config', 'dimensions.yaml');
            if exist(dimCfgPath, 'file'); dimCfg = core.readYaml(dimCfgPath); end

            % per-instance joint variable overrides: joint_config.yaml (keyed by instance name)
            jointCfg = struct();
            if ~isempty(configYaml)
                configYaml = core.PathUtils.resolve(configYaml, here);
                if exist(configYaml, 'file'); jointCfg = core.readYaml(configYaml); end
            end

            % ---- expand instances ----
            assert(isfield(dsl, 'instances') && isstruct(dsl.instances), ...
                'Mechanism %s declares no instances.', obj.MechName);
            instNames = fieldnames(dsl.instances);
            nInst = numel(instNames);

            obj.EdgeGraph_ = ir.EdgeGraph();
            obj.DefCache_ = containers.Map('KeyType', 'char', 'ValueType', 'any');

            obj.Instances = struct('name', {}, 'type', {}, 'md', {}, ...
                'bodies', {}, 'frames', {}, 'joints', {});

            for i = 1:nInst
                iname = instNames{i};
                itype = dsl.instances.(iname).type;
                obj.Instances(i) = obj.localExpandInstance( ...
                    iname, itype, dimCfg, jointCfg);
            end

            % ---- process connections (mate edges) ----
            conns = core.CommonUtils.asList(core.CommonUtils.field(dsl, 'connections', {}));
            obj.ConnectionInfo = struct('socketNode', {}, 'plugNode', {}, ...
                'closed', {}, 'label', {});

            for c = 1:numel(conns)
                cn = conns{c};
                assert(isfield(cn, 'ports') && numel(cn.ports) == 2, ...
                    'Connection %d must list exactly two ports.', c);
                refA = cn.ports{1}; refB = cn.ports{2};
                [ia, pa] = obj.localParsePort(refA);
                [ib, pb] = obj.localParsePort(refB);
                fa = obj.localLookupFrame(ia, pa);
                fb = obj.localLookupFrame(ib, pb);
                assert(~isempty(fa), 'Connection %d: unknown port "%s".', c, refA);
                assert(~isempty(fb), 'Connection %d: unknown port "%s".', c, refB);

                % polarity check: one socket + one plug
                polA = fa.polarity; polB = fb.polarity;
                if strcmp(polA, 'socket') && strcmp(polB, 'plug')
                    sk = fa; pl = fb;
                elseif strcmp(polA, 'plug') && strcmp(polB, 'socket')
                    sk = fb; pl = fa;
                else
                    error('ir:Expander:polarity', ...
                        ['Connection %d [%s ~ %s]: expected one socket + one plug, ' ...
                         'got "%s" + "%s".'], c, refA, refB, ...
                        core.CommonUtils.tern(isempty(polA), 'none', polA), ...
                        core.CommonUtils.tern(isempty(polB), 'none', polB));
                end

                % connection parameters: roll and symmetry
                roll = core.CommonUtils.field(cn, 'roll', 0);
                sym = core.CommonUtils.field(sk, 'symmetry', 4);
                isClosed = isequal(core.CommonUtils.field(cn, 'closed', false), true);

                if ~isClosed
                    obj.EdgeGraph_.addMate(sk.node, pl.node, roll, sym);
                else
                    obj.EdgeGraph_.addClosedMate(sk.node, pl.node, roll, sym);
                end

                obj.ConnectionInfo(end+1) = struct( ...
                    'socketNode', sk.node, 'plugNode', pl.node, ...
                    'closed', isClosed, ...
                    'label', sprintf('%s~%s', sk.node, pl.node)); %#ok<AGROW>
            end

            % ---- ground nodes & FK propagation ----
            if ~obj.EdgeGraph_.hasGroundNodes()
                obj.EdgeGraph_.addGround(obj.Instances(1).bodies{1}.node);
            end
            obj.Poses = obj.EdgeGraph_.propagate();
        end

    end

    % ---- private methods (migrated from +viz/mechanism.m local functions) ----
    methods (Access = private)

        %% expand a single instance: load module def, inject params, expand bodies/frames/edges
        function inst_i = localExpandInstance(obj, iname, itype, dimCfg, jointCfg)
            if isKey(obj.DefCache_, itype)
                md = obj.DefCache_(itype);
            else
                fp = fullfile(obj.LibDir, [itype '.yaml']);
                assert(exist(fp, 'file') > 0, ...
                    'Instance "%s": module type "%s" not found in %s.', ...
                    iname, itype, obj.LibDir);
                md = core.readYaml(fp);
                obj.DefCache_(itype) = md;
            end

            % inject geometric parameters (cubeLength, tipDistance, ...)
            % from <module_library>/config/dimensions.yaml
            params = struct();
            if isfield(dimCfg, itype)
                params = dimCfg.(itype);
            end

            % overlay per-instance joint variable overrides from joint_config.yaml
            if isfield(jointCfg, iname)
                ov = jointCfg.(iname);
                if isstruct(ov)
                    ofn = fieldnames(ov);
                    for q = 1:numel(ofn); params.(ofn{q}) = ov.(ofn{q}); end
                end
            end

            % instance name prefix for all node names: "inst1.body"
            pre = [iname '.'];

            % ---- expand bodies ----
            bodies = core.CommonUtils.asList(core.CommonUtils.field(md, 'bodies', {}));
            bList = cell(1, numel(bodies));
            for k = 1:numel(bodies)
                b = bodies{k};
                bList{k} = struct('node', [pre b.name], 'name', b.name, ...
                    'geometry', core.CommonUtils.field(b, 'geometry', ''));
            end

            % ---- expand frames ----
            frames = core.CommonUtils.asList(core.CommonUtils.field(md, 'frames', {}));
            fList = cell(1, numel(frames));
            for k = 1:numel(frames)
                f = frames{k};
                node = [pre f.name];
                fList{k} = struct('node', node, 'name', f.name, ...
                    'exposed', isfield(f, 'exposed') && isequal(f.exposed, true), ...
                    'polarity', core.CommonUtils.field(f, 'polarity', ''), ...
                    'semantic_tag', core.CommonUtils.field(f, 'semantic_tag', ''), ...
                    'symmetry', core.CommonUtils.field(f, 'symmetry', 4));

                % auto-register ground frames
                if strcmp(core.CommonUtils.field(f, 'semantic_tag', ''), 'ground')
                    obj.EdgeGraph_.addGround(node);
                end
            end

            % ---- expand fixed transforms ----
            fts = core.CommonUtils.asList(core.CommonUtils.field(md, 'fixed_transforms', {}));
            for k = 1:numel(fts)
                t = fts{k};
                tr = core.CommonUtils.evalVec(t.translation, params);
                R = core.RigidBodyMath.rot(t.rotation, params);
                T = core.RigidBodyMath.T(R, tr);
                obj.EdgeGraph_.addFixedTransform([pre t.from_frame], [pre t.to_frame], T);
            end

            % ---- expand joints ----
            jts = core.CommonUtils.asList(core.CommonUtils.field(md, 'joints', {}));
            jList = cell(1, numel(jts));
            for k = 1:numel(jts)
                j = jts{k};
                ax = core.CommonUtils.evalVec(j.axis, params);
                val = core.CommonUtils.field(params, j.variable, 0);
                kind = core.CommonUtils.field(j, 'kind', 'revolute');
                obj.EdgeGraph_.addJoint([pre j.from_frame], [pre j.to_frame], ax, val, kind);
                jList{k} = struct('node', [pre j.from_frame], ...
                    'axis', ax(:) / max(norm(ax), eps), ...
                    'var', j.variable, 'val', val, 'kind', kind);
            end

            % ---- assemble instance struct ----
            inst_i = struct('name', iname, 'type', itype, 'md', md, ...
                'bodies', {bList}, 'frames', {fList}, 'joints', {jList});
        end

        %% split an "instance.port" reference into its two parts
        function [instName, portName] = localParsePort(~, ref)
            d = strfind(ref, '.');
            assert(~isempty(d), ...
                'Malformed port reference "%s" (expected instance.port).', ref);
            instName = ref(1:d(1)-1);
            portName = ref(d(1)+1:end);
        end

        %% find the recorded frame struct for instance/port; [] if absent
        function f = localLookupFrame(obj, instName, portName)
            f = [];
            for i = 1:numel(obj.Instances)
                if ~strcmp(obj.Instances(i).name, instName); continue; end
                for k = 1:numel(obj.Instances(i).frames)
                    if strcmp(obj.Instances(i).frames{k}.name, portName)
                        f = obj.Instances(i).frames{k};
                        return;
                    end
                end
            end
        end

    end

end
