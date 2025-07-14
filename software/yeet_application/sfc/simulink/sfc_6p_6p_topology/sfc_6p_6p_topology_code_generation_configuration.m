Configuration = Simulink.ConfigSet;

% Original configuration set version: 23.1.0
if Configuration.versionCompare('23.1.0') < 0
    error('Simulink:MFileVersionViolation', 'The version of the target configuration set is older than the original configuration set.');
end

% Character encoding: UTF-8

% Do not change the order of the following commands. There are dependencies between the parameters.
Configuration.set_param('Name', 'Configuration'); % Name
Configuration.set_param('Description', ''); % Description

% Original configuration set target is ert.tlc
Configuration.switchTarget('ert.tlc','');

Configuration.set_param('HardwareBoard', 'None');   % Hardware board

Configuration.set_param('TargetLang', 'C++');   % Language

Configuration.set_param('CodeInterfacePackaging', 'C++ class');   % Code interface packaging

Configuration.set_param('GenerateAllocFcn', 'off');   % Use dynamic memory allocation for model initialization

Configuration.set_param('Solver', 'FixedStepDiscrete');   % Solver

% Solver
Configuration.set_param('StartTime', '0.0');   % Start time
Configuration.set_param('StopTime', 'inf');  % Or remove stop time constraint
Configuration.set_param('SolverName', 'FixedStepDiscrete');   % Solver
Configuration.set_param('SolverType', 'Fixed-step');   % Type
Configuration.set_param('SampleTimeConstraint', 'Unconstrained');   % Periodic sample time constraint
Configuration.set_param('FixedStep', '10e-6');  % 10µs for power electronics
Configuration.set_param('EnableFixedStepZeroCrossing', 'off');   % Enable zero-crossing detection for fixed-step simulation
Configuration.set_param('ConcurrentTasks', 'off');   % Allow tasks to execute concurrently on target
Configuration.set_param('EnableMultiTasking', 'on');   % Treat each discrete rate as a separate task
Configuration.set_param('AllowMultiTaskInputOutput', 'off');   % Allow multiple tasks to access inputs and outputs
Configuration.set_param('PositivePriorityOrder', 'off');   % Higher priority value indicates higher task priority
Configuration.set_param('AutoInsertRateTranBlk', 'off');   % Automatically handle rate transition for data transfer

% Data Import/Export
Configuration.set_param('Decimation', '1');   % Decimation
Configuration.set_param('LoadExternalInput', 'off');   % Load external input
Configuration.set_param('SaveFinalState', 'off');   % Save final state
Configuration.set_param('LoadInitialState', 'off');   % Load initial state
Configuration.set_param('LimitDataPoints', 'off');   % Limit data points
Configuration.set_param('SaveFormat', 'StructureWithTime');   % Format
Configuration.set_param('SaveOutput', 'off');   % Save output
Configuration.set_param('SaveState', 'off');   % Save states
Configuration.set_param('SignalLogging', 'off');   % Signal logging
Configuration.set_param('DSMLogging', 'off');   % Data stores
Configuration.set_param('StreamToWks', 'on');   % Stream To Workspace blocks
Configuration.set_param('InspectSignalLogs', 'off');   % Record logged workspace data in Simulation Data Inspector
Configuration.set_param('SaveTime', 'off');   % Save time
Configuration.set_param('ReturnWorkspaceOutputs', 'off');   % Single simulation output
Configuration.set_param('LoggingToFile', 'off');   % Log Dataset data to file
Configuration.set_param('DatasetSignalFormat', 'timeseries');   % Dataset signal format

% Optimization
Configuration.set_param('BlockReduction', 'on');   % Block reduction
Configuration.set_param('BooleanDataType', 'on');   % Implement logic signals as Boolean data (vs. double)
Configuration.set_param('ConditionallyExecuteInputs', 'on');   % Conditional input branch execution
Configuration.set_param('DefaultParameterBehavior', 'Inlined');   % Default parameter behavior
Configuration.set_param('UseDivisionForNetSlopeComputation', 'off');   % Use division for fixed-point net slope computation
Configuration.set_param('GainParamInheritBuiltInType', 'off');   % Gain parameters inherit a built-in integer type that is lossless
Configuration.set_param('UseFloatMulNetSlope', 'off');   % Use floating-point multiplication to handle net slope corrections
Configuration.set_param('InheritOutputTypeSmallerThanSingle', 'off');   % Inherit floating-point output type smaller than single precision
Configuration.set_param('DefaultUnderspecifiedDataType', 'double');   % Default for underspecified data type
Configuration.set_param('UseSpecifiedMinMax', 'off');   % Optimize using the specified minimum and maximum values
Configuration.set_param('InlineInvariantSignals', 'off');   % Inline invariant signals
Configuration.set_param('DataBitsets', 'off');   % Use bitsets for storing Boolean data
Configuration.set_param('StateBitsets', 'off');   % Use bitsets for storing state configuration
Configuration.set_param('LocalBlockOutputs', 'on');   % Enable local block outputs
Configuration.set_param('EnableMemcpy', 'off');   % Use memcpy for vector assignment
Configuration.set_param('OptimizeBlockIOStorage', 'off');   % Signal storage reuse
Configuration.set_param('ExpressionFolding', 'off');   % Eliminate superfluous local variables (expression folding)
Configuration.set_param('BufferReuse', 'off');   % Reuse local block outputs
Configuration.set_param('ReuseModelBlockBuffer', 'off');   % Reuse output buffers of Model blocks
Configuration.set_param('GlobalBufferReuse', 'off');   % Reuse global block outputs
Configuration.set_param('GlobalVariableUsage', 'None');   % Optimize global data access
Configuration.set_param('StrengthReduction', 'off');   % Simplify array indexing
Configuration.set_param('AdvancedOptControl', '');   % Disable incompatible optimizations
Configuration.set_param('BooleansAsBitfields', 'off');   % Pack Boolean data into bitfields
Configuration.set_param('OptimizeBlockOrder', 'off');   % Optimize block operation order in generated code
Configuration.set_param('OptimizeDataStoreBuffers', 'on');   % Reuse buffers for Data Store Read and Data Store Write blocks
Configuration.set_param('BusAssignmentInplaceUpdate', 'on');   % Perform in-place updates for Assignment and Bus Assignment blocks
Configuration.set_param('DifferentSizesBufferReuse', 'off');   % Reuse buffers of different sizes and dimensions
Configuration.set_param('OptimizationCustomize', 'on');   % Specify custom optimizations
Configuration.set_param('MultiThreadedLoops', 'off');   % Generate parallel for loops
Configuration.set_param('BitwiseOrLogicalOp', 'Same as modeled');   % Operator to represent Bitwise and Logical Operator blocks
Configuration.set_param('MemcpyThreshold', 64);   % Memcpy threshold (bytes)
Configuration.set_param('PassReuseOutputArgsAs', 'Individual arguments');   % Pass reusable subsystem outputs as
Configuration.set_param('PassReuseOutputArgsThreshold', 12);   % Maximum number of arguments for subsystem outputs
Configuration.set_param('RollThreshold', 5);   % Loop unrolling threshold
Configuration.set_param('ActiveStateOutputEnumStorageType', 'Native Integer');   % Base storage type for automatically created enumerations
Configuration.set_param('InitFltsAndDblsToZero', 'off');   % Use memset to initialize floats and doubles to 0.0
Configuration.set_param('NoFixptDivByZeroProtection', 'off');   % Remove code that protects against division arithmetic exceptions
Configuration.set_param('EfficientFloat2IntCast', 'off');   % Remove code from floating-point to integer conversions that wraps out-of-range values
Configuration.set_param('EfficientMapNaN2IntZero', 'on');   % Remove code from floating-point to integer conversions with saturation that maps NaN to zero
Configuration.set_param('LifeSpan', 'auto');   % Application lifespan (days)
Configuration.set_param('ClockResolution', '-1');   % Clock resolution (seconds, -1 for inherited)
Configuration.set_param('MaxStackSize', 'Inherit from target');   % Maximum stack size (bytes)
Configuration.set_param('BufferReusableBoundary', 'on');   % Buffer for reusable subsystems
Configuration.set_param('RemoveLocalVariableInitialization', 'off');   % Remove local variable initialization to zero value
Configuration.set_param('SimCompilerOptimization', 'off');   % Compiler optimization level
Configuration.set_param('AccelVerboseBuild', 'off');   % Verbose accelerator builds
Configuration.set_param('UseRowMajorAlgorithm', 'off');   % Use algorithms optimized for row-major array layout
Configuration.set_param('LabelGuidedReuse', 'off');   % Use signal labels to guide buffer reuse
Configuration.set_param('DenormalBehavior', 'GradualUnderflow');   % In accelerated simulation modes, denormal numbers can be flushed to zero using 'flush-to-zero' option.
Configuration.set_param('EfficientTunableParamExpr', 'off');   % Remove code from tunable parameter expressions that saturates out-of-range values

% Diagnostics
Configuration.set_param('RTPrefix', 'error');   % "rt" prefix for identifiers
Configuration.set_param('ConsistencyChecking', 'none');   % Solver data inconsistency
Configuration.set_param('ArrayBoundsChecking', 'none');   % Array bounds exceeded
Configuration.set_param('SignalInfNanChecking', 'none');   % Inf or NaN block output
Configuration.set_param('StringTruncationChecking', 'error');   % String truncation checking
Configuration.set_param('SignalRangeChecking', 'none');   % Simulation range checking
Configuration.set_param('ReadBeforeWriteMsg', 'DisableAll');   % Detect read before write
Configuration.set_param('WriteAfterWriteMsg', 'DisableAll');   % Detect write after write
Configuration.set_param('WriteAfterReadMsg', 'DisableAll');   % Detect write after read
Configuration.set_param('AlgebraicLoopMsg', 'warning');   % Algebraic loop
Configuration.set_param('ArtificialAlgebraicLoopMsg', 'warning');   % Minimize algebraic loop
Configuration.set_param('SaveWithDisabledLinksMsg', 'warning');   % Block diagram contains disabled library links
Configuration.set_param('SaveWithParameterizedLinksMsg', 'warning');   % Block diagram contains parameterized library links
Configuration.set_param('UnderspecifiedInitializationDetection', 'Simplified');   % Underspecified initialization detection
Configuration.set_param('MergeDetectMultiDrivingBlocksExec', 'error');   % Detect multiple driving blocks executing at the same time step
Configuration.set_param('SignalResolutionControl', 'UseLocalSettings');   % Signal resolution
Configuration.set_param('BlockPriorityViolationMsg', 'warning');   % Block priority violation
Configuration.set_param('TimeAdjustmentMsg', 'none');   % Sample hit time adjusting
Configuration.set_param('SolverPrmCheckMsg', 'none');   % Automatic solver parameter selection
Configuration.set_param('InheritedTsInSrcMsg', 'warning');   % Source block specifies -1 sample time
Configuration.set_param('MultiTaskDSMMsg', 'none');   % Multitask data store
Configuration.set_param('MultiTaskCondExecSysMsg', 'error');   % Multitask conditionally executed subsystem
Configuration.set_param('MultiTaskRateTransMsg', 'error');   % Multitask data transfer
Configuration.set_param('SingleTaskRateTransMsg', 'none');   % Single task data transfer
Configuration.set_param('TasksWithSamePriorityMsg', 'warning');   % Tasks with equal priority
Configuration.set_param('SigSpecEnsureSampleTimeMsg', 'warning');   % Enforce sample times specified by Signal Specification blocks
Configuration.set_param('CheckMatrixSingularityMsg', 'none');   % Division by singular matrix
Configuration.set_param('IntegerOverflowMsg', 'warning');   % Wrap on overflow
Configuration.set_param('Int32ToFloatConvMsg', 'warning');   % 32-bit integer to single precision float conversion
Configuration.set_param('ParameterDowncastMsg', 'error');   % Detect downcast
Configuration.set_param('ParameterOverflowMsg', 'error');   % Detect overflow
Configuration.set_param('ParameterUnderflowMsg', 'none');   % Detect underflow
Configuration.set_param('ParameterPrecisionLossMsg', 'warning');   % Detect precision loss
Configuration.set_param('ParameterTunabilityLossMsg', 'error');   % Detect loss of tunability
Configuration.set_param('FixptConstUnderflowMsg', 'none');   % Detect underflow
Configuration.set_param('FixptConstOverflowMsg', 'none');   % Detect overflow
Configuration.set_param('FixptConstPrecisionLossMsg', 'none');   % Detect precision loss
Configuration.set_param('UnderSpecifiedDataTypeMsg', 'none');   % Underspecified data types
Configuration.set_param('UnnecessaryDatatypeConvMsg', 'none');   % Unnecessary type conversions
Configuration.set_param('VectorMatrixConversionMsg', 'none');   % Vector/matrix block input conversion
Configuration.set_param('FcnCallInpInsideContextMsg', 'error');   % Context-dependent inputs
Configuration.set_param('SignalLabelMismatchMsg', 'none');   % Signal label mismatch
Configuration.set_param('UnconnectedInputMsg', 'none');   % Unconnected block input ports
Configuration.set_param('UnconnectedOutputMsg', 'none');   % Unconnected block output ports
Configuration.set_param('UnconnectedLineMsg', 'none');   % Unconnected line
Configuration.set_param('SFcnCompatibilityMsg', 'none');   % S-function upgrades needed
Configuration.set_param('FrameProcessingCompatibilityMsg', 'error');   % Block behavior depends on frame status of signal
Configuration.set_param('UniqueDataStoreMsg', 'none');   % Duplicate data store names
Configuration.set_param('BusObjectLabelMismatch', 'warning');   % Element name mismatch
Configuration.set_param('RootOutportRequireBusObject', 'warning');   % Unspecified bus object at root Outport block
Configuration.set_param('AssertControl', 'UseLocalSettings');   % Model Verification block enabling
Configuration.set_param('AllowSymbolicDim', 'off');   % Allow symbolic dimension specification
Configuration.set_param('ModelReferenceIOMsg', 'none');   % Invalid root Inport/Outport block connection
Configuration.set_param('ModelReferenceVersionMismatchMessage', 'none');   % Model block version mismatch
Configuration.set_param('ModelReferenceIOMismatchMessage', 'none');   % Port and parameter mismatch
Configuration.set_param('UnknownTsInhSupMsg', 'warning');   % Unspecified inheritability of sample time
Configuration.set_param('ModelReferenceDataLoggingMessage', 'warning');   % Unsupported data logging
Configuration.set_param('ModelReferenceNoExplicitFinalValueMsg', 'none');   % No explicit final value for model arguments
Configuration.set_param('ModelReferenceSymbolNameMessage', 'warning');   % Insufficient maximum identifier length
Configuration.set_param('ModelReferenceExtraNoncontSigs', 'error');   % Extraneous discrete derivative signals
Configuration.set_param('StateNameClashWarn', 'none');   % State name clash
Configuration.set_param('OperatingPointInterfaceChecksumMismatchMsg', 'warning');   % Operating point restore interface checksum mismatch
Configuration.set_param('NonCurrentReleaseOperatingPointMsg', 'error');   % Operating point object from a different release
Configuration.set_param('PregeneratedLibrarySubsystemCodeDiagnostic', 'warning');   % Behavior when pregenerated library subsystem code is missing
Configuration.set_param('SubsystemReferenceDiagnosticForUnitTest', 'error');   % Behavior when a matching unit test for subsystem reference is missing
Configuration.set_param('InitInArrayFormatMsg', 'warning');   % Initial state is array
Configuration.set_param('StrictBusMsg', 'ErrorLevel1');   % Bus signal treated as vector
Configuration.set_param('BusNameAdapt', 'WarnAndRepair');   % Repair bus selections
Configuration.set_param('NonBusSignalsTreatedAsBus', 'none');   % Non-bus signals treated as bus signals
Configuration.set_param('SFUnusedDataAndEventsDiag', 'warning');   % Unused data, events, messages and functions
Configuration.set_param('SFUnexpectedBacktrackingDiag', 'warning');   % Unexpected backtracking
Configuration.set_param('SFInvalidInputDataAccessInChartInitDiag', 'warning');   % Invalid input data access in chart initialization
Configuration.set_param('SFNoUnconditionalDefaultTransitionDiag', 'warning');   % No unconditional default transitions
Configuration.set_param('SFTransitionOutsideNaturalParentDiag', 'warning');   % Transition outside natural parent
Configuration.set_param('SFUnreachableExecutionPathDiag', 'warning');   % Unreachable execution path
Configuration.set_param('SFUndirectedBroadcastEventsDiag', 'warning');   % Undirected event broadcasts
Configuration.set_param('SFTransitionActionBeforeConditionDiag', 'warning');   % Transition action specified before condition action
Configuration.set_param('SFOutputUsedAsStateInMooreChartDiag', 'error');   % Read-before-write to output in Moore chart
Configuration.set_param('SFTemporalDelaySmallerThanSampleTimeDiag', 'warning');   % Absolute time temporal value shorter than sampling period
Configuration.set_param('SFSelfTransitionDiag', 'warning');   % Self-transition on leaf state
Configuration.set_param('SFExecutionAtInitializationDiag', 'none');   % 'Execute-at-initialization' disabled in presence of input events
Configuration.set_param('IntegerSaturationMsg', 'warning');   % Saturate on overflow
Configuration.set_param('AllowedUnitSystems', 'all');   % Allowed unit systems
Configuration.set_param('UnitsInconsistencyMsg', 'warning');   % Units inconsistency messages
Configuration.set_param('AllowAutomaticUnitConversions', 'on');   % Allow automatic unit conversions
Configuration.set_param('RCSCRenamedMsg', 'warning');   % Detect non-reused custom storage classes
Configuration.set_param('RCSCObservableMsg', 'warning');   % Detect ambiguous custom storage class final values
Configuration.set_param('ForceCombineOutputUpdateInSim', 'off');   % Combine output and update methods for code generation and simulation
Configuration.set_param('UnderSpecifiedDimensionMsg', 'none');   % Underspecified dimensions
Configuration.set_param('DebugExecutionForFMUViaOutOfProcess', 'off');   % FMU Import blocks
Configuration.set_param('ArithmeticOperatorsInVariantConditions', 'warning');   % Arithmetic operations in variant conditions
Configuration.set_param('VariantConditionMismatch', 'none');   % Variant condition mismatch at signal source and destination
Configuration.set_param('InheritVATfromSVC', 'warning');   % Variant activation time inherited from Simulink.VariantControl
Configuration.set_param('VariantConfigNotUsedByTopModel', 'warning');   % Variant configuration not used by top model
Configuration.set_param('ParamWriterValidationControl', 'UseLocalSettings');   % Parameter Writer block validation

% Hardware Implementation
Configuration.set_param('ProdHWDeviceType', 'Intel->x86-64 (Windows64)');   % Production device vendor and type
Configuration.set_param('ProdLongLongMode', 'off');   % Support long long
Configuration.set_param('ProdEqTarget', 'off');   % Test hardware is the same as production hardware
Configuration.set_param('TargetPreprocMaxBitsSint', 32);   % Maximum bits for signed integer in C preprocessor
Configuration.set_param('TargetPreprocMaxBitsUint', 32);   % Maximum bits for unsigned integer in C preprocessor
Configuration.set_param('HardwareBoardFeatureSet', 'EmbeddedCoderHSP');   % Feature set for selected hardware board

% Model Referencing
Configuration.set_param('UpdateModelReferenceTargets', 'IfOutOfDate');   % Rebuild
Configuration.set_param('EnableRefExpFcnMdlSchedulingChecks', 'on');   % Enable strict scheduling checks for referenced models
Configuration.set_param('EnableParallelModelReferenceBuilds', 'off');   % Enable parallel model reference builds
Configuration.set_param('ParallelModelReferenceErrorOnInvalidPool', 'on');   % Perform consistency check on parallel pool
Configuration.set_param('ModelReferenceNumInstancesAllowed', 'Multi');   % Total number of instances allowed per top model
Configuration.set_param('PropagateVarSize', 'Infer from blocks in model');   % Propagate sizes of variable-size signals
Configuration.set_param('ModelDependencies', '');   % Model dependencies
Configuration.set_param('ModelReferencePassRootInputsByReference', 'on');   % Pass fixed-size scalar root inputs by value for code generation
Configuration.set_param('ModelReferenceMinAlgLoopOccurrences', 'off');   % Minimize algebraic loop occurrences
Configuration.set_param('PropagateSignalLabelsOutOfModel', 'off');   % Propagate all signal labels out of the model
Configuration.set_param('SupportModelReferenceSimTargetCustomCode', 'off');   % Include custom code for referenced models
Configuration.set_param('UseModelRefSolver', 'off');   % Use local solver when referencing model

% Simulation Target
Configuration.set_param('SimCustomSourceCode', '');   % Additional code
Configuration.set_param('SimUserSources', '');   % Source files
Configuration.set_param('SimCustomHeaderCode', '');   % Include headers
Configuration.set_param('SimCustomInitializer', '');   % Initialize code
Configuration.set_param('SimCustomTerminator', '');   % Terminate code
Configuration.set_param('SimReservedNameArray', []);   % Reserved names
Configuration.set_param('SimUserIncludeDirs', '');   % Include directories
Configuration.set_param('SimUserLibraries', '');   % Libraries
Configuration.set_param('SimUserDefines', '');   % Defines
Configuration.set_param('SimCustomCompilerFlags', '');   % Compiler flags
Configuration.set_param('SimCustomLinkerFlags', '');   % Linker flags
Configuration.set_param('SFSimEnableDebug', 'off');   % Allow setting breakpoints during simulation
Configuration.set_param('SFSimEcho', 'off');   % Echo expressions without semicolons
Configuration.set_param('SimCtrlC', 'on');   % Break on Ctrl-C
Configuration.set_param('SimIntegrity', 'on');   % Enable memory integrity checks
Configuration.set_param('SimParseCustomCode', 'on');   % Import custom code
Configuration.set_param('SimDebugExecutionForCustomCode', 'off');   % Simulate custom code in a separate process
Configuration.set_param('SimAnalyzeCustomCode', 'off');   % Enable custom code analysis
Configuration.set_param('SimGenImportedTypeDefs', 'off');   % Generate typedefs for imported bus and enumeration types
Configuration.set_param('CompileTimeRecursionLimit', 50);   % Compile-time recursion limit for MATLAB functions
Configuration.set_param('EnableRuntimeRecursion', 'on');   % Enable run-time recursion for MATLAB functions
Configuration.set_param('EnableImplicitExpansion', 'on');   % Enable implicit expansion in MATLAB functions
Configuration.set_param('MATLABDynamicMemAlloc', 'off');   % Dynamic memory allocation in MATLAB functions
Configuration.set_param('LegacyBehaviorForPersistentVarInContinuousTime', 'off');   % Enable continuous-time MATLAB functions to write to initialized persistent variables
Configuration.set_param('CustomCodeFunctionArrayLayout', []);   % Exception by function...
Configuration.set_param('DefaultCustomCodeFunctionArrayLayout', 'NotSpecified');   % Default function array layout
Configuration.set_param('CustomCodeUndefinedFunction', 'UseInterfaceOnly');   % Undefined function and variable handling
Configuration.set_param('CustomCodeGlobalsAsFunctionIO', 'off');   % Enable global variables as function interfaces
Configuration.set_param('DefaultCustomCodeDeterministicFunctions', 'None');   % Deterministic functions
Configuration.set_param('SimHardwareAcceleration', 'generic');   % Hardware acceleration
Configuration.set_param('GPUAcceleration', 'off');   % GPU acceleration
Configuration.set_param('SimTargetLang', 'C++');   % Language
Configuration.set_param('SimDLTargetLibrary', 'mkl-dnn');   % Target library

% Code Generation
Configuration.set_param('GenerateGPUCode', 'None');   % Generate GPU code
Configuration.set_param('UseOperatorNewForModelRefRegistration', 'off');   % Use dynamic memory allocation for model block instantiation
Configuration.set_param('ExistingSharedCode', '');   % Existing shared code
Configuration.set_param('EmbeddedCoderDictionary', '');   % Shared coder dictionary
Configuration.set_param('TLCOptions', '');   % TLC command line options
Configuration.set_param('Toolchain', 'CMake');   % Toolchain
Configuration.set_param('PackageGeneratedCodeAndArtifacts', 'off');   % Package code and artifacts
Configuration.set_param('PostCodeGenCommand', '');   % Post code generation command
Configuration.set_param('GenerateReport', 'on');   % Create code generation report
Configuration.set_param('RTWVerbose', 'on');   % Verbose build
Configuration.set_param('RetainRTWFile', 'off');   % Retain .rtw file
Configuration.set_param('ProfileTLC', 'off');   % Profile TLC
Configuration.set_param('TLCDebug', 'off');   % Start TLC debugger when generating code
Configuration.set_param('TLCCoverage', 'off');   % Start TLC coverage when generating code
Configuration.set_param('TLCAssert', 'off');   % Enable TLC assertion
Configuration.set_param('BuiltinFFTWCallback', 'off');   % Built-in FFTW library callback
Configuration.set_param('RTWUseSimCustomCode', 'off');   % Use the same custom code settings as Simulation Target
Configuration.set_param('CustomSourceCode', '');   % Additional code
Configuration.set_param('CustomHeaderCode', '');   % Include headers
Configuration.set_param('CustomInclude', '');   % Include directories
Configuration.set_param('CustomSource', '');   % Source files
Configuration.set_param('CustomLibrary', '');   % Libraries
Configuration.set_param('CustomDefine', '');   % Defines
Configuration.set_param('CustomBLASCallback', '');   % Custom BLAS library callback
Configuration.set_param('CustomLAPACKCallback', '');   % Custom LAPACK library callback
Configuration.set_param('CustomFFTCallback', '');   % Custom FFT library callback
Configuration.set_param('CustomInitializer', '');   % Initialize code
Configuration.set_param('CustomTerminator', '');   % Terminate code
Configuration.set_param('PortableWordSizes', 'off');   % Enable portable word sizes
Configuration.set_param('CreateSILPILBlock', 'None');   % Create block
Configuration.set_param('CodeExecutionProfiling', 'off');   % Measure task execution time
Configuration.set_param('CodeProfilingInstrumentation', 'off');   % Measure function execution times
Configuration.set_param('CodeStackProfiling', 'off');   % Measure task stack usage
Configuration.set_param('CodeCoverageSettings', coder.coverage.CodeCoverageSettings([],'off','off','None'));   % Third-party tool
Configuration.set_param('SILPILDebugging', 'off');   % Enable source-level debugging for SIL or PIL
Configuration.set_param('DataTypeReplacement', 'CDataTypesFixedWidth');   % Data type replacement
Configuration.set_param('CoderTypedefsCompatibility', 'off');   % Coder typedefs compatibility
Configuration.set_param('CodeReplacementLibrary', 'None');   % Code replacement library
Configuration.set_param('ArrayLayout', 'Column-major');   % Array layout
Configuration.set_param('HalideCodeGeneration', 'off');   % Generate Halide code
Configuration.set_param('GenerateCodeMetricsReport', 'off');   % Generate static code metrics
Configuration.set_param('ObjectivePriorities', []);   % Prioritized objectives
Configuration.set_param('CheckMdlBeforeBuild', 'Warning');   % Check model before generating code
Configuration.set_param('DLTargetLibrary', 'none');   % Target library
Configuration.set_param('DLLearnablesCompression', 'None');   % Learnables compression
Configuration.set_param('GenerateComments', 'on');   % Include comments
Configuration.set_param('ForceParamTrailComments', 'on');   % Verbose comments for 'Model default' storage class
Configuration.set_param('CommentStyle', 'Auto');   % Comment style
Configuration.set_param('IgnoreCustomStorageClasses', 'off');   % Ignore custom storage classes
Configuration.set_param('IgnoreTestpoints', 'off');   % Ignore test point signals
Configuration.set_param('MaxIdLength', 128);   % Maximum identifier length
Configuration.set_param('ShowEliminatedStatement', 'on');   % Show eliminated blocks
Configuration.set_param('OperatorAnnotations', 'on');   % Operator annotations
Configuration.set_param('SimulinkDataObjDesc', 'on');   % Simulink data object descriptions
Configuration.set_param('SFDataObjDesc', 'on');   % Stateflow object descriptions
Configuration.set_param('MATLABFcnDesc', 'off');   % MATLAB user comments
Configuration.set_param('MangleLength', 1);   % Minimum mangle length
Configuration.set_param('SharedChecksumLength', 8);   % Shared checksum length
Configuration.set_param('CustomSymbolStrGlobalVar', '$R$N$M');   % Global variables
Configuration.set_param('CustomSymbolStrType', '$N$R$M_T');   % Global types
Configuration.set_param('CustomSymbolStrField', '$N$M');   % Field name of global types
Configuration.set_param('CustomSymbolStrFcn', '$R$N$M$F');   % Subsystem methods
Configuration.set_param('CustomSymbolStrFcnArg', 'rt$I$N$M');   % Subsystem method arguments
Configuration.set_param('CustomSymbolStrBlkIO', 'rtb_$N$M');   % Local block output variables
Configuration.set_param('CustomSymbolStrTmpVar', '$N$M');   % Local temporary variables
Configuration.set_param('CustomSymbolStrMacro', '$R$N$M');   % Constant macros
Configuration.set_param('CustomSymbolStrUtil', '$N$C');   % Shared utilities identifier format
Configuration.set_param('CustomSymbolStrEmxType', 'emxArray_$M$N');   % EMX array types identifier format
Configuration.set_param('CustomSymbolStrEmxFcn', 'emx$M$N');   % EMX array utility functions identifier format
Configuration.set_param('CustomUserTokenString', '');   % Custom token text
Configuration.set_param('EnableCustomComments', 'off');   % Custom comments (MPT objects only)
Configuration.set_param('DefineNamingRule', 'None');   % #define naming
Configuration.set_param('ParamNamingRule', 'None');   % Parameter naming
Configuration.set_param('SignalNamingRule', 'None');   % Signal naming
Configuration.set_param('InsertBlockDesc', 'on');   % Simulink block descriptions
Configuration.set_param('InsertPolySpaceComments', 'off');   % Insert Polyspace comments
Configuration.set_param('SimulinkBlockComments', 'on');   % Simulink block comments
Configuration.set_param('StateflowObjectComments', 'off');   % Stateflow object comments
Configuration.set_param('BlockCommentType', 'BlockPathComment');   % Trace to model using
Configuration.set_param('MATLABSourceComments', 'off');   % MATLAB source code as comments
Configuration.set_param('InternalIdentifier', 'Shortened');   % System-generated identifiers
Configuration.set_param('InlinedPrmAccess', 'Literals');   % Generate scalar inlined parameters as
Configuration.set_param('ReqsInCode', 'off');   % Requirements in block comments
Configuration.set_param('UseSimReservedNames', 'off');   % Use the same reserved names as Simulation Target
Configuration.set_param('ReservedNameArray', []);   % Reserved names
Configuration.set_param('EnumMemberNameClash', 'error');   % Duplicate enumeration member names
Configuration.set_param('TargetLibSuffix', '');   % Suffix applied to target library name
Configuration.set_param('TargetPreCompLibLocation', '');   % Precompiled library location
Configuration.set_param('TargetLangStandard', 'C++11 (ISO)');   % Language standard
Configuration.set_param('UtilityFuncGeneration', 'Auto');   % Shared code placement
Configuration.set_param('MultiwordTypeDef', 'System defined');   % Multiword type definitions
Configuration.set_param('DynamicStringBufferSize', 256);   % Buffer size of dynamically-sized string (bytes)
Configuration.set_param('GenerateFullHeader', 'on');   % Generate full file banner
Configuration.set_param('InferredTypesCompatibility', 'off');   % Create preprocessor directive in rtwtypes.h
Configuration.set_param('GenerateSampleERTMain', 'off');   % Generate an example main program
Configuration.set_param('IncludeMdlTerminateFcn', 'on');   % Terminate function required
Configuration.set_param('CombineOutputUpdateFcns', 'on');   % Single output/update function
Configuration.set_param('CombineSignalStateStructs', 'on');   % Combine signal/state structures
Configuration.set_param('MatFileLogging', 'off');   % MAT-file logging
Configuration.set_param('SuppressErrorStatus', 'off');   % Remove error status field in real-time model data structure
Configuration.set_param('ERTCustomFileBanners', 'on');   % Enable custom file banner
Configuration.set_param('SupportAbsoluteTime', 'on');   % Support absolute time
Configuration.set_param('PurelyIntegerCode', 'off');   % Support floating-point numbers
Configuration.set_param('SupportNonFinite', 'on');   % Support non-finite numbers
Configuration.set_param('SupportComplex', 'off');   % Support complex numbers
Configuration.set_param('SupportContinuousTime', 'off');   % Support continuous time
Configuration.set_param('SupportNonInlinedSFcns', 'off');   % Support non-inlined S-functions
Configuration.set_param('RemoveDisableFunc', 'off');   % Remove disable function
Configuration.set_param('RemoveResetFunc', 'on');   % Remove reset function
Configuration.set_param('SupportVariableSizeSignals', 'off');   % Support variable-size signals
Configuration.set_param('ParenthesesLevel', 'Nominal');   % Parentheses level
Configuration.set_param('CastingMode', 'Explicit');   % Casting modes
Configuration.set_param('LUTObjectStructOrderExplicitValues', 'Size,Breakpoints,Table');   % LUT object struct order for explicit value specification
Configuration.set_param('LUTObjectStructOrderEvenSpacing', 'Size,Breakpoints,Table');   % LUT object struct order for even spacing specification
Configuration.set_param('ERTHeaderFileRootName', '$R$E');   % Header files
Configuration.set_param('ERTSourceFileRootName', '$R$E');   % Source files
Configuration.set_param('ERTFilePackagingFormat', 'Compact');   % File packaging format
Configuration.set_param('ERTDataFileRootName', '$R_data');   % Data files
Configuration.set_param('InstructionSetExtensions', {'None'});   % Leverage target hardware instruction set extensions
Configuration.set_param('ExtMode', 'off');   % External mode
Configuration.set_param('ExtModeTransport', 0);   % Transport layer
Configuration.set_param('ExtModeMexFile', 'ext_comm');   % MEX-file name
Configuration.set_param('ExtModeStaticAlloc', 'off');   % Static memory allocation
Configuration.set_param('EnableUserReplacementTypes', 'off');   % Specify custom data type names
Configuration.set_param('GenerateDestructor', 'on');   % Generate destructor
Configuration.set_param('IncludeModelTypesInModelClass', 'on');   % Include model types in model class
Configuration.set_param('DSAsUniqueAccess', 'off');   % Implement each data store block as a unique access point
Configuration.set_param('ExtModeTesting', 'off');   % External mode testing
Configuration.set_param('ExtModeMexArgs', '');   % MEX-file arguments
Configuration.set_param('ExtModeIntrfLevel', 'Level1');   % External mode interface level
Configuration.set_param('MultiInstanceErrorCode', 'Error');   % Multi-instance code error diagnostic
Configuration.set_param('RTWCAPISignals', 'off');   % Generate C API for signals
Configuration.set_param('RTWCAPIParams', 'off');   % Generate C API for parameters
Configuration.set_param('RTWCAPIStates', 'off');   % Generate C API for states
Configuration.set_param('RTWCAPIRootIO', 'off');   % Generate C API for root-level I/O
Configuration.set_param('ERTSrcFileBannerTemplate', 'ert_code_template.cgt');   % Source file template
Configuration.set_param('ERTHdrFileBannerTemplate', 'ert_code_template.cgt');   % Header file template
Configuration.set_param('ERTDataSrcFileTemplate', 'ert_code_template.cgt');   % Source file template
Configuration.set_param('ERTDataHdrFileTemplate', 'ert_code_template.cgt');   % Header file template
Configuration.set_param('SignalDisplayLevel', 10);   % Signal display level
Configuration.set_param('ParamTuneLevel', 10);   % Parameter tune level
Configuration.set_param('PreserveExpressionOrder', 'off');   % Preserve operand order in expression
Configuration.set_param('PreserveIfCondition', 'off');   % Preserve condition expression in if statement
Configuration.set_param('ConvertIfToSwitch', 'on');   % Convert if-elseif-else patterns to switch-case statements
Configuration.set_param('PreserveExternInFcnDecls', 'on');   % Preserve extern keyword in function declarations
Configuration.set_param('SuppressUnreachableDefaultCases', 'on');   % Suppress generation of default cases for switch statements if unreachable
Configuration.set_param('EnableSignedLeftShifts', 'on');   % Replace multiplications by powers of two with signed bitwise shifts
Configuration.set_param('EnableSignedRightShifts', 'on');   % Allow right shifts on signed integers
Configuration.set_param('IndentStyle', 'K&R');   % Indent style
Configuration.set_param('IndentSize', '2');   % Indent size
Configuration.set_param('NewlineStyle', 'Default');   % Newline style
Configuration.set_param('MaxLineWidth', 128);   % Maximum line width
Configuration.set_param('ReplacementTypes', struct('double','','single','','int32','','int16','','int8','','uint32','','uint16','','uint8','','boolean','','int','','uint','','char','','uint64','','int64',''));   % Data type names
Configuration.set_param('MaxIdInt64', 'INT64_MAX');   % 64-bit integer maximum identifier
Configuration.set_param('MinIdInt64', 'INT64_MIN');   % 64-bit integer minimum identifier
Configuration.set_param('MaxIdUint64', 'UINT64_MAX');   % 64-bit unsigned integer maximum identifier
Configuration.set_param('MaxIdInt32', 'INT32_MAX');   % 32-bit integer maximum identifier
Configuration.set_param('MinIdInt32', 'INT32_MIN');   % 32-bit integer minimum identifier
Configuration.set_param('MaxIdUint32', 'UINT32_MAX');   % 32-bit unsigned integer maximum identifier
Configuration.set_param('MaxIdInt16', 'INT16_MAX');   % 16-bit integer maximum identifier
Configuration.set_param('MinIdInt16', 'INT16_MIN');   % 16-bit integer minimum identifier
Configuration.set_param('MaxIdUint16', 'UINT16_MAX');   % 16-bit unsigned integer maximum identifier
Configuration.set_param('MaxIdInt8', 'INT8_MAX');   % 8-bit integer maximum identifier
Configuration.set_param('MinIdInt8', 'INT8_MIN');   % 8-bit integer minimum identifier
Configuration.set_param('MaxIdUint8', 'UINT8_MAX');   % 8-bit unsigned integer maximum identifier
Configuration.set_param('BooleanTrueId', 'true');   % Boolean true identifier
Configuration.set_param('BooleanFalseId', 'false');   % Boolean false identifier
Configuration.set_param('TypeLimitIdReplacementHeaderFile', '');   % Type limit identifier replacement header file
Configuration.set_param('ArrayContainerType', 'std::array');   % Array container type

% Simulink Coverage
Configuration.set_param('CovEnable', 'off');   % Enable coverage analysis
Configuration.set_param('RecordCoverage', 'off');   % Record coverage for this model
Configuration.set_param('CovModelRefEnable', 'off');   % Record coverage for referenced models

% Simscape
try
  Configuration_componentCC = SSC.SimscapeCC;
  Configuration.attachComponent(Configuration_componentCC);
  Configuration.set_param('EditingMode', 'Full');   % Editing Mode
  Configuration.set_param('ExplicitSolverDiagnosticOptions', 'warning');   % Explicit solver used in model containing Physical Networks blocks
  Configuration.set_param('GlobalZcOffDiagnosticOptions', 'warning');   % Zero-crossing control is globally disabled in Simulink
  Configuration.set_param('SimscapeNormalizeSystem', 'off');   % Normalize using nominal values
  Configuration.set_param('SimscapeLogType', 'none');   % Log simulation data
  Configuration.set_param('SimscapeUseOperatingPoints', 'off');   % Enable operating point initialization
  Configuration.set_param('SimscapeCompileComponentReuse', 'off');   % Reuse components during compilation
  Configuration.set_param('SimscapeMultithreadedCompilation', 'on');   % Enable multithreaded compilation
catch ME
  warning('Simulink:ConfigSet:AttachComponentError', '%s', ME.message);
nk
  cs.set_param('SimscapeNormalizeSystem', 'off');   % Normalize using nominal values
  cs.set_param('SimscapeLogType', 'none');   % Log simulation data
  cs.set_param('SimscapeUseOperatingPoints', 'off');   % Enable operating point initialization
end