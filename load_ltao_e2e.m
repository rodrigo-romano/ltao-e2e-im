%
% load_IMS_5pt3b_asm.m
% Load data required to run the ASMS single segment simulations.
%

% clear
simulink_fname = 'ltao_e2e';
fprintf("\n*** Loading End-to-end simulation variables (Version:%s) ***\n\n",simulink_fname);



%% General settings
%%

% Zenith angle string (affects the control model for now)
sZa = "30";
% Set telescope structural dynamics damping
sysDamp = 0.02;
% Set FEM discretization sampling period
FEM_Ts = 1/8e3;    % [s]


% - - - - - Simulation setting flags (0:disables) - - - - -
clear osim
osim.reduce_model = 0;  % DO NOT ENABLE UNTIL TODO IS DONE![bool] model reduction feature
osim.dc_mm_comp = 1;    % [bool] DC mismatch compensation
osim.bpless_wl = 1;     % [bool] Smoothing wind load
osim.wload_en = 1;      % [bool] Wind load input
% MNT
osim.mntC_en = 1;       % [bool] Mount feedback control loop switch
osim.en_servo = 0;      % [bool] Enable/disable mount trajectory
osim.mnt_FFc_en = 1;    % [bool] Azimuth feedforward action switch      
% M1
osim.m1olf_en = 1;      % [bool] M1 outer force loop switch
% M2
osim.m2_asm_en = 1;         % [bool] M2 ASM inner control loop
osim.m2_asm_ff_en = 1;      % [bool] M2 ASM FF inner control loop
osim.m2_asmfdamp_en = 1;    % [bool] M2 ASM fluid damping pseudo control loop
osim.m2_pos_en = 1;         % [bool] M2 Positioner control loop
% AO (Adaptive Optics)
osim.gtt_fb_en = 1;     % [bool] Global tip-tilt controller
osim.ltws_fb_en = 1;    % [bool] (on-axis) segment TT LTWS controller 

% Log decimation factor
asm_modes_decim = 1; %1/FEM_Ts/1e3;



%% Load telescope structural dynamics and static solution models
%%

ModelFolder = fullfile(im.lfFolder,"20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111");
FileName = "modal_state_space_model_2ndOrder.mat";
    
if(~exist('inputTable','var') || 0)
    load(fullfile(ModelFolder,FileName),'inputs2ModalF','modalDisp2Outputs',...
        'eigenfrequencies','proportionalDampingVec','inputTable','outputTable');    
    fprintf('Model %s loaded from\n%s\n', FileName, ModelFolder);
end

if(~exist('gainMatrix','var') || 0)
    % Static solution gain matrix
    staticSolFile = fullfile(ModelFolder,"static_reduction_model.mat");
    try
        load(staticSolFile,'gainMatrixMountControlled');
        gainMatrix = gainMatrixMountControlled;
    catch, load(staticSolFile,'gainMatrix');
    end
    fprintf('Static gain matrix loaded from \n%s\n', ModelFolder);
end

%% Pick model IOs according to Input/Output tables
%%

% INPUTS
desiredInputLabels = [...
    "OSS_ElDrive_Torque";...
    "OSS_AzDrive_Torque";...
    "OSS_RotDrive_Torque";...
    "M1_actuators_segment_1";...
    "M1_actuators_segment_2";...
    "M1_actuators_segment_3";...
    "M1_actuators_segment_4";...
    "M1_actuators_segment_5";...
    "M1_actuators_segment_6";...
    "M1_actuators_segment_7";...
    "OSS_M1_lcl_6F";...
    "MC_M2_SmHex_F";...
    "MC_M2_lcl_6F";...
    "MC_M2_S1_VC_delta_F";...
    "MC_M2_S1_fluid_damping_F";...
    "MC_M2_S2_VC_delta_F";...
    "MC_M2_S2_fluid_damping_F";...
    "MC_M2_S3_VC_delta_F";...
    "MC_M2_S3_fluid_damping_F";...
    "MC_M2_S4_VC_delta_F";...
    "MC_M2_S4_fluid_damping_F";...
    "MC_M2_S5_VC_delta_F";...
    "MC_M2_S5_fluid_damping_F";...
    "MC_M2_S6_VC_delta_F";...
    "MC_M2_S6_fluid_damping_F";...
    "MC_M2_S7_VC_delta_F";...
    "MC_M2_S7_fluid_damping_F";...
    "MC_M2_PMA_1F";...
    "CFD_202110_6F"...
    ];
isDesired = zeros(size(inputs2ModalF,2),1);
modelInputDims = zeros(numel(desiredInputLabels),1);

for i1 = 1:numel(desiredInputLabels)
    aux = inputTable{desiredInputLabels{i1},"indices"}{1}(:);
    isDesired(aux) = 1;
    modelInputDims(i1) = length(aux);
end
indDesInputs = find(isDesired ~= 0);
modelInputDims(modelInputDims == 0) = [];

% OUTPUTS
desiredOutputLabels = [...
    "OSS_ElEncoder_Angle";...
    "OSS_AzEncoder_Angle";...
    "OSS_RotEncoder_Angle";...
    "OSS_Hardpoint_D";...
    "OSS_M1_lcl";...
    "OSS_M1_edge_sensors";...
    "MC_M2_SmHex_D";...
    "MC_M2_RB_6D";...
    "MC_M2_lcl_6D";...
    "MC_M2_S1_VC_delta_D";...
    "MC_M2_S2_VC_delta_D";...
    "MC_M2_S3_VC_delta_D";...
    "MC_M2_S4_VC_delta_D";...
    "MC_M2_S5_VC_delta_D";...
    "MC_M2_S6_VC_delta_D";...
    "MC_M2_S7_VC_delta_D";...
    "MC_M2_PMA_1D";...
    "M2_segment_1_axial_d";...
    "M2_segment_2_axial_d";...
    "M2_segment_3_axial_d";...
    "M2_segment_4_axial_d";...
    "M2_segment_5_axial_d";...
    "M2_segment_6_axial_d";...
    "M2_segment_7_axial_d";...
    "M2_edge_sensors"...
    ];
isDesired = zeros(size(modalDisp2Outputs,1),1);
modelOutputDims = zeros(numel(desiredOutputLabels),1);

for i1 = 1:numel(desiredOutputLabels)
    aux = outputTable{desiredOutputLabels{i1},"indices"}{1}(:);
    isDesired(aux) = 1;
    modelOutputDims(i1) = length(aux);
end
indDesOutputs = find(isDesired ~= 0);
modelOutputDims(modelOutputDims == 0) = [];




%% Structural model discretization
%%

if osim.reduce_model
    % Compute the approximate Hankel singular values
    % <-- TO DO: Always keep the first 3 modes
    [gamma,~] = utils.approxhsv(sqrt(om2), sysDamp*ones(size(om2)), phiB, phiC);
    [~,si] = sort(gamma,'descend');
    th1 = 1e-7;
    gammamax = max(gamma(:,1+3));
    
    nr = length(find((gamma./gammamax) >= th1));
    warning('\n-> The number of modes for TH=%.2g is %d\n',th1,nr);
    mode_ind_vec = si(1:nr);
else
    nr = length(eigenfrequencies(:));
    mode_ind_vec = 1:nr;
end

% om^2 and 2*zeta*om vectors 
om2 = (2*pi*eigenfrequencies(mode_ind_vec)).^2;
twice_zom = 2*sysDamp .* sqrt(om2);

fprintf('Plant model damping ratio set to:%.2g\n',(twice_zom(4)/2/sqrt(om2(4))))
% Perform discretization and provide the 2nd order form DT simulation parameters
PG = zeros(length(om2),6);
for i = 1:length(om2)
    PhiGamma = expm([0 1 0; -om2(i) -twice_zom(i) 1; 0 0 0]*FEM_Ts);
    PG(i,:) = [PhiGamma(1,1:3), PhiGamma(2,1:3)];
end

s_rate_msg = 'Telescope structural model sampling rate set to %gkHz\n';
if((FEM_Ts ~= 1/8e3) && (FEM_Ts ~= 1e-3)), warning(s_rate_msg,1/FEM_Ts/1e3);
else, fprintf(s_rate_msg,1/FEM_Ts/1e3);
end


%% Load parameters of controllers and other subsystems
%%

% -------------------
% % Load ODC 2023 FDR mount controller and driver parameters
% try
%     o = get_odc_mnt_dt(sZa);
% catch
%     error("Unable to load ODC data! "+...
%         "If functions are not available try loading from a standalone file.\n");
% end
% 
% % function filter = notchF(fc,F,delta)
% notchF = @(fc,F,delta) tf([1, 4*pi*fc/(F*delta), 4*(pi*fc)^2],...
%     [1, 4*pi*fc/F, 4*(pi*fc)^2]);
% % Mount (FB & FF) controller discretization
% c2d_opt = c2dOptions('method','tustin');
% mnt_TF_Ts = FEM_Ts;
% % AZ FB & FF
% warning("Changing AZ controller!")
% aznotchF17 = notchF(16.9, 3.5, 1.8);
% mount.az.SSdtHfb = balreal(c2d(ss(aznotchF17*o.az.c.Hp), mnt_TF_Ts, c2d_opt));
% mount.az.SSdtHff = balreal(c2d(ss(o.az.c.Hff), mnt_TF_Ts, c2d_opt));
% % EL FB & FF
% mount.el.SSdtHfb = balreal(c2d(ss(o.el.c.Hp), mnt_TF_Ts, c2d_opt));
% mount.el.SSdtHff = balreal(c2d(ss(o.el.c.Hff), mnt_TF_Ts, c2d_opt));
% % GIR FB & FF
% mount.gir.SSdtHfb = balreal(c2d(ss(o.gir.c.Hp), mnt_TF_Ts, c2d_opt));
% mount.gir.SSdtHff = balreal(c2d(ss(o.gir.c.Hff), mnt_TF_Ts, c2d_opt));
% 
% % Mount driver model parameters
% mount.delay = 4.0e-3;     % [s] DRV delay: 4ms (GMT25-ANA-40000-0007-2.0 - Pg26)
% drv_delay = ceil(mount.delay/FEM_Ts);
% fprintf('Driver delay set to %d sampling periods (Ts=%gms).\n',drv_delay,1e3*FEM_Ts);
% if(rem(mount.delay,FEM_Ts))
%     warning('Driver delay is not a multiple of the sampling period!')
% end
% % Current loop dynamics tranfer function
% Hdrive_d = c2d(o.Hdrive(1,1), FEM_Ts, 'tustin');

% -------------------
% File with controller and interface parameters
ctrl_filename = sprintf('controls_5pt1g8K_z%s_llTT_oad',sZa);
load(fullfile("/Users/rromano/Workspace/gmt-ims",...
    ctrl_filename),'m2pos','m1sys','mount');
if(~exist("fem","var")), fem.Ts = FEM_Ts; end


% Load KL modal basis
% -------------------
clear XiKL XiZN
n_m = 3;
for iseg = 1:7
    try
        variableName = sprintf('KL_%d',iseg);
        %     load(fullfile(ModelFolder,'KLmodes.mat'),variableName);
        load(fullfile(ModelFolder,'KLmodesQR.mat'),variableName);
    catch, warning('Check if the correct segment was selected!');
    end
    asm_modes = 1:n_m;
    eval([sprintf('XiKL_S%d=',iseg),variableName,'(:,asm_modes);']);
end
fprintf('Number of vector basis vectors (n_m):%d \n',n_m)

% Load LOM data
% -------------------
filename1 = fullfile(im.lfFolder,'ceo-data','lom_tt_dt.mat');
load(filename1,'D_seg_tt');
% fprintf('\nTT sensitivity matrix loaded from \n%s\n',filename1);
filename2 = fullfile(im.lfFolder,'ceo-data','D_seg_piston_dt.mat');
load(filename2,'D_seg_piston');
% fprintf('Piston sensitivity matrix loaded from \n%s\n',filename2);
filename3 = fullfile(im.lfFolder,'ceo-data','rbm2gtt.mat');
load(filename3,'rbm2gtt');
% fprintf('Global TT sensitivity matrix loaded from \n%s\n',filename3);

% Compute matrix (Sm1m2_gtt) that retaining the (on-axis) global TT in M1
% and M2 RBM
% -------------------
m1_out_idxs = outputTable{'OSS_M1_lcl','indices'}{1};
m2_out_idxs = outputTable{'MC_M2_lcl_6D','indices'}{1};
mntAZ_out = outputTable{'OSS_AzEncoder_Angle',"indices"}{1};
mntEL_out = outputTable{'OSS_ElEncoder_Angle',"indices"}{1};

Cm1m2 = [modalDisp2Outputs(m1_out_idxs,1:3);modalDisp2Outputs(m2_out_idxs,1:3)];
Cmnt = [mean(modalDisp2Outputs(mntAZ_out,1:3),1);...
    mean(modalDisp2Outputs(mntEL_out,1:3),1)];
Hk_mnt_m1m2 = Cm1m2*pinv(Cmnt);

% mnt2gtt = rbm2gtt * Hk_mnt_m1m2 %(for verification purposes)
gtt2mnt = eye(2)/(rbm2gtt * Hk_mnt_m1m2);
%
Sm1m2_gtt = (Hk_mnt_m1m2*gtt2mnt) * rbm2gtt;

if(0), test__Sm1m2_gtt(D_seg_tt, rbm2gtt, Sm1m2_gtt, 10); end

%% M1ES Reconstruction matrix
% -------------------
try in_hp_axF = inputTable{'OSS_Hardpoint_delta_F',"indices"}{1}'; % HP forces
catch   % Handle typo
    in_hp_axF = inputTable{'OSS_Harpoint_delta_F',"indices"}{1}'; % HP forces
end

try out_m1es = outputTable{'OSS_M1_edge_sensors',"indices"}{1}';
catch, out_m1es = outputTable{'M1_edge_sensors',"indices"}{1}';
end

% HP_axF to M1ES
load(fullfile('/Users/rromano/Workspace/gmt-ims/studies/m1es_reconst',...
    'M1_edge_sensor_conversion.mat'), 'A1');
KhpF2es = A1*gainMatrix(out_m1es,in_hp_axF);
% HP_axF to M1RBM 
KhpF2m1rbm = gainMatrix(m1_out_idxs,in_hp_axF);
% M1RBM to M1ES node displacements convertion
Km1rbm_es = (KhpF2es/KhpF2m1rbm);

% M1RBM Reconstruction
% Pi_m1 = (eye(42) - (Hk_mnt_m1m2(1:42,:)*gtt2mnt) * rbm2gtt(:,1:42));
% Rm1es = Pi_m1*[pinv(Km1rbm_es(:,1:36), 1e-2); zeros(6,48)];
if(0), Rm1es = [pinv(Km1rbm_es(:,1:36), 1e-2); zeros(6,48)];
else, Rm1es = pinv(Km1rbm_es, 1e-2);
end

if(1)
    [~,Ses,~] = svd(Km1rbm_es, "econ");
    figure(99)
    sigma = diag(Ses);
    semilogy(sigma,'+'); hold on;
    Tmodes = 37:42; semilogy(Tmodes, sigma(Tmodes),'o','MarkerSize',12);
    ylabel('Singular values of the M1RBM-to-ES\_MEAS transformation');
    xlabel('Singular mode #');
    
    [~,Ses_,~] = svd(Km1rbm_es(:,1:36), "econ");
    sigma_ = diag(Ses_);
    semilogy(sigma_,'x'); hold on;
    legend('D_{M1ES} singular values','Filtered (ill-conditioned) modes',...
        '\sigma(D_{M1ES}(:,1:36))','fontsize',12,'location','southwest');
    grid on; hold off; xlim([0,43]);

    figure(101)
    set(gcf,'Position',[321   267   280*3/2*1.8   300]);
    subplot(121)
    imagesc(Rm1es*(Km1rbm_es));
    ylabel('Reconstructed M1-RBM','Fontsize',14);
    xlabel('M1-RBM (S1TxyzRxyz,...,S7TxyzRxyz)','Fontsize',14);
    cbar = colorbar;
    ylabel(cbar,'M1RBM rec - Txyz[um/um] or Rxyz[um/urad]','Fontsize',12)
    subplot(122)
    imagesc([D_seg_tt(:,1:42);D_seg_piston(:,1:42)]*Rm1es*(Km1rbm_es));
    ylabel('Rec M1-induced TTP ','Fontsize',14);
    xlabel('M1-RBM (S1TxyzRxyz,...,S7TxyzRxyz)','Fontsize',14);
    cbar = colorbar;
    ylabel(cbar,'M1 rec-induced PTT','Fontsize',12)
end

%% M2ES Reconstruction matrix
% -------------------

% Indices of M2 force inputs
in_m2p_mc = inputTable{'MC_M2_SmHex_F',"indices"}{1}(1:2:end);	%macrocell side
in_m2p_m2 = inputTable{'MC_M2_SmHex_F',"indices"}{1}(2:2:end);	%mirror segment side
% Indices of M2 POS displacements
out_m2p_mc = outputTable{'MC_M2_SmHex_D',"indices"}{1}(1:2:end);	%macrocell side
out_m2p_m2 = outputTable{'MC_M2_SmHex_D',"indices"}{1}(2:2:end);	%mirror seg side
% M2 reference body (RB) RBM (local CS) output indices
out_asm_RB = outputTable{'MC_M2_RB_6D',"indices"}{1};
% M2 ES output indices
out_m2es = outputTable{'M2_edge_sensors',"indices"}{1};

% M2 POS 6F -> M2 POS 6D
K_m2p = gainMatrix(out_m2p_mc,in_m2p_mc) -gainMatrix(out_m2p_mc,in_m2p_m2) ...
        - gainMatrix(out_m2p_m2,in_m2p_mc) +gainMatrix(out_m2p_m2,in_m2p_m2);

% M2 POS 6F -> ASM RB
K_m2p_2_asm = gainMatrix(out_asm_RB,in_m2p_mc) - gainMatrix(out_asm_RB,in_m2p_m2);
% ASM RB -> M2P 6D
K_asm_2_m2p = K_m2p / K_m2p_2_asm;
% M2 POS 6F -> M2ES
K_m2p_2_es = gainMatrix(out_m2es,in_m2p_mc)-gainMatrix(out_m2es,in_m2p_m2);
% ASM RB -> M2ES
K_asm_2_es = K_m2p_2_es / K_m2p_2_asm;

if(0), Rm2es = [pinv(K_asm_2_es(:,1:36), 1e-2); zeros(6,48)];
else, Rm2es = pinv(K_asm_2_es, 1e-2);
end

if(0)
    [Ues,Ses,Ves] = svd(K_asm_2_es, "econ");
    figure(100)
    sigma = diag(Ses);
    semilogy(sigma,'+'); hold on;
    Tmodes = 37:42; semilogy(Tmodes, sigma(Tmodes),'o','MarkerSize',12);
    ylabel('Singular values of the M2RBM-to-ES\_MEAS transformation');
    xlabel('Singular mode #');
    
    [Ues_,Ses_,Ves_] = svd(K_asm_2_es(:,1:36), "econ");
    sigma_ = diag(Ses_);
    semilogy(sigma_,'x'); hold on;
    legend('D_{M2ES} singular values','Filtered (ill-conditioned) modes',...
        '\sigma(D_{M2ES}(:,1:36))','fontsize',12,'location','southwest');
    grid on; hold off; xlim([0,43]);

    figure(102)
    set(gcf,'Position',[321   267   280*3/2*1.8   300]);
    subplot(121)
    imagesc(Rm2es*(K_asm_2_es));
    ylabel('Reconstructed M2-RBM','Fontsize',14);
    xlabel('M2-RBM (S1TxyzRxyz,...,S7TxyzRxyz)','Fontsize',14);
    cbar = colorbar;
    ylabel(cbar,'M2RBM rec - Txyz[um/um] or Rxyz[um/urad]','Fontsize',12)
    subplot(122)
    imagesc([D_seg_tt(:,43:84);D_seg_piston(:,43:84)]*Rm2es*(K_asm_2_es));
    ylabel('Rec M2-induced TTP ','Fontsize',14);
    xlabel('M2-RBM (S1TxyzRxyz,...,S7TxyzRxyz)','Fontsize',14);
    cbar = colorbar;
    ylabel(cbar,'M2 rec-induced PTT','Fontsize',12)
end

%%
rng('default');
% m1_rbm_dist = pinv(D_seg_piston(:,1:42))*...
%     1e-6*[-0.1659   -0.5880    0.8959   -0.8359   -0.7886   -0.7159   -0.6671]';
% 
% m1_rbm_dist = 1e-5*randn(42,1);
% m1_rbm_dist = pinv([0*D_seg_tt(:,1:42); D_seg_piston(:,1:42)])*...
%     1e-5*randn(21,1);
m1_rbm_dist = zeros(42,1);
% m1_rbm_dist(40) = 1e-5;

m2_rbm_dist = 0*randn(42,1);
% m2_rbm_dist(40) = 1e-5;
% m2_rbm_dist = pinv([D_seg_tt(:,43:84); D_seg_piston(:,43:84)])*...
%     1e-5*randn(21,1);



%% Load parameters of controllers and other subsystems
%%

% ASM inner loop an AO controller discretization method
% c2d_opts = c2dOptions('Method','foh'); % <- Lead to instability due to Hpd_d
c2d_opts = c2dOptions('Method','Tustin');

% File with controller and interface parameters
fprintf('\nLoading the controller TFs using getcontrol_asm()\n');
asm_script_folder = '/Users/rromano/Workspace/GMT-IMS/controllers';
addpath(asm_script_folder)
st = getcontrol_asm();
fprintf('ASM inner loop controller parameters\n')
fprintf('Kp=%.3gN/m, Ki=%.3gN/N, Kd=%.3gNs/m, Kfd=%.3gNs/m, Km=%.4gNs^2/m\n',...
    st.asm.Kp, st.asm.Ki, st.asm.Kd, st.asm.Kfd, st.asm.Km);
rmpath(asm_script_folder)

fao_d = c2d(st.ltao.fao, FEM_Ts, c2d_opts);
fao_d4k = c2d(st.ltao.fao, FEM_Ts, c2d_opts);

%% M2-ES filter
%%
fc_es = 50;
fes = c2d(tf((fc_es^2)*4*(pi^2),[1, 2*0.8*fc_es*2*pi, (fc_es^2)*4*(pi^2)]),...
    1/8e3,c2d_opts);%st.ltao.T

%% ASM inner loop controller TFs
%%

% PI compensator
Cpi_d = c2d(st.asm.fpi, FEM_Ts, c2d_opts);
% Numerical differentiation
Hpd_d = c2d(st.asm.fpd, FEM_Ts, 'Tustin');

% Rebuild ASM feedforward (FF) modal controller and calculate cell array
% (segPTT2asmCMD) mapping segPTT into asmKL012 commands
Ks = cell(7,1);
m1_out_idxs = outputTable{"OSS_M1_lcl","indices"}{1}(:);
if(~exist('m2_out_idxs','var')), m2_out_idxs = outputTable{"MC_M2_lcl_6D","indices"}{1}(:); end
segPTT2asmCMD = cell(7,1);
T6_ = cell(7,1);
for iseg = 1:7
    % VC IO indexes
    in_idxs = inputTable{sprintf('MC_M2_S%d_VC_delta_F',iseg),"indices"}{1}(:);
    out_idxs = outputTable{sprintf('MC_M2_S%d_VC_delta_D',iseg),"indices"}{1}(:);

    % Update Ks with ASM modal stiffness
    if(osim.dc_mm_comp)
        asm_ssG = gainMatrix(out_idxs, in_idxs);
        asmsegF_2_segTTP = [D_seg_tt(iseg:7:14,:); D_seg_piston(iseg,:)]...
            * gainMatrix([m1_out_idxs;m2_out_idxs], in_idxs);
    else
        % Compute the ASM stiffness matrix
        asm_ssG = modalDisp2Outputs(out_idxs,(1+3):end) *...
            diag(1./((2*pi*eigenfrequencies((1+3):end)).^2)) *...
            inputs2ModalF((1+3):end,in_idxs);
        % Matrix relating DCg from ASM_VC_F to segment PTT
        asmsegF_2_segTTP = [D_seg_tt(iseg:7:14,:); D_seg_piston(iseg,:)]*...
            modalDisp2Outputs([m1_out_idxs;m2_out_idxs],(1+3):end) *...
            diag(1./((2*pi*eigenfrequencies((1+3):end)).^2)) *...
            inputs2ModalF((1+3):end,in_idxs);
    end
    
    fprintf('Calculating static ASM FF term for segment %d\n',iseg);
    eval(sprintf('XiKL = XiKL_S%d;',iseg));
    asm_ssG = XiKL' * asm_ssG * XiKL;

    if (rank(asm_ssG) < size(asm_ssG,1))
        warning('Rank defficient asm_ssG. Using pseudo-inverse to get Ks!')
        Ks{iseg} = pinv(asm_ssG);
    else
        Ks{iseg} = eye(size(asm_ssG))/asm_ssG;
    end

    segPTT2asmCMD{iseg} = asm_ssG/ (asmsegF_2_segTTP* XiKL);
    T6_{iseg} = XiKL'*(gainMatrix(out_idxs,in_m2p_mc) - gainMatrix(out_idxs,in_m2p_m2));
end

if(0) % DEBUG
    sttp2asm = blkdiag(segPTT2asmCMD{1},segPTT2asmCMD{2},segPTT2asmCMD{3},segPTT2asmCMD{4},...
        segPTT2asmCMD{5},segPTT2asmCMD{6},segPTT2asmCMD{7});
    gtt2sttp = [D_seg_tt; D_seg_piston]*Hk_mnt_m1m2*gtt2mnt;
    Tgtt2asm = sttp2asm*gtt2sttp;
    T5 = rbm2gtt(:,43:end)*(gainMatrix(m2_out_idxs,in_m2p_mc) - gainMatrix(m2_out_idxs,in_m2p_m2));
    T6 = [T6_{1};T6_{2};T6_{3};T6_{4};T6_{5};T6_{6};T6_{7}];
    Tgtt2asm_ = T6*pinv(T5);
    figure(70);
    subplot(121)
    imagesc(Tgtt2asm);
    subplot(122)
    imagesc(Tgtt2asm_);
    figure(71);
    gtt2rbm = Hk_mnt_m1m2*gtt2mnt;
    gtt2rbm_ = (gainMatrix(m2_out_idxs,in_m2p_mc) - gainMatrix(m2_out_idxs,in_m2p_m2))*...
        pinv(T5);
    subplot(121)
    plot(1:42,gtt2rbm_(:,1)','o--', 1:42, gtt2rbm(43:84,1)','+-.'); axis tight;
    subplot(122)
    plot(1:42,gtt2rbm_(:,2)','o--', 1:42, gtt2rbm(43:84,2)','+-.'); axis tight;
end

% Set to 0 to use original pre shape filter (fc=2200Hz)
if(0)
    % [Hz] prefilter break frequency ;%2200 %
    fpre_bw = 1/FEM_Ts; %#ok<*UNRCH>
    dd = [1.00000000 3.12393994 4.39155033 3.20108587 1.00000000]; % for wc=1
    dd = dd.*(2*pi*fpre_bw).^(0:4); % for wc=2*pi*fpre_bw
    nn = [0 0 0 0 dd(end)]; % for unity dc gain
    fpre = tf(nn,dd);
else
    fpre = st.asm.fpre;
end

% FF contributions
[a,b,c]=ssdata(fpre);
flag=ss(a,b,[c;c*a;c*(a^2)],[0;c*b;c*a*b]); % [1;s;s^2]*fpre

if(1)
    % Discrete-time FF controller TFs - 4th-order Bessel approx
    fpre_d = tf(c2d(fpre,FEM_Ts,c2d_opts));
    flag_1stOdyn_d = tf(c2d(flag(2,1),FEM_Ts,c2d_opts));
    flag_2ndOdyn_d = tf(c2d(flag(3,1),FEM_Ts,c2d_opts));
else
    % ASM Shaping filter - AdOptica' implementation
    sm.tSF = 1/(1.1/AOctrl.T);    %1/2200;
    sm.d1SF = 60/sm.tSF^3;
    sm.d2SF = -180/sm.tSF^4;
    sm.d3SF = 120/sm.tSF^5;
    sm.TCmd = AOctrl.T;
end



%% Input & output transformations
%%

% Labels of the inputs to transform
InLabels2Tr = [...
    "M1_actuators_segment_1";...
    "M1_actuators_segment_2";...
    "M1_actuators_segment_3";...
    "M1_actuators_segment_4";...
    "M1_actuators_segment_5";...
    "M1_actuators_segment_6";...
    "M1_actuators_segment_7";...
    "MC_M2_S1_VC_delta_F"; "MC_M2_S1_fluid_damping_F";...
    "MC_M2_S2_VC_delta_F"; "MC_M2_S2_fluid_damping_F";...
    "MC_M2_S3_VC_delta_F"; "MC_M2_S3_fluid_damping_F";...
    "MC_M2_S4_VC_delta_F"; "MC_M2_S4_fluid_damping_F";...
    "MC_M2_S5_VC_delta_F"; "MC_M2_S5_fluid_damping_F";...
    "MC_M2_S6_VC_delta_F"; "MC_M2_S6_fluid_damping_F";...
    "MC_M2_S7_VC_delta_F"; "MC_M2_S7_fluid_damping_F"...
    ];
% Corresponding input transformation matrices
InTrMatrices = {...
    m1sys{1}.Kbal;...
    m1sys{2}.Kbal;...
    m1sys{3}.Kbal;...
    m1sys{4}.Kbal;...
    m1sys{5}.Kbal;...
    m1sys{6}.Kbal;...
    m1sys{7}.Kbal;...
    XiKL_S1; XiKL_S1;...
    XiKL_S2; XiKL_S2;...
    XiKL_S3; XiKL_S3;...
    XiKL_S4; XiKL_S4;...
    XiKL_S5; XiKL_S5;...
    XiKL_S6; XiKL_S6;...
    XiKL_S7; XiKL_S7...
    };
% New labels of the transformed inputs
NewTrInLabels = [...
    "M1-S1_SA_CG_FM";...
    "M1-S2_SA_CG_FM";...
    "M1-S3_SA_CG_FM";...
    "M1-S4_SA_CG_FM";...
    "M1-S5_SA_CG_FM";...
    "M1-S6_SA_CG_FM";...
    "M1-S7_SA_CG_FM";...
    "ASM-S1_modal_F"; "ASM-S1_modal_Fd";...
    "ASM-S2_modal_F"; "ASM-S2_modal_Fd";...
    "ASM-S3_modal_F"; "ASM-S3_modal_Fd";...
    "ASM-S4_modal_F"; "ASM-S4_modal_Fd";...
    "ASM-S5_modal_F"; "ASM-S5_modal_Fd";...
    "ASM-S6_modal_F"; "ASM-S6_modal_Fd";...
    "ASM-S7_modal_F"; "ASM-S7_modal_Fd"...
    ];
% Input transformation table
InputTr_dt = table(InLabels2Tr,InTrMatrices,NewTrInLabels);

KinputT = [];
InputLabels = desiredInputLabels;
modelMuxDims = modelInputDims;
for ii = 1:numel(desiredInputLabels)
    iu = find(strcmp(desiredInputLabels(ii), InputTr_dt.InLabels2Tr));
    if(isempty(iu))
        fprintf("Unchanged input: <%s> [%d]\n",desiredInputLabels(ii),ii);
        KinputT = blkdiag(KinputT, eye(modelInputDims(ii)));        
    else
        fprintf("Input transformation [%d]: <%s> -> <%s>\n",...
            iu,InputTr_dt.InLabels2Tr(iu),NewTrInLabels(iu));
        assert(size(InputTr_dt.InTrMatrices{iu},1) == modelInputDims(ii),...
            '*** Incompatible transformation !!! ***')
        KinputT = blkdiag(KinputT, InputTr_dt.InTrMatrices{iu});
        InputLabels(ii) = NewTrInLabels(iu);
        modelMuxDims(ii) = size(InputTr_dt.InTrMatrices{iu},2);
    end
end
% Check input transformation matrix
assert(isequal(size(KinputT), [sum(modelInputDims),sum(modelMuxDims)]),...
    'Inconsistent input transformation matrix!!!');
fprintf('---\nInput transformation n_u: %d (before) -> %d (after)\n---\n',...
    size(KinputT,1), size(KinputT,2));

% Labels of the outputs to transform
OutLabels2Tr = [...
    "OSS_M1_edge_sensors";...    
    "MC_M2_S1_VC_delta_D";...
    "MC_M2_S2_VC_delta_D";...
    "MC_M2_S3_VC_delta_D";...
    "MC_M2_S4_VC_delta_D";...
    "MC_M2_S5_VC_delta_D";...
    "MC_M2_S6_VC_delta_D";...
    "MC_M2_S7_VC_delta_D";...
    "M2_segment_1_axial_d";...
    "M2_segment_2_axial_d";...
    "M2_segment_3_axial_d";...
    "M2_segment_4_axial_d";...
    "M2_segment_5_axial_d";...
    "M2_segment_6_axial_d";...
    "M2_segment_7_axial_d"...
    ];
% Corresponding output transformation matrices
load(fullfile('/Users/rromano/Workspace/gmt-ims/studies/m1es_reconst',...
    'M1_edge_sensor_conversion.mat'), 'A1');
OutTrMatrices = {A1;    
    XiKL_S1'; XiKL_S2'; XiKL_S3'; XiKL_S4'; XiKL_S5'; XiKL_S6'; XiKL_S7'; ...
    XiKL_S1'; XiKL_S2'; XiKL_S3'; XiKL_S4'; XiKL_S5'; XiKL_S6'; XiKL_S7'...
    };
% New labels of the transformed outputs
NewTrOutLabels = [...
    "M1-ES_MEAS";...    
    "ASM-S1_FS-RB_modalD";...
    "ASM-S2_FS-RB_modalD";...
    "ASM-S3_FS-RB_modalD";...
    "ASM-S4_FS-RB_modalD";...
    "ASM-S5_FS-RB_modalD";...
    "ASM-S6_FS-RB_modalD";...
    "ASM-S7_FS-RB_modalD";...
    "ASM-S1_modalD";...
    "ASM-S2_modalD";...
    "ASM-S3_modalD";...
    "ASM-S4_modalD";...
    "ASM-S5_modalD";...
    "ASM-S6_modalD";...
    "ASM-S7_modalD"...
    ];
% Output transformation table
OutputTr_dt = table(OutLabels2Tr,OutTrMatrices,NewTrOutLabels);

KoutputT = [];
OutputLabels = desiredOutputLabels;
modelDemuxDims = modelOutputDims;
for ii = 1:numel(desiredOutputLabels)
    iy = find(strcmp(desiredOutputLabels(ii), OutputTr_dt.OutLabels2Tr));
    if(isempty(iy))
        fprintf("Unchanged output: <%s> [%d]\n",desiredOutputLabels(ii),ii);
        KoutputT = blkdiag(KoutputT, eye(modelOutputDims(ii)));        
    else
        fprintf("Output transformation [%d]: <%s> -> <%s>\n",...
            iy,OutputTr_dt.OutLabels2Tr(iy),NewTrOutLabels(iy));
        assert(size(OutputTr_dt.OutTrMatrices{iy},2) == modelOutputDims(ii),...
            '*** Incompatible transformation !!! ***')
        KoutputT = blkdiag(KoutputT, OutputTr_dt.OutTrMatrices{iy});
        OutputLabels(ii) = NewTrOutLabels(iy);
        modelDemuxDims(ii) = size(OutputTr_dt.OutTrMatrices{iy},1);
    end
end
% Check output transformation matrix
assert(isequal(size(KoutputT), [sum(modelDemuxDims),sum(modelOutputDims)]),...
    'Inconsistent output transformation matrix!!!');
fprintf('---\nOutput transformation n_y: %d (before) -> %d (after)\n---\n',...
    size(KoutputT,2), size(KoutputT,1));

% Compute input-output modal model matrices
phiB = inputs2ModalF(mode_ind_vec,indDesInputs) * KinputT;
phiC = KoutputT * modalDisp2Outputs(indDesOutputs,mode_ind_vec);


%% Open Simulink model
%%
open_system(simulink_fname);



%% Load wind load time series
%%
wl_demux = [270,42,42]; %#ok<*NASGU>
if(osim.wload_en)
    [IMLoads, wl_demux] = load_WLdt(1,ModelFolder);
    set_param(simulink_fname+"/Wind Loads",'Commented','off');
else
    set_param(simulink_fname+"/Wind Loads",'Commented','on');
end


%% Static gain mismatch compensation
%%
struct_dyn_label = "/Telescope model/Structural Dynamics GMT";
memory_label = simulink_fname+struct_dyn_label+"/Psi_ss_memory";
matmult_label = simulink_fname+struct_dyn_label+"/Psi_ss";
zoh_label = simulink_fname+struct_dyn_label+"/Psi_ssZOH";
rt_label = simulink_fname+struct_dyn_label+"/Psi_ssRT";

if osim.dc_mm_comp
    try
        K_ss = phiC(:,4:end)* diag(1./((2*pi*eigenfrequencies(4:end)).^2))* phiB(4:end,:);        
        Psi_ss = KoutputT *gainMatrix(indDesOutputs,indDesInputs)* KinputT - K_ss;
        %
        num_mnt_ax_in = contains(desiredInputLabels,...
            ["OSS_ElDrive_Torque"; "OSS_AzDrive_Torque"; "OSS_RotDrive_Torque"]);
        v_in = [1; 1; 1; zeros(numel(desiredInputLabels)-3,1)];
        num_mnt_ax_out = contains(desiredOutputLabels,...
            ["OSS_ElEncoder_Angle"; "OSS_AzEncoder_Angle"; "OSS_RotEncoder_Angle"]);
        v_out = [1; 1; 1; zeros(numel(desiredOutputLabels)-3,1)];
        
        if(all(num_mnt_ax_in == v_in) && all(num_mnt_ax_out == v_out))
            Psi_ss(1:sum(outputTable.size(1:3)),:) = 0;
            Psi_ss(:,1:sum(inputTable.size(1:3))) = 0;
        else
            warning("No entries of Psi_ss set to zero!"+...
                "\nCheck for the chosen MNT IOs.\n");
        end
        
        Psi_ssTs = FEM_Ts;        
        set_param(memory_label,'Commented','off');
        set_param(matmult_label,'Commented','off');
        set_param(zoh_label,'Commented','off');
        set_param(rt_label,'Commented','off');
        fprintf('DC mismatch compensation enabled!\n');
    catch
        warning('Unable to compute static compensation matrix.');
        warning('Disabling static compensation matrix.');
        set_param(memory_label,'Commented','on');
        set_param(matmult_label,'Commented','on');
        set_param(zoh_label,'Commented','on');
        set_param(rt_label,'Commented','on');
        StaticModelFolder = [];
    end
else
    set_param(memory_label,'Commented','on');
    set_param(matmult_label,'Commented','on');
    set_param(zoh_label,'Commented','on');
    set_param(rt_label,'Commented','on');
    StaticModelFolder = [];
end


%% Mount trajectory profile
%%
if osim.en_servo
    try
        % Load trajectory file
        load('mount_sp_tj0.mat', 'az_sp','el_sp','gir_sp');
        Ts_tj = 1;      %[s] Trajectory time-series sampling period
        deg2rad = pi/180;   %[rad/deg] Unit convertion constant
        
        % Compute relative trajectory
        az_sp = deg2rad*(az_sp-az_sp(1));
        el_sp = deg2rad*(el_sp-el_sp(1));
        gir_sp = deg2rad*(gir_sp-gir_sp(1));
        
        mount_tj.time = (0:length(az_sp)-1)';
        mount_tj.signals.values = en_servo*[az_sp, el_sp, gir_sp];
        mount_tj.signals.dimensions = 3;
    catch
        warning('Unable to load mount trajectory data!')
    end
else
    mount_tj.time = 0;
%     mount_tj.signals.values = en_servo*[5e-6, 0e-6, 0e-6];
    mount_tj.signals.values = [0*pi/180/3600, 0*pi/180/3600,...
    0*pi/180/3600]; % test1-7
% mount_tj.signals.values = [0, 0, 1e-6];
    mount_tj.signals.dimensions = 3;
end



%%
if(0)
    % eval_asm_ctrl_disc(st.asm.fpi + st.asm.Kd*st.asm.fpd, FEM_Ts);
    eval_asm_ctrl_disc(st.asm.fpd, FEM_Ts);
    % eval_asm_ctrl_disc(flag(3,1), FEM_Ts);
end

% ----------------------------------------------------------------------------------------
%% Auxiliary functions
%% ---------------------------------------------------------------------------------------

%% Function to get 
function o = get_odc_mnt_dt(sZa)

% oTest.sZa: elevation zenith angle (ZA) as string e.g. '00','30','60'
oTest.sZa = sZa; %'30', %'00' or '60'
% oTest.sVer: FEM version as string e.g. '19'
oTest.sVer = '20';
% oTest.sSubVer: FEM subversion as string e.g. '1'
oTest.sSubVer = '11'; %'2'; %'9';
% oTest.sDamping: now '02' means 2% structural dumping
oTest.sDamping ='02';
% oTest.bUseReducedModel: [true|false] if true: a reduced model is used
%  which was created by the balred method
oTest.bUseReducedModel = true;

odc_file_folder = '/Users/rromano/Workspace/mnt-odc';
odc_main_folder = "fdr2023/MatlabFilesE2E_2023-05-10";
odc_base_util_folder = fullfile(odc_file_folder,odc_main_folder,'base/util');
odc_base_conf_folder = fullfile(odc_file_folder,odc_main_folder,'base/conf');
addpath(odc_base_util_folder, odc_base_conf_folder);
fprintf('+Including folder\n%s\ninto MatLab path.\n',...
    fullfile(odc_file_folder,odc_main_folder));
fprintf('Getting ODC mount model parameters ...\n');

% ODC Simulink model used (located in ../base)
oTest.sRoot = 'root';
% oTest.sHbsConf: name of HBS configuration: e.g. 'HbTp19'
oTest.sHbsConf = 'HaTp19'; %'HbTp19'
% oTest.sViscFrCase: one of 3 cases w.r.t. viscosity: ['ViscFrLow', 'ViscFrMedium', 'ViscFrHigh']  see fun_mueByTempAzEl
oTest.sViscFrCase = 'ViscFrLow'; %lowest viscosity=>lowest damping
% oTest.sModelDirIn: directory relative to ../ss_model/ where the state space models are located
% returns structure [o] with all configuration parameters
oTest.sModelDirIn = 'v20.11/n100HzR800';

o = fun_confBase(oTest);
% Remove folders from Matlab path
fprintf('-Removing folders\n%s\n%s\nfrom MatLab path.\n',...
    odc_base_util_folder, odc_base_conf_folder);
rmpath(odc_base_util_folder, odc_base_conf_folder);

end

%% Load WL time series
function [windload_dt, wl_demux] = load_WLdt(wl_src, ModelFolder)

if(nargin < 2 && wl_src == 1)
    error('Inform the model folder to enable time series adjustment!')
end

switch wl_src
    case 1
        WL_dt_filename = fullfile(im.lfFolder,...
            'b2021_WL/rust_gen_wl/zen30az000_OS7','windloads.mat');
        try
            load(WL_dt_filename,'MountLoads','M1Loads','M2Loads','time');
        catch
            error('Unable to load wind load time series from file \n%s\n',WL_dt_filename);
        end
        % Handle model extra inputs
        % Load inputTable
        load(fullfile(ModelFolder,'modal_state_space_model_2ndOrder.mat'),'inputTable');
        % List of identifiers of the inputs to be disregarded
        wl_descr = ["M2 cell";...
            "LPA Servicing";...
            "Azimuth disk";...
            "Total force on all M1 cells walls";];
        wl_sub_inds = zeros(length(wl_descr),2);
        % Take the starting index (first column) and the size of those inputs
        for i1 = 1:length(wl_descr)
            aux = find(startsWith(inputTable.descriptions{'CFD_202110_6F'}(:),wl_descr(i1)));
            wl_sub_inds(i1,:) = [aux(1), length(aux)];
        end
        % Insert 0 at columns of the time series related to disregarded
        % model inputs
        [~,i__] = sort(wl_sub_inds);
        for i1 = 1:length(wl_descr)
            MountLoads = [MountLoads(1:wl_sub_inds(i__(i1),1)-1,:);...
                zeros(wl_sub_inds(i__(i1),2),size(MountLoads,2));...
                MountLoads(wl_sub_inds(i__(i1),1):end,:)];
        end
        
        timeFEM = time;
        inputFEM = [MountLoads; M1Loads; M2Loads];
        wl_demux = [size(MountLoads,1),42,42];
    otherwise
        WL_dt_filename = fullfile(im.lfFolder,'b2021_WL/zen30az000_OS7','convertedToFEM.mat');
        try
            load(WL_dt_filename,'inputFEM','timeFEM');
        catch
            error('Unable to load wind load time series from file \n%s\n',WL_dt_filename);
        end
        wl_demux = [270,42,42];
end


fHz_wl = floor(1/diff(timeFEM(1:2)));
fprintf("Wind load time series (%dHz) loaded from\n%s\n",fHz_wl,WL_dt_filename);

% Take data corresponding to the last 400s
windload_dt.time = timeFEM(end-(400*fHz_wl):end);
% Enforce first time instant to zero
windload_dt.time = windload_dt.time - windload_dt.time(1);
windload_dt.signals.values = inputFEM(:,end-(400*fHz_wl):end)';
if(false)
    windload_dt.signals.values = windload_dt.signals.values - windload_dt.signals.values(1,:);
    warning('Removing WL (initial) offset value!');
end
end

%% ---------------------------------------------------------------------------------------
% Auxiliar analysis

function eval_asm_ctrl_disc(Casm,Ts)
    Casm_d2 = c2d(Casm,Ts,'foh');
    Casm_d3 = c2d(Casm,Ts,'Tustin');
    
    if (any(zero(Casm) == 0) && (numel(pole(Casm))<2))
        Td=1; N=1/Casm.den{1}(1); TdpNh = Td + N*Ts;
        Casm_d4 = (Td*N)/TdpNh *tf([1 -1],[1, -Td/TdpNh],Ts);
        lbl_c4 = 'Backward diff @%gkHz';
        leg_loc = 'southwest'; leg_ncol = 1;
        fig_idx = 46;
    else
        Casm_d4 = c2d(Casm,Ts,'zoh');
        lbl_c4 = 'ZOH @%gkHz';
        leg_loc = 'best'; leg_ncol = 2;
        fig_idx = 44;
    end
    
    % Bode plot
    hbode = bodeoptions;
    hbode.TickLabel.FontSize = 12;
    hbode.YLabel.FontSize = 12;
    hbode.XLabel.FontSize = 12;
    hbode.FreqUnits = 'Hz';
    hbode.Grid = 'on';
    hbode.PhaseMatching = 'on';
    hbode.PhaseMatchingValue = -180;
    
    figure(fig_idx);
    bode(Casm, Casm_d2, Casm_d3, Casm_d4, hbode);%
    legend('CT',...
        sprintf('Tustin @%gkHz',1/Casm_d2.Ts/1e3),...
        sprintf('FOH @%gkHz',1/Casm_d3.Ts/1e3),...
        sprintf(lbl_c4,1/Casm_d4.Ts/1e3),...
        'NumColumns',leg_ncol,'Location',leg_loc,'Fontsize',12);
    legend box off;
    
end

function test__Sm1m2_gtt(D_seg_tt, rbm2gtt, Sm1m2_gtt, zero_gtt)

zero_gtt = isfinite(1/zero_gtt);
if(zero_gtt), figOffset = 10; else, figOffset = 0; end

S_ = pinv(rbm2gtt)*rbm2gtt;
% S_ = rbm2gtt'*rbm2gtt;

% Test 1
m1m2 = [zeros(84,1)];
m1m2((40:41)+42) = 1e-3;
figure(2+figOffset);
subplot(211)
plot(1:14, D_seg_tt*m1m2,'o',...
    1:14, D_seg_tt*(eye(84)-S_)*m1m2,'+',...
    1:14, D_seg_tt*(eye(84)-Sm1m2_gtt)*m1m2,'x'); grid on;
xlabel('segment TT')
legend('seg TT (raw)','XXX','seg TT (after removing globalTT)','fontsize',12)
subplot(212)
xlabel('global TT')
ylabel('global TT (rad)')
plot(1:2,zero_gtt*rbm2gtt*m1m2,'m+',...
    1:2,rbm2gtt*(eye(84)-Sm1m2_gtt)*m1m2,'*',...
    1:2,rbm2gtt*(eye(84)-Sm1m2_gtt)*m1m2,'o'); grid on;
xlim([0,3])

% Test 2
m1m2 = [zeros(42,1);randn(42,1)];
figure(3+figOffset)
subplot(211)
plot(1:14, D_seg_tt*m1m2,'o',...
    1:14, D_seg_tt*(eye(84)-S_)*m1m2,'+',...
    1:14, D_seg_tt*(eye(84)-Sm1m2_gtt)*m1m2,'x'); grid on;
xlabel('segment TT')
legend('seg TT (raw)','XXX','seg TT (after removing globalTT)','fontsize',12)
subplot(212)
plot(1:2,zero_gtt*rbm2gtt*m1m2,'m+',...
    1:2,rbm2gtt*(eye(84)-Sm1m2_gtt)*m1m2,'*',...
    1:2,rbm2gtt*(eye(84)-Sm1m2_gtt)*m1m2,'o'); grid on;
xlabel('global TT')
ylabel('global TT (rad)')
xlim([0,3])
end