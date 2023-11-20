%
% evalM2asm_modal.m
%
% Script to analyze the performance of the M2 ASM actuator control loop.
%



%% General analysis settings
%%
% Choose M2 segment
m2_seg = 1;

% Flag to plot ASM inner loop results
show_ASMact_plots = true;
% Flag to plot AO loop results 
show_ao_plots = false;
% Flag to choose KL mode shapes as the ASM modal basis
asm_KL_modes = true; %false; % 
% Flag to apply model reduction
reduce_model = false;
% Flag to remove the first 3 modes from the model
rm_first_3modes = true;

% Flag to load static model data
load_static_model = true; %false;  % 

% Flag to keep the current ASM plots
hold_plots = false;

% Set telescope structural dynamics damping
sysDamp = 0.01;   %0.02; 

% Number of frequency response points
Nom = 4000;
wrange = [2,4e3];      % Analysis frequency range
% Frequency points (rad/s)
w = logspace(log10(wrange(1)),log10(wrange(2)),Nom)*2*pi;

% - - - 
% Frequency response plot options
% - - - 
% Figure number to avoid overwrite
figNumber = 2000;
% Font size for plots
plotFontSize = 12;
% Bode plot
hbode = bodeoptions;
hbode.TickLabel.FontSize = 12;
hbode.YLabel.FontSize = 12;
hbode.XLabel.FontSize = 12;
hbode.FreqUnits = 'Hz';
hbode.Grid = 'on';
hbode.PhaseMatching = 'on';
hbode.PhaseMatchingValue = -180;
% - - - 
% Nichols plot
hnichols = nicholsoptions;
hnichols.TickLabel.FontSize = 12;
hnichols.YLabel.FontSize = 12;
hnichols.XLabel.FontSize = 12;
hnichols.FreqUnits = 'Hz';
hnichols.Grid = 'on';
hnichols.PhaseMatching = 'on';
hnichols.PhaseMatchingValue = -180;
% - - - 


%% Load telescope SS model
%%
% User shall check model file path and name * 

% ModelFolder = fullfile(im.lfFolder,"20210611_1336_MT_mount_v202104_ASM_full_epsilon");
ModelFolder = fullfile(im.lfFolder,"20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111");
% ModelFolder = fullfile(im.lfFolder,"20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/300Hz");

if(~exist('inputTable','var') || 0)
    
    FileName = "modal_state_space_model_2ndOrder.mat";
    %     FileName = "modal_state_space_model_2ndOrder_300Hz.mat";
    load(fullfile(ModelFolder,FileName),'inputs2ModalF','modalDisp2Outputs',...
        'eigenfrequencies','proportionalDampingVec','inputTable','outputTable');
    % Handle modal parameters
    om2 = (2*pi*eigenfrequencies(:)).^2;
    twice_zom = 2*sysDamp.*(2*pi*eigenfrequencies(:));
    
    % State-space model matrices
    B = [zeros(size(inputs2ModalF));inputs2ModalF];
    C = [modalDisp2Outputs,zeros(size(modalDisp2Outputs))];
    n_m = size(inputs2ModalF,1);
    A = [zeros(n_m),eye(n_m);...
        -diag(om2), -diag(twice_zom)];
end

if(load_static_model)
    try
        staticSolFile = fullfile(ModelFolder,"static_reduction_model.mat");
        try
            load(staticSolFile,'gainMatrixMountControlled');
            gainMatrix = gainMatrixMountControlled;
        catch
            load(staticSolFile,'gainMatrix');
        end
    catch
        load_static_model = 0;
        warning('Unable to load static gain matrix\n')
    end
end

if rm_first_3modes
    % Remove the first 3 modes and Retrieve modal-form parameters
    [~,aux2] = spdiags(full(A));
    om2 = om2(1+3:end);
    twice_zom = twice_zom(1+3:end);
    nm = length(om2);
    A = [zeros(nm),eye(nm);-diag(om2), -diag(twice_zom)];
    B = [zeros(nm,size(B,2));B(aux2(end)+1+3:end,:)];
    C = [C(:,1+3:nm+3),zeros(size(C,1),nm)];
else
    % Set the frequency of the first three modes to 0.
    om2(1:3) = [0;0;0];     %#ok<*UNRCH>
    twice_zom(1:3) = [0;0;0];
    nm = length(om2);
end

% Mode frequency vetor (rad/s)
om0 = sqrt(om2);

% Structural damping ratio
damp = sysDamp*ones(size(om2));



%% Extract M2-ASMS subsystem <m2_seg>
%%

% CP and FS nodes
nDim = inputTable(sprintf('MC_M2_S%d_VC_delta_F',m2_seg),1).size;
xFS = zeros(nDim,1); yFS = zeros(nDim,1);
xCP = zeros(nDim,1); yCP = zeros(nDim,1);

for ii = 1:nDim
    props = inputTable(sprintf('MC_M2_S%d_VC_delta_F',m2_seg),:).properties{1}{ii};
    xFS(ii) = props.location(2,1);
    yFS(ii) = props.location(2,2);
    xCP(ii) = props.location(1,1);
    yCP(ii) = props.location(1,2);
end
% MODE SHAPES MATRIX
pmax = 3;
[XiFS,~,~,n2] = utils.zernike(complex(xFS,yFS),pmax);
%     XiwoPTT = XI(:,4:end);
if(asm_KL_modes)
    try
        variableName = sprintf('KL_%d',m2_seg);
        load(fullfile(ModelFolder,'KLmodesQR.mat'),variableName);
        eval(['XiFS = ',variableName,'(:,:);']);
    catch
        warning('Check if the correct segment was selected\n');
    end
end

% n2 = 40;
asm_modes = 1:3;
% [4,338,342,343,348,349,...
%     353,354,355,356,359,...
%     360,361,362,363,364,...
%     367,368,369,370,381,...
%     382,385,386,416,417]';   %6; %1:n2; %4;%64; %1:n2
XiFS_ = XiFS(:,asm_modes);
n_Zmodes = length(asm_modes);
fprintf('Number of vector basis vectors:%d \n',n_Zmodes)

% Choose 1 to plot the mode shapes
if(0) 
    tri = delaunay(xFS,yFS);
    figure(575);
    nrow=1; ncol=1;
    for imode = 1:min(n_Zmodes,nrow*ncol)
        subplot(nrow,ncol,imode);
        trisurf(tri,xFS,yFS,XiFS(:,imode),'Facecolor','interp','Linestyle','none');
        title(sprintf('(Mode #%d)',imode));
        set(gca,'XTickLabel',[],'YTickLabel',[]);
        axis equal; axis tight; colormap('jet'); colorbar; view(2);
    end
end

% INPUT MATRIX
in1 = inputTable{sprintf('MC_M2_S%d_VC_delta_F',m2_seg),"indices"}{1}(:);
in2 = inputTable{sprintf('MC_M2_S%d_fluid_damping_F',m2_seg),"indices"}{1}(:);
b_ = [B(:,in1)*XiFS_,B(:,in2)*XiFS_];
nu = size(b_,2);

% OUTPUT MATRIX
out1 = outputTable{sprintf('MC_M2_S%d_VC_delta_D',m2_seg),"indices"}{1}(:);
out2 = outputTable{sprintf('M2_segment_%d_axial_d',m2_seg),"indices"}{1}(:);
c_ = [XiFS_'*C(out1,:); XiFS_'*C(out2,:)];
ny = size(c_,1);

D = zeros(ny,nu);


%% Model reduction analysis
%%

if reduce_model
    % Compute the approximate Hankel singular values
    [gamma,~] = utils.approxhsv(sparse(A), b_, c_);
    [~,si] = sort(gamma,'descend');
    th1 = 1e-16;
    if rm_first_3modes, gammamax = max(gamma(:,1));
    else, gammamax = max(gamma((1+3):end,1));
    end
    
    nr = length(find((gamma./gammamax) >= th1))+3;
    fprintf('\n-> The number of modes for TH=%.2g is %d\n',th1,nr);
    [a_,b_,c_] = utils.hsvmred(sparse(A),b_,c_,nr,gamma);
    
    rom2 = om2(si(1:nr));
    Gast = c_(:,1:nr) *diag(1./rom2) *b_(nr+1:end,:);
else
    a_ = A;
    
    if rm_first_3modes, Gast = c_(:,1:nm) *diag(1./om2) *b_(nm+1:end,:);
    else, Gast = c_(:,4:nm) *diag(1./om2(4:end)) *b_(nm+4:end,:);
    end
end



%% Create CMM object
%%
comp_name = struct('name','M2_ASM','shortname',[]);%,'nastran',[],'mfile',[]);
sys_struct = struct('A',a_,'B',b_,'C',c_,'D',D);
ins{1} = struct('name','asm_seg_in','indexes',1:nu,'ctrace',[]);
outs{1} = struct('name','asm_seg_out','indexes',1:ny);
cmmObj = component_mode_model(comp_name,sys_struct,ins,outs);


%% Telescope model Frequency responses

Ts = 1/8e3;   % Master sampling frequency
if(0 && ~load_static_model)
    % Structural model discretization
    PG = zeros(length(om2),6);
    for i = 1:length(om2)
        PhiGamma = expm([0 1 0; -om2(i) -twice_zom(i) 1; 0 0 0]*Ts);
        PG(i,:) = [PhiGamma(1,1:3) PhiGamma(2,1:3)];
    end
    [aux,temp] = spdiags(full(A));
    PhiB = cmmObj.sys.B(temp(end)+1:end,:);
    PhiC = cmmObj.sys.C(:,1:temp(end));
    G_fresp = frd(bode_DT2nd_order(PG,PhiB,PhiC,Ts,w), w, Ts);
else
    % Full FR matrix
    if(1), G_fresp = frd(bode_second_order(cmmObj,w,om0,damp,1:ny,1:nu), w, Ts);
    % FR matrix restricted to the main diagonal terms    
    else
        try
            G_fresp = frd(bode_diag_2nd_order(cmmObj,w,sqrt(rom2),damp(1:nr),1:ny), w, Ts);
            warning('Computing FR after model reduction.\n')
        catch
            G_fresp = frd(bode_diag_2nd_order(cmmObj,w,sqrt(om2),damp,1:ny), w, Ts);
        end
    end
    
    if(load_static_model)
        D_ = blkdiag(XiFS_,XiFS_)'*...
            gainMatrix([out1;out2],[in1;in2])*blkdiag(XiFS_,XiFS_) - Gast;
        fprintf('\nIncluding the DC mismatch compensation contribution **.\n');
%         fprintf('\nIncluding the DC mismatch compensation contribution.\n');
        G_fresp = G_fresp + frd(D_*tf(1,1,Ts,'IOdelay',1), w);%frd(D_*tf(1,1), w);%
    end
end



%% Load parameters of controllers and other subsystems
%%
% File with controller and interface parameters
fprintf('Loading the controller TFs using getcontrol_asm()\n');
asm_script_folder = '/Users/rromano/Workspace/GMT-IMS/controllers';
addpath(asm_script_folder)
st = getcontrol_asm();
rmpath(asm_script_folder)

% Controller discretization
% ASM inner loop controller discretization method
c2d_opts = c2dOptions('Method','foh');%,'PrewarpFrequency',fms_wc/2); %

% PI compensator
Cpi_d = eye(n_Zmodes)* c2d(st.asm.fpi, Ts, c2d_opts);
% Numerical differentiation
Hpd_d = eye(n_Zmodes)* c2d(st.asm.fpd, Ts, c2d_opts);

Cfb_asm = [Cpi_d + st.asm.Kd*Hpd_d; st.asm.Kfd*Hpd_d];


% AO controller
AO_case = "LTAO";
switch AO_case
    case "NGAO"
        if(1)
            fc=44.62;       % [Hz] controller unit-magnitude crossover
            f2=22.4;        % [Hz] low-freq-lag
            st.ngao.T=1e-3; % [s] samping time
            st.ngao.fao=tf(2*pi*fc*1,[1 0])*tf([1 2*pi*f2],[1 0]);
        end
        fao_d = c2d(st.ngao.fao, st.ngao.T, c2d_opts);
        fao_d2k = c2d(st.ngao.fao, 1/2e3, c2d_opts);
        fao_d4k = c2d(st.ngao.fao, 1/4e3, c2d_opts);
        fao_d8k = c2d(st.ngao.fao, 1/8e3, c2d_opts);
        tau_ell = st.ngao.tau_ell;
    case "LTAO"
        fao_d = c2d(st.ltao.fao, st.ltao.T, c2d_opts);
        fao_d2k = c2d(st.ltao.fao, 1/2e3, c2d_opts);
        fao_d4k = c2d(st.ltao.fao, 1/4e3, c2d_opts);
        fao_d8k = c2d(st.ltao.fao, 1/8e3, c2d_opts);
        tau_ell = st.ltao.tau_ell;
    case "GLAO"
        fao_d = c2d(st.glao.fao, st.glao.T, c2d_opts);
        tau_ell = st.glao.tau_ell;
end

% Wavefront sensor dynamics
zoh_fr = frd((1-exp(-1j*w*fao_d.Ts))./(1j*w*fao_d.Ts),w,Ts);
latency_wfs = frd(exp(-1j*w*(tau_ell)),w,Ts);
% latency_asm = frd(exp(-1j*w*(tau_asm)),w);
fwfs_fr = zoh_fr * latency_wfs; % zoh_fr * !!! fao_d is a discrete-time controller

% FF contributions
[aff,bff,cff]=ssdata(st.asm.fpre);
flag=ss(aff,bff,[cff;cff*aff;cff*(aff^2)],...
    [0;cff*bff;cff*aff*bff]); % [1;s;s^2]*fpre

% % Dual-rate filter: dd1 and dd2 employ a zero at z=-1 to "notch" peaks due
% % to mirroring
Mao = fao_d.Ts/Ts;
dd1 = upsample(zpk(-1,0,1/2,fao_d.Ts/2), Mao/2);
dd2 = upsample(zpk(-1,0,1/2,fao_d.Ts/4), Mao/4);
fdr_d = tf(dd1*dd2);

% Discrete-time FF controller TFs - 4th-order Bessel approx
fpre_d = tf(c2d(st.asm.fpre, Ts, c2d_opts));
flag_1stOdyn_d = tf(c2d(flag(2,1), Ts, c2d_opts));
flag_2ndOdyn_d = tf(c2d(flag(3,1), Ts, c2d_opts));


% Compute ASM modal stiffness matrices
if(load_static_model)
    invKs = XiFS_'* gainMatrix(out1,in1)*XiFS_;
else
    if rm_first_3modes
        invKs = c_(1:n_Zmodes,1:nm) *diag(1./om2) *b_(nm+1:end,1:n_Zmodes);
    else
        invKs = c_(1:n_Zmodes,4:nm) *diag(1./om2(4:end)) *b_(nm+4:end,1:n_Zmodes);
    end
end
% Feedforward controller
Cffasm = eye(n_Zmodes)*(st.asm.Km*flag_2ndOdyn_d +...
    st.asm.Kb*flag_1stOdyn_d) + (eye(n_Zmodes)/invKs)*fpre_d;



%% ASM controller frequency response
%%

% Frequency responses of inner ASM control loop TFs
Cfb_fresp = frd(Cfb_asm, w);
Lasm = G_fresp(1:n_Zmodes,:)*Cfb_fresp;
% Sasm = eye(n_Zmodes)/(eye(n_Zmodes) + Lasm);
feedin = (1:n_Zmodes)+n_Zmodes; % RB-FS modal inputs ('MC_M2_S%d_fluid_damping_F')
feedout = 1:n_Zmodes;           % RB-FS modal outputs

G_inner_fb = feedback(G_fresp, frd(Cfb_asm, w), 1:nu, feedout, -1);

Tasm_woFF = G_inner_fb(1:n_Zmodes,1:n_Zmodes) * frd(Cpi_d, w)*fpre_d;
Tasm__FF = G_inner_fb(1:n_Zmodes,1:n_Zmodes) * (frd(Cffasm, w) + frd(Cpi_d, w)*fpre_d);



%% ASM loop analysis plots
%%

% ASM modal transfer function
figure(figNumber)
subplot(211)
for i1 = 1:n_Zmodes, bodemag(G_fresp(i1,i1), w, hbode); hold on; end
xlim([min(w/2/pi),max(w/2/pi)]);
title('ASM modal TF FR magnitude','FontSize',plotFontSize);
ylabel('|\sigma(G(j\omega))|','FontSize',plotFontSize);
xlabel('Frequency (Hz)','FontSize',plotFontSize);
grid on;
if (~hold_plots), hold off; end
subplot(212)
% ASM modal TF including the fluid damping (w.o. FB/FF controller)
% G_with_fd = feedback(G_fresp(1:n_Zmodes,:),frd(st.asm.Kfd*Hpd_d,w),feedin,feedout,-1);
G_with_fd = feedback(G_fresp(1:n_Zmodes,:),frd(st.asm.Kfd*Hpd_d,w),feedin,feedout,-1);
for i1 = 1:n_Zmodes, bodemag(G_with_fd(i1,i1), w, hbode); hold on; end
title('ASM modal TF FR magnitude (with fluid damping)','FontSize',plotFontSize);
xlim([min(w/2/pi),max(w/2/pi)]);
ylabel('|\sigma(G(j\omega))|','FontSize',plotFontSize);
xlabel('Frequency (Hz)','FontSize',plotFontSize);
grid on; 
if (~hold_plots), hold off; end

% ASM actuator loop transfer function nichols plot (robustness)
if(show_ASMact_plots && 1)
    hTFn = figure(figNumber+1); %#ok<*NASGU>
    for i1 = 1:n_Zmodes
        nichols(Lasm(i1,i1,:),hnichols);
        hold on;
    end
%     ylim([-40,16]);
    xlim([-270,0]);
    grid on;
    if (~hold_plots)
        plot_nichols_ellipse(0.5);
        hold off;
    end
end


% ASM loop TF
if(show_ASMact_plots && 1)
    figure(figNumber+2);
    for i1 = 1:n_Zmodes
        bode(Lasm(i1,i1,:),hbode);
        hold on;        
    end
    xlim([min(w/2/pi),max(w/2/pi)]); grid on;
    if (~hold_plots), hold off; end 
% % ASM actuator closed-loop transfer function / w.o. FF (bandwidth)    
%     for ik = 1:n_Zmodes
%         bodemag(Tasm_woFF(ik,ik),w,hbode); hold on;
%     end
%     
%     plot_cl_OAD_req(800,2500);
%     ylim([-40,16]);
%     xlim([10,w(end)/2/pi]);
%     set(gca,'Fontsize',14)
%     hold off;
end

% ASM actuator closed-loop transfer function (bandwidth)
if(show_ASMact_plots)
    hx = figure(figNumber+3);
    for ik = 1:n_Zmodes
        bodemag(Tasm__FF(ik,ik),w,hbode); hold on;
    end
    
    if (~hold_plots)
        plot_cl_OAD_req(800,2500);
        ylim([-40,16]);
        xlim([10,w(end)/2/pi]);
        set(gca,'Fontsize',14);
        hold off;
    end
end


%% AO loop analysis plots
%%
if(show_ao_plots || 1)
    
    % Frequency responses of AO control loop TFs
    [response,freq] = frdata(frd(fao_d4k,w));
    fao_d_fr = frd(response, freq, Ts);
    Cfb_ao_fr = eye(n_Zmodes)*fao_d_fr;
    asmFSidx = (1:n_Zmodes)+n_Zmodes;
    Tasm_2_FS =  G_inner_fb(asmFSidx,1:n_Zmodes) *...
        (frd(Cffasm, w) + frd(Cpi_d, w)*fpre_d);
    L_ao = Tasm_2_FS * Cfb_ao_fr * fwfs_fr;   %unitTsDelay_fr *
    S_ao = eye(n_Zmodes)/(eye(n_Zmodes) + L_ao);
    
    % Loop and sensitivity TFs considering the reconstruction filter dd1*dd2
    L_ao_dd = Tasm_2_FS * frd(fdr_d,w) * Cfb_ao_fr * fwfs_fr;   %unitTsDelay_fr *
    S_ao_dd = eye(n_Zmodes)/(eye(n_Zmodes) + L_ao_dd);
    
    % AO loop transfer function nichols plot (robustness)
    if(1)
        hTFn = figure(figNumber+4);
        for i1 = 1:n_Zmodes
            nichols(fdel(L_ao(i1,i1,:),max(w)),hnichols);
            hold on;
        end
        plot_nichols_ellipse(0.5)
        xlim([-440,0]);
        hold off; grid on;
    end
    
    
    % AO loop transfer function nichols plot (robustness)
    if(1)
        figure(figNumber+5);
        for i1 = 1:n_Zmodes
            bode(L_ao(i1,i1,:),w(1:Nom-1),hbode);
            hold on;
        end
        hold off; grid on;
    end
    
    % AO sensitivity transfer function (disturbance rejection)
    if(1)
        figure(figNumber+6);
        for ik = 1:n_Zmodes
            bodemag(S_ao(ik,ik),w(1:Nom-1),hbode); hold on;
        end
        plot_S_envelope(10)
        set(gca,'Fontsize',14);
        hold off;
    end
end


%% % - - - - - -
return

%% Extra analyses
%%
%% Plots used to report the issues with AO dual-rate
%%

% Bode plot options
hbode_ = hbode;
hbode_.PhaseMatchingValue = 0;
hbode_.Title.String = '';
% Nichols plot options
hnichols_ = hnichols;
hnichols_.PhaseMatching = 'off';
hnichols_.Title.String = '';

Cff0_4 = fpre_d/invKs(4,4);
Cff1 = st.asm.Kb*flag_1stOdyn_d;
Cff2 = st.asm.Km*flag_2ndOdyn_d;

if(0)    
    hTF = figure(figNumber+11);
    set(gcf,'Position',[521+figNumber+21   267   280*3/2   420]);
    bode(Cff0_4,Cff1,Cff2,w(1:Nom-1),hbode_);
    
    children = get(hTF, 'Children');	% use this handle to obtain list of figure's children
    magChild = children(end);           % Pick a handle to axes of magnitude in bode diagram
    axes(magChild);                     % Make those axes current
    
    legend({'$$ K\_{s,4} F\_{\_\textrm{pre}}(j\omega)$$ (Defocus)',...
        '$$ K\_{b} j\omega F\_{\_\textrm{pre}}(j\omega)$$',...
        '$$ K\_m (j\omega)^2 F\_{\_\textrm{pre}}(j\omega) $$'},...
        'Interpreter','latex','Location','southeast','Fontsize',12);
    legend box off
    
    hTF = figure(figNumber+12);
    set(gcf,'Position',[521+figNumber+21   267   280*3/2   420]);
    ff_leg_str = cell(1,6);
    for ik = 1:6
        Cff0 = fpre_d/invKs(ik,ik);
        bode(Cff0+Cff1+Cff2,w(1:Nom-1),hbode_); hold on;
        ff_leg_str{ik} = sprintf('$$C\\_{\\_{\\textrm{FF,%i}}}(j\\omega)$$',ik);
    end
    
    children = get(hTF, 'Children');	% use this handle to obtain list of figure's children
    magChild = children(end);           % Pick a handle to axes of magnitude in bode diagram
    axes(magChild);                     % Make those axes current
    
    legend(ff_leg_str,'Interpreter','latex',...
        'Location','southeast','Fontsize',12,'NumColumns',2);
    legend box off
    hold off;
end


if(0)
    bcolor = [.3 .3 .3];
    hTF = figure(figNumber+13);
    set(gcf,'Position',[321+figNumber+21   267   280*3/2*1.8   300]);
    set(gca,'Units','Normalized','Position',[.06 .11 .92 .92])
    bodemag(fao_d_fr,w(1:Nom-1),hbode_); hold on;
    bodemag(Cff0_4+Cff1+Cff2,w(1:Nom-1),hbode_);
    bodemag((Cff0_4+Cff1+Cff2)*fao_d_fr,w(1:Nom-1),hbode_);
    
    % AO controller nyquist frequency
    ax = findobj(gcf,'type','axes');
    ylim_ = get(ax(end),'YLim');
    plot(0.5/fao_d.Ts*[1 1],ylim,'--','color',bcolor,'Linewidth',1.5);
    text(0.55*0.5/fao_d.Ts,0.7*ylim_(2),...
        sprintf('NGAO controller\nNyquist \nFrequency'),...
        'HorizontalAlignment','center','VerticalAlignment','middle',...
        'color',bcolor,'Fontsize',13,'Interpreter','latex');
    
    legend({'$$\left|C\_{\_\textrm{AO}}^\ast(j\omega)\right|$$',...
        '$$\left|C\_{\_{\textrm{FF,4}}}(j\omega)\right|$$',...
        '$$\left|C\_{\_{\textrm{FF,4}}}(j\omega)\right| \left|C\_{\_\textrm{AO}}^\ast(j\omega)\right|$$'},...
        'Interpreter','latex','Location','northwest','Fontsize',12,'NumColumns',3);
    legend box off
    hold off
end


if(0)    
    % Bode plot
    hbode_ = hbode;
    hbode_.PhaseMatchingValue = 0;
    hbode_.Title.String = '';
    
    hTF = figure(figNumber+91);
    set(gcf,'Position',[521+figNumber+21   267   280*3/2   420]);
    bode(dd1,dd2,dd1*dd2,w(1:Nom-1),hbode_);
    
    children = get(hTF, 'Children');	% use this handle to obtain list of figure's children
    magChild = children(end);           % Pick a handle to axes of magnitude in bode diagram
    axes(magChild);                     % Make those axes current
    
    legend({'$$H\_1^\ast(j\omega)$$','$$H\_2^\ast(j\omega)$$',...
        '$$ H\_{8\uparrow}(j\omega) = H\_1^\ast(j\omega)H\_2^\ast(j\omega) $$'},...
        'Interpreter','latex','Location','southwest','Fontsize',13);
    legend box off
    
    hTF = figure(figNumber+92);
    set(gcf,'Position',[521+figNumber+21   267   280*3/2   420]);
    ff_leg_str = cell(1,6);
    bode((Cff0_4+Cff1+Cff2)*fao_d_fr,w(1:Nom-1),'b-',hbode_); hold on;
    bode((Cff0_4+Cff1+Cff2)*frd(fdr_d,w)*fao_d_fr,w(1:Nom-1),'m-.',hbode_);
    
    children = get(hTF, 'Children');	% use this handle to obtain list of figure's children
    magChild = children(end);           % Pick a handle to axes of magnitude in bode diagram
    axes(magChild);                     % Make those axes current
    
    legend('$$C\_{\_{\textrm{FF,4}}}(j\omega)C\_{\_\textrm{AO}}^\ast(j\omega)$$',...
        '$$C\_{\_{\textrm{FF,4}}}(j\omega)H\_{8\uparrow}(j\omega)C\_{\_\textrm{AO}}^\ast(j\omega)$$',...
        'Interpreter','latex','Location','northwest','Fontsize',13,'NumColumns',1);
    legend box off
    hold off;
end


if(0)
    % AO loop transfer function nichols plot (robustness)
    figure(10*figNumber+4);
    set(gcf,'Position',[521+figNumber+21   467   280*3/2   380]);
    for i1 = 1:n_Zmodes
        nichols(fdel(L_ao(i1,i1,:),max(w)),'-',hnichols_);
        hold on;
        nichols(fdel(L_ao_dd(i1,i1,:),max(w)),'m-.',hnichols_);
    end
    axis([-990 -120 -60 35])
    plot_nichols_ellipse(0.5);
    legend({'$$L\_{\textrm{AO}}(j\omega) $$',...
        '$$L\_{\textrm{AO}}(j\omega)$$ with $$H\_{8\uparrow}(z)$$'},...
        'Interpreter','latex','Location','northwest','Fontsize',13,'NumColumns',1);
    legend box off
    set(gca,'Units','Normalized','Position',[.16 .115 .83 .87],'FontSize',13)
    hold off; grid on;
    
    % AO sensitivity transfer function (disturbance rejection)
    figure(10*figNumber+6);
    set(gcf,'Position',[521+figNumber+21   267   280*3/2   380]);
    
    [MagsigS, ~] = sigma(S_ao,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(MagsigS(1,:)),'-',...
        'color',[0.9290 0.6940 0.1250],'Linewidth',1.3);
    hold on;
    semilogx(w(1:Nom-1)/2/pi,20*log10(MagsigS(end,:)),'-',...
        'color',[0 0.4470 0.7410],'Linewidth',1.3);
    [MagsigS, ~] = sigma(S_ao_dd,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(MagsigS(1,:)),'-.',...
        'color',[0.4940 0.1840 0.5560],'Linewidth',1.3);
    semilogx(w(1:Nom-1)/2/pi,20*log10(MagsigS(end,:)),'-.',...
        'color',[0.4660 0.6740 0.1880],'Linewidth',1.3);
    plot_S_envelope(10);
    axis([w(1)/2/pi, 4e3, -35, 12]);
    ylabel('Magnitude (dB)');
    xlabel('Frequency (Hz)');
    set(gca,'Units','Normalized','Position',[.11 .115 .88 .87],'FontSize',13)
    grid on; hold off;
    legend({'$$\left|\sigma_{\textrm{max}} S_{\textrm{AO}}(j\omega)\right|$$',...
        '$$\left|\sigma_{\textrm{min}} S_{\textrm{AO}}(j\omega)\right|$$',...
        '$$\left|\sigma_{\textrm{max}} S_{\textrm{AO}}(j\omega)\right|$$ with $$H_{8\uparrow}(z)$$',...
        '$$\left|\sigma_{\textrm{min}} S_{\textrm{AO}}(j\omega)\right|$$ with $$H_{8\uparrow}(z)$$'},...
        'Interpreter','latex','Location','southeast','Fontsize',13,'NumColumns',1);
    legend box off
    
end

if(0)
    data_folder = '/Users/rromano/Workspace/reports_slides/flexFS_ASM_IM_report/flexASMrep_sim_data';
    load(fullfile(data_folder,'ASM_RbFs_ngaoH8_bessel4_2pct'),'ASM_Fs');
    
    hTF = figure(figNumber+14);
    set(gcf,'Position',[321+figNumber+21   267   280*3/2*1.8   300]);
    set(gca,'Units','Normalized','Position',[.06 .11 .92 .83],'Fontsize',13);
    indexOfInterest = (ASM_Fs.time < 4) & (ASM_Fs.time > 2.5); % range of t
    plot(ASM_Fs.time(indexOfInterest),ASM_Fs.signals.values(indexOfInterest,1:6));
    xlabel('Time (s)');
    ylabel('ASM-FS mode coeff');
    grid on; hold on;
    
    xmin=3.1; xmax=3.11;
    indexOfInterest = (ASM_Fs.time < xmax) & (ASM_Fs.time > xmin); % range of t
    ymin=min(ASM_Fs.signals.values(indexOfInterest,1:6),[],'all');
    ymax=1.1*max(ASM_Fs.signals.values(indexOfInterest,1:6),[],'all');
    plot([xmin xmax xmax xmin xmin],[ymin ymin ymax ymax ymin],'k--','Linewidth',2)
    % Legend
    legend('mode 1','mode 2','mode 3','mode 4','mode 5','mode 6',...
        'Interpreter','latex','Location','northwest','Fontsize',13,'NumColumns',2);
    legend box off
    hold off
    
    % create a new pair of axes inside current figure
    axes('position',[.15 .175 .65 .25])
    box on % put box around new pair of axes
    
    plot(ASM_Fs.time(indexOfInterest),ASM_Fs.signals.values(indexOfInterest,1:6));
    axis tight;
%     ax_ = axes(); ax_(4) = 1.1*ax_(4); axes(ax_);
    
end



if(0)
    [response,freq] = frdata(frd(fao_d,w));
    fao_d_fr = frd(response, freq, Ts);
    [response,freq] = frdata(frd(fao_d2k,w));
    fao_d2k_fr = frd(response, freq, Ts);
    [response,freq] = frdata(frd(fao_d4k,w));
    fao_d4k_fr = frd(response, freq, Ts);
    [response,freq] = frdata(frd(fao_d8k,w));
    fao_d8k_fr = frd(response, freq, Ts);
    
    hTF = figure(figNumber+25);
    set(gcf,'Position',[321+figNumber+21   267   280*3/2*1.8   300]);
    set(gca,'Units','Normalized','Position',[.06 .16 .92 .83])
    [Mcff,~] = bode(Cff0_4+Cff1+Cff2,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(Mcff(:)),'k:','Linewidth',1.5);
    hold on;
    [M1k,~] = bode((Cff0_4+Cff1+Cff2)*fao_d_fr*fwfs_fr,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(M1k(:)),'--','Linewidth',1.3);
    [M2k,~] = bode((Cff0_4+Cff1+Cff2)*fao_d2k_fr*fwfs_fr,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(M2k(:)),'--','Linewidth',1.3);
    [M4k,~] = bode((Cff0_4+Cff1+Cff2)*fao_d4k_fr*fwfs_fr,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(M4k(:)),'-','Linewidth',1.3);
    [M8k,~] = bode((Cff0_4+Cff1+Cff2)*fao_d8k_fr*fwfs_fr,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(M8k(:)),'-','Linewidth',1.3);
    [MM,~] = bode((Cff0_4+Cff1+Cff2)*frd(fdr_d,w)*fao_d_fr*fwfs_fr,w(1:Nom-1));
    semilogx(w(1:Nom-1)/2/pi,20*log10(MM(:)),':','Linewidth',2);
    
    axis tight; ylim([-35 135]); grid on;
    xlabel('Frequency (Hz)','Fontsize',13);
    ylabel('Magnitude (dB)','Fontsize',13)
    
%     % AO controller nyquist frequency
%     ax = findobj(gcf,'type','axes');
%     ylim_ = get(ax(end),'YLim');
%     bcolor = [.3 .3 .3];
%     plot(0.5/fao_d.Ts*[1 1],ylim,'--','color',bcolor,'Linewidth',1.5);
%     text(0.55*0.5/fao_d.Ts,0.7*ylim_(2),...
%         sprintf('NGAO controller\nNyquist \nFrequency'),...
%         'HorizontalAlignment','center','VerticalAlignment','middle',...
%         'color',bcolor,'Fontsize',13,'Interpreter','latex');
    
    legend({'$$\left|C_{_{\textrm{FF,4}}}(j\omega)\right|$$',...
        '$$f_{_{\textrm{AO}}} = 1$$kHz','$$f_{_{\textrm{AO}}} = 2$$kHz',...
        '$$f_{_{\textrm{AO}}} = 4$$kHz','$$f_{_{\textrm{AO}}} = 8$$kHz',...
        '$$\left|C_\textrm{FF}(j\omega)H_{8\uparrow}(j\omega)C_\textrm{AO}^{\ast}(j\omega)H_\textrm{zoh}(j\omega)\right|$$'},...
        'Interpreter','latex','Location','southwest','Fontsize',13,'NumColumns',1);
    legend box off
    hold off;
    
end


return



%% Auxiliar functions
%%
% - - - - - -
% Function to plot -6dB margin after crossover frequency (RTF).
function plot_w0_6db_bode(hTF,w0db) %#ok<*DEFNU>
try
    children = get(hTF, 'Children');	% use this handle to obtain list of figure's children
    magChild = children(end);           % Pick a handle to axes of magnitude in bode diagram
    axes(magChild);                     % Make those axes current
catch
end
ax = findobj(gcf,'type','axes');
xlim_ = get(ax(end),'XLim');
S_env_w = [w0db, w0db, xlim_(2)]';
S_env_mag = [0, -6, -6]';
semilogx(S_env_w,S_env_mag,'k--','Linewidth',2);
end

% Function to plot OAD BW requirement
function hlr = plot_cl_OAD_req(CL_Bw0,hifi_w)

if(nargin < 2), hifi_w = 2*CL_Bw0; end

bcolor = [0.9 0.0 0.0];
ax = findobj(gcf,'type','axes');

xlim_ = get(ax(end),'XLim');
ylim_ = get(ax(end),'YLim');

semilogx([xlim_(1) CL_Bw0],[-3 -3],'r--','Linewidth',2.5);
semilogx([xlim_(1) CL_Bw0],[6 6],'r--','Linewidth',2.5);
hlr = semilogx([CL_Bw0 CL_Bw0],[ylim_(1) -3],'r--','Linewidth',2.5);
semilogx([hifi_w xlim_(2)],[-6 -6],'r--','Linewidth',2.5);
semilogx([CL_Bw0 hifi_w],[6 -6],'r:','Linewidth',1.5);
% semilogx([hifi_w hifi_w],[-6 ylim_(2)],'r:','Linewidth',1.5);

text(0.9*CL_Bw0,-10,sprintf('%.3g Hz\nbandwidth\nlimit',CL_Bw0),...
    'HorizontalAlignment','right',...
    'VerticalAlignment','top',...
    'color',bcolor,'Fontsize',12);
text(0.9*hifi_w,-2,sprintf('-6dB\nHF\nbound'),...
    'HorizontalAlignment','left',...
    'VerticalAlignment','bottom',...
    'color',bcolor,'Fontsize',12);


end

%% Function to plot sensitivity function boundary
function plot_S_envelope(f_20dbrej)
bcolor = [0.9 0.0 0.0];

ax = findobj(gcf,'type','axes');
xlim_ = get(ax(1),'XLim');
S_env_w = [0.1*f_20dbrej,f_20dbrej, f_20dbrej, xlim_(2)];
S_env_mag = [-40,-20, 6, 6];
semilogx(S_env_w,S_env_mag,'--','color',bcolor,'Linewidth',2.5);

text(1.05*f_20dbrej,7,sprintf('RTF envelope'),...
    'HorizontalAlignment','left',...
    'VerticalAlignment','bottom',...
    'color',bcolor,'Fontsize',12);

end

%% Function to plot nichols robustness boundaries
function plot_nichols_ellipse(VM)
bcolor = [0.9 0.0 0.0];

ax = findobj(gcf,'type','axes');
xlim_ = get(ax(1),'XLim');
ylim_ = get(ax(1),'YLim');

thetaS = linspace(-0.8*pi/2,0,501)';
thetaT = linspace(0,0.9*pi/2,501)';

y = (1/VM)*exp(1i*thetaS);

S = y./(1-y);
ph=angle(S)*(180/pi); ph=ph-(360*ceil(ph/360)); mag=20*log10(abs(S));
line(ph,mag,'color',bcolor,'LineStyle','--','Linewidth',2);

y = (1/VM)*exp(1i*thetaT);
T = (1-y)./y;
ph=angle(T)*(180/pi); ph=ph-(360*ceil(ph/360)); mag=20*log10(abs(T));
line(ph,mag,'color',bcolor,'LineStyle','--','Linewidth',2.5);

if 1
    GM = 6; %PM = 30;
    line([xlim_(1) -180],GM*[-1 -1],'color',bcolor,'LineStyle','--','Linewidth',2.5)
    line([-180 -180],[GM ylim_(2)],'color',bcolor,'LineStyle','--','Linewidth',2.5)
end

text(0.5*xlim_(1),-GM,sprintf('robustness\nboundary'),...
    'HorizontalAlignment','left',...
    'VerticalAlignment','bottom',...
    'color',bcolor,'Fontsize',13);

end