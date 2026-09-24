function MAK324_D_Simscape_Calistir(mode)
% MAK324_D_SIMSCAPE_CALISTIR
% MAK 324 Makina Teorisi projesinin D bolumu icin Simscape Multibody
% modelini kurar, alt montaj dalinda kinematik analiz yapar, C bolumundeki
% analitik MATLAB sonuclariyla karsilastirir ve GIF animasyonu uretir.
%
% Gereksinimler:
%   MATLAB R2025a
%   Simulink
%   Simscape
%   Simscape Multibody
%
% Calistirmak icin bu dosyanin bulundugu klasoru MATLAB Current Folder
% yapin ve Command Window'a su komutu yazin:
%   MAK324_D_Simscape_Calistir("model")  % once 3B model testi
%   MAK324_D_Simscape_Calistir("all")    % tam analiz ve ciktilar

clc;
close all;

if nargin < 1
    mode = "all";
end
mode = lower(string(mode));
if ~any(mode == ["model" "all"])
    error('Gecersiz mod. "model" veya "all" kullanin.');
end

rootDir = fileparts(mfilename('fullpath'));
outputDir = fullfile(rootDir,'MAK324_D_Ciktilar');
if ~isfolder(outputDir)
    mkdir(outputDir);
end

fprintf('\nMAK 324 - Simscape Multibody CAE Analizi\n');
fprintf('=========================================\n');
fprintf('1/5 3B mekanizma nesnesi kuruluyor...\n');

[mechanism,initialOP] = buildWiperMechanism();

fprintf('2/5 Model derleniyor ve alt montaj dali kuruluyor...\n');
compiledMechanism = compile(mechanism);
initialState = computeState(compiledMechanism,initialOP);

if initialState.Status ~= simscape.multibody.StateStatus.Valid
    disp(initialState);
    error(['Simscape Multibody modeli gecerli bir baslangic durumuna ' ...
           'kurulamadi. Yukaridaki assembly diagnostics bilgisini kaydedin.']);
end

% Simscape Multibody'nin kendi 3B goruntuleyicisini acar.
visualize(compiledMechanism,initialState,'MAK324_Alt_Montaj');

fprintf('3/5 Simulink/Simscape blok modeli olusturuluyor...\n');
createBlockDiagram(mechanism,initialOP,rootDir);

if mode == "model"
    fprintf('\n3B MODEL TESTI TAMAMLANDI.\n');
    fprintf(['Multibody Explorer ve MAK324_D_Simscape_Model.slx acildi. ' ...
             'Model gorunuyorsa tam analiz icin su komutu calistirin:\n']);
    fprintf('MAK324_D_Simscape_Calistir("all")\n\n');
    return;
end

fprintf('4/5 0-360 derece kinematik tarama yapiliyor...\n');
thetaDeg = (0:1:360).';
simscapeResults = runKinematicSweep(compiledMechanism,thetaDeg);

resultFile = fullfile(outputDir,'MAK324_D_Simscape_Sonuclar.csv');
writetable(simscapeResults,resultFile);

fprintf('5/5 Grafikler, sayisal karsilastirma ve GIF uretiliyor...\n');
analyticFile = fullfile(rootDir,'MAK324_C_Sayisal_Sonuclar.csv');
if isfile(analyticFile)
    comparison = compareWithAnalytical(simscapeResults,analyticFile,outputDir);
    writeSummary(simscapeResults,comparison,outputDir);
else
    warning(['MAK324_C_Sayisal_Sonuclar.csv ayni klasorde bulunamadi. ' ...
             'Simscape sonuclari olusturuldu ancak C-D karsilastirmasi atlandi.']);
    plotSimscapeOnly(simscapeResults,outputDir);
end

createSimulationGif(compiledMechanism,outputDir);

fprintf('\nISLEM TAMAMLANDI.\n');
fprintf('3B CAE modeli : MAK324_D_Simscape_Model.slx\n');
fprintf('Sonuclar       : %s\n',resultFile);
fprintf('Grafik ve GIF  : %s\n',outputDir);
fprintf(['\nMultibody Explorer penceresinde modeli ustten ve izometrik gorunuste ' ...
         'inceleyip ekran goruntusu alabilirsiniz.\n\n']);
end


function [mechanism,op] = buildWiperMechanism()
% Geometri [mm]
r1 = simscape.Value(40,'mm');
r2 = simscape.Value(120,'mm');
r3 = simscape.Value(100,'mm');
r4 = simscape.Value(120,'mm');
r5 = simscape.Value(100,'mm');

mechanism = simscape.multibody.Multibody;
mechanism.Gravity = simscape.Value([0 0 0],'m/s^2');

ground = makeGround();
crank = makeLink(r1,[0.82 0.16 0.12],0,14,7);
% A noktasindaki uc elemanli pim mafsali, ayni konumda iki ayri frame
% connector ile temsil edilir. Boylece sag ve sol biyeller krankla bagimsiz
% revolute joint bloklari uzerinden baglanir.
addFrame(crank,'A_right','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.StandardAxisTranslation( ...
    r1/2,simscape.multibody.Axis.PosX)));
addFrame(crank,'A_left','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.StandardAxisTranslation( ...
    r1/2,simscape.multibody.Axis.PosX)));
addConnector(crank,'A_right');
addConnector(crank,'A_left');
rightCoupler = makeLink(r2,[0.10 0.42 0.78],4,12,6);
rightRocker = makeRocker(r3,[0.04 0.25 0.52],0,100);
leftCoupler = makeLink(r4,[0.95 0.53 0.08],-4,12,6);
leftRocker = makeRocker(r5,[0.72 0.31 0.04],0,100);

addComponent(mechanism,'Ground',ground);
addComponent(mechanism,'Crank',crank);
addComponent(mechanism,'Right_Coupler',rightCoupler);
addComponent(mechanism,'Right_Rocker',rightRocker);
addComponent(mechanism,'Left_Coupler',leftCoupler);
addComponent(mechanism,'Left_Rocker',leftRocker);
addComponent(mechanism,'World',simscape.multibody.WorldFrame);

joint = simscape.multibody.RevoluteJoint;
jointNames = {'J_A0_Crank','J_A_Right','J_B_Right', ...
              'J_B0_Right','J_A_Left','J_C_Left','J_C0_Left'};
for k = 1:numel(jointNames)
    addComponent(mechanism,jointNames{k},joint);
end

% Zemini dunya koordinat sistemine sabitle.
connect(mechanism,'World/W','Ground/reference');

% Sag kapali cevrim: A0-A-B-B0-A0
connectVia(mechanism,'J_A0_Crank', ...
    'Ground/A0','Crank/neg_end');
connectVia(mechanism,'J_A_Right', ...
    'Crank/A_right','Right_Coupler/neg_end');
connectVia(mechanism,'J_B_Right', ...
    'Right_Coupler/pos_end','Right_Rocker/pos_end');
connectVia(mechanism,'J_B0_Right', ...
    'Ground/B0','Right_Rocker/neg_end');

% Sol kapali cevrim: A0-A-C-C0-A0
connectVia(mechanism,'J_A_Left', ...
    'Crank/A_left','Left_Coupler/neg_end');
connectVia(mechanism,'J_C_Left', ...
    'Left_Coupler/pos_end','Left_Rocker/pos_end');
connectVia(mechanism,'J_C0_Left', ...
    'Ground/C0','Left_Rocker/neg_end');

% Alt montaj dalini secen baslangic hedefleri. Giris krank acisi yuksek,
% sarkac hedefleri ise montaj dalini secmek icin dusuk onceliklidir.
op = simscape.op.OperatingPoint;
op('J_A0_Crank/Rz/q') = simscape.op.Target(0,'deg','High');
op('J_B0_Right/Rz/q') = ...
    simscape.op.Target(-59.4897625939,'deg','Low');
op('J_C0_Left/Rz/q') = ...
    simscape.op.Target(-70.7115887759,'deg','Low');
end


function ground = makeGround()
ground = simscape.multibody.RigidBody;

zeroRT = simscape.multibody.RigidTransform;
addFrame(ground,'A0','reference',zeroRT);
addFrame(ground,'B0','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([100 40 0],'mm'))));
addFrame(ground,'C0','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([-100 40 0],'mm'))));

addConnector(ground,'reference');
addConnector(ground,'A0');
addConnector(ground,'B0');
addConnector(ground,'C0');

% Gri taban plakasi yalnizca 3B gorunumu zenginlestirir.
plateFrame = simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([0 20 -8],'mm')));
addFrame(ground,'plate_frame','reference',plateFrame);

plateGeometry = simscape.multibody.Brick( ...
    simscape.Value([260 100 5],'mm'));
plateMass = simscape.multibody.UniformDensity( ...
    simscape.Value(7800,'kg/m^3'));
plateVisual = simscape.multibody.SimpleVisualProperties([0.55 0.58 0.62]);
plate = simscape.multibody.Solid(plateGeometry,plateMass,plateVisual);
addComponent(ground,'Base_Plate','plate_frame',plate);
end


function link = makeLink(lengthValue,color,zOffset,widthMM,heightMM)
link = simscape.multibody.RigidBody;

halfLength = lengthValue/2;
addFrame(link,'neg_end','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.StandardAxisTranslation( ...
    halfLength,simscape.multibody.Axis.NegX)));
addFrame(link,'pos_end','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.StandardAxisTranslation( ...
    halfLength,simscape.multibody.Axis.PosX)));
addConnector(link,'neg_end');
addConnector(link,'pos_end');

bodyFrame = simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([0 0 zOffset],'mm')));
addFrame(link,'body_frame','reference',bodyFrame);

lengthMM = value(lengthValue,'mm');
geometry = simscape.multibody.Brick( ...
    simscape.Value([lengthMM widthMM heightMM],'mm'));
mass = simscape.multibody.UniformDensity( ...
    simscape.Value(2700,'kg/m^3'));
visual = simscape.multibody.SimpleVisualProperties(color);
body = simscape.multibody.Solid(geometry,mass,visual);
addComponent(link,'Body','body_frame',body);
end


function rocker = makeRocker(lengthValue,color,zOffset,bladeArmLengthMM)
rocker = makeLink(lengthValue,color,zOffset,12,6);
lengthMM = value(lengthValue,'mm');

% Opsiyonel b1/b2 uzantisi: proje foyune uygun olarak 100 mm.
armCenterX = -(lengthMM/2 + bladeArmLengthMM/2);
armFrame = simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([armCenterX 0 zOffset],'mm')));
addFrame(rocker,'wiper_arm_frame','reference',armFrame);

armGeometry = simscape.multibody.Brick( ...
    simscape.Value([bladeArmLengthMM 8 4],'mm'));
armMass = simscape.multibody.UniformDensity( ...
    simscape.Value(2700,'kg/m^3'));
armVisual = simscape.multibody.SimpleVisualProperties([0.18 0.18 0.20]);
armSolid = simscape.multibody.Solid(armGeometry,armMass,armVisual);
addComponent(rocker,'Wiper_Arm','wiper_arm_frame',armSolid);

bladeCenterX = -(lengthMM/2 + bladeArmLengthMM);
bladeFrame = simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([bladeCenterX 0 zOffset],'mm')));
addFrame(rocker,'blade_tip','reference',bladeFrame);
addConnector(rocker,'blade_tip');

bladeHalfSpan = 70;
addFrame(rocker,'blade_pos','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([bladeCenterX bladeHalfSpan zOffset],'mm'))));
addFrame(rocker,'blade_neg','reference', ...
    simscape.multibody.RigidTransform( ...
    simscape.multibody.CartesianTranslation( ...
    simscape.Value([bladeCenterX -bladeHalfSpan zOffset],'mm'))));
addConnector(rocker,'blade_pos');
addConnector(rocker,'blade_neg');

bladeGeometry = simscape.multibody.Brick( ...
    simscape.Value([6 2*bladeHalfSpan 3],'mm'));
bladeMass = simscape.multibody.UniformDensity( ...
    simscape.Value(1100,'kg/m^3'));
bladeVisual = simscape.multibody.SimpleVisualProperties([0.03 0.03 0.03]);
bladeSolid = simscape.multibody.Solid(bladeGeometry,bladeMass,bladeVisual);
addComponent(rocker,'Wiper_Blade','blade_tip',bladeSolid);
end


function createBlockDiagram(mechanism,op,rootDir)
modelName = 'MAK324_D_Simscape_Model';
modelFile = fullfile(rootDir,[modelName '.slx']);

if bdIsLoaded(modelName)
    close_system(modelName,0);
end

if isfile(modelFile)
    fprintf('    Mevcut %s korundu ve yeniden acildi.\n',[modelName '.slx']);
    open_system(modelFile);
    return;
end

makeBlockDiagram(mechanism,op,modelName);
set_param(modelName,'StopTime','1');
save_system(modelName,modelFile);
open_system(modelName);
end


function results = runKinematicSweep(compiledMechanism,thetaDeg)
N = numel(thetaDeg);
rightAngle = zeros(N,1);
leftAngle = zeros(N,1);
rightVelocity = zeros(N,1);
leftVelocity = zeros(N,1);
rightAcceleration = zeros(N,1);
leftAcceleration = zeros(N,1);

previousRight = -59.4897625939;
previousLeft = -70.7115887759;
omegaInput = 360; % 60 rpm = 360 deg/s

accelerationDictionary = simscape.multibody.JointAccelerationDictionary;
inputAcceleration = simscape.multibody.RevolutePrimitiveAcceleration( ...
    simscape.Value(0,'deg/s^2'));
accelerationDictionary('J_A0_Crank/Rz') = inputAcceleration;

% Krank ivmesi hareket girdisi olarak verildiginde, bu hareketi saglayan
% motor torku ters dinamik cozumunde bilinmeyen olmalidir. "Computed"
% secimi, Simscape'e gerekli aktüator torkunu otomatik hesaplatir ve
% dinamik denklem sisteminin serbestlik derecesini kapatir.
actuationDictionary = simscape.multibody.JointActuationDictionary;
computedMotorTorque = ...
    simscape.multibody.RevolutePrimitiveActuationTorque("Computed");
actuationDictionary('J_A0_Crank/Rz') = computedMotorTorque;

for k = 1:N
    op = simscape.op.OperatingPoint;
    op('J_A0_Crank/Rz/q') = ...
        simscape.op.Target(thetaDeg(k),'deg','High');
    op('J_A0_Crank/Rz/w') = ...
        simscape.op.Target(omegaInput,'deg/s','High');
    op('J_B0_Right/Rz/q') = ...
        simscape.op.Target(previousRight,'deg','Low');
    op('J_C0_Left/Rz/q') = ...
        simscape.op.Target(previousLeft,'deg','Low');

    state = computeState(compiledMechanism,op);
    if state.Status ~= simscape.multibody.StateStatus.Valid
        disp(state);
        error('Kinematik cozum theta1 = %.1f derecede gecersiz.',thetaDeg(k));
    end

    rightAngle(k) = value(jointPrimitivePosition( ...
        compiledMechanism,'J_B0_Right/Rz',state),'deg');
    leftAngle(k) = value(jointPrimitivePosition( ...
        compiledMechanism,'J_C0_Left/Rz',state),'deg');
    rightVelocity(k) = value(jointPrimitiveVelocity( ...
        compiledMechanism,'J_B0_Right/Rz',state),'deg/s');
    leftVelocity(k) = value(jointPrimitiveVelocity( ...
        compiledMechanism,'J_C0_Left/Rz',state),'deg/s');

    dynamicsResult = computeDynamics(compiledMechanism,state, ...
        actuationDictionary,accelerationDictionary);
    rightAcceleration(k) = value(jointPrimitiveAcceleration( ...
        compiledMechanism,'J_B0_Right/Rz',dynamicsResult),'deg/s^2');
    leftAcceleration(k) = value(jointPrimitiveAcceleration( ...
        compiledMechanism,'J_C0_Left/Rz',dynamicsResult),'deg/s^2');

    previousRight = rightAngle(k);
    previousLeft = leftAngle(k);

    if mod(k-1,60) == 0 || k == N
        fprintf('    Krank acisi: %6.1f / 360 deg\n',thetaDeg(k));
        drawnow;
    end
end

results = table(thetaDeg,rightAngle,leftAngle, ...
    rightVelocity,leftVelocity,rightAcceleration,leftAcceleration, ...
    'VariableNames',{'Krank_Acisi_deg', ...
    'Simscape_theta3_deg','Simscape_theta5_deg', ...
    'Simscape_omega3_deg_s','Simscape_omega5_deg_s', ...
    'Simscape_alpha3_deg_s2','Simscape_alpha5_deg_s2'});
end


function comparison = compareWithAnalytical(simResults,analyticFile,outputDir)
analytic = readtable(analyticFile);
x = simResults.Krank_Acisi_deg;

comparison.theta3 = interp1(analytic.Krank_Acisi_deg, ...
    analytic.theta3_deg,x,'pchip');
comparison.theta5 = interp1(analytic.Krank_Acisi_deg, ...
    analytic.theta5_deg,x,'pchip');
comparison.omega3 = interp1(analytic.Krank_Acisi_deg, ...
    analytic.omega3_deg_s,x,'pchip');
comparison.omega5 = interp1(analytic.Krank_Acisi_deg, ...
    analytic.omega5_deg_s,x,'pchip');
comparison.alpha3 = interp1(analytic.Krank_Acisi_deg, ...
    analytic.alpha3_deg_s2,x,'pchip');
comparison.alpha5 = interp1(analytic.Krank_Acisi_deg, ...
    analytic.alpha5_deg_s2,x,'pchip');

comparison.errTheta3 = simResults.Simscape_theta3_deg-comparison.theta3;
comparison.errTheta5 = simResults.Simscape_theta5_deg-comparison.theta5;
comparison.errOmega3 = simResults.Simscape_omega3_deg_s-comparison.omega3;
comparison.errOmega5 = simResults.Simscape_omega5_deg_s-comparison.omega5;
comparison.errAlpha3 = simResults.Simscape_alpha3_deg_s2-comparison.alpha3;
comparison.errAlpha5 = simResults.Simscape_alpha5_deg_s2-comparison.alpha5;

comparisonTable = table(x,comparison.theta3, ...
    simResults.Simscape_theta3_deg,comparison.errTheta3, ...
    comparison.theta5,simResults.Simscape_theta5_deg,comparison.errTheta5, ...
    comparison.omega3,simResults.Simscape_omega3_deg_s,comparison.errOmega3, ...
    comparison.omega5,simResults.Simscape_omega5_deg_s,comparison.errOmega5, ...
    comparison.alpha3,simResults.Simscape_alpha3_deg_s2,comparison.errAlpha3, ...
    comparison.alpha5,simResults.Simscape_alpha5_deg_s2,comparison.errAlpha5, ...
    'VariableNames',{'Krank_Acisi_deg', ...
    'Analitik_theta3_deg','Simscape_theta3_deg','Hata_theta3_deg', ...
    'Analitik_theta5_deg','Simscape_theta5_deg','Hata_theta5_deg', ...
    'Analitik_omega3_deg_s','Simscape_omega3_deg_s','Hata_omega3_deg_s', ...
    'Analitik_omega5_deg_s','Simscape_omega5_deg_s','Hata_omega5_deg_s', ...
    'Analitik_alpha3_deg_s2','Simscape_alpha3_deg_s2','Hata_alpha3_deg_s2', ...
    'Analitik_alpha5_deg_s2','Simscape_alpha5_deg_s2','Hata_alpha5_deg_s2'});
writetable(comparisonTable, ...
    fullfile(outputDir,'MAK324_D_Analitik_Simscape_Karsilastirma.csv'));

makeComparisonFigure(x,comparison.theta3,simResults.Simscape_theta3_deg, ...
    comparison.theta5,simResults.Simscape_theta5_deg, ...
    'Açısal Konum','Açı [deg]', ...
    fullfile(outputDir,'MAK324_D_Karsilastirma_Konum.png'));
makeComparisonFigure(x,comparison.omega3,simResults.Simscape_omega3_deg_s, ...
    comparison.omega5,simResults.Simscape_omega5_deg_s, ...
    'Açısal Hız','Açısal hız [deg/s]', ...
    fullfile(outputDir,'MAK324_D_Karsilastirma_Hiz.png'));
makeComparisonFigure(x,comparison.alpha3,simResults.Simscape_alpha3_deg_s2, ...
    comparison.alpha5,simResults.Simscape_alpha5_deg_s2, ...
    'Açısal İvme','Açısal ivme [deg/s^2]', ...
    fullfile(outputDir,'MAK324_D_Karsilastirma_Ivme.png'));

makeErrorFigure(x,comparison,outputDir);
end


function makeComparisonFigure(x,analyticRight,simscapeRight, ...
    analyticLeft,simscapeLeft,quantityName,yLabelText,outputFile)
fig = figure('Color','w','Position',[80 80 1200 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(x,analyticRight,'Color',[0.08 0.27 0.53], ...
    'LineWidth',2.2,'DisplayName','Analitik MATLAB');
hold on;
plot(x,simscapeRight,'--','Color',[0.90 0.36 0.08], ...
    'LineWidth',1.7,'DisplayName','Simscape Multibody');
grid on;
xlim([0 360]);
ylabel(yLabelText);
title(['Sağ silecek - ' quantityName]);
legend('Location','best');

nexttile;
plot(x,analyticLeft,'Color',[0.08 0.27 0.53], ...
    'LineWidth',2.2,'DisplayName','Analitik MATLAB');
hold on;
plot(x,simscapeLeft,'--','Color',[0.90 0.36 0.08], ...
    'LineWidth',1.7,'DisplayName','Simscape Multibody');
grid on;
xlim([0 360]);
xlabel('Krank açısı, \theta_1 [deg]');
ylabel(yLabelText);
title(['Sol silecek - ' quantityName]);
legend('Location','best');

sgtitle(['MAK 324: Analitik Kod ve Simscape Multibody ' quantityName ...
    ' Karşılaştırması']);
exportgraphics(fig,outputFile,'Resolution',300);
end


function makeErrorFigure(x,c,outputDir)
fig = figure('Color','w','Position',[90 90 1200 850]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(x,c.errTheta3,'LineWidth',1.7); hold on;
plot(x,c.errTheta5,'--','LineWidth',1.7);
grid on; xlim([0 360]);
ylabel('Hata [deg]');
title('Konum farkı: Simscape - analitik');
legend('\theta_3','\theta_5','Location','best');

nexttile;
plot(x,c.errOmega3,'LineWidth',1.7); hold on;
plot(x,c.errOmega5,'--','LineWidth',1.7);
grid on; xlim([0 360]);
ylabel('Hata [deg/s]');
title('Hız farkı: Simscape - analitik');
legend('\omega_3','\omega_5','Location','best');

nexttile;
plot(x,c.errAlpha3,'LineWidth',1.7); hold on;
plot(x,c.errAlpha5,'--','LineWidth',1.7);
grid on; xlim([0 360]);
xlabel('Krank açısı, \theta_1 [deg]');
ylabel('Hata [deg/s^2]');
title('İvme farkı: Simscape - analitik');
legend('\alpha_3','\alpha_5','Location','best');

sgtitle('MAK 324: Sayısal Fark Eğrileri');
exportgraphics(fig, ...
    fullfile(outputDir,'MAK324_D_Karsilastirma_Hata.png'),'Resolution',300);
end


function plotSimscapeOnly(results,outputDir)
x = results.Krank_Acisi_deg;
fig = figure('Color','w','Position',[100 100 1200 850]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(x,results.Simscape_theta3_deg,'LineWidth',1.8); hold on;
plot(x,results.Simscape_theta5_deg,'--','LineWidth',1.8);
grid on; ylabel('Açı [deg]'); legend('\theta_3','\theta_5');
nexttile;
plot(x,results.Simscape_omega3_deg_s,'LineWidth',1.8); hold on;
plot(x,results.Simscape_omega5_deg_s,'--','LineWidth',1.8);
grid on; ylabel('Hız [deg/s]'); legend('\omega_3','\omega_5');
nexttile;
plot(x,results.Simscape_alpha3_deg_s2,'LineWidth',1.8); hold on;
plot(x,results.Simscape_alpha5_deg_s2,'--','LineWidth',1.8);
grid on; xlabel('Krank açısı [deg]'); ylabel('İvme [deg/s^2]');
legend('\alpha_3','\alpha_5');
sgtitle('MAK 324 - Simscape Multibody Kinematik Sonuçları');
exportgraphics(fig,fullfile(outputDir,'MAK324_D_Simscape_Grafikleri.png'), ...
    'Resolution',300);
end


function writeSummary(results,c,outputDir)
rightSweep = max(results.Simscape_theta3_deg)-min(results.Simscape_theta3_deg);
leftSweep = max(results.Simscape_theta5_deg)-min(results.Simscape_theta5_deg);

summaryFile = fullfile(outputDir,'MAK324_D_Ozet.txt');
fid = fopen(summaryFile,'w');
if fid < 0
    warning('Ozet dosyasi acilamadi: %s',summaryFile);
    return;
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid,'MAK 324 D BOLUMU - SIMSCAPE MULTIBODY OZETI\n');
fprintf(fid,'=============================================\n');
fprintf(fid,'Giris hizi: 60 rpm (360 deg/s)\n');
fprintf(fid,'Simscape adimi: 1 deg\n');
fprintf(fid,'Montaj dali: Alt montaj\n');
fprintf(fid,'Sag silecek supurme acisi: %.9f deg\n',rightSweep);
fprintf(fid,'Sol silecek supurme acisi: %.9f deg\n',leftSweep);
fprintf(fid,'Maksimum mutlak konum farki theta3: %.6e deg\n', ...
    max(abs(c.errTheta3)));
fprintf(fid,'Maksimum mutlak konum farki theta5: %.6e deg\n', ...
    max(abs(c.errTheta5)));
fprintf(fid,'Maksimum mutlak hiz farki omega3: %.6e deg/s\n', ...
    max(abs(c.errOmega3)));
fprintf(fid,'Maksimum mutlak hiz farki omega5: %.6e deg/s\n', ...
    max(abs(c.errOmega5)));
fprintf(fid,'Maksimum mutlak ivme farki alpha3: %.6e deg/s^2\n', ...
    max(abs(c.errAlpha3)));
fprintf(fid,'Maksimum mutlak ivme farki alpha5: %.6e deg/s^2\n', ...
    max(abs(c.errAlpha5)));
end


function createSimulationGif(compiledMechanism,outputDir)
gifFile = fullfile(outputDir,'MAK324_D_Simscape_Animasyon.gif');
frameAngles = 0:6:360;
previousRight = -59.4897625939;
previousLeft = -70.7115887759;

fig = figure('Color','w','Position',[120 80 1100 720]);

for k = 1:numel(frameAngles)
    op = simscape.op.OperatingPoint;
    op('J_A0_Crank/Rz/q') = ...
        simscape.op.Target(frameAngles(k),'deg','High');
    op('J_B0_Right/Rz/q') = ...
        simscape.op.Target(previousRight,'deg','Low');
    op('J_C0_Left/Rz/q') = ...
        simscape.op.Target(previousLeft,'deg','Low');
    state = computeState(compiledMechanism,op);

    previousRight = value(jointPrimitivePosition( ...
        compiledMechanism,'J_B0_Right/Rz',state),'deg');
    previousLeft = value(jointPrimitivePosition( ...
        compiledMechanism,'J_C0_Left/Rz',state),'deg');

    A0 = [0;0;0];
    B0 = [100;40;0];
    C0 = [-100;40;0];
    A = framePoint(compiledMechanism,'Crank/pos_end',state);
    B = framePoint(compiledMechanism,'Right_Coupler/pos_end',state);
    C = framePoint(compiledMechanism,'Left_Coupler/pos_end',state);
    rTip = framePoint(compiledMechanism,'Right_Rocker/blade_tip',state);
    rPos = framePoint(compiledMechanism,'Right_Rocker/blade_pos',state);
    rNeg = framePoint(compiledMechanism,'Right_Rocker/blade_neg',state);
    lTip = framePoint(compiledMechanism,'Left_Rocker/blade_tip',state);
    lPos = framePoint(compiledMechanism,'Left_Rocker/blade_pos',state);
    lNeg = framePoint(compiledMechanism,'Left_Rocker/blade_neg',state);

    clf(fig);
    hold on;
    fill3([-135 135 135 -135],[-15 -15 70 70],[-10 -10 -10 -10], ...
        [0.86 0.88 0.90],'EdgeColor',[0.50 0.52 0.55], ...
        'DisplayName','Sabit gövde');

    drawBar(A0,A,[0.82 0.16 0.12],7,'Krank A_0A');
    drawBar(A,B,[0.10 0.42 0.78],6,'Sağ biyel AB');
    drawBar(B0,B,[0.04 0.25 0.52],6,'Sağ sarkaç B_0B');
    drawBar(A,C,[0.95 0.53 0.08],6,'Sol biyel AC');
    drawBar(C0,C,[0.72 0.31 0.04],6,'Sol sarkaç C_0C');
    drawBar(B0,rTip,[0.18 0.18 0.20],4,'Sağ silecek kolu');
    drawBar(C0,lTip,[0.18 0.18 0.20],4,'Sol silecek kolu');
    drawBar(rNeg,rPos,[0.02 0.02 0.02],5,'Sağ silecek');
    drawBar(lNeg,lPos,[0.02 0.02 0.02],5,'Sol silecek');

    P = [A0 B0 C0 A B C];
    scatter3(P(1,:),P(2,:),P(3,:),55,'k','filled', ...
        'HandleVisibility','off');
    text(A0(1)+4,A0(2)-7,2,'A_0');
    text(B0(1)+4,B0(2)+4,2,'B_0');
    text(C0(1)-14,C0(2)+4,2,'C_0');
    text(A(1)+4,A(2)-5,2,'A');
    text(B(1)+4,B(2)-5,2,'B');
    text(C(1)-12,C(2)-5,2,'C');

    axis equal;
    xlim([-250 250]);
    ylim([-185 190]);
    zlim([-20 30]);
    view(0,90);
    grid on;
    xlabel('x [mm]');
    ylabel('y [mm]');
    title(sprintf(['Simscape Multibody Kinematik Simülasyonu - ' ...
        '\\theta_1 = %.0f°'],frameAngles(k)));
    legend('Location','eastoutside');
    drawnow;

    frame = getframe(fig);
    rgbImage = frame2im(frame);
    [indexedImage,colorMap] = rgb2ind(rgbImage,256);
    if k == 1
        imwrite(indexedImage,colorMap,gifFile,'gif', ...
            'LoopCount',inf,'DelayTime',0.06);
    else
        imwrite(indexedImage,colorMap,gifFile,'gif', ...
            'WriteMode','append','DelayTime',0.06);
    end
end
end


function pointMM = framePoint(compiledMechanism,framePath,state)
T = transformation(compiledMechanism,'World/W',framePath,state);
point = transformPoint(T,simscape.Value([0;0;0],'mm'));
pointMM = value(point,'mm');
end


function drawBar(P1,P2,color,lineWidth,displayName)
plot3([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], ...
    'Color',color,'LineWidth',lineWidth,'DisplayName',displayName);
end
