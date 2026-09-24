%% MAK 324 - Makina Teorisi Dönem Projesi
% C Bölümü: Çift silecek mekanizmasının konum, hız ve ivme analizi
%
% Mekanizma:
%   Sağ çevrim : A0-A-B-B0-A0
%   Sol çevrim : A0-A-C-C0-A0
%
% Koordinatlar [mm]:
%   A0 = (0,0), B0 = (100,40), C0 = (-100,40)
%
% Uzunluklar [mm]:
%   A0A = 40, AB = AC = 120, B0B = C0C = 100
%
% Açı tanımı:
%   theta_i, yönlendirilmiş i. uzvun global +x ekseniyle yaptığı açıdır;
%   saat yönünün tersi pozitiftir. Yönler: theta1 A0->A, theta2 A->B,
%   theta3 B0->B, theta4 A->C, theta5 C0->C.
%
% Varsayım:
%   Giriş hızı 60 rpm ve sabit, dolayısıyla omega1 = 2*pi rad/s
%   ve alpha1 = 0 rad/s^2.
%
% Bu betik Optimization Toolbox gerektirmez. Konum analizi özel bir
% Newton-Raphson algoritması ile yapılır. Hız ve ivme analizlerinde aynı
% Jacobian matrisi kullanılır.

clear; close all; clc;

%% 1. Model parametreleri
p.r1 = 40;      % A0A [mm]
p.r2 = 120;     % AB  [mm]
p.r3 = 100;     % B0B [mm]
p.r4 = 120;     % AC  [mm]
p.r5 = 100;     % C0C [mm]
p.B0 = [100; 40];
p.C0 = [-100; 40];

inputRPM = 60;
omega1 = 2*pi*inputRPM/60;  % [rad/s]
alpha1 = 0;                 % [rad/s^2]

theta1_deg = 0:0.5:360;
theta1 = deg2rad(theta1_deg);
N = numel(theta1);

% Şekildeki alt montaj koluna karşılık gelen başlangıç tahmini
% q = [theta2 theta3 theta4 theta5]^T
q0 = deg2rad([-22.6198649480; ...
              -59.4897625939; ...
             -153.0493331151; ...
              -70.7115887759]);

q = zeros(4,N);
qdot = zeros(4,N);
qddot = zeros(4,N);
residualInf = zeros(1,N);
iterationCount = zeros(1,N);
conditionJ = zeros(1,N);

%% 2. Konum, hız ve ivme çözümleri
qGuess = q0;

for k = 1:N
    [q(:,k), iterationCount(k), residualInf(k)] = ...
        solvePositionNewton(theta1(k), qGuess, p);

    J = constraintJacobian(q(:,k), p);
    conditionJ(k) = cond(J);

    % Hız denklemi: J*qdot = -Phi_theta1*omega1
    velocityRHS = [ p.r1*sin(theta1(k))*omega1; ...
                   -p.r1*cos(theta1(k))*omega1; ...
                    p.r1*sin(theta1(k))*omega1; ...
                   -p.r1*cos(theta1(k))*omega1 ];

    qdot(:,k) = J \ velocityRHS;

    % İvme denklemi: J*qddot = b
    b = accelerationRHS(theta1(k), q(:,k), ...
                        omega1, alpha1, qdot(:,k), p);
    qddot(:,k) = J \ b;

    % Bir sonraki krank açısında aynı montaj kolunu korur.
    qGuess = q(:,k);
end

%% 3. Sonuçların işlenmesi
qUnwrapped = unwrap(q,[],2);
qDeg = rad2deg(qUnwrapped);
qdotDeg = rad2deg(qdot);
qddotDeg = rad2deg(qddot);

% Sağ ve sol silecek kolları, theta3 ve theta5 sarkaçlarına rijit bağlıdır.
% Sabit pi radyanlik yön farkı hız ve ivmeyi değiştirmez.
rightWiperAngle = qDeg(2,:);
leftWiperAngle = qDeg(4,:);

rightSweep = max(rightWiperAngle)-min(rightWiperAngle);
leftSweep = max(leftWiperAngle)-min(leftWiperAngle);

fprintf('\nMAK 324 C Bolumu - Sayisal Sonuclar\n');
fprintf('----------------------------------\n');
fprintf('Giris hizi                 : %.3f rpm\n', inputRPM);
fprintf('Sag silecek supurme acisi : %.6f derece\n', rightSweep);
fprintf('Sol silecek supurme acisi : %.6f derece\n', leftSweep);
fprintf('Maksimum konum kalintisi  : %.3e mm\n', max(residualInf));
fprintf('Maksimum Jacobian kosulu  : %.3f\n', max(conditionJ));
fprintf('Maksimum Newton iterasyonu: %d\n\n', max(iterationCount));

%% 4. CSV ciktilari
resultsTable = table( ...
    theta1_deg(:), ...
    qDeg(1,:).', qDeg(2,:).', qDeg(3,:).', qDeg(4,:).', ...
    qdotDeg(1,:).', qdotDeg(2,:).', qdotDeg(3,:).', qdotDeg(4,:).', ...
    qddotDeg(1,:).', qddotDeg(2,:).', qddotDeg(3,:).', qddotDeg(4,:).', ...
    residualInf(:), iterationCount(:), conditionJ(:), ...
    'VariableNames', { ...
    'Krank_Acisi_deg', ...
    'theta2_deg','theta3_deg','theta4_deg','theta5_deg', ...
    'omega2_deg_s','omega3_deg_s','omega4_deg_s','omega5_deg_s', ...
    'alpha2_deg_s2','alpha3_deg_s2','alpha4_deg_s2','alpha5_deg_s2', ...
    'Kisit_Kalintisi_mm','Newton_Iterasyon','Jacobian_Cond'});

writetable(resultsTable,'MAK324_C_Sayisal_Sonuclar.csv');

summaryFile = fopen('MAK324_C_Ozet.txt','w');
fprintf(summaryFile,'MAK 324 - C Bolumu Sayisal Ozet\n');
fprintf(summaryFile,'================================\n');
fprintf(summaryFile,'Giris hizi: %.3f rpm\n',inputRPM);
fprintf(summaryFile,'omega1: %.9f rad/s\n',omega1);
fprintf(summaryFile,'alpha1: %.3f rad/s^2\n',alpha1);
fprintf(summaryFile,'Sag silecek supurme acisi: %.9f derece\n',rightSweep);
fprintf(summaryFile,'Sol silecek supurme acisi: %.9f derece\n',leftSweep);
fprintf(summaryFile,'Sag silecek acisal hiz araligi: %.9f ile %.9f derece/s\n', ...
        min(qdotDeg(2,:)),max(qdotDeg(2,:)));
fprintf(summaryFile,'Sol silecek acisal hiz araligi: %.9f ile %.9f derece/s\n', ...
        min(qdotDeg(4,:)),max(qdotDeg(4,:)));
fprintf(summaryFile,'Sag silecek acisal ivme araligi: %.9f ile %.9f derece/s^2\n', ...
        min(qddotDeg(2,:)),max(qddotDeg(2,:)));
fprintf(summaryFile,'Sol silecek acisal ivme araligi: %.9f ile %.9f derece/s^2\n', ...
        min(qddotDeg(4,:)),max(qddotDeg(4,:)));
fprintf(summaryFile,'Maksimum konum kalintisi: %.12e mm\n',max(residualInf));
fprintf(summaryFile,'Maksimum Newton iterasyonu: %d\n',max(iterationCount));
fprintf(summaryFile,'Maksimum Jacobian kosul sayisi: %.9f\n',max(conditionJ));
fclose(summaryFile);

%% 5. Konum grafigi
fig1 = figure('Color','w','Position',[100 100 1200 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(theta1_deg,qDeg(1,:),'LineWidth',1.8); hold on;
plot(theta1_deg,qDeg(3,:),'--','LineWidth',1.8);
grid on; xlim([0 360]);
ylabel('Biyel acisi [deg]');
title('Biyel Acilarinin Krank Acisina Bagli Degisimi');
legend('\theta_2: AB','\theta_4: AC','Location','best');

nexttile;
plot(theta1_deg,qDeg(2,:),'LineWidth',2.0); hold on;
plot(theta1_deg,qDeg(4,:),'--','LineWidth',2.0);
grid on; xlim([0 360]);
xlabel('Krank acisi, \theta_1 [deg]');
ylabel('Sarkac acisi [deg]');
title(sprintf('Silecek Cikis Acilari - Supurme: Sag %.3f deg, Sol %.3f deg', ...
      rightSweep,leftSweep));
legend('\theta_3: sag silecek','\theta_5: sol silecek','Location','best');

exportgraphics(fig1,'MAK324_C_Konum.png','Resolution',300);

%% 6. Hiz grafigi
fig2 = figure('Color','w','Position',[100 100 1200 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(theta1_deg,qdotDeg(1,:),'LineWidth',1.8); hold on;
plot(theta1_deg,qdotDeg(3,:),'--','LineWidth',1.8);
grid on; xlim([0 360]);
ylabel('Acilisal hiz [deg/s]');
title('Biyel Acisal Hizlari');
legend('\omega_2','\omega_4','Location','best');

nexttile;
plot(theta1_deg,qdotDeg(2,:),'LineWidth',2.0); hold on;
plot(theta1_deg,qdotDeg(4,:),'--','LineWidth',2.0);
yline(0,':','HandleVisibility','off');
grid on; xlim([0 360]);
xlabel('Krank acisi, \theta_1 [deg]');
ylabel('Acilisal hiz [deg/s]');
title('Sag ve Sol Silecek Acisal Hizlari');
legend('\omega_3: sag silecek','\omega_5: sol silecek','Location','best');

exportgraphics(fig2,'MAK324_C_Hiz.png','Resolution',300);

%% 7. Ivme grafigi
fig3 = figure('Color','w','Position',[100 100 1200 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(theta1_deg,qddotDeg(1,:),'LineWidth',1.8); hold on;
plot(theta1_deg,qddotDeg(3,:),'--','LineWidth',1.8);
grid on; xlim([0 360]);
ylabel('Acilisal ivme [deg/s^2]');
title('Biyel Acisal Ivmeleri');
legend('\alpha_2','\alpha_4','Location','best');

nexttile;
plot(theta1_deg,qddotDeg(2,:),'LineWidth',2.0); hold on;
plot(theta1_deg,qddotDeg(4,:),'--','LineWidth',2.0);
yline(0,':','HandleVisibility','off');
grid on; xlim([0 360]);
xlabel('Krank acisi, \theta_1 [deg]');
ylabel('Acilisal ivme [deg/s^2]');
title('Sag ve Sol Silecek Acisal Ivmeleri');
legend('\alpha_3: sag silecek','\alpha_5: sol silecek','Location','best');

exportgraphics(fig3,'MAK324_C_Ivme.png','Resolution',300);

%% 8. Mekanizma animasyonu
gifName = 'MAK324_C_Mekanizma.gif';
fig4 = figure('Color','w','Position',[100 100 1000 650]);
animationIndices = 1:12:N; % Her 6 derecede bir kare

for frameNo = 1:numel(animationIndices)
    k = animationIndices(frameNo);
    drawMechanism(theta1(k),q(:,k),p,fig4);
    title(sprintf('MAK 324 Silecek Mekanizmasi - \theta_1 = %.1f deg', ...
          theta1_deg(k)));
    drawnow;

    frame = getframe(fig4);
    rgbImage = frame2im(frame);
    [indexedImage,colorMap] = rgb2ind(rgbImage,256);

    if frameNo == 1
        imwrite(indexedImage,colorMap,gifName,'gif', ...
                'LoopCount',inf,'DelayTime',0.04);
    else
        imwrite(indexedImage,colorMap,gifName,'gif', ...
                'WriteMode','append','DelayTime',0.04);
    end
end

fprintf('Dosyalar basariyla olusturuldu:\n');
fprintf('  MAK324_C_Sayisal_Sonuclar.csv\n');
fprintf('  MAK324_C_Ozet.txt\n');
fprintf('  MAK324_C_Konum.png\n');
fprintf('  MAK324_C_Hiz.png\n');
fprintf('  MAK324_C_Ivme.png\n');
fprintf('  MAK324_C_Mekanizma.gif\n');

%% Yerel fonksiyonlar
function [q,iteration,residualInf] = solvePositionNewton(theta1,qInitial,p)
    q = qInitial;
    tolerance = 1e-11;
    maximumIterations = 30;

    for iteration = 1:maximumIterations
        Phi = constraintVector(theta1,q,p);
        residualInf = norm(Phi,inf);

        if residualInf < tolerance
            return;
        end

        J = constraintJacobian(q,p);
        deltaQ = J \ (-Phi);
        q = q + deltaQ;

        if norm(deltaQ,inf) < 1e-13
            Phi = constraintVector(theta1,q,p);
            residualInf = norm(Phi,inf);
            if residualInf < tolerance
                return;
            end
        end
    end

    error('Newton-Raphson yakinmadi. theta1 = %.6f rad, kalinti = %.3e', ...
          theta1,residualInf);
end

function Phi = constraintVector(theta1,q,p)
    theta2 = q(1); theta3 = q(2);
    theta4 = q(3); theta5 = q(4);

    Phi = [ ...
        p.r1*cos(theta1)+p.r2*cos(theta2)-p.r3*cos(theta3)-p.B0(1); ...
        p.r1*sin(theta1)+p.r2*sin(theta2)-p.r3*sin(theta3)-p.B0(2); ...
        p.r1*cos(theta1)+p.r4*cos(theta4)-p.r5*cos(theta5)-p.C0(1); ...
        p.r1*sin(theta1)+p.r4*sin(theta4)-p.r5*sin(theta5)-p.C0(2) ];
end

function J = constraintJacobian(q,p)
    theta2 = q(1); theta3 = q(2);
    theta4 = q(3); theta5 = q(4);

    J = [ ...
       -p.r2*sin(theta2),  p.r3*sin(theta3), 0, 0; ...
        p.r2*cos(theta2), -p.r3*cos(theta3), 0, 0; ...
        0, 0, -p.r4*sin(theta4),  p.r5*sin(theta5); ...
        0, 0,  p.r4*cos(theta4), -p.r5*cos(theta5) ];
end

function b = accelerationRHS(theta1,q,omega1,alpha1,qdot,p)
    theta2 = q(1); theta3 = q(2);
    theta4 = q(3); theta5 = q(4);
    omega2 = qdot(1); omega3 = qdot(2);
    omega4 = qdot(3); omega5 = qdot(4);

    inputX = p.r1*(cos(theta1)*omega1^2 + sin(theta1)*alpha1);
    inputY = p.r1*(sin(theta1)*omega1^2 - cos(theta1)*alpha1);

    b = [ ...
        inputX+p.r2*cos(theta2)*omega2^2-p.r3*cos(theta3)*omega3^2; ...
        inputY+p.r2*sin(theta2)*omega2^2-p.r3*sin(theta3)*omega3^2; ...
        inputX+p.r4*cos(theta4)*omega4^2-p.r5*cos(theta5)*omega5^2; ...
        inputY+p.r4*sin(theta4)*omega4^2-p.r5*sin(theta5)*omega5^2 ];
end

function drawMechanism(theta1,q,p,figHandle)
    figure(figHandle); clf(figHandle);

    A0 = [0;0];
    A = A0+p.r1*[cos(theta1);sin(theta1)];
    B = p.B0+p.r3*[cos(q(2));sin(q(2))];
    C = p.C0+p.r5*[cos(q(4));sin(q(4))];

    bladeLength = 100;
    rightBladeTip = p.B0-bladeLength*[cos(q(2));sin(q(2))];
    leftBladeTip = p.C0-bladeLength*[cos(q(4));sin(q(4))];

    hold on;
    plot([p.C0(1) A0(1) p.B0(1)], ...
         [p.C0(2) A0(2) p.B0(2)],'k-','LineWidth',4);
    plot([A0(1) A(1)],[A0(2) A(2)],'Color',[0.90 0.35 0.10], ...
         'LineWidth',5);
    plot([A(1) B(1)],[A(2) B(2)],'Color',[0.10 0.45 0.85], ...
         'LineWidth',4);
    plot([p.B0(1) B(1)],[p.B0(2) B(2)],'Color',[0.10 0.65 0.35], ...
         'LineWidth',4);
    plot([A(1) C(1)],[A(2) C(2)],'Color',[0.55 0.25 0.75], ...
         'LineWidth',4);
    plot([p.C0(1) C(1)],[p.C0(2) C(2)],'Color',[0.05 0.65 0.70], ...
         'LineWidth',4);
    plot([p.B0(1) rightBladeTip(1)], ...
         [p.B0(2) rightBladeTip(2)],'-','Color',[0.15 0.15 0.15], ...
         'LineWidth',7);
    plot([p.C0(1) leftBladeTip(1)], ...
         [p.C0(2) leftBladeTip(2)],'-','Color',[0.15 0.15 0.15], ...
         'LineWidth',7);

    joints = [A0 A B p.B0 C p.C0];
    scatter(joints(1,:),joints(2,:),75,'w','filled', ...
            'MarkerEdgeColor','k','LineWidth',1.5);

    text(A0(1)-10,A0(2)+13,'A_0','FontWeight','bold');
    text(A(1)+6,A(2)-13,'A','FontWeight','bold');
    text(B(1)+6,B(2)-10,'B','FontWeight','bold');
    text(p.B0(1)+5,p.B0(2)+10,'B_0','FontWeight','bold');
    text(C(1)-15,C(2)-10,'C','FontWeight','bold');
    text(p.C0(1)-18,p.C0(2)+10,'C_0','FontWeight','bold');

    axis equal; axis([-230 230 -170 190]);
    grid on;
    xlabel('x [mm]'); ylabel('y [mm]');
    set(gca,'FontSize',11,'LineWidth',1);
end
