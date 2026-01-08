clc;
Lf = 1e-3; Cf = 22e-6; Rf = 35e-3;
fsw = 20e3; Ts = 1/(2*fsw);

Vdc = 700; f = 50; w = 2*pi*f;

A = [-Rf/Lf  0     -1/Lf   0    ;
      0     -Rf/Lf  0     -1/Lf ;
      1/Cf   0      0      0    ;
      0      1/Cf   0      0   ];

B = [1/Lf   0    ;
      0     1/Lf ;
      0     0    ;
      0     0   ];

P = [ 0     0    ;
      0     0    ;
     -1/Cf  0    ;
      0    -1/Cf];

nx = size(B,1); nu = size(B,2);
C = eye(nx); D = zeros(nx,nu);

ss_VSI = ss(A,B,C,D); [Ad,Bd,~,~] = ssdata(c2d(ss_VSI,Ts,'zoh'));

currentFolder = pwd;
C_Folder = extractBefore(currentFolder,'MATLAB');
C_folder = append(C_Folder,'PLECS/');
cd(C_folder)
save System_Vars.mat Lf Cf Rf Vdc
cd(currentFolder)