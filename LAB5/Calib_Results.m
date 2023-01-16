% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 617.004690469377351 ; 674.441469709880607 ];

%-- Principal point:
cc = [ 319.500000000000000 ; 239.500000000000000 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.266949697690537 ; 0.172341841564588 ; -0.006911774077804 ; 0.008194511618120 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.879021823465337 ; 2.206942051407609 ];

%-- Principal point uncertainty:
cc_error = [ 0.000000000000000 ; 0.000000000000000 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013976351740430 ; 0.050271669305430 ; 0.000777677593684 ; 0.000649273656121 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 0;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.657154e+00 ; 1.625711e+00 ; -6.498770e-01 ];
Tc_1  = [ -7.955426e+01 ; -2.908230e+01 ; 3.153572e+02 ];
omc_error_1 = [ 2.488669e-03 ; 2.837041e-03 ; 4.190845e-03 ];
Tc_error_1  = [ 1.304153e-01 ; 1.999704e-01 ; 9.814309e-01 ];

%-- Image #2:
omc_2 = [ 1.844585e+00 ; 1.877005e+00 ; -3.857563e-01 ];
Tc_2  = [ -6.985192e+01 ; -5.681964e+01 ; 2.816891e+02 ];
omc_error_2 = [ 3.116552e-03 ; 3.545154e-03 ; 6.004128e-03 ];
Tc_error_2  = [ 1.175738e-01 ; 1.737455e-01 ; 9.596576e-01 ];

%-- Image #3:
omc_3 = [ 1.749290e+00 ; 2.068077e+00 ; -5.302086e-01 ];
Tc_3  = [ -5.877228e+01 ; -6.256088e+01 ; 2.916241e+02 ];
omc_error_3 = [ 3.119466e-03 ; 3.704530e-03 ; 6.353302e-03 ];
Tc_error_3  = [ 1.372571e-01 ; 1.888585e-01 ; 9.197511e-01 ];

%-- Image #4:
omc_4 = [ 1.811516e+00 ; 2.102665e+00 ; -1.102862e+00 ];
Tc_4  = [ -3.418509e+01 ; -5.491545e+01 ; 2.934678e+02 ];
omc_error_4 = [ 2.827428e-03 ; 2.556682e-03 ; 4.244539e-03 ];
Tc_error_4  = [ 1.746760e-01 ; 1.830215e-01 ; 7.465433e-01 ];

%-- Image #5:
omc_5 = [ 1.118672e+00 ; 1.939221e+00 ; -2.819907e-01 ];
Tc_5  = [ -4.663473e+01 ; -8.547997e+01 ; 2.835153e+02 ];
omc_error_5 = [ 1.757721e-03 ; 2.597136e-03 ; 3.657843e-03 ];
Tc_error_5  = [ 1.618244e-01 ; 1.504367e-01 ; 9.128203e-01 ];

%-- Image #6:
omc_6 = [ -1.699476e+00 ; -1.957444e+00 ; -8.460679e-01 ];
Tc_6  = [ -6.367932e+01 ; -2.826837e+01 ; 1.623834e+02 ];
omc_error_6 = [ 2.205264e-03 ; 2.435242e-03 ; 3.601937e-03 ];
Tc_error_6  = [ 1.470667e-01 ; 1.387050e-01 ; 7.556950e-01 ];

%-- Image #7:
omc_7 = [ 1.952532e+00 ; 1.946002e+00 ; 1.321716e+00 ];
Tc_7  = [ -3.769668e+01 ; -2.772285e+01 ; 1.632590e+02 ];
omc_error_7 = [ 2.503271e-03 ; 2.398678e-03 ; 3.568727e-03 ];
Tc_error_7  = [ 1.808542e-01 ; 1.464294e-01 ; 8.544880e-01 ];

%-- Image #8:
omc_8 = [ 1.920504e+00 ; 1.836542e+00 ; 1.331619e+00 ];
Tc_8  = [ -7.326757e+01 ; -3.704622e+01 ; 1.710554e+02 ];
omc_error_8 = [ 2.822928e-03 ; 2.759524e-03 ; 4.113559e-03 ];
Tc_error_8  = [ 2.718394e-01 ; 1.678255e-01 ; 9.424807e-01 ];

%-- Image #9:
omc_9 = [ -1.416903e+00 ; -2.055529e+00 ; 3.538206e-01 ];
Tc_9  = [ -7.984101e+00 ; -8.367498e+01 ; 2.863935e+02 ];
omc_error_9 = [ 2.776574e-03 ; 4.369748e-03 ; 5.727800e-03 ];
Tc_error_9  = [ 2.165717e-01 ; 1.011871e-01 ; 9.807828e-01 ];

%-- Image #10:
omc_10 = [ -1.576418e+00 ; -2.168012e+00 ; 2.247761e-01 ];
Tc_10  = [ -2.104641e+01 ; -1.112265e+02 ; 3.346609e+02 ];
omc_error_10 = [ 4.768309e-03 ; 6.563905e-03 ; 9.842847e-03 ];
Tc_error_10  = [ 2.505354e-01 ; 1.418673e-01 ; 1.299626e+00 ];

%-- Image #11:
omc_11 = [ -1.811360e+00 ; -2.092086e+00 ; -5.535817e-01 ];
Tc_11  = [ -6.772217e+01 ; -8.447407e+01 ; 2.605833e+02 ];
omc_error_11 = [ 4.504113e-03 ; 4.911134e-03 ; 8.834759e-03 ];
Tc_error_11  = [ 1.955090e-01 ; 3.078553e-01 ; 1.277990e+00 ];

%-- Image #12:
omc_12 = [ -1.849996e+00 ; -2.112550e+00 ; -5.837078e-01 ];
Tc_12  = [ -5.948772e+01 ; -6.350982e+01 ; 2.226450e+02 ];
omc_error_12 = [ 3.642281e-03 ; 4.086094e-03 ; 7.105606e-03 ];
Tc_error_12  = [ 1.777606e-01 ; 2.270799e-01 ; 1.057756e+00 ];

%-- Image #13:
omc_13 = [ -1.921094e+00 ; -2.138076e+00 ; -6.548298e-01 ];
Tc_13  = [ -5.841803e+01 ; -5.134550e+01 ; 1.997412e+02 ];
omc_error_13 = [ 3.386886e-03 ; 3.653598e-03 ; 6.273488e-03 ];
Tc_error_13  = [ 1.776098e-01 ; 1.880359e-01 ; 9.478729e-01 ];

%-- Image #14:
omc_14 = [ -1.955667e+00 ; -2.145071e+00 ; -6.377591e-01 ];
Tc_14  = [ -5.434931e+01 ; -4.918231e+01 ; 1.802270e+02 ];
omc_error_14 = [ 3.140473e-03 ; 3.401693e-03 ; 5.777398e-03 ];
Tc_error_14  = [ 1.591508e-01 ; 1.674400e-01 ; 8.450664e-01 ];

%-- Image #15:
omc_15 = [ 2.098951e+00 ; 2.255502e+00 ; 5.493156e-01 ];
Tc_15  = [ -8.420255e+01 ; -4.827241e+01 ; 1.724361e+02 ];
omc_error_15 = [ 3.605489e-03 ; 3.850770e-03 ; 7.386302e-03 ];
Tc_error_15  = [ 2.029505e-01 ; 1.394844e-01 ; 8.700904e-01 ];

%-- Image #16:
omc_16 = [ 1.908366e+00 ; 2.346842e+00 ; -1.441323e-01 ];
Tc_16  = [ -1.481891e+01 ; -6.196715e+01 ; 2.614315e+02 ];
omc_error_16 = [ 5.354316e-03 ; 5.550366e-03 ; 1.126790e-02 ];
Tc_error_16  = [ 1.330469e-01 ; 1.078991e-01 ; 1.138398e+00 ];

%-- Image #17:
omc_17 = [ -1.641714e+00 ; -1.994044e+00 ; -4.037069e-01 ];
Tc_17  = [ -5.952935e+01 ; -5.047286e+01 ; 1.828117e+02 ];
omc_error_17 = [ 2.052406e-03 ; 2.784948e-03 ; 4.519384e-03 ];
Tc_error_17  = [ 1.151349e-01 ; 1.436557e-01 ; 7.476286e-01 ];

%-- Image #18:
omc_18 = [ -1.378539e+00 ; -1.718541e+00 ; -3.369236e-01 ];
Tc_18  = [ -8.028478e+01 ; -5.770489e+01 ; 1.676174e+02 ];
omc_error_18 = [ 1.354141e-03 ; 1.847082e-03 ; 2.625660e-03 ];
Tc_error_18  = [ 9.503510e-02 ; 1.211060e-01 ; 6.426769e-01 ];

%-- Image #19:
omc_19 = [ -1.894372e+00 ; -1.874928e+00 ; -1.463735e+00 ];
Tc_19  = [ -4.617387e+01 ; -2.844051e+01 ; 1.234094e+02 ];
omc_error_19 = [ 2.328514e-03 ; 1.792556e-03 ; 2.849289e-03 ];
Tc_error_19  = [ 1.586661e-01 ; 1.314882e-01 ; 6.966543e-01 ];

%-- Image #20:
omc_20 = [ 1.853509e+00 ; 1.610999e+00 ; 1.464989e+00 ];
Tc_20  = [ -6.263796e+01 ; -3.178882e+01 ; 1.484773e+02 ];
omc_error_20 = [ 2.387796e-03 ; 2.317065e-03 ; 3.177036e-03 ];
Tc_error_20  = [ 2.409234e-01 ; 1.468443e-01 ; 8.550425e-01 ];

