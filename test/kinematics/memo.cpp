cout << endl;
cout << L1 << ", " << powL1 << endl;
cout << L2 << ", " << powL2 << endl;
cout << px << ", " << pow_px << endl;
cout << py << ", " << pow_py << endl;
cout << pz << ", " << pow_pz << endl;
cout << z_corr << ", " << pow_z_corr << endl;
cout << c << ", " << pow_c << endl;

cout << endl;

cout <<  "D1: " << D1 << ", " << "D2: " << D2 << endl;
double t = pow_c + powL1 - powL2;
double t1 = 2*c*L1;
cout << t << endl;
cout << t1 << endl;
cout << t / t1 << endl;

//cout << cos(1.047) << endl; // 1.047 = 60'
//cout << acos(0.5) << endl;  // cos60(1.047) = 1/2(0.5)
cout << endl;

cout << cos(D2) << endl;
cout << acos(0.520752) << endl;

cout << endl;
