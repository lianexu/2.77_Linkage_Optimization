function [] = build4BarElevatedKinematics()
    B = casadi.SX.sym('B');
    C = casadi.SX.sym('C');
    D = casadi.SX.sym('D');
    E = casadi.SX.sym('E');
    th = casadi.SX.sym('th');
    alpha = casadi.SX.sym('alpha');
    par = casadi.SX.sym('p', 4, 1);
    m = par(1);
    g = par(2);
    I = par(3);
    J = par(4);

    % Set up geometry
    i_hat = [1; 0; 0];
    j_hat = [0; 1; 0];
    % k_hat = cross(i_hat, j_hat);


    % r_e = [0; 0; 0];
    r_a = E * j_hat;
    r_d = D * i_hat;
    r_c = r_d + C*(cos(th)*i_hat + sin(th)*j_hat);
    r_b = r_c + B*(-cos(alpha-th)*i_hat + sin(alpha-th)*j_hat);
    
    r_dc = r_c-r_d;
    r_cb = r_b-r_c; 
    r_ba = r_a-r_b; 
    r_db = r_b-r_d;
    r = [r_dc, r_cb, r_ba, r_db, r_b];

    A = norm(r_ba);
       
    % Calulate external torque from gravity (at the hinge)
    F_g = m*g;  
    Phi = atan(I/J); %
    T_ext = sqrt(I^2+J^2)*F_g*sin(Phi-th); 
    
    % Obtain direction of force of actuation
    u_ba = r_ba / (norm(r_ba) + 1e-9); % Unit vector from b to a

    % Calculate the effective torque arm magnitude using the cross product.
    torque_arm = norm(cross(r_db, u_ba));
    
    % Calculate the force magnitude required to balance the torque from gravity.
    % Add epsilon to the denominator to prevent division by zero.
    F_act = T_ext / (torque_arm + 1e-9);

    r_fn = casadi.Function('r_fn', {B, C, D, E, th, alpha}, {r});
    r_fn.save('codegen/r_fn.casadi');

    A_fn = casadi.Function('A_fn', {B, C, D, E, th, alpha}, {A});
    A_fn.save('codegen/A_fn.casadi');

    F_act_fn = casadi.Function('F_act_fn', {B, C, D, E, th, alpha, par}, {F_act});
    F_act_fn.save('codegen/F_act_fn.casadi');

    T_ext_fn = casadi.Function('T_ext_fn', {par, th}, {T_ext});
    T_ext_fn.save('codegen/T_ext_fn.casadi');
end