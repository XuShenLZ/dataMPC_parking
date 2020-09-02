classdef bike_dynamics_rk4
    properties
        L_r;
        L_f;
        dt;
        M;
    end
    
    methods
        function self = bike_dynamics_rk4(L_r, L_f, dt, M)
            self.L_r = L_r;
            self.L_f = L_f;
            self.dt = dt;
            self.M = M;
        end
        
        function x_dot = f_ct(self, x, u)
            beta = @(d) atan2(self.L_r*tan(d), self.L_r+self.L_f);
    
            x_dot= [x(4)*cos(x(3)+beta(u(1)));
                x(4)*sin(x(3)+beta(u(1)));
                x(4)*sin(beta(x(3)))/self.L_r;
                u(2)];
        end
        
        function x_kp1 = f_dt(self, x_k, u_k)
            h_k = self.dt/self.M;

            % Runge-Kutta to obtain discrete time dynamics
            x_kp1 = x_k;
            for i = 1:self.M
                a1 = self.f_ct(x_kp1, u_k);
                a2 = self.f_ct(x_kp1+h_k*a1/2, u_k);
                a3 = self.f_ct(x_kp1+h_k*a2/2, u_k);
                a4 = self.f_ct(x_kp1+h_k*a3, u_k);
                x_kp1 = x_kp1 + h_k*(a1 + 2*a2 + 2*a3 + a4)/6;
            end
        end
        
        function x_kp1 = f_dt_aug(self, x_k, u_k)
            x_kp1 = vertcat(self.f_dt(x_k, u_k), u_k);
        end
    end
end