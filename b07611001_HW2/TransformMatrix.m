classdef TransformMatrix
    %TransformMatrix Summary of this class goes here
    %   Detailed explanation goes here    
    properties
        mat
        syms
    end
    
    methods
        function obj = TransformMatrix(alpha_i_neg_one,a_i_neg_one,d_i,theta_i, flag)
            if (flag)
                obj.mat = [cosd(theta_i), -sind(theta_i), 0, a_i_neg_one;
                        sind(theta_i)*cosd(alpha_i_neg_one), cosd(theta_i)*cosd(alpha_i_neg_one), -sind(alpha_i_neg_one), -sind(alpha_i_neg_one)*d_i;
                        sind(theta_i)*sind(alpha_i_neg_one), cosd(theta_i)*sind(alpha_i_neg_one), cosd(alpha_i_neg_one), cosd(alpha_i_neg_one)*d_i;
                                0, 0, 0, 1];
            else
                obj.syms = [cos(sym(theta_i)), -sin(sym(theta_i)), sym(0), sym(a_i_neg_one);
                        sin(sym(theta_i))*cos(sym(alpha_i_neg_one)), cos(sym(theta_i))*cos(sym(alpha_i_neg_one)), -sin(sym(alpha_i_neg_one)), -sin(sym(alpha_i_neg_one))*sym(d_i);
                        sin(sym(theta_i))*sin(sym(alpha_i_neg_one)), cos(sym(theta_i))*sin(sym(alpha_i_neg_one)), cos(sym(alpha_i_neg_one)), cos(sym(alpha_i_neg_one))*sym(d_i);
                                sym(0), sym(0), sym(0), sym(1)];
            end
        end
        
        function outputArg = inverse(obj)
            outputArg = inv(obj.syms);
        end

    end
end




