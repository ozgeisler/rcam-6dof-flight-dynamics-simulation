function [FVAL] = RCAM_model_implicit(XDOT,X,U)

FVAL= RCAM_Model_D(X,U) - XDOT;
end
