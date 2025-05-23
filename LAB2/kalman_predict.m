function [x,P] = kalman_predict(x,P,F,Gu,Q)

    % Matriz Variância
    P = F*P*F' + Q;
    % Projection over Sn subspace (keep Pk symmetric)
    P = (P+P')/2;
    
    % State prediction
    x = F*x + Gu;

end