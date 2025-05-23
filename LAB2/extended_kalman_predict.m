function [x,P] = extended_kalman_predict(x,P,F,Gu,Q,Bw,w_mean)

    % Project covariance matrix ahead
    P = F*P*F' + Bw*Q*Bw';
    % Projection over Sn subspace (keep Pk symmetric)
    P = (P+P')/2;

    x = F*x + Gu +Bw*w_mean;
    
end
