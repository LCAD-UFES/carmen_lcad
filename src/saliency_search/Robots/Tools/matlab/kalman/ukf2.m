function [x,P,muPredicted,covPredicted]=ukf2(fstate,mu,P,hmeas,z,Q,R)
% UKF   Unscented Kalman Filter for nonlinear dynamic systems
% [x, P] = ukf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system (for simplicity, noises are assumed as additive):
%           x_k+1 = f(x_k) + w_k
%           z_k   = h(x_k) + v_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   f: function handle for f(x)
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%           h: fanction handle for h(x)
%           z: current measurement
%           Q: process noise covariance 
%           R: measurement noise covariance
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance
%


numStates = numel(mu);
numObservations = numel(z);

%Predict
%Scaling factor which determin how far sigma points a spread from the mean
k=0;                 %Scalling factor
alpha = 1e-3;        %Scalling factor   

beta=2;               %used to incude high order information about the distrbution

lambda = alpha^2*(numStates+k)-numStates; 
gamma = sqrt(numStates + lambda);

%Calculate the weights for the mean and coveriance ahead of time
muWeight = zeros(1,1+numStates*2);
covWeight = zeros(1,1+numStates*2);

%Initial weight for the mean
muWeight(1) = lambda/(numStates + lambda);
covWeight(1) = (lambda/(numStates + lambda)) + (1-alpha^2+beta);

weight = 1/(2*(numStates+lambda));
for i=2:1+numStates*2
  muWeight(i) = weight;
  covWeight(i) = weight;
end

%Calculate sigma points for simulation
A = gamma*chol(P)';  %More stable
%A = gamma*sqrt(P);
mu1 = mu;
mu2 = mu(:,ones(1,numStates)) + A;
mu3 = mu(:,ones(1,numStates)) - A;

X=[mu1 mu2 mu3];
%Predict the new state by simulating the process at the sigma points
%and then computing a new mean by a weigted sum
Xnew=zeros(size(X));
muPredicted = zeros(numStates,1);
for k=1:size(X,2)
  Xnew(:,k) = fstate(X(:,k));
  muPredicted = muPredicted + muWeight(k)*Xnew(:,k);
end

covPredicted = zeros(size(P));
%Compute the predicted covariance in a similar manner
for k=1:size(X,2)
  variance = (Xnew(:,k)-muPredicted)*(Xnew(:,k)-muPredicted)';
  covPredicted = covPredicted + covWeight(k)*variance;
end
covPredicted = covPredicted + R;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Predict the measurement by simulating the process at the sigma points
%and then computing a new mean by a weighted sum
Z1=zeros(numObservations, size(Xnew,2));
zPredicted = zeros(numObservations,1);
for k=1:size(X,2)
  Z1(:,k) = hmeas(Xnew(:,k));
  zPredicted = zPredicted + muWeight(k)*Z1(:,k);
end

zcovPredicted = zeros(size(Q));
%Compute the predicted covariance in a similar manner
for k=1:size(X,2)
  variance = (Z1(:,k)-zPredicted)*(Z1(:,k)-zPredicted)';
  zcovPredicted = zcovPredicted + covWeight(k)*variance;
end

zcovPredicted = zcovPredicted + Q;


P12 = zeros(numStates,numObservations);
for k=1:size(X,2)
  variance = (Xnew(:,k)-muPredicted)*(Z1(:,k)-zPredicted)';
  P12 = P12 + covWeight(k)*variance;
end

K=P12*inv(zcovPredicted);
x=muPredicted +K*(z-zPredicted);           %state update
P=covPredicted-K*P12';                     %covariance update

