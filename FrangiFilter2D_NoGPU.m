function [outIm,whatScale,Direction] = FrangiFilter2D_NoGPU(I, options)
% This function FRANGIFILTER2D uses the eigenvectors of the Hessian to
% compute the likeliness of an image region to vessels, according
% to the method described by Frangi:2001 (Chapter 2).
%
% [J,Scale,Direction] = FrangiFilter2D(I, Options)
%
% inputs,
%   I : The input image (vessel image)
%   Options : Struct with input options,
%       .FrangiScaleRange : The range of sigmas used, default [1 8]
%       .FrangiScaleRatio : Step size between sigmas, default 2
%       .FrangiBetaOne : Frangi correction constant, default 0.5
%       .FrangiBetaTwo : Frangi correction constant, default 15
%       .BlackWhite : Detect black ridges (default) set to true, for
%                       white ridges set to false.
%       .verbose : Show debug information, default true
%
% outputs,
%   J : The vessel enhanced image (pixel is the maximum found in all scales)
%   Scale : Matrix with the scales on which the maximum intensity 
%           of every pixel is found
%   Direction : Matrix with directions (angles) of pixels (from minor eigenvector)   
%
% Example,
%   I=double(imread ('vessel.png'));
%   Ivessel=FrangiFilter2D(I);
%   figure,
%   subplot(1,2,1), imshow(I,[]);
%   subplot(1,2,2), imshow(Ivessel,[0 0.25]);
%
% Written by Marc Schrijver, 2/11/2001
% Re-Written by D.Kroon University of Twente (May 2009)
%tic
defaultoptions = struct('FrangiScaleRange', [2 4], 'FrangiScaleRatio', 1, 'FrangiBetaOne', 1, 'FrangiBetaTwo', 2, 'verbose',true,'BlackWhite',true);

% Process inputs
if(~exist('options','var')) 
    options=defaultoptions; 
else
    tags = fieldnames(defaultoptions);
    for i=1:length(tags)
         if(~isfield(options,tags{i})),  options.(tags{i})=defaultoptions.(tags{i}); end
    end
    if(length(tags)~=length(fieldnames(options))) 
        warning('FrangiFilter2D:unknownoption','unknown options found');
    end
end
%tCom = toc;
%disp(['Computational Time is: ' num2str(tCom)]);
%%
%tic
sigmas = options.FrangiScaleRange(1):options.FrangiScaleRatio:options.FrangiScaleRange(2);
%sigmas = gpuArray(sigmas);
sigmas = sort(sigmas, 'ascend');

beta  = 2*options.FrangiBetaOne^2;
c     = 2*options.FrangiBetaTwo^2;
%tCom = toc;
%disp(['Computational Time is: ' num2str(tCom)]);
%%
% Make matrices to store all filterd images
%tic
%I = gpuArray(I);

%ALLfiltered=zeros([size(I) length(sigmas)], 'gpuArray');
%ALLangles=zeros([size(I) length(sigmas)], 'gpuArray');

ALLfiltered=zeros([size(I) length(sigmas)]);
ALLangles=zeros([size(I) length(sigmas)]);
%ALLfiltered=zeros([size(I) length(sigmas)]);
%ALLangles=zeros([size(I) length(sigmas)]);

%
% Frangi filter for all sigmas
for i = 1:length(sigmas)
    % Show progress
    %{
    if(options.verbose)
        disp(['Current Frangi Filter Sigma: ' num2str(sigmas(i)) ]);
    end
    %}
    
    %tic
    % Make 2D hessian
    [Dxx,Dxy,Dyy] = Hessian2D_NoGPU(I,sigmas(i));
    %tCom = toc; disp(['Computational Time of Hessian is: ' num2str(tCom)]);
    
    % Correct for scale
    Dxx = (sigmas(i)^2)*Dxx;
    Dxy = (sigmas(i)^2)*Dxy;
    Dyy = (sigmas(i)^2)*Dyy;
    
    %tic
    % Calculate (abs sorted) eigenvalues and vectors
    [Lambda2,Lambda1,Ix,Iy]=eig2image(Dxx,Dxy,Dyy);
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]);
    
    %tic
    % Compute the direction of the minor eigenvector
    angles = atan2(Ix,Iy);

    % Compute some similarity measures
    Lambda1(Lambda1==0) = eps;
    Rb = (Lambda2./Lambda1).^2;
    S2 = Lambda1.^2 + Lambda2.^2;
    %Rb = gpuArray(Rb); S2 = gpuArray(S2);
    
    % Compute the output image
    Ifiltered = exp(-Rb/beta) .*(ones(size(I))-exp(-S2/c));
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]);
    
    % see pp. 45
    if(options.BlackWhite)
        Ifiltered(Lambda1<0)=0;
    else
        Ifiltered(Lambda1>0)=0;
    end
    % store the results in 3D matrices
    ALLfiltered(:,:,i) = Ifiltered;
    ALLangles(:,:,i) = angles;
end
%tCom = toc;
%disp(['Computational Time of Loop is: ' num2str(tCom)]);

% Return for every pixel the value of the scale(sigma) with the maximum 
% output pixel value
if length(sigmas) > 1
    [outIm,whatScale] = max(ALLfiltered,[],3);
    outIm = reshape(outIm,size(I));
    if(nargout>1)
        whatScale = reshape(whatScale,size(I));
    end
    if(nargout>2)
        Direction = reshape(ALLangles((1:numel(I))'+(whatScale(:)-1)*numel(I)),size(I));
    end
else
    outIm = reshape(ALLfiltered,size(I));
    if(nargout>1)
            whatScale = ones(size(I));
    end
    if(nargout>2)
        Direction = reshape(ALLangles,size(I));
    end
end
%tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]);
%%
