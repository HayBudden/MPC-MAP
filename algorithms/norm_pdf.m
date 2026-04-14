function y = norm_pdf(x, mu, sigma)
%NORM_PDF  Gaussian probability density function.
y = (1 ./ (sigma * sqrt(2*pi))) .* exp(-(x - mu).^2 ./ (2 * sigma.^2));
end
