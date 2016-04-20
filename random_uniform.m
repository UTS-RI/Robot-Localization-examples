function [r] = random_uniform(a,b)
	r = a + (b-a).*rand(1,1);
end