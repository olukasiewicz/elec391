step_pwm = readtable("tuned.csv");
figure;
plot(step_pwm.timestamp, step_pwm.target, step_pwm.timestamp, step_pwm.current);
title("Open Loop Step Response");
xlabel('Time (ms)') 
ylabel('Position') 