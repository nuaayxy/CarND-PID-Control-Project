# CarND-Controls-PID Writeup
PID control project reflection

---

## reflection
1. P component represents steering the car in a proportional factor of the cross track error, this alone can overshoot
2. D component  the difference between the current CTE and previous CTE to prevent oscillations or overshooting 
3. I component is useful to comenstate for biases 
4. Here is a video of the implemenation running the car with the PID control


## Disscussion

1. manually selected initial PID parameters after trial and error
2. car is able to drive within safe area for loops
3. Exponential smoothing to the steering is used for a more smooth and comfortable ride, although may add a bit latency but its a trade off 

