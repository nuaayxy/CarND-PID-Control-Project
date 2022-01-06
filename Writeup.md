# CarND-Controls-PID Writeup
PID control project reflection

---

## reflection
1. P component represents steering the car in a proportional factor of the cross track error, this alone can overshoot
2. D component  the difference between the current CTE and previous CTE to prevent oscillations or overshooting 
3. I component is useful to comenstate for biases 
4. Here is a video of the implemenation running the car with the PID control


https://user-images.githubusercontent.com/8016115/148288345-71ebca6b-eb26-4f28-a07b-1019f44884c6.mp4



## Disscussion

1. manually selected starting initial PID parameters after trial and error
2. twiddle is implemented in a seperate thread that takes in CTE error and adjust the parameters for optimal error minimization
3. car is able to drive within safe area for loops
4. Exponential smoothing to the steering is used for a more smooth and comfortable ride, although may add a bit latency but its a trade off 
5. it seems it is important to select a good starting parameters, otherwise the parameters optimiaztion might stuck in some local minimal. 
6. More advanced algortihm like Genetic ALgorithm which spawns a large population of parameters and use mutation and breeding algorithm to find the optimal solution to prevent local minimal

