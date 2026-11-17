## Reporting the dynamics aspect of the Project

### Setting:
We dipose of a comfort zone temperature interval $I=[17,22]$, and of $N$ rooms where $N\ge3$. In each of those rooms we find a heater which also has its own temperature that is taken into account, but most importantly it has a paramater that represents our input: $u$, its power setting. The temperature of $room_i$ is modeled by $T_i$.


### Goal:
Upon setting the power of all heaters of the rooms, we want to *KEEP* their temperatures withing the comfort zone. It is a "stay inside of" and not a "reach a destination" problem 



### Bounds proof:
Let's verify and understand what does **"this system is monotone means"**:

Let's compute the gradient:

$\frac{\partial T_i^+}{\partial T_i} = 1 - 2\alpha - \beta - \gamma \ u_i(t) \ge 0.001 > 0 \ \ (\text{when: } \ u_i(t) = 0.6)$

$\frac{\partial T_i^+}{\partial T_i\pm1} = \alpha > 0$

$\frac{\partial T_i^+}{\partial T_i} = \gamma(T_h - T_i(t)) > 0 \ \ (\text{since :} T_h > T_i(t))$


***Note:*** This allows us to avoid computing the computation heavy overapproximation, we compute it straight from the high/low bounds of the interval as seen in the implementation.
