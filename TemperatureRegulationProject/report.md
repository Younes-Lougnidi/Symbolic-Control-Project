## Reporting the dynamics aspect of the Project

### Setting:
We dipose of a comfort zone temperature interval $I=[17,22]$, and of $N$ rooms where $N\ge3$. In each of those rooms we find a heater which also has its own temperature; a constant $T_h = 50$ that is taken into account, but most importantly it has a parameter that represents our input: $u$, its power setting. The temperature of $room_i$ is modeled by $T_i$.

$*add the constants please* *also the values of u (3) and the 10 cells of each room*$

The dynamics/interactions of/between the rooms in this project is modeled using the following function:

Given a room $T_i$ at a new state $T_i(t+1)$:

$*You write it here*$

It involves the two neighbors of the room $T_i$: $T_{i+1}$ and $T_{i-1}$ alongside the constants invoked above.



### Goal:
Upon setting the power $u$ of all heaters of the rooms, we want to *KEEP* their temperatures within the comfort zone. It is a "stay inside of" and not a "reach a destination" problem. 

To do so we will first establish the dynamics logic, which serves as the building block of the project's solution:

## dynamics.hpp:
In this file we set the all the constants that we are going to use, as well as two important methods:

#### The function $f$ (stated above):
This method takes as input, all the $N$ rooms temperatures as well as the power of the heater in each one of them, and it returns the new state of all the rooms via the dynamics equation given by the project's description.

#### reachable_bounds:
This method takes all the lower bounds and the upper bounds of all temperatures in each room, as well as each room's heater power (vector u) and returns the next state's reachable bound. Since **the system is monotone**, all we need to do, is compute the image of each bound by $f$ to know the next reachable state of each room. Down below is a proof to understand, why can we do that:

##### Bounds proof:
Let's verify and understand what does **"this system is monotone means"**:

Let's compute the gradient:

$\frac{\partial T_i^+}{\partial T_i} = 1 - 2\alpha - \beta - \gamma \ u_i(t) \ge 0.001 > 0 \ \ (\text{when: } \ u_i(t) = 0.6)$

$\frac{\partial T_i^+}{\partial T_{i\pm1}} = \alpha > 0$

$\frac{\partial T_i^+}{\partial u_i} = \gamma(T_h - T_i(t)) > 0 \ \ (\text{since :} T_h > T_i(t))$

For any other rooms not adjacent to $T_i$ the derivative with respect to them is simply 0.


***Note:*** This allows us to avoid computing the computation's heavy overapproximation (like in the ROBOT project), we compute it straight from the high/low bounds of the interval as seen in the implementation.


## coord2state.hpp:
We will adopt a simple example ($N = 3$) rooms to build up our understanding and then it is easy to generalize for N rooms:

**Input:** 
- A vector I containing $3$ indices: The three indices stem from $3$ temperature values (one for each room) $T_1 \ T_2  \ T_3$. Those 3 indices represent the respective bins in which, the each temperature is contained. (After discretization)

- A vector p_T which contains the base multipliers in our case $1$, $10$, $10^2$.

**Output:** A unique cell ID 1-indexed, from 1 to 100.

**Analogy/Motivation:** 

We will think of it as units, tens and hundreds in base10. We get (3,8,9) and we return the following number $3\times 10^0 + 8\times 10^1 + 9\times 10^2 = 983$

But how does this relate to our problem:

Room $k$ has a cell index $I[k]$ from 1 to 10, we subtract 1 to make it 0 indexed, we denote this one $a_k$ it takes values from 0 to 9.

So if we only had one room, we would only have 10 cells to deal with, we get a temperature $T$ we know which bin it is located in from 1 to 10 and that's it.

Now if we had two rooms we would have a 10x10 grid, think about is like the $\mathbb{R^2} plane in which each room's temperature's bin is a coord, the x axis is for Room1 and for Room2 is the y-axis and so, for 1-unit increase in y we need to skip a line which is 10 cells...

We could now extend this with adding Room3, yielding a cube 10x10x10.

So we now can consider that, Room 1 is the units digit (weight 1), room 2 is the tens (weight 10), room 3 is the hundreds (weight 100), and so on.
So the weight of room k is $p_T[k−1] = 10^{k−1}$. These are the multipliers .

The number built from the digits is $\sum a_k  p_T[k−1]$. It runs from $0$ to $10^N − 1$. We add $1$ so the IDs run from $1$ to $10^N$:

$ID = 1 + \sum (I[k-1]) · p_T[k−1]$

**NOTE:** For inputs $u$, it is a similar approach. However, the base is 3, and so multipliers for $N=3$ will be of the form {1,3,9}.

## state2coord.hpp:
This is just the inverse, going from an ID formed in the base, so the indices that formed it.