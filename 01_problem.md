# TSIDProblem Class Documentation

## Short Description

The `TSIDProblem` class is designed as a specialized convex optimization problem for Task Space Inverse Dynamics (TSID) using the CVXGen solver interfaces. It takes in task and contact definitions, along with system dimensions, to build an optimization problem that computes control inputs and system accelerations which satisfy a set of dynamics, contact, and task constraints.

## Problem Formulation
The general formulation of the Task Space Inverse Dynamics optimization problem looks like the following:
<!-- Beautiful version, not supported by Katex version in GitLab -->
<!-- $$
\begin{align} 
& \min_{\dot{v}, u, f_j} && \sum_{i=1}^{N_t} w_i||J_i(q, v)\dot{v} + \varphi_i(q, v)||_2^2 \\ %
  &&& + \sum_{j=i}^{N_c} w_j||f_j - f_j^*||_2^2 \nonumber \\
  &&& + w_u ||u - u^*|| \nonumber \\
  &&& + w_v ||\dot{v} - \dot{v}^*|| \nonumber \\
& \textrm{s.t.} && c_k ||J_k(q, v) \dot{v} - \varphi_k(q, v)|| = 0 & \forall k = 1 \ldots N_t\\
&&& c_j (f_j - f_j^*) = 0 & \forall j = 1 \ldots N_c\\
&&& f_j \in cone & \forall j = 1 \ldots N_c\\
&&& u_{min} \leq u \leq u_{max} \nonumber \\
&&& \dot{v}_{max} \leq \dot{v} \leq \dot{v}_{max} \nonumber \\
&&& M(q)\dot{v} + h(q, v) = Bu + \sum_{j=1}^{N_c}J_j^T(q)f_j \\
&&& \psi_b(q, v) \dot{v} + \varphi_b(q, v) \geq 0 & \forall b = 1 \ldots N_b\\
\end{align}
$$ -->
$$
\begin{aligned} 
& \min_{\dot{v}, u} && w_t||J_t(q, v)\dot{v} + \varphi_t(q, v)||_2^2 &&&& (1)  \\
& \textrm{s.t.}
&& u_{min} \leq u \leq u_{max}  \\
&&& \dot{v}_{max} \leq \dot{v} \leq \dot{v}_{max}  \\
&&& M(q)\dot{v} + h(q, v) = Bu & (5)\\
&&& \psi_b(q, v) \dot{v} + \varphi_b(q, v) \geq 0 & \forall b = 1 \ldots N_b &&& (6)\\
\end{aligned}
$$

where (1) refers to [tasks](03_tasks.md) in the system, (2)-(3) represents optional hard constraints on the tasks and contacts, (4) refers to [linearized friction cones](04_contacts.md), (5) is system dynamics, and (6) is a [barrier functions](06_barrier.md).  
## How to Use
To use the `TSIDProblem` class, follow these steps:

1. Initialize the class with dictionaries defining tasks and contacts, the number of variables (nv), control inputs (nu), and optionally the number of generalized positions (nq), among other settings.
2. Optionally, set the `build` parameter to `True` to build the problem at initialization. If set to `False`, call the `build()` method manually after initialization.
3. If the `generate` parameter is set to `True`, the solver code will be generated upon initialization. Otherwise, you can call `generate_ccode()` manually.
4. Access the resulting `cp.Problem` instance through the `problem` attribute after building it.

## Data Classes Description
#### Task
Represents a specific [task](03_tasks.md) in the TSIDProblem with attributes for defining its Jacobian, weight, desired outcome, and if it's a hard constraint or not

| Attribute  | Type                     | Description                                                  | Symbol from formulation              |
|------------|--------------------------|--------------------------------------------------------------|--------------------------------------|
| jacobian   | cp.Parameter             | The Jacobian matrix of the task.                             | $J$                                  |
| weight     | cp.Parameter             | The weight of the task in the optimization problem.          | $w$                                  |
| rhs        | cp.Parameter             | The right-hand side of the task's equality constraint.       | $\varphi$                            |
| residual   | cp.Variable              | The slack variable representing the task's residual.         | $r = J(q, v)\dot{v} + \varphi(q, v)$ |
| is_hard    | cp.Parameter (binary)    | A binary indicator specifying if the task constraint is hard.| $c$                                  | 

#### Contact

Defines a contact (point or wrench) in the TSIDProblem with attributes for its activation status, Jacobian, weight, reference force, force constraints, and residual forces.

| Attribute   | Type                | Description                                                     | Symbol from formulation   |
|-------------|---------------------|-----------------------------------------------------------------|---------------------------|
| activation  | cp.Parameter        | Activation status of the contact force.                         | $c$                       |
| jacobian    | cp.Parameter        | The Jacobian matrix of the contact.                             | $J$                       |
| weight      | cp.Parameter        | The weight of the contact in the optimization problem.          | $w$                       |
| reference   | cp.Parameter        | The reference force for the contact.                            | $f^&ast;$                     |
| cone        | cp.Parameter        | The cone constraints for the contact force.                     | $C$                       |
| force       | cp.Variable         | The contact force variable.                                     | $f$                       |
| residual    | cp.Variable         | The slack variable representing the contact's residual force.   | $r = f_j - f_j^&ast;$         |

#### Contact

Defines a contact (point or wrench) in the TSIDProblem with attributes for its activation status, Jacobian, weight, reference force, force constraints, and residual forces.

| Attribute   | Type                | Description                                                     | Symbol from formulation   |
|-------------|---------------------|-----------------------------------------------------------------|---------------------------|
| activation  | cp.Parameter        | Activation status of the contact force.                         | $c$                       |
| jacobian    | cp.Parameter        | The Jacobian matrix of the contact.                             | $J$                       |
| weight      | cp.Parameter        | The weight of the contact in the optimization problem.          | $w$                       |
| reference   | cp.Parameter        | The reference force for the contact.                            | $f^&ast;$                     |
| cone        | cp.Parameter        | The cone constraints for the contact force.                     | $C$                       |
| force       | cp.Variable         | The contact force variable.                                     | $f$                       |
| residual    | cp.Variable         | The slack variable representing the contact's residual force.   | $r = f_j - f_j^&ast;$         |

#### Limits

Specifies the bounds for variables within the TSIDProblem, such as joint limits or actuator saturation limits, with attributes for the minimum and maximum values.

**class Limit**
| Attribute | Type            | Description                              |
|-----------|-----------------|------------------------------------------|
| min       | cp.Parameter    | The minimum bound for the limit.         | 
| max       | cp.Parameter    | The maximum bound for the limit.         |

**class Limits**
| Attribute     | Type            | Description                              | Symbol from formulation |
|---------------|-----------------|------------------------------------------|-------------------------|
| acceleration  | Limit           | Limits on joint positions.               | $a&#95;{min}$, $a&#95;{max}$    |
| control       | Limit           | The maximum bound for the limit.         | $u&#95;{min}$, $u&#95;{max}$    |


### OptiVariable
Stores information about an optimization variable (forces, accelerations, ) within an optimization problem.

| Attribute     | Type            | Description                                 | Symbol from formulation                                                 |
|---------------|-----------------|---------------------------------------------|-------------------------------------------------------------------------|
| variable  | cp.Variable           | Optimization variable.                    | $\dot{v}$, $u$, $f_j$                                                   |
| weight    | cp.Parameter            | The weight in the variable error task.  | $w_v$, $w_u$, $w_j$                                                     |
| residual  | cp.Variable           | The error of the variable.                |  ${ \|\|\dot{v} - \dot{v}^&ast;\|\|_2^2 }$, ${ \|\|u - u^&ast;\|\|_2^2 }$, ${\|\|f_j - f_j^&ast;\|\|_2^2 }$
| reference | cp.Parameter           | Reference for the optimization variable. | $\dot{v}^&ast;$, $u^&ast;$, $f_j^&ast;$                                             |
| limits    | Limit           | the lower and upper bounds for the variables.   | $\dot{v}&#95;{min/max}$, $u&#95;{min/max}$                                      |

<!-- | residual  | cp.Variable           | The error of the variable.                | $||\dot{v} - \dot{v}^*||_2^2$, $||u - u^*||_2^2$, $||f_j - f_j^*||_2^2$ | -->

#### Dynamics

Describes the dynamics of the system within the TSIDProblem, including variables for control inputs, system accelerations, and parameters for inertia, biases, and input mappings.

| Attribute       | Type            | Description                                              | Symbol from formulation   |
|-----------------|-----------------|----------------------------------------------------------|---------------------------|
| control         | cp.Variable     | The control input variable.                              | $u$                       |
| acceleration    | cp.Variable     | The system acceleration variable.                        | $\dot{v}$                 |
| inertia_matrix  | cp.Parameter    | The inertia matrix of the system.                        | $M(q)$                    |
| bias_force      | cp.Parameter    | The bias forces (e.g., Coriolis, gravity) of the system. | $h(q, v)$                 |
| selector        | cp.Parameter    | A selector matrix to map control inputs to forces.       | $B$                       |

### Weights

Captures the importance of different components in the optimization cost function, including control inputs, accelerations, and specific task and contact weights.

| Attribute       | Type                    | Description                                           | Symbol from formulation   |
|-----------------|-------------------------|-------------------------------------------------------|---------------------------|
| control         | cp.Parameter            | The weight of the control input in the cost function. | $w_u$                     |
| acceleration    | cp.Parameter            | The weight of the acceleration in the cost function.  | $w_v$                     |
| task            | Dict[str, cp.Parameter] | A dictionary of weights for each task.                | $w_i$                     |
| contact         | Dict[str, cp.Parameter] | A dictionary of weights for each contact.             | $w_j$                     |

#### Barrier

Specifies barrier constraint, aplied to the TSIDProblem.


| Attribute | Type            | Description                              | Symbol from formulation |
|-----------|-----------------|------------------------------------------|-------------------------|
| jacobian  | cp.Parameter    | Jacobian of the barrier.                 | $\psi_b$.               |
| bias.     | cp.Parameter    | Bias of the barrier.                     | $\varphi_b$             |                          

### Example

```python
from typing import Dict

# Define tasks and contacts with their dimensions
task_dict = {'balance': 3, 'posture': 4}
contact_dict = {'foot': 6, 'hand': 3}

# Create a TSID problem instance
tsid_problem = TSIDProblem(
    task_dict=task_dict,
    contact_dict=contact_dict,
    nv=10,
    nu=4,
    nq=10,
    build=True,
    generate=True
)

# Access the built CVXPY problem
cvxpy_problem = tsid_problem.problem

# Solve the problem
cvxpy_problem.solve()
```
