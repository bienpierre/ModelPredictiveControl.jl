# Tools

Tools related to MPC are depicted in this section.

## Terminal matrix computation

```
julia> P = terminal_weight(Ad, Bd, Q, R)

```
P is computed thanks to Discrete Algebraic Riccati Equation. Where Ad and Bd are the discrete state and input matrices, Q and R are the weighting matrices with appropriate dimensions.



