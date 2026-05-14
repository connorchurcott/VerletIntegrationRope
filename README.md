# **Verlet Integration Rope Simulation**
A Unity implementation of a Verlet integration rope simulation

# **What I Implemented**

- Verlet integration to update particle positions each timestep using current and previous positions
- Iterative distance-constraint satisfaction to maintain segment lengths, with configurable iteration count for stiffness control
- Anchor re-pinning inside the constraint loop to prevent drift at the fixed end
- Mass-gradient weighting for asymmetric constraint correction, giving the rope a physically convincing heavy-end feel
- Ground-plane collision with positional clamping and prev-position correction to avoid artificial bounce

# **Context**

This was written as an assignment submission. The scaffolding (LineRenderer setup, mouse-follow logic, StateController integration) was provided. The simulation logic itself is my own work.
