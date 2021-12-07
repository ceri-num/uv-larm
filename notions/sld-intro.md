---
marp: true
theme: imt
paginate: true
backgroundImage: url('../style/bg-imt.svg')
---

# My beatiful tittle
### and an optional subtittle

<br />
<br />
<br />
<br />

**Guillaume Lozenguez**
[@imt-lille-douai.fr](mailto:guillaume.lozenguez@imt-lille-douai.fr)

![bg](../style/bg-tittle.svg)

---

1. Random trajectories (a lot)
   - ....
   - .... 
2. Until each transition is visited several times.
   - ....
   - ....
3. Compute an optimal policy.
   - ....

![bg](../style/bg-toc.svg)

---
<!-- --------------------------------------------------------------- -->


## Markov Decision Process

<br />

**MDP:** $\langle S, A, T, R \rangle$:

*S :* set of system's states
*A :* set of possible actions
*T :* S × A × S → [0, 1] : transitions
*R :* S × A → R : cost/rewards

![bg right 100%](../figs/MDP.svg)

**Optimal policy:**

The policy $\pi^*$ maximizing Bellman

---
<!-- --------------------------------------------------------------- -->

## Bellman Equation

### State evaluation for a given policy $\pi$:

$$V^\pi(s)= R(s, a) + \gamma \sum_{s'\in S} T(s,a,s') \times V^\pi(s')$$
(with $\pi(s) = a$, $s'$ the potential state at the next time step and $\gamma$ the discount factor)

### As a sum of gains:

- The immediate reward: $R(s, a)$
- The future gains $V^\pi(s')$, proportional to the probability to reach them $T(s,a,s')$
- with the parameter $\gamma \in [0, 1]$, balancing immediate and future gains

---
<!-- --------------------------------------------------------------- -->

#### In conclusion....
