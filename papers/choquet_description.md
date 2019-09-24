**=== INSTRUCTION ===**

How does the Choquet Fuzzy Integral work?
1) Identify a frontier $p$
2) Evaluate the frontier $p$ using $n$ available criteria, $u(c_i)$ (in our example we use $n=3$)
3) Order the criteria from the last significant to the most important one based on the previous evaluation (e.g.,$u(c2) < u(c3) <u(c1)$ then  $c2 - c3 - c1$)
4) Starting from the less significant one, for each criterion find the list (combination) of criteria whose evaluation is greater than that one ( given $c2$ -> $(c1,c2,c3)$)
   * If it's the first consider criterion, multiply it for the weight of the coalition  ( $u(c2) * \eta(c1,c2,c3)$)
   * If it's not the first considered one, subtract the evaluation of the previous criterion first and then multiply the result for the weight of the coalition ($(u(c3) - u(c2)) * \eta(c3,c1)$)


**=== Toy example ===**

| IG   |  TD   |  ST   | IG,TD | IG,ST | TD,ST | IG, TD, ST |
| :--- | :---: | :---: | :---: | :---: | :---: | :--------: |
| 0.5  | 0.35  | 0.15  | 0.95  | 0.75  |  0.6  |     1      |

1) Assume we are considering a frontier $p$
2) $u(IG) = 0.236686$   
   $u(TD) = 0.67935$  
   $u(ST) = 0$

3) Based on the previous evaluation, we can order the criteria as following:  
    $ST - IG - TD$

4) The first criteria is $ST$, the associated list of criteria is therefore $(ST, IG, TD)$, hence:  
   $f(p) += u(ST) * \eta(ST, TD, IG) = 0 * 0.15 = 0$
    * We proceed than with the secon criterion ($IG$):  
        $f(p) += (u(IG) - u(ST)) * \eta(IG,TD) = (0.236686 - 0 ) * 0.95 = 0.22477$
    * And then with the last one, the most important for this pose ($TD$):  
        $f(p) += (u(TD) - u(IG)) * \eta(TD) += (0.67935 - 0.236686) * 0.35 = 0.3797045$


**=== NOTE ===**  
From the paper [1]:  
> Another important advantage of MCDM is its generality. Indeed, different aggregation operators turn out to be particular cases of the Choquet integral, up to a proper choice of weights for the fuzzy measure $\mu$. For instance, a class of aggregation operators that can be expressed with the Choquet integral are *weighted means*. A weighted mean is defined as $\sum_n^{i=1} w_iu_i(p)$ where $w_i$ is the weight of criterion $i$ and $\sum_n^{i=1} w_i = 1$. This aggregation operator can be obtained from Choquet integral by setting $\eta({i}) = w_i$ for all $i \in N$ and by constraining $\eta$ to be additive:  
$\eta(S) = \sum_{i \in S}w_i \quad \forall S \in \Rho(N)$  
Note that additivity of $\eta$ reflects *independence* between criteria, namely joint contributions are exactly the sum of marginal ones. Therefore, weighted means should be considered suitable when such independence between criteria actually holds.

> The arithmetic mean and the $k$-th criterion projection (namely, considering only a criterion) can be obtained as further particular cases of weighted means by imposing $w_i = 1/n \forall i \in N$ and $w_k = 1, w_i = 0 \forall i \in N \setminus {k}$, respectively.

> Compared to the weighted sum, the Choquet integral allows to model preferences over alternatives, taking into account also *conditional relative importance* among criteria.  


**=== REFERENCES ===**

[1] Basilico, N., & Amigoni, F. (2011). Exploration strategies based on multi-criteria decision making for searching environments in rescue operations. Autonomous Robots, 31(4), 401.  
[2] Grabisch, M., & Labreuche, C. (2010). A decade of application of the Choquet and Sugeno integrals in multi-criteria decision aid. Annals of Operations Research, 175(1), 247-286.  