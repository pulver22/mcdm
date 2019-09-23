**=== INSTRUCTION ===**

How does the Choquet Fuzzy Integral work?
1) Identify a frontier $p$
2) Evaluate the frontier $p$ using the $n$ criteria (in our example we use $n=3$)
3) Order the criteria from the last significant to the most important one based on the previous evaluation (e.g., $c2 - c3 - c1$)
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
   $f(p) += u(ST) * \eta(ST) = 0 * 0.15 = 0$
    * We proceed than with the secon criterion ($IG$):  
        $f(p) += (u(IG) - u(ST)) * \eta(IG,TD) = (0.236686 - 0 ) * 0.95 = 0.22477$
    * And then with the last one, the most important for this pose ($TD$):  
        $f(p) += (u(TD) - u(IG)) * \eta(TD) += (0.67935 - 0.236686) * 0.35 = 0.3797045$