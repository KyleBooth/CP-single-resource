**Single Resource Transformation:** CPOptimizer and MiniZinc Implementations  
**Author:** Kyle E. C. Booth (kbooth@mie.utoronto.ca) 

This repository investigates the single resource transformation (SRT) in constraint programming (CP) for vehicle routing and scheduling problems. Our initial paper that explored this idea with application to electric vehicle routing:

**Paper:** "[A Constraint Programming Approach to Electric Vehicle Routing with Time Windows](https://tidel.mie.utoronto.ca/pubs/Booth-CPAIOR2019.pdf)".

**Citation:**  
Booth, K.E.C. & Beck, J.C., "A Constraint Programming Approach to Electric Vehicle Routing with Time Windows", Proceedings of Sixteenth International Conference on the Integration of Constraint Programming, Artificial Intelligence, and Operations Research (CPAIOR2019), 129-145, 2019.
```
@inproceedings{booth2019constraint,
  title={A Constraint Programming Approach to Electric Vehicle Routing with Time Windows},
  author={Booth, Kyle EC and Beck, J Christopher},
  booktitle={International Conference on Integration of Constraint Programming, Artificial Intelligence, and Operations Research},
  pages={129--145},
  year={2019},
  organization={Springer}
}
```

If you are interested in the source code of the above paper, please contact me.

### File/directory descriptions:

*"cpoptimizer":* implementation of SRT for multiple traveling salesman problem (mTSP) in C++ using CPOptimizer.

*"minizinc":* implementation of SRT for mTSP in MiniZinc.

