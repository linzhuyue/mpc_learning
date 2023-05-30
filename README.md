# <p align="center">  A series of codes/notes of MPC was used for the study.</p>
This is a note about MPC, refer to the course taught by Prof. Bing Zhu. **Check The Course On Bilibili** [here](https://space.bilibili.com/511139883/channel/collectiondetail?sid=566863).

### Repository Organization
The code is organized as follows:
```
.
├──common/
├──constraints_mpc/
├──fig/
├──MPC_tools_try/
├──nonlinear_mpc/
├──Robust_mpc/
├──setup.m
├──unconstraint_mpc/
```
1. `common`
    - Contains the common function such as the MPC matrix Gains(Phi).
2. `constraints_mpc`
    - It mainly introduces the course case codes of the chapter-2, and the name of the code file is its code meaning.
3. `MPC_tools_try`
    - Contains the method from MATLAB MPC toolbox.
4.  `nonlinear_mpc`
    - Contains the code of Chapter-3.
5.  `Robust_mpc`
    - Contains the code of Chapter-4.
6.  `unconstraint_mpc`
    - Contains the code of Chapter-1.
7.  `setup.m`
    - Before you try to run the codes, you should run this first.
## [Explicit MPC]
<p align="center">
  <img width="400" height="225"
       src="https://github.com/linzhuyue/mpc_learning/blob/main/dr_zhubing_code_2022_spring/fig/explcit.png">
</p>
<p align="center">
  Figure: Explicit MPC  
</p>