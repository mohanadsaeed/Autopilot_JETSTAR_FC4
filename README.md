# Autopilot JETSTAR FC4
## JETSTAR FC4 Autopilot  
This is the autopilot system for JETSTAR aircraft at flight condition 4. In this project, we can mention the steps as following:<br/>  
1- Test Some Numerical Methods for Solving PDEs (Aircraft Equations of Motion) to choose the best method<br/><br/>
2- Build Airplane Non Linear Simulator to solve the equations of motion and calculate the states of the aircraft from the control actions<br/><br/> 
3- Test the Non Linear Simulator by comparing its results with benchmarks of BOEING 747 Results<br/><br/> 
4- Linearize the aircraft equations of motion to get the transfer functions which will be used to design controllers<br/><br/> 
5- Use the linearized state space model of the longitudinal dynamics (4x4) to represent the motion of the airplane in the longitudinal plane, which is a (Multi input Multi Output system MIMO), and to design our controllers we will use our previous studies about the (linear time invariant Single Input Single Output system SISO) to design our controllers like (linear PID and compensators). This method is called “Successive loop closure”<br/><br/>
6- Use the linearized state space model of the lateral dynamics (5x5) to represent the motion of the airplane in the lateral plane to design our controllers<br/><br/> 
7- Test the full autopilot system (longitudinal &amp; lateral) on the non linear model and tuning the gains to get the desired response for each state<br/><br/>
