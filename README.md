Nonlinear systems have been of growing interest and frequently researched upon pertaining to the field of control theory and designing controllers is of vital importance to regulate and control the behavior of systems. Traditionally, designing controllers for nonlinear systems has been a complex and demanding task, relying extensively on precise system models that can account for the non-linearity. The current scope of the work discussed as part of this thesis is done to make the modeling process less demanding by employing the power of data-driven approaches, making it accessible even to those without extensive expertise in system dynamics. The core objective of this work is to develop a versatile model identification tool capable of generating system models for subsequent controller design. 
Conventional methods for modeling nonlinear systems are often bundled with complex mathematical modeling accounting for every tiny aspect of the non-linearity thus often ending up as time-consuming and heavily reliant on expert input. In contrast, data-based modeling leverages real-world observations and patterns, offering a more accurate representation of system behavior. There is a trade-off of complex equations in favor of learning from actual examples, making it better equipped to handle system intricacies and uncertainties. Data-based modeling minimizes model reduction errors by accounting for all physical parameters, leading to more reliable predictions and enhanced control capabilities. 
One such approach that has been gaining traction is the Sparse Identification of Nonlinear Dynamics (SINDy). It identifies governing equations from data, even in complex systems, streamlining the modeling process compared to traditional methods and enhancing accuracy compared to classical databased modeling and neural networks, which require vast datasets to mitigate noise. 
In addition to the SINDy approach, the Nonlinear Autoregressive Exogenous Model (NARX) neural network has also been used for the identification of the Wheeled Inverted Pendulum (WIP) system. The results obtained from the NARX network are compared with those obtained from the SINDy approach to evaluate the effectiveness of both methods in modeling the system dynamics. This comparative analysis can provide valuable insights into the advantages and limitations of each approach and can help in selecting the most appropriate method for a given application. 
The research done as part of the work combines SINDy with Model Predictive Control (MPC) as Sparse Identification of Nonlinear Dynamics with Control (SINDYc), revolutionizing the identification and control of nonlinear systems. The combination of data-driven modeling and MPC promises advancements in controlling complex systems, benefiting fields like robotics and autonomous vehicles. The proposed approach encompasses five tasks, starting with understanding SINDYc using a simple pendulum to understand the workings of the approach and later progressing to more complex systems like the WIP. Moreover, the work involves using a neural network for system identification, specifically comparing the efficiency of models generated by SINDYc and the NARX model. Every aspect of the work on the aforementioned systems involves system identification and a custom control design for each use case, ultimately aiming to optimize system performance and stability. The fusion of these cutting-edge techniques is demonstrated practically on the evoBOT robotic platform, emphasizing the ability to learn nonlinear dynamic models from limited measurements. 
This master’s thesis is carried out with the desire to gradually explore nonlinear system identification and control, combining data-driven modeling and advanced control strategies. This work also builds the groundwork for a potential further scope of development in the direction of the modeling and control of nonlinear systems across various real-time applications.