# Project: Vehicle Platoon Using PID Control



## Description
In order to cope with uncertainties in a platoon, this paper proposes a reconfifigurable multi-agent architecture to address the platoon safety problem by handling two modes: the normal mode and the degraded mode. At this stage of research, the normal mode is characterized by the interaction between agents over a Vehicle-to-Vehicle (V2V) communication network while the degraded mode simply involves sensors for a local perception. The switching from the normal mode to the degraded one is triggered when the communication quality is considered not fully reliable. A PID (Proportional Integral Derivative) controller is proposed to regulate the inter-vehicle distance and orientation. 
Two models are proposed in this paper: in the first one, the management operations 
such as splitting and joining are set up while the second is mainly modeled to assess 
the implemented controller quality. In this paper, the safety of a platoon is represented by the quality of tracking and the inter-vehicle distance. The mentioned features are assessed for both modes through a formal verification using the Uppaal software. We prove the inefficiency of the proposed platoon model for several situations such as merging, following or leaving the platoon by verifying different properties using the model checking. The evaluation of the second model, simulated by the ** Webots software**, proves the impact of the number of vehicles on the platoon performance and the vehicle tracking quality. We conclude that when the platoon reaches a certain number of vehicles, the safety criterion is no more reliable. 



## Simulation Video: 
![platoon](https://user-images.githubusercontent.com/4749204/194799677-f1d4a553-048c-4cdf-bc1f-ee85e8910a86.gif)




If you use the code, please cite our papers: 
[1] Oussama Karoui, Mohamed Khalgui, Anis Koubaa,  Emna Guerfala, Zhiwu Li, and Eduardo Tovar, “Dual mode for vehicular platoon safety: Simulation and formal verification”, Information Sciences, vol. 402, pp. 216—232 , 2017.
[2] Oussama Karoui, Emna Guerfala, Anis Koubaa,  Mohamed Khalgui, Eduardo Tovar, Naiqi Wu,  Abdulrahman Al-Ahmari,  Zhiwu Li,  “Performance evaluation of vehicular platoons using Webots”, IET Intelligent Transport Systems, vol. 11, no. 8, pp. 441—449, 2017.
