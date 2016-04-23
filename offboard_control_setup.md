# Pixhawk OFFBOARD Control Setup



---

OFFBOARD control means that we would like to be able to send (usually) high-level control commands to *Pixahwk*. For example, sending position, velocity , or acceleration set-points. Then, *Pixhawk* will receive those set-points and perform the neccessary low-level control (e.g. attitude/engines control).

In general, sending high-level commands is done off-board (board here refers to *Pixhawk*). In other words, an offboard computer is usually used to execute some code to take some high-level decisions. Then, high-level decisions are translated to set-points (e.g. position set-points) which, then, are sent to the *Pixhawk* to be executed. For example, an offboard computer can be used to do run some image processing algorithm for object tracking. The output of the algorithm is position set-points to tell *Pixhawk* to move to the direction of the tracked object.

In general, executing such offboard tasks are not feasible due to the limited resources on *Pixhwak*. Therefore, more powerful computers are used.

Offboard computers can be single board computer (or SBC in brief), e.g. [ODROID XU4](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825). Or, it can be a fully loaded workstaton, desktop, or laptop.

In summary, *Pixhawk* is used as a flight controllers. Whereas, offboard controller are used to execute more sophisticated tasks.

In this guide, we will learn how to do offboard control from an SBC (ODROID XU4), and from desktop/laptop that runs *MATLAB*.