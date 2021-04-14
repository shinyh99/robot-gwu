# %%
# import matplotlib
# from tclab import clock, setup, Historian, Plotter


# def pid(k_p, k_i, k_d, mv_bar=0.0):
#     # initialize stored data
#     e_prev = 0
#     t_prev = -100
#     i = 0

#     # initial control
#     m_v = mv_bar

#     while True:
#         # yield mv, wait for new t, p_v, s_p
#         t, p_v, s_p = yield m_v

#         # PID calculation
#         error = s_p - p_v

#         p = k_p * error
#         i = i + k_i * error * (t - t_prev)
#         d = k_d * (error - e_prev) / (t - t_prev)

#         m_v = mv_bar + p + i + d

#         # update stored data for next iteration
#         e_prev = error
#         t_prev = t

#     def proportional(k_p, s_p):
#         """create proportional controlllers with specified gain and setpoint

#         Args:
#             Kp (float): Proportional gain
#             Sp (float): Setpoint
#         """
#         # manipulated variable
#         m_v = 0.0
#         while True:
#             # process variable
#             p_v = yield m_v
#             m_v = k_p * (s_p - p_v)


# # controller1 = pid.proportional(10, 40)
# # controller1.send(None)

# # controller2 = pid.proportional(1, 40)
# # controller2.send(None)

# # pv = 35

# # print("Controller 1: MV = ", controller1.send(pv))
# # print("Controller 2: MV = ", controller2.send(pv))


# class New:
#     def __init__(self) -> None:
#         self.TCLab = setup(connected=False, speedup=10)

#         self.controller = pid(2, 0.1, 2)  # create pid control
#         self.controller.send(None)  # initialize

#     def run(self):

#         tfinal = 800

#         with self.TCLab() as lab:
#             h = Historian(
#                 [
#                     ("SP", lambda: SP),
#                     ("T1", lambda: lab.T1),
#                     ("MV", lambda: MV),
#                     ("Q1", lab.Q1),
#                 ]
#             )
#             p = Plotter(h, tfinal)
#             T1 = lab.T1
#             for t in clock(tfinal, 2):
#                 SP = T1 if t < 50 else 50  # get setpoint
#                 PV = lab.T1  # get measurement
#                 MV = self.controller.send(
#                     [t, PV, SP]
#                 )  # compute manipulated variable
#                 lab.U1 = MV  # apply
#                 p.update(t)  # update information display


# new = New()
# new.run()


# # TCLab = setup(connected=False, speedup=10)

# # controller = pid(2, 0.1, 2)  # create pid control
# # controller.send(None)  # initialize

# # tfinal = 800

# # with TCLab() as lab:
# #     h = Historian(
# #         [
# #             ("SP", lambda: SP),
# #             ("T1", lambda: lab.T1),
# #             ("MV", lambda: MV),
# #             ("Q1", lab.Q1),
# #         ]
# #     )
# #     p = Plotter(h, tfinal)
# #     T1 = lab.T1
# #     for t in clock(tfinal, 2):
# #         SP = T1 if t < 50 else 50  # get setpoint
# #         PV = lab.T1  # get measurement
# #         MV = controller.send([t, PV, SP])  # compute manipulated variable
# #         lab.U1 = MV  # apply
# #         p.update(t)  # update information display

# %%
for i in range(-5, 5):
    print(i)
# %%
