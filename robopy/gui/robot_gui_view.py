from tkinter import *
import numpy as np
import multiprocessing
import atexit
# from robopy.base.FaceStar import *

# simulation = None
title = 'Helvetica 24 bold'
h1 = 'Helvetica 18 bold'
h2 = 'Helvetica 12 bold'
selected_kinematics = 1

def run_gui():
    window = Tk()
    window.title("Robot Controller GUI")
    window.geometry('700x700')
    # window.configure(bg='')

    # mainframe = Frame(window)
    # mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
    # mainframe.columnconfigure(0, weight=1)
    # mainframe.rowconfigure(0, weight=1)
    # mainframe.pack(pady=100, padx=100)


    mainTitle = Label(window, text="Robot Control Parameters: ", font=title)
    mainTitle.grid(column=0, row=0)

    v = IntVar()
    v.set(1)  # initializing the choice, i.e. Python

    kinematics_options = [
        ("Forward", 1),
        ("Inverse", 2),
    ]

    def ShowChoice():
        global selected_kinematics
        selected_kinematics = v.get()

        print(selected_kinematics)

    ik_label = Label(window,
             text="Select kinematics to use",
             justify=LEFT,
             padx=20)
    ik_label.grid(column=0, row=2)


    for val, option in enumerate(kinematics_options):
        btn = Radiobutton(window,
                       text=option[0],
                       indicatoron=0,
                       width=20,
                       padx=20,
                       variable=v,
                       command=ShowChoice,
                       value=val,
                       bg='blue', fg='white', highlightbackground='#3E4149')
        btn.grid(column=0, row=3+val)

    create_base_entries(window, 5)

    create_point_selection(window, 11)

    create_angles_entries(window, 17)

    create_gripper_entries(window, 23)

    create_structure_selection(window, 29)

    create_port_entries(window, 35)

    # Create a Tkinter variable


    def clicked():
        global simulation
        print("Running code on robot")


        # blueprint = np.array([
        #     [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        #     [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        #     [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        #     [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        #     [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        #     [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
        # ])
        #
        # base = np.matrix([[1, 0, 0, 0.5],
        #                   [0, 1, 0, 0.5],
        #                   [0, 0, 1, 1.],
        #                   [0, 0, 0, 1]])
        #
        # num_steps = 20
        #
        # # if simulation:
        # #     simulation.terminate()
        # #
        # # from robopy.gui.robot_gui_controller import robot_simulation
        # #
        # # simulation = multiprocessing.Process(target=robot_simulation, args=(None, blueprint, base, num_steps, (1, 0, 0,
        # #                                                                                                      'top'),
        # #                                                                     (5, 0, 0, 'top')))
        # # simulation.start()
        # # atexit.register(simulation.terminate)

    simulate_btn = Button(window, text="Run Simulation", command=clicked, bg='blue', fg='white', highlightbackground='#3E4149')
    simulate_btn.grid(column=1, row=40)
    # simulate_btn.configure(bg='#1846FF')
    # simulate_btn.pack()

    submit_btn = Button(window, text="Upload to Robot", command=clicked, bg='blue', fg='white', highlightbackground='#3E4149')
    submit_btn.grid(column=3, row=40)
    # submit_btn.pack()

    # submit_btn.configure(bg='#1846FF')


    window.mainloop()

    # class GuiSimulation():
    #         def __init__(self, blueprint, base, num_steps, start_face, end_face):
    #             self.simulation = multiprocessing.Process(target=robot_simulation, args=(blueprint, base, num_steps,
    #                                                                                      start_face, end_face)
    #
    #             atexit.register(self.simulation.terminate)
    #
    #         def initialize(self):
    #             self.simulation.start()


def create_base_entries(window, start_row):
    base_lbl = Label(window, text="Base: ", font=h1)
    base_lbl.grid(column=0, row=start_row)

    base_lbl_x = Label(window, text="X: ", font=h2)
    base_lbl_x.grid(column=0, row=start_row+1)
    base_lbl_txt_x = Entry(window, width=10)
    base_lbl_txt_x.grid(column=1, row=start_row+1)

    base_lbl_y = Label(window, text="Y: ", font=h2)
    base_lbl_y.grid(column=2, row=start_row+1)
    base_lbl_txt_y = Entry(window, width=10)
    base_lbl_txt_y.grid(column=3, row=start_row+1)

    base_lbl_z = Label(window, text="Z: ", font=h2)
    base_lbl_z.grid(column=4, row=start_row+1)
    base_lbl_txt_z = Entry(window, width=10)
    base_lbl_txt_z.grid(column=5, row=start_row+1)


def create_angles_entries(window, start_row):
    angles_lbl = Label(window, text="Angles: ", font=h1)
    angles_lbl.grid(column=0, row=start_row)

    angles_lbl_j1 = Label(window, text="J1: ", font=h2)
    angles_lbl_j1.grid(column=0, row=start_row + 1)
    angles_lbl_text_j1 = Entry(window, width=10)
    angles_lbl_text_j1.grid(column=1, row=start_row + 1)
    angles_lbl_text_j1.config(state="disabled" if selected_kinematics == 2 else "normal")

    angles_lbl_j2 = Label(window, text="J2: ", font=h2)
    angles_lbl_j2.grid(column=2, row=start_row + 1)
    angles_lbl_text_j2 = Entry(window, width=10)
    angles_lbl_text_j2.grid(column=3, row=start_row + 1)
    angles_lbl_text_j2.config(state="disabled" if selected_kinematics == 2 else "normal")

    angles_lbl_j2 = Label(window, text="J3: ", font=h2)
    angles_lbl_j2.grid(column=4, row=start_row + 1)
    angles_lbl_text_j3 = Entry(window, width=10)
    angles_lbl_text_j3.grid(column=5, row=start_row + 1)
    angles_lbl_text_j3.config(state="disabled" if selected_kinematics == 2 else "normal")


def create_gripper_entries(window, start_row):
    grippers_lbl = Label(window, text="Grippers: ", font=h1)
    grippers_lbl.grid(column=0, row=start_row)

    grippers_lbl_g1 = Label(window, text="G1: ", font=h2)
    grippers_lbl_g1.grid(column=0, row=start_row + 1)
    grippers_lbl_text_g1 = Entry(window, width=10)
    grippers_lbl_text_g1.grid(column=1, row=start_row + 1)

    grippers_lbl_g2 = Label(window, text="G2: ", font=h2)
    grippers_lbl_g2.grid(column=2, row=start_row + 1)
    grippers_lbl_text_g2 = Entry(window, width=10)
    grippers_lbl_text_g2.grid(column=3, row=start_row + 1)

def create_point_selection(window, start_row):
    tkvar = StringVar(window)

    # Dictionary with options
    choices = {"(5, 0, 0, 'Top')", "(7, 1, 3, 'Top')", "(3, 2, 1, 'Left')", "(3, 0, 0, 'Top')"}
    tkvar.set("(5, 0, 0, 'Top')")  # set the default option

    popupMenu = OptionMenu(window, tkvar, *choices)
    Label(window, text="Point", font=h1).grid(row=start_row, column=0)
    popupMenu.grid(row=start_row+1, column=2)
    popupMenu.config(state="disabled" if selected_kinematics == 1 else "normal")

    # on change dropdown value
    def change_dropdown(*args):
        print(tkvar.get())

    # link function to change dropdown
    tkvar.trace('w', change_dropdown)

def create_structure_selection(window, start_row):
    tkvar = StringVar(window)

    # Dictionary with options
    choices = {"Playground"}
    tkvar.set("Playground")  # set the default option

    popupMenu = OptionMenu(window, tkvar, *choices)
    Label(window, text="Structure", font=h1).grid(row=start_row, column=0)
    popupMenu.grid(row=start_row + 1, column=2)
    popupMenu.configure(highlightbackground='#3E4149')

    # on change dropdown value
    def change_dropdown(*args):
        print(tkvar.get())

    # link function to change dropdown
    tkvar.trace('w', change_dropdown)

def create_port_entries(window, start_row):
    port_lbl = Label(window, text="Port: ", font=h1)
    port_lbl.grid(column=0, row=start_row)
    port_lbl_txt = Entry(window, width=10)
    port_lbl_txt.grid(column=1, row=start_row)

if __name__ == '__main__':
    run_gui()
