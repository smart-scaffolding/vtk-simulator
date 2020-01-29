from tkinter import *
from robopy.gui.robot_gui_controller import robot_simulation
import numpy as np

title = 'Helvetica 24 bold'
h1 = 'Helvetica 18 bold'
h2 = 'Helvetica 12 bold'


blueprint = np.array([
            [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
            [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
            [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
            [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
            [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
            [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
        ])

class RobotGui:
    def __init__(self, win):


        self.mainTitle = Label(window, text="Robot Control Parameters: ", font=title)

        self.kinematics_radio_buttons()

        self.create_base_entries()
        self.create_angles_entries()
        self.create_gripper_entries()
        self.create_point_selection()
        self.create_structure_selection()

        self.create_port_entries()
        # self.lbl1=Label(win, text='First number')
        # self.lbl2=Label(win, text='Second number')
        # self.lbl3=Label(win, text='Result')
        # self.t1=Entry(bd=3)
        # self.t2=Entry()
        # self.t3=Entry()
        # self.btn1 = Button(win, text='Add')
        # self.btn2=Button(win, text='Subtract')

        self.mainTitle.place(x=5, y=5)
        # self.lbl1.place(x=100, y=100)
        # self.t1.place(x=200, y=100)
        # self.lbl2.place(x=100, y=150)
        # self.t2.place(x=200, y=150)
        #
        self.b1=Button(win, text='Simulate', command=self.simulate)
        self.b2=Button(win, text='Run', command=self.add)
        # self.b2.bind('<Button-1>', self.sub)
        #
        self.b1.place(x=500, y=5)
        self.b2.place(x=600, y=5)
        # self.lbl3.place(x=100, y=250)
        # self.t3.place(x=200, y=250)

    def simulate(self):
        base = (float(self.base_lbl_txt_x.get()), float(self.base_lbl_txt_y.get()), float(self.base_lbl_txt_z.get()))
        grippers = (self.grippers_lbl_text_g1.get(), self.grippers_lbl_text_g2.get())
        structure = self.selected_structure.get()

        port = self.port_label_text.get()

        point = None
        angles = None
        if self.selected_kinematics.get() == 1: #FKWIN
            angles = (float(self.angles_lbl_text_j1.get()), float(self.angles_lbl_text_j2.get()),
                      float(self.angles_lbl_text_j3.get()))
        else: #IK
            point = self.ik_destination.get()

        print(f"Base: {base}\nGrippers: {grippers}\n Structure: {structure}\nPoint: {point}\nAngles: {angles}\n Port: "
              f"{port}")

        results = robot_simulation(blueprint=blueprint, base=base, num_steps=20, startFace=(1, 0, 0, 'top'),
                                 endFace=point)

        print(results)

    def add(self):
        self.t3.delete(0, 'end')
        num1=int(self.t1.get())
        num2=int(self.t2.get())
        result=num1+num2
        self.t3.insert(END, str(result))

    def sub(self, event):
        self.t3.delete(0, 'end')
        num1=int(self.t1.get())
        num2=int(self.t2.get())
        result=num1-num2
        self.t3.insert(END, str(result))

    def kinematics_radio_buttons(self):
        self.selected_kinematics = IntVar()
        self.selected_kinematics.set(1)  # initializing the choice, i.e. Python

        self.kinematics_options = [
            ("Forward", 1),
            ("Inverse", 2),
        ]


        self.ik_label = Label(window,
                              text="Select kinematics: ",
                              justify=LEFT,
                              padx=20,
                              font=h1
                              )
        self.ik_label.place(x=70, y=50)

        self.ik_btn_fwk = Radiobutton(window,
                                      text=self.kinematics_options[0][0],
                                      indicatoron=0,
                                      width=20,
                                      padx=20,
                                      variable=self.selected_kinematics,
                                      command=self.update_kinematics_selection,
                                      value=self.kinematics_options[0][1],
                                      bg='blue', fg='white', highlightbackground='#3E4149')
        self.ik_btn_fwk.place(x=300, y=50)

        self.ik_btn_ik = Radiobutton(window,
                                     text=self.kinematics_options[1][0],
                                     indicatoron=0,
                                     width=20,
                                     padx=20,
                                     variable=self.selected_kinematics,
                                     command=self.update_kinematics_selection,
                                     value=self.kinematics_options[1][1],
                                     bg='blue', fg='white', highlightbackground='#3E4149')
        self.ik_btn_ik.place(x=600, y=50)

    def update_kinematics_selection(self):
        # self.selected_kinematics
        print(self.selected_kinematics.get())
        if self.selected_kinematics.get() == 1:
            fwk_update = "normal"
            ik_update = "disabled"

        else:
            fwk_update = "disabled"
            ik_update = "normal"

        self.angles_lbl_text_j1.config(state=fwk_update)
        self.angles_lbl_text_j2.config(state=fwk_update)
        self.angles_lbl_text_j3.config(state=fwk_update)

        self.ik_destination_selection.config(state=ik_update)

    def create_base_entries(self, start_x=75, start_y=100, label_increment=120, entry_increment=130,
                            entry_height_offset=30, label_height_offset=35):
        self.base_lbl = Label(window, text="Base: ", font=h1)
        self.base_lbl.place(x=start_x, y=start_y)

        self.base_lbl_x = Label(window, text="X: ", font=h2)
        self.base_lbl_x.place(x=start_x+label_increment-7, y=start_y+label_height_offset)
        self.base_lbl_txt_x = Entry(window, width=10, bd=3)
        self.base_lbl_txt_x.place(x=start_x+entry_increment, y=start_y+entry_height_offset)

        self.base_lbl_y = Label(window, text="Y: ", font=h2)
        self.base_lbl_y.place(x=start_x+label_increment*2, y=start_y+label_height_offset)
        self.base_lbl_txt_y = Entry(window, width=10, bd=3)
        self.base_lbl_txt_y.place(x=start_x+entry_increment*2, y=start_y+entry_height_offset)

        self.base_lbl_z = Label(window, text="Z: ", font=h2)
        self.base_lbl_z.place(x=start_x+label_increment*3+7, y=start_y+label_height_offset)
        self.base_lbl_txt_z = Entry(window, width=10, bd=3)
        self.base_lbl_txt_z.place(x=start_x+entry_increment*3, y=start_y+entry_height_offset)

    def create_angles_entries(self, start_x=75, start_y=200, label_increment=120, entry_increment=130,
                            entry_height_offset=30, label_height_offset=35):
        self.angles_lbl = Label(window, text="Angles: ", font=h1)
        self.angles_lbl.place(x=start_x, y=start_y)

        self.angles_lbl_j1 = Label(window, text="J1: ", font=h2)
        self.angles_lbl_j1.place(x=start_x+label_increment-10, y=start_y+label_height_offset)
        self.angles_lbl_text_j1 = Entry(window, width=10, bd=3)
        self.angles_lbl_text_j1.place(x=start_x+entry_increment, y=start_y+entry_height_offset)
        self.angles_lbl_text_j1.config(state="disabled" if self.selected_kinematics.get() == 2 else "normal")

        self.angles_lbl_j2 = Label(window, text="J2: ", font=h2)
        self.angles_lbl_j2.place(x=start_x+label_increment*2, y=start_y+label_height_offset)
        self.angles_lbl_text_j2 = Entry(window, width=10, bd=3)
        self.angles_lbl_text_j2.place(x=start_x+entry_increment*2, y=start_y+entry_height_offset)
        self.angles_lbl_text_j2.config(state="disabled" if self.selected_kinematics.get() == 2 else "normal")

        self.angles_lbl_j3 = Label(window, text="J3: ", font=h2)
        self.angles_lbl_j3.place(x=start_x+label_increment*3+7, y=start_y+label_height_offset)
        self.angles_lbl_text_j3 = Entry(window, width=10, bd=3)
        self.angles_lbl_text_j3.place(x=start_x+entry_increment*3, y=start_y+entry_height_offset)
        self.angles_lbl_text_j3.config(state="disabled" if self.selected_kinematics.get() == 2 else "normal")

    def create_gripper_entries(self, start_x=75, start_y=300, label_increment=120, entry_increment=130,
                            entry_height_offset=30, label_height_offset=35):
        self.grippers_lbl = Label(window, text="Grippers: ", font=h1)
        self.grippers_lbl.place(x=start_x, y=start_y)

        self.grippers_lbl_g1 = Label(window, text="G1: ", font=h2)
        self.grippers_lbl_g1.place(x=start_x+label_increment-15, y=start_y+label_height_offset)
        self.grippers_lbl_text_g1 = Entry(window, width=10, bd=3)
        self.grippers_lbl_text_g1.place(x=start_x+entry_increment, y=start_y+entry_height_offset)

        self.grippers_lbl_g2 = Label(window, text="G2: ", font=h2)
        self.grippers_lbl_g2.place(x=start_x+label_increment*2-5, y=start_y+label_height_offset)
        self.grippers_lbl_text_g2 = Entry(window, width=10, bd=3)
        self.grippers_lbl_text_g2.place(x=start_x+entry_increment*2, y=start_y+entry_height_offset)

    def create_point_selection(self, start_x=75, start_y=400, label_increment=120, entry_increment=130,
                            entry_height_offset=30, label_height_offset=35):
        self.ik_destination = StringVar(window)

        # Dictionary with options
        choices = {"(5, 0, 0, 'Top')", "(7, 1, 3, 'Top')", "(3, 2, 1, 'Left')", "(3, 0, 0, 'Top')"}
        self.ik_destination.set("(5, 0, 0, 'Top')")  # set the default option

        self.ik_destination_selection = OptionMenu(window, self.ik_destination, *choices)
        self.ik_destination_selection_label = Label(window, text="Point:", font=h1).place(x=start_x, y=start_y)
        self.ik_destination_selection.place(x=start_x+label_increment-15, y=start_y+label_height_offset)
        self.ik_destination_selection.config(state="disabled" if self.selected_kinematics == 1 else "normal")

        # link function to change dropdown
        self.ik_destination.trace('w', self.select_ik_destination)

    def select_ik_destination(self, *args):
        print(self.ik_destination.get())


    def create_structure_selection(self, start_x=75, start_y=500, label_increment=120, entry_increment=130,
                            entry_height_offset=30, label_height_offset=35):
        self.selected_structure = StringVar(window)

        # Dictionary with options
        choices = {"Playground"}
        self.selected_structure.set("Playground")  # set the default option

        self.structure_selection = OptionMenu(window, self.selected_structure, *choices)
        self.structure_selection_label = Label(window, text="Structure:", font=h1).place(x=start_x, y=start_y)
        self.structure_selection.place(x=start_x+label_increment-15, y=start_y+label_height_offset)
        self.structure_selection.config(state="disabled" if self.selected_kinematics == 1 else "normal")

        # link function to change dropdown
        self.selected_structure.trace('w', self.select_structure_menu)

    def select_structure_menu(self, *args):
        print(self.selected_structure.get())

    def create_port_entries(self, start_x=75, start_y=600, label_increment=120, entry_increment=130,
                            entry_height_offset=30, label_height_offset=35):
        self.port_lbl = Label(window, text="Port: ", font=h1)
        self.port_lbl.place(x=start_x, y=start_y)

        self.port_label_text = Entry(window, width=50, bd=3)
        self.port_label_text.place(x=start_x+entry_increment-10, y=start_y+entry_height_offset)

window=Tk()
mywin=RobotGui(window)
window.title('Robot Controller GUI')
window.geometry("1200x700+10+10")
window.mainloop()