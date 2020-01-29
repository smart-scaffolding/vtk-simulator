from tkinter import *
import numpy as np

# from robopy.base.FaceStar import *

window = Tk()
window.title("Robot Controller GUI")
window.geometry('350x200')

lbl = Label(window, text="Hello")
lbl.grid(column=0, row=0)

txt = Entry(window, width=10)
txt.grid(column=1, row=0)

def clicked():
    res = "Welcome to " + txt.get()
    lbl.configure(text=res)


    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    base = np.matrix([[1, 0, 0, 0.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])

    num_steps = 20

    from .robot_gui_controller import robot_simulation
    robot_simulation(blueprint=blueprint, base=base, num_steps=num_steps, startFace=(1, 0, 0, 'top'), endFace=(5, 0, 0, 'top'))


btn = Button(window, text="Click Me", command=clicked)
btn.grid(column=2, row=2)

window.mainloop()