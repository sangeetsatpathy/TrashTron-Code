import tkinter
import threading
import time
import math

end_program = False
flag = False
val = 0
flag_multiplier = 1
non_flag_multiplier = 1

###########################################################################
###########################################################################
def robot():
    global val
    global end_program
    global flag_multiplier
    global non_flag_multiplier

    print("Robot thread started!")

    counter = 0

    while not end_program:
        if flag:
            val = flag_multiplier * math.sin(counter)
        else:
            val = non_flag_multiplier*math.sin(counter)
        time.sleep(0.25)
        counter += 0.1
###########################################################################
###########################################################################

###########################################################################
###########################################################################
def onclick():
    global flag
    if flag:
        flag = False
    else:
        flag = True
###########################################################################
###########################################################################

###########################################################################
###########################################################################
def endprogram():
    global end_program

    print("End program thread started!")

    while not end_program:
        key = input("press q to exit \n")
        if (key == 'q'):
            end_program = True
###########################################################################
###########################################################################

###########################################################################
###########################################################################
def Update_GUI():
    global flab_label
    global robot_label
    global window
    global end_program
    global slider
    global flag_multiplier
    global textbox
    global textbox_label
    global non_flag_multiplier

    if not end_program:
        flag_label.config(text="Flag: " + str(flag))
        robot_label.config(text=val)
        textbox_label.config(text="String length: " + str(len(textbox.get())))
        non_flag_multiplier = 1 + len(textbox.get())
        flag_multiplier = slider.get()
        window.after(5, Update_GUI)
###########################################################################
###########################################################################

###########################################################################
###########################################################################
if __name__ == "__main__":
    robot_thread = threading.Thread(target=robot)
    robot_thread.start()

    end_thread = threading.Thread(target=endprogram)
    end_thread.start()

    window = tkinter.Tk()
    tkinter.Label(window, text='Hello World!').pack()

    btn = tkinter.Button(window, text="Click to toggle flag", command=onclick)
    btn.pack()

    slider = tkinter.Scale(window,
                           orient=tkinter.HORIZONTAL,
                           label="Flag multiplier:",
                           tickinterval=0,
                           from_=1,
                           to=10)
    slider.pack()

    textbox_label = tkinter.Label(text="String length: ")
    textbox_label.pack()

    textbox = tkinter.Entry(window)
    textbox.pack()

    robot_label = tkinter.Label(window, text="")
    robot_label.pack()

    flag_label = tkinter.Label(window, text="Flag: " + str(flag))
    flag_label.pack()

    gui_update_thread = threading.Thread(target=Update_GUI)
    gui_update_thread.start()

    window.mainloop()
    window.quit()
    window.destroy()

