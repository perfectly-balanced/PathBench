import tkinter as tk

root = tk.Tk()
var = tk.StringVar(value="one")
menubutton = tk.Menubutton(root, textvariable=var, indicatoron=True,
                           borderwidth=1, relief="raised", width=20)
main_menu = tk.Menu(menubutton, tearoff=False)
menubutton.configure(menu=main_menu)

for item in (("Numbers", "one", "two", "three"),
             ("Colors", "red", "green", "blue")
):
    menu = tk.Menu(main_menu, tearoff=False)
    main_menu.add_cascade(label=item[0], menu=menu)
    for value in item[1:]:
        menu.add_radiobutton(value=value, label=value, variable=var)

menubutton.pack(side="top", padx=20, pady=20)

root.mainloop()