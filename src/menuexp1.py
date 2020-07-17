import tkinter as tk

class Example(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)

        items = {"one": ["a","b","c"],
                 "two": ["d","e","f"],
                 "three": ["g","h","i"]}

        self.the_value = tk.StringVar()
        self.the_value.set("a")

        self.menubutton = tk.Menubutton(self, textvariable=self.the_value, indicatoron=True)
        self.topMenu = tk.Menu(self.menubutton, tearoff=False)
        self.menubutton.configure(menu=self.topMenu)

        for key in sorted(items.keys()):
            menu = tk.Menu(self.topMenu)
            self.topMenu.add_cascade(label=key, menu=menu)
            for value in items[key]:
                menu.add_radiobutton(label=value, variable = self.the_value, value=value)

        self.menubutton.pack()

if __name__ == "__main__":
    root = tk.Tk()
    Example(root).pack(fill="both", expand=True)
    root.mainloop()