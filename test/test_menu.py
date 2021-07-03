

from kivymd.app import MDApp
from kivy.uix.screenmanager import Screen
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.lang import Builder

from consolemenu import ConsoleMenu, SelectionMenu
from consolemenu.items import FunctionItem, SubmenuItem, CommandItem

# Create the menu
menu = SelectionMenu("Title", "Subtitle", prologue_text='hello', epilogue_text='@ULAVAL')

# Create some items

# A FunctionItem runs a Python function when selected
function_item = FunctionItem("Call a Python function", input, ["Enter an input"])

# A CommandItem runs a console command
command_item = CommandItem("Run a console command",  "touch hello.txt")

# A SelectionMenu constructs a menu from a list of strings
selection_menu = SelectionMenu(["item1", "item2", "item3"])

# A SubmenuItem lets you add a menu (the selection_menu above, for example)
# as a submenu of another menu
submenu_item = SubmenuItem("Submenu item", selection_menu, menu)

# Once we're done creating them, we just add the items to the menu
menu.append_item(function_item)
menu.append_item(command_item)
menu.append_item(submenu_item)

# Finally, we call show to show the menu and allow the user to interact
menu.show()