# PROCCONTROL

## Default values by comand line

The map `button_slot_id` was defined as `<module_name, {Start Program, Stop Program, Show Output, No Output}>` containing the identifiers of each button, whose hash key is the module name. To set a default value of a button in a module, just pass it via the command line, and treat in the main function.

Map:

```c
button_slot_id[module_name][0]; // Start Program
button_slot_id[module_name][1]; // Stop Program
button_slot_id[module_name][2]; // Show Output
button_slot_id[module_name][3]; // No Output
```

Access:
```c
qdisplay->showClicked(button_slot_id["logger"][2];
```

Slots:
```c
void startClicked( int ); // Start Program
void stopClicked( int ); // Stop Program
void showClicked( int ); // Show Output
void noClicked( int ); // No Output
```

For example, to enable log output, just make in the bash `./proccontrol_gui -show logger`, and in the code, it's something like this: `qdisplay->showClicked(button_slot_id["logger"][2]);`.
