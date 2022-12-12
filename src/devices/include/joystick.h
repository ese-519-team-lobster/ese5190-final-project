





void joystick_init(int x_pin, int y_pin, int sel_pin);
void get_joystick_inputs(int * x_axis, int * y_axis, int * z_axis);
void get_test_inputs(int * x_axis, int * y_axis, int * z_axis);
void get_console_inputs(int * x_axis, int * y_axis, int * z_axis, int * wrist, double * wrist_rot, int * gripper, int* prog_cmd);