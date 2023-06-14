import os
import datetime
import numpy as np


class DataLogger:
    def __init__(self, filename):
        self.filename = os.path.join("/home/nvidia/pod_comm_py/misc/data", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S_") + filename)
        self.initialized = False
        self.variable_names = []
        self.values = []
        self.value_updated = []
        self.name_type_map = {}
        self.array_name_start_end_map = {}

    def initialize(self, variable_info):
        if not self.initialized:
            self.initialized = True
            for info in variable_info:
                variable_name, variable_type = info
                self.parse_variable_name(variable_name, variable_type)

            self.write_header()
            # from json import dumps
            # print(dumps(self.variable_names, indent=4))
            # print(dumps(self.name_type_map, indent=4))

            self.value_updated = [False] * len(self.variable_names)
            self.values = [None] * len(self.variable_names)

    def log(self, variable_name, value):
        # print(f'{variable_name}: {value.__class__.__name__} with value {value}')
        if isinstance(value, np.generic) \
                or isinstance(value, bool) \
                or isinstance(value, int) \
                or isinstance(value, float):
            self.parse_variable_value(variable_name, value)
        elif isinstance(value, np.matrix) or isinstance(value, np.ndarray):
            self.parse_matrix_value(variable_name, value)
        elif isinstance(value, list):
            self.parse_list_value(variable_name, value)
        elif hasattr(value, 'x') and hasattr(value, 'y') and hasattr(value, 'z'):
            self.parse_point_value(variable_name, value)
        else:
            print(f"Unknown type: {value.__class__.__name__}")

    def newline(self):
        if self.initialized:
            if not all(self.value_updated):
                print("Some value has not been updated!!!")
            timestamp = datetime.datetime.now().strftime("%Y, %m, %d, %H, %M, %S, %f")[:-3]
            line = f"{timestamp}"
            for value, updated in zip(self.values, self.value_updated):
                if not updated:
                    print("Some value has not been updated!!!")
                    break
                line += f", {value}"
            line += "\n"

            with open(self.filename, "a") as file:
                file.write(line)

            self.value_updated = [False] * len(self.variable_names)

    def parse_variable_name(self, variable_name, variable_type):
        variable_type = variable_type.lower()
        if variable_type in ["int", "double", "enum", "bool"]:
            self.name_type_map[variable_name] = variable_type
            self.variable_names.append(variable_name)
        elif "matrix" in variable_type or "vector" in variable_type:
            name = variable_name[:variable_name.find("[")]
            self.name_type_map[name] = variable_type
            num_rows = int(variable_name[variable_name.find("[") + 1:variable_name.find("]")])
            num_cols = int(variable_name[variable_name.rfind("[") + 1:variable_name.rfind("]")])

            for i in range(num_rows):
                for j in range(num_cols):
                    var_name = f"{name}[{i}][{j}]"
                    self.variable_names.append(var_name)
        elif "point" in variable_type:
            self.name_type_map[variable_name] = 'point'
            self.variable_names.append(f"{variable_name}.x")
            self.variable_names.append(f"{variable_name}.y")
            self.variable_names.append(f"{variable_name}.z")
        elif "list" in variable_type:
            self.name_type_map[variable_name] = 'list'
            name = variable_name[:variable_name.find("[")]

            if "-" in variable_name:
                start_pos = int(variable_name[variable_name.find("[") + 1:variable_name.find("-")])
                end_pos = int(variable_name[variable_name.find("-") + 1:variable_name.find("]")])
            else:
                start_pos = 0
                end_pos = int(variable_name[variable_name.find("[") + 1:variable_name.find("]")])

            self.array_name_start_end_map[name] = (start_pos, end_pos)

            for i in range(start_pos, end_pos):
                var_name = f"{name}[{i}]"
                self.variable_names.append(var_name)
        else:
            print(f"Unknown variable type: {variable_type}")

    def parse_variable_value(self, variable_name, value):
        index = self.variable_names.index(variable_name)
        self.values[index] = str(value)
        self.value_updated[index] = True

    def parse_matrix_value(self, variable_name, matrix):
        for index, element in np.ndenumerate(matrix):
            indices = variable_name + ''.join(f'[{i}]' for i in index)
            ind = self.variable_names.index(indices)
            self.values[ind] = str(element)
            self.value_updated[ind] = True
            # print(f"{indices}: {element}")

    def parse_list_value(self, variable_name, array):
        start_pos, end_pos = self.array_name_start_end_map[variable_name]
        for i in range(start_pos, end_pos):
            index = self.variable_names.index(f"{variable_name}[{i}]")
            self.values[index] = str(array[i])
            self.value_updated[index] = True

    def parse_point_value(self, variable_name, value):
        self.values[self.variable_names.index(f"{variable_name}.x")] = str(value.x)
        self.values[self.variable_names.index(f"{variable_name}.y")] = str(value.y)
        self.values[self.variable_names.index(f"{variable_name}.z")] = str(value.z)
        self.value_updated[self.variable_names.index(f"{variable_name}.x")] = True
        self.value_updated[self.variable_names.index(f"{variable_name}.y")] = True
        self.value_updated[self.variable_names.index(f"{variable_name}.z")] = True

    def write_header(self):
        with open(self.filename, "w") as file:
            file.write("Year,Month,Day,Hour,Minute,Second,Millisecond")
            for name in self.variable_names:
                file.write(f",{name}")
            file.write("\n")
