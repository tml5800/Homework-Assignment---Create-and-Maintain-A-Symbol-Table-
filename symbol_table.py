class SymbolTable:
    def __init__(self):
        self.table = {}

    def declare_variable(self, name, var_type, value):
        if name not in self.table:
            self.table[name] = {'type': var_type, 'value': value}
        else:
            print(f"Error: Variable '{name}' already declared.")

    def update_variable(self, name, value):
        if name in self.table:
            self.table[name]['value'] = value
        else:
            print(f"Error: Variable '{name}' not declared.")

    def print_table(self, title="Symbol Table"):
        print("\n" + title)
        for name, details in self.table.items():
            print(f"{name} -> Type: {details['type']}, Value: {details['value']}")

def main():
    st = SymbolTable()
    
    print("=== Variable Declaration ===")
    while True:
        var_input = input("Declare variable (name type value) or 'done': ")
        if var_input.lower() == "done":
            break
        try:
            name, var_type, value = var_input.split()
            if var_type == "int":
                value = int(value)
            elif var_type == "float":
                value = float(value)
            st.declare_variable(name, var_type, value)
        except ValueError:
            print("Invalid format. Use: name type value (e.g., x int 10)")

    st.print_table("Before Update")

    print("\n=== Update Variables ===")
    while True:
        update_input = input("Update variable (name new_value) or 'done': ")
        if update_input.lower() == "done":
            break
        try:
            name, value = update_input.split()
            if name in st.table:
                var_type = st.table[name]['type']
                if var_type == "int":
                    value = int(value)
                elif var_type == "float":
                    value = float(value)
                st.update_variable(name, value)
            else:
                print("Variable not declared.")
        except ValueError:
            print("Invalid format. Use: name new_value (e.g., x 50)")

    st.print_table("After Update")

if __name__ == "__main__":
    main()
