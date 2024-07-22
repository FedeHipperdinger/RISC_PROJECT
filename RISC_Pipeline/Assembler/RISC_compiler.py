instruction_set = {
    'load': '000',
    'store': '010',
    'loadi': '001',
    'storei': '011',
    'mov': '100',
    'or': '000',
    'inv': '001',
    'and': '010',
    'add': '000',
    'sub': '001',
    'addc': '010',
    'subc': '011',
    'nop': '111',
    'jump': '000',
    'bz': '001',
    'bnz': '010',
    'bc': '011',
    'bv': '100',
    'jal': '101',
    'jral': '110',
    'ret': '111',
    'stop': '111' # End program
}

type_map = {
    'load': '00',
    'store': '00',
    'loadi': '00',
    'storei': '00',
    'mov': '00',
    'or': '10',
    'inv': '10',
    'and': '10',
    'add': '11',
    'sub': '11',
    'addc': '11',
    'subc': '11',
    'nop': '11',
    'jump': '01',
    'bz': '01',
    'bnz': '01',
    'bc': '01',
    'bv': '01',
    'jal': '01',
    'jral': '01',
    'ret': '01',
    'stop': '00' # End program
}

# MOVEMENT = '00'
# LOGIC = '10'
# ARITHMETIC = '11'
# CONTROL = '01'


def to_bin(value, bits):
    return format(int(value), f'0{bits}b')

def code_instruction(instruction):
    parts = instruction.split()
    instr = parts[0]
    operands = parts[1].split(',')

    # Get the type and opcode
    T = type_map[instr]
    OPC = instruction_set[instr]

    # Generate the binary instruction
    if instr == 'stop':
        binary_instruction = f"{T}{OPC}{'0'*27}"
    elif instr == 'nop':
        binary_instruction = f"{T}{OPC}{'0'*27}"
    elif instr in ['load','jral']:
        Ra = to_bin(operands[0], 5)
        Rb = to_bin(operands[1], 5)
        Immediate_rc = to_bin(operands[2], 17)
        binary_instruction = f"{T}{OPC}{Immediate_rc}{Rb}{Ra}"
    elif instr in ['store', 'bz', 'bnz', 'bc', 'bv']:
        Ra = to_bin('0', 5)
        Rb = to_bin(operands[0], 5)
        Rc = to_bin(operands[1], 5)
        Immediate = to_bin(operands[2], 12)
        binary_instruction = f"{T}{OPC}{Immediate}{Rc}{Rb}{Ra}"
    elif instr in ['loadi','jal']:
        Ra = to_bin(operands[0], 5)
        Rb = to_bin('0', 5)
        Immediate_rc = to_bin(operands[1], 17)
        binary_instruction = f"{T}{OPC}{Immediate_rc}{Rb}{Ra}"
    elif instr == 'storei':
        Rb = to_bin(operands[0], 5)
        Ra = to_bin('0', 5)
        Immediate_rc = to_bin(operands[1], 17)
        binary_instruction = f"{T}{OPC}{Immediate_rc}{Rb}{Ra}"
    elif instr in [ 'or', 'and', 'add', 'sub', 'addc', 'subc']:
        Ra = to_bin(operands[0], 5)
        Rb = to_bin(operands[1], 5)
        Rc = to_bin(operands[2], 5)
        Immediate = to_bin('0', 12)
        binary_instruction = f"{T}{OPC}{Immediate}{Rc}{Rb}{Ra}"
    elif instr in ['mov', 'inv']:
        Ra = to_bin(operands[0], 5)
        Rb = to_bin(operands[1], 5)
        Rc = to_bin('0', 5)
        Immediate = to_bin('0', 12)
        binary_instruction = f"{T}{OPC}{Immediate}{Rc}{Rb}{Ra}"
    elif instr in ['jump', 'ret']:
        Ra = to_bin('0', 5)
        Rb = to_bin(operands[0], 5)
        Rc = to_bin('0', 5)
        Immediate = to_bin('0', 12)
        binary_instruction = f"{T}{OPC}{Immediate}{Rc}{Rb}{Ra}"
    else:
        raise ValueError(f"Instrucción no reconocida: {instr}")
    
    return binary_instruction



class RISCCompiler:
    def __init__(self):
        self.instructions = []

    def read_file(self, filename):
        with open(filename, 'r') as file:
            self.instructions = file.readlines()

    def compile(self, input_file, output_file):
        self.read_file(input_file)
        with open(output_file, 'w') as file:
            binary_instruction = code_instruction("nop 0,0,0") # Useless instruction to get the first instruction
            binary_instruction = binary_instruction[0:8] + ' ' + binary_instruction[8:16] + ' ' + binary_instruction[16:24] + ' ' + binary_instruction[24:32]
            file.write(binary_instruction + '\n')
            for instruction in self.instructions:
                try:
                    binary_instruction = code_instruction(instruction)
                    binary_instruction = binary_instruction[0:8] + ' ' + binary_instruction[8:16] + ' ' + binary_instruction[16:24] + ' ' + binary_instruction[24:32]
                    file.write(binary_instruction + '\n')
                except ValueError as e:
                    print(f"Error compilando la instrucción '{instruction.strip()}': {e}")
            binary_instruction = code_instruction("stop 0,0,0")
            binary_instruction = binary_instruction[0:8] + ' ' + binary_instruction[8:16] + ' ' + binary_instruction[16:24] + ' ' + binary_instruction[24:32]
            file.write(binary_instruction + '\n')

# Uso del compilador
compiler = RISCCompiler()
# input_file = input("Ingrese el nombre del archivo de entrada: ")
compiler.compile('program1.asm', 'output.bin')
