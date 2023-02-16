# Define Registers Values --------------------------------------------------------------------------#

X = 0 # X refer's to "don't care values", represented by *'s in the data sheet

# REG2 
PDALL = 0 
PDPLL = 0
PDVCO = 0
PDOUT = 0
PDFN = 0 
MTCAL = 1
OMUTE = 0 # **Change to 1 if oscillator port is left as open circuit 
POR = 0

# REG3
ALCEN = 0
ALCMON = 0
ALCCAL = 1
ALCULOK = 1 
AUTOCAL = 0
AUTORST = 1
DITHEN = 1 
INTN = 1

# REG4
BD = [0,1,0,1]
CPLE = 0
LDOEN = 1
LDOV = [1,1]

# REG5
SEED = [0,0,0,1,0,0,0,1]

# REG6 and REG7
RD = [0,1,1,0,0] # [0,0,0,1,0]
ND = [0,0,0,1,0,1,0,0,0,0] 

# REG7 - REGA 
NUM = [0,0,1,0,1,0,0,0,1,1,1,1,0,1,0,1,1,1] # 41943
RSTFN = 0
CAL = 0

# REGB
BST = 0; 
FILT = [0,0]
RFO = [1,1] 
OD = [0,1,0]

# REGC 
LKWIN = [1,0,1]
LKCT = [1,1]
CP = [1,1,1]

# REGD 
CPCHI = 0
CPCLO = 0
CPMID = 0
CPINV = 0 
CPWIDE = 0
CPRST = 0
CPUP = 0
CPDN = 0 

ALLCAL = 1

# Define Registers Contents --------------------------------------------------------------------------#

# REG0 and REG1 are left as default 
REG2 = [PDALL, PDPLL, PDVCO, PDOUT, PDFN, MTCAL, OMUTE, POR]
REG3 = [ALCEN, ALCMON, ALCCAL, ALCULOK, AUTOCAL, AUTORST, DITHEN, INTN]
REG4 = [BD[-1-3], BD[-1-2], BD[-1-1], BD[-1], CPLE, LDOEN, LDOV[-1-1], LDOV[-1]]
REG5 = SEED
REG6 = [RD[-1-4], RD[-1-3], RD[-1-2], RD[-1-1], RD[-1], X, ND[-1-9], ND[-1-8]] 
REG7 = [ND[-1-7], ND[-1-6], ND[-1-5], ND[-1-4], ND[-1-3], ND[-1-2], ND[-1-1], ND[-1]]
REG8 = [X, X, NUM[-1-17], NUM[-1-16], NUM[-1-15], NUM[-1-14], NUM[-1-13], NUM[-1-12]] 
REG9 = [NUM[-1-11], NUM[-1-10], NUM[-1-9], NUM[-1-8], NUM[-1-7], NUM[-1-6], NUM[-1-5], NUM[-1-4]]
REGA = [NUM[-1-3], NUM[-1-2], NUM[-1-1], NUM[-1], X, X, RSTFN, CAL]
REGB = [BST, FILT[-1-1], FILT[-1], RFO[-1-1], RFO[-1], OD[-1-2], OD[-1-1], OD[-1]]
REGC = [LKWIN[-1-2], LKWIN[-1-1], LKWIN[-1], LKCT[-1-1], LKCT[-1], CP[-1-2], CP[-1-1], CP[-1]]
REGD = [CPCHI, CPCLO, CPMID, CPINV, CPWIDE, CPRST, CPUP, CPDN]

# Functions ----------------------------------------------------------------------------------------#

# Prints the values in a given register in binary, hex, and binary
def print_register_values(REG, REG_NUM): 

    reg_contents_binary = ""
    reg_contents_binary_spaced = ""
    reg_contents_decimal = 0

    for k in range(0, len(REG)): 
        reg_contents_binary += str(REG[k])
        reg_contents_binary_spaced += str(REG[k]) + " "
        reg_contents_decimal = int(reg_contents_binary, 2)
    
    print("REG{}:  {}  =  0x{:02x}  =  {}".format(REG_NUM, reg_contents_binary_spaced, reg_contents_decimal, reg_contents_decimal))

# Main 
def main(): 
    print_register_values(REG2, "2")
    print_register_values(REG3, "3")
    print_register_values(REG4, "4")
    print_register_values(REG5, "5")
    print_register_values(REG6, "6")
    print_register_values(REG7, "7")
    print_register_values(REG8, "8")
    print_register_values(REG9, "9")
    print_register_values(REGA, "A")
    print_register_values(REGB, "B")
    print_register_values(REGC, "C")
    print_register_values(REGD, "D")

if __name__ == '__main__':
    main()